#include "VMS_UD3.h"
#include "DutyCompressor.h"
#include "SignalGenerator.h"
#include "VMSWrapper.h"
#include "ZCDtoPWM.h"
#include "clock.h"
#include "interrupter.h"
#include "qcw.h"
#include "tsk_audio.h"

/** @brief Check if pulse timer is currently running */
#define SigGen_isTimerRunning() (interrupterTimebase_ReadControlRegister() & interrupterTimebase_CTRL_ENABLE)
/** @brief Start pulse timer and enable ISR */
#define SigGen_startTimer() interrupterTimebase_WriteControlRegister(interrupterTimebase_ReadControlRegister() | interrupterTimebase_CTRL_ENABLE); SigGen_enableTimerISR();
/** @brief Stop pulse timer and disable ISR */
#define SigGen_stopTimer() interrupterTimebase_WriteControlRegister(interrupterTimebase_ReadControlRegister() & ~interrupterTimebase_CTRL_ENABLE); SigGen_disableTimerISR();
/** @brief Check if timer ISR is enabled */
#define SigGen_isTimerISREnabled() interrupterIRQ_GetState()
/** @brief Disable timer ISR */
#define SigGen_disableTimerISR() interrupterIRQ_Disable()
/** @brief Enable timer ISR */
#define SigGen_enableTimerISR() interrupterIRQ_Enable()

/** @brief Convert microseconds to timer period counts (32kHz timer: 32 counts/µs) */
#define SIGGEN_US_TO_PERIOD_COUNT(X) (X) * 32
/** @brief Convert microseconds to on-time counts (1:1 for hardware PWM) */
#define SIGGEN_US_TO_OT_COUNT(X) (X)
/** @brief Convert siggen volume (0-INT16_MAX) to DAC current value using linear scaling */
#define SIGGEN_VOLUME_TO_CURRENT_DAC_VALUE(X) (params.min_tr_cl_dac_val + (((X) * params.diff_tr_cl_dac_val) >> 15))

static SigGen_PulseBuffer pulse_buffer;

/** @brief Current pulse being timed by hardware ISR (pre-loaded for next period) */
static SigGen_pulseData_t readPulse;

/**
 * @brief 8kHz system tick ISR - clock and QCW control
 *
 * High-priority ISR called at MIDI_ISR_Hz (8000 Hz) by hardware timer.
 * Handles:
 * - Global clock tick (for uptime tracking)
 * - QCW mode ramping (if QCW_enable_Control active)
 *
 * Execution time: ~10µs (clock_tick) or ~50µs (qcw_handle)
 *
 * @note In QCW mode, bypasses normal signal generation to run qcw_handle()
 */
CY_ISR(isr_synth) {   
    clock_tick();
    if(QCW_enable_Control){
        qcw_handle();
        return;
    }
}

/**
 * @brief Hardware pulse timer ISR - consume pulses from ring buffer
 *
 * Called when interrupterTimebase timer expires (variable rate, depends on pulse periods).
 * Workflow:
 * 1. Command previous pulse to hardware via interrupter_oneshotRaw()
 * 2. Read next pulse from ring buffer
 * 3. Load next pulse period into timer compare register
 * 4. If buffer empty, stop timer
 *
 * Execution time: ~5-15µs depending on buffer state
 *
 * @note Uses ISR-safe RingBuffer_readFromISR() for thread safety
 * @note Zero-period pulses are rejected and retried (invalid state)
 * @note Timer stops automatically when buffer empty
 */
CY_ISR(SigGen_PulseTimerISR){
    interrupterTimebase_ReadStatusRegister();
    interrupterIRQ_ClearPending();
    
    //start the previous pulse
    if(!(readPulse.current == 0 || readPulse.onTime == 0)){
        if(configuration.is_qcw == 0 || SigGen_getSynthMode() == SYNTH_TR){ //Don't command a pulse in QCW mode... For now.
            interrupter_oneshotRaw(readPulse.onTime, readPulse.current);
        }
    }
    
    //try to read the next pulse
    while(1){
        if(RingBuffer_readFromISR(pulse_buffer.pulseBuffer, (void*)&readPulse, 1) == 1){
            //check if we got valid pulse and if not retry
            if(readPulse.period == 0){ 
                readPulse.period = 1;
                continue;
            }
            
            //and finally reduce the buffer size
            pulse_buffer.bufferLengthInCounts -= readPulse.period;
            
            if(readPulse.period < SIGGEN_MIN_PERIOD){ 
                //TODO evaluate occurance of this happening. Should be impossible and if it does happen it ruins the entire note timebase...
                readPulse.period = SIGGEN_MIN_PERIOD;
            }
            
            //load timer registers
            interrupterTimebase_WriteCompare(readPulse.period);
            
            //we got a valid time => exit loop
            break;

            //is the timer already running longer than the period? if so make it trigger as soon as possible
            //TODO evaluate if this is actually neccessary. After all the timer compare mode is set to ">=", so setting a compare value lower than the counter should trigger a pulse right away anyway
        }else{
            //no more pulses in the buffer or other error. Turn off the timer 
            SigGen_stopTimer();
            
            //also there is no way that there is still some time left in the buffer... clear it just in case
            pulse_buffer.bufferLengthInCounts = 0;
            
            //no more pulses could be read out => jsut exit from the loop
            break;
        }
        
    }
}

/**
 * @brief Queue a pulse for hardware output (converts units and writes to ring buffer)
 *
 * Conversion process:
 * 1. Input pulse in microseconds and siggen volume (0-INT16_MAX)
 * 2. Convert period: µs → timer counts (32 counts/µs at 32kHz timer)
 * 3. Convert onTime: µs → timer counts (1:1 for hardware PWM)
 * 4. Convert current: siggen volume → DAC counts via linear scaling
 *    DAC value = min_tr_cl_dac_val + ((current * diff_tr_cl_dac_val) >> 15)
 * 5. Write to ring buffer, update bufferLengthInCounts
 *
 * Thread safety:
 * - Uses RingBuffer_write() (thread-safe)
 * - Disables timer ISR during bufferLengthInCounts update
 *
 * @param pulse Pointer to pulse descriptor in microseconds (period, onTime, current)
 * @return 1 if pulse queued successfully, 0 if buffer full
 *
 * @note Called by SigGen_task() to feed pulses to hardware ISR
 * @note Buffer capacity: 64 entries (SIGGEN_PULSEBUFFER_SIZE)
 */
uint8_t SigGen_queuePulse(SigGen_pulseData_t const* pulse) {
    //convert the period, volume and ontime to the values that will need to be written into the hardware upon pulse execution
    SigGen_pulseData_t raw_pulse;
    raw_pulse.period = SIGGEN_US_TO_PERIOD_COUNT(pulse->period);
    raw_pulse.onTime = SIGGEN_US_TO_OT_COUNT(pulse->onTime);
    raw_pulse.current = SIGGEN_VOLUME_TO_CURRENT_DAC_VALUE(pulse->current);

    //write it into the buffer
    if(RingBuffer_write(pulse_buffer.pulseBuffer, (void*)&raw_pulse, 1, 0) != 1){
        //write failed, not enough space available...
        return 0;
    }else{
        //increase the buffersize
        SigGen_disableTimerISR();
        pulse_buffer.bufferLengthInCounts += raw_pulse.period;
        SigGen_enableTimerISR();
        return 1;
    }
}

uint8_t callback_siggen(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    SigGen_update_min_ot();
    
    return pdPASS;
}

static int32_t get_buffer_length() {
    return pulse_buffer.bufferLengthInCounts;
}

/** @brief Forward declaration of main signal generation task */
static void SigGen_task(void * params) {
    while(1){
        vTaskDelay(1);
        SigGen_generate(SigGen_queuePulse, get_buffer_length);
        
        //it is possible that the timer is turned of at this point in the code but a pulse is waiting. If that is the case the timer needs to be kickstarted so it can begin reading out more pulses by itself
        if(!SigGen_isTimerRunning() && RingBuffer_getDataCount(pulse_buffer.pulseBuffer) > 0){
            //time is off but pulses are waiting. Load the first pulse and start the timer
            
            //get the next pulse
            if(RingBuffer_read(pulse_buffer.pulseBuffer, (void*)&readPulse, 1) == 1){
                pulse_buffer.bufferLengthInCounts -= readPulse.period;
                
                if(readPulse.period < SIGGEN_MIN_PERIOD) readPulse.period = SIGGEN_MIN_PERIOD;
                
                //reset counter to trigger asap
                interrupterTimebase_WriteCounter(0);
                
                //load timer registers
                interrupterTimebase_WriteCompare(readPulse.period);
                
                //clear the irq incase its still pending
                interrupterIRQ_ClearPending();
                
                //and finally re-enable the timer
                SigGen_startTimer();
            }
            
            //wait what? Read failed although there is supposedly data in the buffer... anyway, forget what we are doing and just carry on the loop
        }
#ifdef SIMULATOR
        simulator_process_audio(&pulse_buffer, &readPulse);
#endif
    }
}

    
void SigGen_init() {
    SigGen_init_data();

    pulse_buffer.pulseBuffer = RingBuffer_create(64, sizeof(SigGen_pulseData_t));
    pulse_buffer.bufferLengthInCounts = 0;
    
    //initialize timers
    
    //Timer 2&3 generate the signal period. 32Bit mode, no prescaler
    interrupterTimebase_Init();
    interrupterIRQ_StartEx(SigGen_PulseTimerISR);
    
    isr_midi_StartEx(isr_synth);

    xTaskCreate(SigGen_task, "SigGen", configMINIMAL_STACK_SIZE+128, NULL, tskIDLE_PRIORITY + 4, NULL);
}

void SigGen_killAudio() {
    SigGen_killAudio_data();
    if (!pulse_buffer.pulseBuffer) { return; }
    //reset the buffer, which must be done with the timer interrupt disabled to prevent intereference with the bufferLengthInCounts write
    SigGen_disableTimerISR();
    RingBuffer_flush(pulse_buffer.pulseBuffer);
    pulse_buffer.bufferLengthInCounts = 0;
    //kill the timer
    SigGen_stopTimer();
    
    SigGen_enableTimerISR();
}

static void COMP_task(void * params){
    while(1){
        //TODO maybe make this thread safe? Or at least verify that this will not try to compress right in the middle of some voice values being updated
        COMP_compress();
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void Comp_init(){
    xTaskCreate(COMP_task, "COMP", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
}

static void VMSW_task(void * params){
    while(1){
        //run VMS service
        
        //does the synth mode require vms?
        if(param.synth == SYNTH_MIDI){
            //yes => run it
            VMS_run();
        }else{
            //no => save some cpu cycles
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        vTaskDelay(1);
    }
}

void VMSW_init(){
    VMSW_init_data();
    xTaskCreate(VMSW_task, "VMS Task", configMINIMAL_STACK_SIZE+256, NULL, tskIDLE_PRIORITY + 3, NULL);
}

