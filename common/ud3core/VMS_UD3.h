#ifndef VMS_UD3_H
#define VMS_UD3_H
    #include <stdint.h>
    #include "RingBuffer/include/RingBuffer.h"
    #include "SignalGenerator.h"

    typedef struct {
        volatile int32_t bufferLengthInCounts; /**< Target pulse buffer fill level in timer counts */
        volatile RingBuffer_t * pulseBuffer;   /**< Ring buffer for pulse queue (128 entries) */
    } SigGen_PulseBuffer;

    /**
     * @brief Parameter change callback for siggen config
     * @param params Parameter table
     * @param index Index of changed parameter
     * @param handle Terminal handle for error messages
     * @return pdTRUE to accept change, pdFALSE to reject
     */
    uint8_t callback_siggen(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);

    void SigGen_init();

    void SigGen_killAudio();

    /* ===== Pulse Queue Interface ===== */
    
    /**
     * @brief Queue a pulse for hardware output
     * @param pulse Pointer to pulse descriptor (period, onTime, current)
     * @return 1 if pulse queued successfully, 0 if buffer full
     *
     * Units:
     * - Input: period/onTime in microseconds, current in siggen volume (0-INT16_MAX)
     * - Internally converted to timer counts and DAC values before queuing
     *
     * Thread-safe (uses ring buffer primitives).
     */
    uint8_t SigGen_queuePulse(SigGen_pulseData_t const* pulse);

    /**
     * @brief Initialize duty cycle compressor
     */
    void Comp_init();

    /**
     * @brief Initialize VMS wrapper
     *
     * Allocates voice data arrays, initializes VMS core, and creates VMS task.
     * Must be called once at system startup.
     */
    void VMSW_init();


#endif
