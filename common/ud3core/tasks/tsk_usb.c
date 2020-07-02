/* ======================================================================== */
/*
 * Copyright (c) 2015, E2ForLife.com
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the E2ForLife.com nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL E2FORLIFE.COM BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/* ======================================================================== */
#include <CyLib.h>
#include <cytypes.h>
#include <stdio.h>

#include "USBMIDI_1.h"
#include "USBMIDI_1_cdc.h"
#include "tsk_priority.h"
#include "tsk_usb.h"
#include "tsk_fault.h"
#include "alarmevent.h"


/* ======================================================================== */

#include "cli_common.h"
#include "clock.h"
#include "version.h"
#include "interrupter.h"
#include "tsk_midi.h"
#include "tsk_cli.h"
#include "tsk_min.h"
#include "min_id.h"
uint8 tsk_usb_initVar;
xTaskHandle tsk_usb_TaskHandle;
struct min_context min_ctx;
struct _time time;


#define BUFFER_SIZE 256
uint8_t tx_buf[BUFFER_SIZE];
uint16_t tx_buf_size = 0;
uint16_t min_tx_space(uint8_t port){
    return BUFFER_SIZE - tx_buf_size;
}

uint32_t min_rx_space(uint8_t port){
    return 64 - USBMIDI_1_GetCount();
}

void min_tx_byte(uint8_t port, uint8_t byte){
    tx_buf[tx_buf_size] = byte;
    ++tx_buf_size;
}



/* ======================================================================== */
void tsk_usb_Start(void) {
	if (tsk_usb_initVar != 1) {
		tsk_usb_Init();
	}

	/* Check for enumeration after initialization */
	tsk_usb_Enable();
}
/* ------------------------------------------------------------------------ */
void tsk_usb_Init(void) {
	/*  Initialize the USB COM port */

	if (USBMIDI_1_initVar == 0) {
/* Start USBFS Operation with 3V operation */
#if (CYDEV_VDDIO1_MV < 5000)
		USBMIDI_1_Start(0u, USBMIDI_1_3V_OPERATION);
#else
		USBMIDI_1_Start(0u, USBMIDI_1_5V_OPERATION);
#endif
	}

	xTaskCreate(tsk_usb_Task, "USB-Svc", STACK_USB, NULL, PRIO_USB, &tsk_usb_TaskHandle);

	tsk_usb_initVar = 1;
}
/* ------------------------------------------------------------------------ */
void tsk_usb_Enable(void) {
	if (tsk_usb_initVar != 0) {
		/*
		 * COMIO was initialized, and now is bing enabled for use.
		 * Enter user extension enables within the merge region below.
		 */
		/* `#START COMIO_ENABLE` */

		/* `#END` */
	}
}

uint16 count;
uint8 buffer[tsk_usb_BUFFER_LEN];

void tx_usb() {
 /* When component is ready to send more data to the PC */
    if ((USBMIDI_1_CDCIsReady() != 0u) && (tx_buf_size > 0)) {
        /*
         * Read the data from the transmit queue and buffer it
         * locally so that the data can be utilized.
         */

        /* Send data back to host */
        rx_blink_Write(1);

        for (uint16_t i = 0; i * 32 < tx_buf_size; ++i) {
            while (USBMIDI_1_CDCIsReady() == 0u) {
                vTaskDelay(1);
            }
            uint16_t offset = i * 32;
            uint16_t length = tx_buf_size - offset;
            if (length > 32) {
                length = 32;
            }
            USBMIDI_1_PutData(tx_buf + offset, length);
            /* If the last sent packet is exactly maximum packet size, 
             *  it shall be followed by a zero-length packet to assure the
             *  end of segment is properly identified by the terminal.
             */
            if (length == tsk_usb_BUFFER_LEN) {
                /* Wait till component is ready to send more data to the PC */
                while (USBMIDI_1_CDCIsReady() == 0u) {
                    vTaskDelay(1);
                }
                USBMIDI_1_PutData(NULL, 0u); /* Send zero-length packet to PC */
            }
        }
        tx_buf_size = 0;
    }   
}

/*===============================*/
#define LOCAL_UART_BUFFER_SIZE  127     //bytes
#define FLOW_RETRANSMIT_TICKS 50

#define ITEMS			32								//Stärke des Lowpass-Filters

typedef struct {
	uint8_t i;
	int32_t total;
	int32_t last;
	int32_t samples[ITEMS];
} average_buff;

average_buff sample;

/*******************************************************************************************
Makes avaraging of given values
In	= buffer (avarage_buff) for storing samples
IN	= new_sample (int)
OUT = avaraged value (int) 
********************************************************************************************/
int average (average_buff *buffer, int new_sample){
	buffer->total -= buffer->samples[buffer->i];
	buffer->total += new_sample;
	buffer->samples[buffer->i] = new_sample;
	buffer->i = (buffer->i+1) % ITEMS;
	buffer->last = buffer->total / ITEMS;
	return buffer->last;
}



uint32_t min_time_ms(void){
  return (xTaskGetTickCount() * portTICK_RATE_MS);
}

void min_reset(uint8_t port){
    kill_accu();
    USBMIDI_1_callbackLocalMidiEvent(0, (uint8_t*)kill_msg);   
    alarm_push(ALM_PRIO_WARN,warn_min_reset,ALM_NO_VALUE);
}

void min_tx_start(uint8_t port){
}
void min_tx_finished(uint8_t port){
}
void time_cb(uint32_t remote_time){
    time.remote = remote_time;
    time.diff_raw = time.remote-l_time;
    time.diff = average(&sample,time.diff_raw);
    if(time.diff>1000 ||time.diff<-1000){
        clock_set(time.remote);
        time.resync++;   
        //clock_reset_inc();
    }else{
        clock_trim(time.diff);
    }   
}

uint8_t flow_ctl=1;
static const uint8_t min_stop = 'x';
static const uint8_t min_start = 'o';


struct _socket_info socket_info[NUM_MIN_CON];

void process_synth(uint8_t *min_payload, uint8_t len_payload){
    len_payload--;
    switch(*min_payload++){
        case SYNTH_CMD_FLUSH:
            if(qSID!=NULL){
                xQueueReset(qSID);
            }
            break;
        case SYNTH_CMD_SID:
            param.synth=SYNTH_SID;
            switch_synth(SYNTH_SID);
            break;
        case SYNTH_CMD_MIDI:
            param.synth=SYNTH_MIDI;
            switch_synth(SYNTH_MIDI);
            break;
        case SYNTH_CMD_OFF:
            param.synth=SYNTH_OFF;
            switch_synth(SYNTH_OFF);
            break;
  }
}

void send_command(struct min_context *ctx, uint8_t cmd, char *str){
    uint8_t len=0;
    uint8_t buf[40];
    buf[0] = cmd;
    len=strlen(str);
    if(len>sizeof(buf)-1)len = sizeof(buf)-1;
    memcpy(&buf[1],str,len);
    min_queue_frame(ctx,MIN_ID_COMMAND,buf,len+1);
}
uint8_t transmit_features=0;

void min_application_handler(uint8_t min_id, uint8_t *min_payload, uint8_t len_payload, uint8_t port)
{
    switch(min_id){
        case 0 ... 9:
            if(min_id>(NUM_MIN_CON-1)) return;
            if(socket_info[min_id].socket==SOCKET_DISCONNECTED) return;
            xStreamBufferSend(min_port[min_id].rx,min_payload, len_payload,1);
            break;
        case MIN_ID_MIDI:
            switch(param.synth){
                case SYNTH_OFF:
       
                    break;
                case SYNTH_MIDI:
                    process_midi(min_payload,len_payload);     
                    break;
                case SYNTH_SID:
                    process_sid(min_payload, len_payload);
                    break;
                case SYNTH_MIDI_QCW:
                    process_midi(min_payload,len_payload);     
                    break;
                case SYNTH_SID_QCW:
                    process_sid(min_payload, len_payload);
                    break;
            }
            break;
        case MIN_ID_WD:
                if(len_payload==4){
                	time.remote  = ((uint32_t)min_payload[0]<<24);
			        time.remote |= ((uint32_t)min_payload[1]<<16);
			        time.remote |= ((uint32_t)min_payload[2]<<8);
			        time.remote |= (uint32_t)min_payload[3];
                    time_cb(time.remote);
                }
                WD_reset();
            break;
        case MIN_ID_SOCKET:
            if(*min_payload>(NUM_MIN_CON-1)) return;
            socket_info[*min_payload].socket = *(min_payload+1);
            strncpy(socket_info[*min_payload].info,(char*)min_payload+2,sizeof(socket_info[0].info));
            if(socket_info[*min_payload].socket==SOCKET_CONNECTED){
                command_cls("",&min_port[*min_payload]);
                send_string(":>", &min_port[*min_payload]);
                if(!transmit_features){
                    transmit_features=sizeof(version)/sizeof(char*);
                }
            }else{
                min_port[*min_payload].term_mode = PORT_TERM_VT100;    
                stop_overlay_task(&min_port[*min_payload]);   
            }
            break;
        case MIN_ID_SYNTH:
            process_synth(min_payload,len_payload);
            break;
    }
}
uint8_t assemble_command(uint8_t cmd, char *str, uint8_t *buf){
    uint8_t len=0;
    *buf = cmd;
    buf++;
    len=strlen(str);
    memcpy(buf,str,len);
    return len+1;
}



void send_command_wq(struct min_context *ctx, uint8_t cmd, char *str){
    uint8_t len=0;
    uint8_t buf[40];
    buf[0] = cmd;
    len=strlen(str);
    if(len>sizeof(buf)-1)len = sizeof(buf)-1;
    memcpy(&buf[1],str,len);
    min_send_frame(ctx,MIN_ID_COMMAND,buf,len+1);
}

void min_reset_flow(void){
    flow_ctl=0;
}

/* ======================================================================== */
void tsk_usb_Task(void *pvParameters) {
    min_init_context(&min_ctx, 0);
    
    for(uint8_t i=0;i<NUM_MIN_CON;i++){
        socket_info[i].socket=SOCKET_DISCONNECTED;   
        socket_info[i].old_state=SOCKET_DISCONNECTED;
        socket_info[i].info[0] = '\0';
    }
    
        
	/* `#END` */
    uint16_t bytes_waiting=0;
    
    uint32_t next_sid_flow = 0;
    alarm_push(ALM_PRIO_INFO,warn_task_min, ALM_NO_VALUE);
    
    send_command_wq(&min_ctx,CMD_HELLO_WORLD, configuration.ud_name);
    alarm_push(ALM_PRIO_INFO,warn_task_usb, ALM_NO_VALUE);
	for (;;) {
		/* Handle enumeration of USB port */
		if (USBMIDI_1_IsConfigurationChanged() != 0u) /* Host could send double SET_INTERFACE request */
		{
			if (USBMIDI_1_GetConfiguration() != 0u) /* Init IN endpoints when device configured */
			{
				/* Enumeration is done, enable OUT endpoint for receive data from Host */
				USBMIDI_1_CDC_Init();
				USBMIDI_1_MIDI_Init();
			}
		}
		/*
		 *
		 */
		if (USBMIDI_1_GetConfiguration() != 0u) {
/*
			 * Process received data from the USB, and store it in to the
			 * receiver message Q.
			 */

#if (!USBMIDI_1_EP_MANAGEMENT_DMA_AUTO)
			USBMIDI_1_MIDI_OUT_Service();
#endif

			if (USBMIDI_1_DataIsReady() != 0u) {
				count = USBMIDI_1_GetAll(buffer);
				if (count != 0u) {
					/* insert data in to Receive FIFO */
                    rx_blink_Write(1);
				}
                min_poll(&min_ctx, buffer, count);
			}
			/*
			 * Send a block of data ack through the USB port to the PC,
			 * by checkig to see if there is data to send, then sending
			 * up to the BUFFER_LEN of data (64 bytes)
			 */
            
			tx_usb();
            if(transmit_features){
                uint8_t temp=(sizeof(version)/sizeof(char*))-transmit_features;
                min_queue_frame(&min_ctx, MIN_ID_FEATURE, (uint8_t*)version[temp],strlen(version[temp]));  
                transmit_features--;
            }
        
            for(uint8_t i=0;i<NUM_MIN_CON;i++){
                if(socket_info[i].socket==SOCKET_DISCONNECTED) continue;
                 tx_usb();
                size_t eth_bytes=xStreamBufferBytesAvailable(min_port[i].tx);
                if(eth_bytes){
                    if (eth_bytes > 32) eth_bytes = 32;
                    uint8_t bytes_cnt = xStreamBufferReceive(min_port[i].tx, buffer, eth_bytes, 0);
                    if(bytes_cnt){
                        uint8_t res=0;
                        res = min_queue_frame(&min_ctx,i,buffer,bytes_cnt);
                        for(uint8_t j = 0; !res && j < 255; ++j){
                            tx_usb();
                            vTaskDelay(1 / portTICK_PERIOD_MS);   
                            res = min_queue_frame(&min_ctx,i,buffer,bytes_cnt);
                            min_poll(&min_ctx, NULL, 0);
                        }
                    }
                }
                
                /*if(param.synth==SYNTH_SID || param.synth==SYNTH_SID_QCW){
                    if(uxQueueSpacesAvailable(qSID) < 30 && flow_ctl){
                        min_queue_frame(&min_ctx, MIN_ID_MIDI, (uint8_t*)&min_stop,1);
                        flow_ctl=0;
                    }else if(uxQueueSpacesAvailable(qSID) > 45 && !flow_ctl){ //
                        min_queue_frame(&min_ctx, MIN_ID_MIDI, (uint8_t*)&min_start,1);
                        flow_ctl=1;
                    }else if(uxQueueSpacesAvailable(qSID) > 59){
                        if(xTaskGetTickCount()>next_sid_flow){
                            next_sid_flow = xTaskGetTickCount() + FLOW_RETRANSMIT_TICKS;
                            min_send_frame(&min_ctx, MIN_ID_MIDI, (uint8_t*)&min_start,1);
                            flow_ctl=1;
                        }
                    }
                }*/
            }
		}
        vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}
/* ======================================================================== */
/* [] END OF FILE */
