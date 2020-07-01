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
#include "interrupter.h"
#include "tsk_midi.h"
#include "tsk_cli.h"
#include "tsk_min.h"
#include "min_id.h"
uint8 tsk_usb_initVar;
xTaskHandle tsk_usb_TaskHandle;

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

        for (uint16_t i = 0; i * tsk_usb_BUFFER_LEN < tx_buf_size; ++i) {
            while (USBMIDI_1_CDCIsReady() == 0u) {
                vTaskDelay(1);
            }
            uint16_t offset = i * tsk_usb_BUFFER_LEN;
            uint16_t length = tx_buf_size - offset;
            if (length > tsk_usb_BUFFER_LEN) {
                length = tsk_usb_BUFFER_LEN;
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

/* ======================================================================== */
void tsk_usb_Task(void *pvParameters) {
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
        
            for(uint8_t i=0;i<NUM_MIN_CON;i++){
                if(socket_info[i].socket==SOCKET_DISCONNECTED) continue;
                size_t eth_bytes=xStreamBufferBytesAvailable(min_port[i].tx);
                if(eth_bytes){
                    if (eth_bytes > 64) eth_bytes = 64;
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
            }
		}
        vTaskDelay(10 / portTICK_PERIOD_MS);
	}
   for (;;) {
     vTaskDelay(10 / portTICK_PERIOD_MS);
   }
}
/* ======================================================================== */
/* [] END OF FILE */
