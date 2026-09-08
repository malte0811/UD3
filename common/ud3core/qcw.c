/*
 * UD3
 *
 * Copyright (c) 2018 Jens Kerrinnes
 * Copyright (c) 2015 Steve Ward
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "qcw.h"
#include "hardware.h"
// TODO sim only

#include "ZCDtoPWM.h"
#include "helper/teslaterm.h"
#include "min_id.h"
#include "tasks/tsk_min.h"
#include "telemetry.h"

#define QCW_CORRECT_LINEAR 0
#define QCW_CORRECT_VOLTAGE 1
#define QCW_CORRECT_POWER 2
// See scripts/generate_cosine_correction.py
static uint8_t equiv_voltage[256] = {
    0, 1, 1, 2, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 10, 11, 11, 12, 13, 13, 14, 15, 15, 16, 17, 17, 18, 19, 19, 20, 20,
    21, 22, 22, 23, 24, 24, 25, 26, 26, 27, 28, 28, 29, 29, 30, 31, 31, 32, 33, 33, 34, 35, 35, 36, 37, 37, 38, 39, 39,
    40, 41, 41, 42, 43, 43, 44, 44, 45, 46, 46, 47, 48, 48, 49, 50, 50, 51, 52, 52, 53, 54, 54, 55, 56, 57, 57, 58, 59,
    59, 60, 61, 61, 62, 63, 63, 64, 65, 65, 66, 67, 68, 68, 69, 70, 70, 71, 72, 72, 73, 74, 75, 75, 76, 77, 77, 78, 79,
    80, 80, 81, 82, 82, 83, 84, 85, 85, 86, 87, 88, 88, 89, 90, 91, 91, 92, 93, 94, 94, 95, 96, 97, 97, 98, 99, 100,
    101, 101, 102, 103, 104, 104, 105, 106, 107, 108, 108, 109, 110, 111, 112, 113, 113, 114, 115, 116, 117, 118, 118,
    119, 120, 121, 122, 123, 124, 125, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 136, 137, 138, 139,
    140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 162, 163, 164,
    165, 166, 168, 169, 170, 171, 173, 174, 175, 177, 178, 180, 181, 183, 184, 186, 187, 189, 190, 192, 194, 195, 197,
    199, 201, 203, 205, 207, 209, 212, 214, 217, 220, 223, 226, 230, 235, 241, 255
};
static uint8_t equiv_power[256] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4,
    4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13,
    14, 14, 14, 15, 15, 16, 16, 16, 17, 17, 18, 18, 19, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 26,
    27, 27, 28, 28, 29, 29, 30, 30, 31, 32, 32, 33, 33, 34, 34, 35, 36, 36, 37, 37, 38, 39, 39, 40, 41, 41, 42, 43, 43,
    44, 45, 45, 46, 47, 48, 48, 49, 50, 50, 51, 52, 53, 53, 54, 55, 56, 57, 57, 58, 59, 60, 61, 61, 62, 63, 64, 65, 66,
    67, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94,
    96, 97, 98, 99, 100, 101, 103, 104, 105, 106, 108, 109, 110, 111, 113, 114, 115, 117, 118, 120, 121, 122, 124, 125,
    127, 128, 130, 131, 133, 135, 136, 138, 140, 141, 143, 145, 147, 149, 150, 152, 154, 156, 158, 160, 163, 165, 167,
    169, 172, 174, 177, 179, 182, 185, 188, 191, 194, 198, 201, 205, 210, 214, 220, 226, 235, 255
};

ramp_params volatile ramp;

TimerHandle_t xQCW_Timer;

static uint8_t shift_for_relative_voltage(uint8_t relative_voltage) {
    if (configuration.qcw_correction == QCW_CORRECT_LINEAR) {
        return relative_voltage;
    } else if (configuration.qcw_correction == QCW_CORRECT_VOLTAGE) {
        return equiv_voltage[relative_voltage];
    } else /*if (configuration.qcw_correction == QCW_CORRECT_POWER)*/ {
        return equiv_power[relative_voltage];
    }
}

static void qcw_modulate(uint8_t relative_voltage){
    CT1_dac_SetValue(relative_voltage);
    /*
    //linearize modulation value based on fb_filter_out period
	uint8_t shift_period = (((uint16_t) relative_shift) * (params.pwm_top - fb_filter_out)) >> 8;
	//assign new modulation value to the params.pwmb_psb_val ram
	if ((shift_period + params.pwmb_start_psb_val) > (params.pwmb_start_prd - 4)) {
		params.pwmb_psb_val = 4;
	} else {
		params.pwmb_psb_val = params.pwm_top - (shift_period + params.pwmb_start_psb_val);
	}
    */
}

void qcw_handle() {
    if (ramp.index >= ramp.stop_index) {
        //qcw_modulate(0);
        QCW_enable_Control = 0;
        params.pwmb_psb_val = 0;
        ramp.index = 0;
    }else{
        qcw_modulate(ramp.data[ramp.index]);
        ramp.index++;
    }
}

bool qcw_ramp_changed = false;

void send_qcw_ramp_to_tt() {
    uint16_t active_length = QCW_RAMP_SAMPLES;
    // Ensure that active_length is always positive so we send at least one packet even for zero ramps
    while (active_length > 1 && ramp.data[active_length - 1] == 0) {
        --active_length;
    }
    uint8_t ramp_byte_per_frame = 200;
    uint8_t payload_length = 2 + ramp_byte_per_frame;
    uint8_t* temp_buffer = pvPortMalloc(payload_length);
    for (uint16_t next_byte = 0; next_byte < active_length; next_byte += ramp_byte_per_frame) {
        bool is_last = next_byte + ramp_byte_per_frame >= active_length;
        uint8_t ramp_bytes_this_frame = is_last ? active_length - next_byte : ramp_byte_per_frame;
        temp_buffer[0] = (next_byte >> 8) | (is_last << 7);
        temp_buffer[1] = next_byte & 0xff;
        memcpy(temp_buffer + 2, ramp.data + next_byte, ramp_bytes_this_frame);
        min_queue_frame(&min_ctx, MIN_ID_QCW_RAMP, temp_buffer, ramp_bytes_this_frame + 2);
    }
    vPortFree(temp_buffer);
}

void qcw_regenerate_ramp(){
    if(!ramp.changed){ return; }
    uint32_t modulation_period = roundf((10.0f / (float)param.qcw_freq) / 0.00025f);  //Frequency in tenths

    // Top of QCW ramp. If modulation is used, this refers to the lower level
    uint32_t ramp_max = param.qcw_max;
    // Clamp the max down to fit the volume
    if((ramp_max + param.qcw_vol) > 255) { ramp_max = 255 - param.qcw_vol;  }

    uint16_t pw = param.qcw_pw;
    if (pw > configuration.max_qcw_pw) { pw = configuration.max_qcw_pw; }

    uint32_t max_active = (pw*10)/MIDI_ISR_US;
    if (max_active > sizeof(ramp.data)) { max_active = sizeof(ramp.data); }
    ramp.stop_index = max_active;

    float const ramp_exponent = param.qcw_exponent / 100.f;

    // Generate ramp: Stay at qcw_offset for qcw_holdoff samples. Afterwards, increase the "base ramp" at ramp_increment
    // per sample until ramp_max is reached. If modulation is enabled, add a square wave from 0 to qcw_vol and
    // frequency qcw_freq to this ramp.
    uint8_t modulation_high = pdFALSE;
    float ramp_increment = param.qcw_ramp / 100.0;
    float ramp_val = 0;
    for(uint16_t i=0;i<max_active;i++){
        uint16_t value=floorf(ramp_val);
        if(i>=param.qcw_holdoff){
            ramp_val += ramp_increment;

            if(param.qcw_vol > 0){
                if((i % modulation_period) == 0){
                    modulation_high = !modulation_high;
                }
                if(modulation_high){
                    value += param.qcw_vol;
                }
            }
        }
        float relative_current = pow(((float) value) / 255.f, ramp_exponent);
        relative_current += param.qcw_offset / 255.f;
        if (relative_current > 1) { relative_current = 1; }
        ramp.data[i] = current_to_ct1_dac_value(relative_current * configuration.max_qcw_current);
    }
    ramp.data[0] = current_to_ct1_dac_value(param.qcw_first_sample);
    // Fill inactive portion of QCW buffer with zeroes for clean display in TT
    memset(ramp.data + max_active, 0, QCW_RAMP_SAMPLES - max_active);
    ramp.changed = pdFALSE;

    send_qcw_ramp_to_tt();
}

void qcw_process_ramp_packet(uint8_t* data, uint8_t msg_length) {
    if (msg_length < 3) { return; }
    bool is_last = (data[0] & 0x80) != 0;
    uint16_t offset = ((data[0] & 0x7f) << 8) | data[1];
    uint8_t data_length = msg_length - 2;
    uint16_t byte_after = offset + data_length;
    if (byte_after > QCW_RAMP_SAMPLES) { return; }
    // TODO also stop after max qcw pw from config
    memcpy(ramp.data + offset, data + 2, data_length);
    if (is_last) {
        memset(ramp.data + byte_after, 0, QCW_RAMP_SAMPLES - byte_after);
        //send_qcw_ramp_to_tt();
        qcw_ramp_changed = true;
    }
}

void qcw_cmd_midi_pulse(int32_t volume, int32_t frequencyTenths){
    param.qcw_freq = frequencyTenths;
    ramp.changed = pdTRUE;
    qcw_regenerate_ramp();
    qcw_start();
}

void qcw_ramp_point(uint16_t x,uint8_t y){
    if(x<sizeof(ramp.data)){
        ramp.data[x] = y;
    }
}

void qcw_ramp_line(uint16_t x0,uint8_t y0,uint16_t x1, uint8_t y1){
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int err = (dx > dy ? dx : -dy) / 2, e2;

	for (;;) {
		qcw_ramp_point(x0, y0);
		if (x0 == x1 && y0 == y1)
			break;
		e2 = err;
		if (e2 > -dx) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy) {
			err += dx;
			y0 += sy;
		}
	}
}


void qcw_ramp_visualize(CHART *chart, TERMINAL_HANDLE * handle){
    for(uint16_t i = 0; i<sizeof(ramp.data)-1;i++){
        send_chart_line(chart->offset_x+i,chart->height+chart->offset_y-ramp.data[i],chart->offset_x+i+1,chart->height+chart->offset_y-ramp.data[i+1], TT_COLOR_GREEN ,handle);
    }

    uint16_t red_line = configuration.max_qcw_pw*10 / MIDI_ISR_US;
    send_chart_line(chart->offset_x+red_line,chart->offset_y,chart->offset_x+red_line,chart->offset_y+chart->height, TT_COLOR_RED, handle);

    uint16_t blue_line = param.qcw_pw*10 / MIDI_ISR_US;
    send_chart_line(chart->offset_x+blue_line,chart->offset_y,chart->offset_x+blue_line,chart->offset_y+chart->height, TT_COLOR_BLUE, handle);

}

void qcw_start(){
    if(tt.n.dutycycle.value > configuration.max_qcw_duty) return;  //Don't command a pulse if duty is too high

    ramp.index=0;
    qcw_modulate(ramp.data[0]);
	//the next stuff is time sensitive, so disable interrupts to avoid glitches
	CyGlobalIntDisable;
	//now enable the QCW interrupter
	QCW_enable_Control = 1;
	params.pwmb_psb_val = params.pwm_top - params.pwmb_start_psb_val;
	CyGlobalIntEnable;
}

void qcw_stop(){
    QCW_enable_Control = 0;
    params.pwmb_psb_val = 0;
}

uint8_t callback_rampFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    ramp.changed = pdTRUE;
    if(!QCW_enable_Control){
        qcw_regenerate_ramp();
    }

    return pdPASS;
}



uint8_t CMD_ramp(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    if(argCount==0 || strcmp(args[0], "-?") == 0){
        ttprintf(   "Usage: ramp line x1 y1 x2 y2\r\n"
                    "       ramp point x y\r\n"
                    "       ramp clear\r\n"
                    "       ramp draw\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }
    if (!configuration.is_qcw) {
       ttprintf("Ramp control is only available for QCW coils\r\n");
       return TERM_CMD_EXIT_SUCCESS;
    }


    if(strcmp(args[0], "point") == 0 && argCount == 3){
        int x = atoi(args[1]);
        int y = atoi(args[2]);
        qcw_ramp_point(x,y);
        return TERM_CMD_EXIT_SUCCESS;

    } else if(strcmp(args[0], "line") == 0 && argCount == 5){
        int x0 = atoi(args[1]);
        int y0 = atoi(args[2]);
        int x1 = atoi(args[3]);
        int y1 = atoi(args[4]);
        qcw_ramp_line(x0,y0,x1,y1);
        return TERM_CMD_EXIT_SUCCESS;

    } else if(strcmp(args[0], "clear") == 0){
        for(uint16_t i = 0; i<sizeof(ramp.data);i++){
            ramp.data[i] = 0;
        }
        return TERM_CMD_EXIT_SUCCESS;
    } else if(strcmp(args[0], "draw") == 0){
        port_str * ptr = handle->port;
        if(ptr->term_mode == PORT_TERM_VT100){
            ttprintf("Command only available with Teslaterm\r\n");
            return TERM_CMD_EXIT_SUCCESS;
        }
        send_chart_clear(handle, "QCW ramp");
        CHART temp;
        temp.height = RAMP_CHART_HEIGHT;
        temp.width = RAMP_CHART_WIDTH;
        temp.offset_x = RAMP_CHART_OFFSET_X;
        temp.offset_y = RAMP_CHART_OFFSET_Y;
        temp.div_x = RAMP_CHART_DIV_X;
        temp.div_y = RAMP_CHART_DIV_Y;

        tt_chart_init(&temp,handle);
        qcw_ramp_visualize(&temp,handle);
        return TERM_CMD_EXIT_SUCCESS;
    }
     return TERM_CMD_EXIT_SUCCESS;
}

/*****************************************************************************
* Timer callback for the QCW autofire
******************************************************************************/
void vQCW_Timer_Callback(TimerHandle_t xTimer){
    qcw_regenerate_ramp();
    qcw_start();
    if(param.qcw_repeat<100) param.qcw_repeat = 100;
    xTimerChangePeriod( xTimer, param.qcw_repeat / portTICK_PERIOD_MS, 0 );
}

BaseType_t QCW_delete_timer(void){
    if (xQCW_Timer != NULL) {
    	if(xTimerDelete(xQCW_Timer, 200 / portTICK_PERIOD_MS) != pdFALSE){
            xQCW_Timer = NULL;
            return pdPASS;
        }else{
            return pdFAIL;
        }
    }else{
        return pdFAIL;
    }
}

/*****************************************************************************
* starts the QCW mode. Spawns a timer for the automatic QCW pulses.
******************************************************************************/
uint8_t CMD_qcw(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    if(argCount==0 || strcmp(args[0], "-?") == 0){
        ttprintf("Usage: qcw [start|stop]\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }

    if(configuration.is_qcw == pdFALSE){
        ttprintf("This is not a QCW coil. Set [qcw_coil] to 1.\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }

	if(strcmp(args[0], "start") == 0){
        if(param.qcw_repeat>99){
            if(xQCW_Timer==NULL){
                xQCW_Timer = xTimerCreate("QCW-Tmr", param.qcw_repeat / portTICK_PERIOD_MS, pdFALSE,(void * ) 0, vQCW_Timer_Callback);
                if(xQCW_Timer != NULL){
                    xTimerStart(xQCW_Timer, 0);
                    ttprintf("QCW Enabled\r\n");
                }else{
                    ttprintf("Cannot create QCW Timer\r\n");
                }
            }
        }else{
            qcw_regenerate_ramp();
		    qcw_start();
            ttprintf("QCW single shot\r\n");
        }
		
		return TERM_CMD_EXIT_SUCCESS;
	}
	if(strcmp(args[0], "stop") == 0){
        if (xQCW_Timer != NULL) {
				if(!QCW_delete_timer()){
                    ttprintf("Cannot delete QCW Timer\r\n");
                }
		}
        qcw_stop();
		ttprintf("QCW Disabled\r\n");
		return TERM_CMD_EXIT_SUCCESS;
	}
	return TERM_CMD_EXIT_SUCCESS;
}
