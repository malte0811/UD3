#ifndef tsk_audio_H
#define tsk_audio_H

#include "SignalGenerator.h"
#include "VMS_UD3.h"

void tsk_audio_Start();

void simulator_process_audio(SigGen_PulseBuffer* buffer, SigGen_pulseData_t* read_pulse);

#endif
