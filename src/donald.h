#ifndef VC_ROBOT_MONSTER_DONALD_H
#define VC_ROBOT_MONSTER_DONALD_H

#include "pitched_voice.h"

#define DONALD_PROCESS_ERROR PITCHED_VOICE_PROCESS_ERROR
#define DONALD_PROCESS_NOT_READY PITCHED_VOICE_PROCESS_NOT_READY
#define DONALD_PROCESS_OUTPUT PITCHED_VOICE_PROCESS_OUTPUT

typedef PitchedVoiceState DonaldState;
typedef PitchedVoiceProfile DonaldProfile;

DonaldState *donald_init(int sample_rate);
void donald_free(DonaldState *state);
int donald_process(DonaldState *state,
                   const float input[DSP_HOP_SIZE],
                   float output[DSP_HOP_SIZE]);
void donald_get_profile(const DonaldState *state, DonaldProfile *profile);

#endif
