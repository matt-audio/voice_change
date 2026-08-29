#ifndef VC_ROBOT_MONSTER_FEMALE_H
#define VC_ROBOT_MONSTER_FEMALE_H

#include "pitched_voice.h"

#define FEMALE_PROCESS_ERROR PITCHED_VOICE_PROCESS_ERROR
#define FEMALE_PROCESS_NOT_READY PITCHED_VOICE_PROCESS_NOT_READY
#define FEMALE_PROCESS_OUTPUT PITCHED_VOICE_PROCESS_OUTPUT

typedef PitchedVoiceState FemaleState;
typedef PitchedVoiceProfile FemaleProfile;

FemaleState *female_init(int sample_rate);
void female_free(FemaleState *state);
int female_process(FemaleState *state,
                   const float input[DSP_HOP_SIZE],
                   float output[DSP_HOP_SIZE]);
void female_get_profile(const FemaleState *state, FemaleProfile *profile);

#endif
