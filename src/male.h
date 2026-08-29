#ifndef VC_ROBOT_MONSTER_MALE_H
#define VC_ROBOT_MONSTER_MALE_H

#include "pitched_voice.h"

#define MALE_PROCESS_ERROR PITCHED_VOICE_PROCESS_ERROR
#define MALE_PROCESS_NOT_READY PITCHED_VOICE_PROCESS_NOT_READY
#define MALE_PROCESS_OUTPUT PITCHED_VOICE_PROCESS_OUTPUT

typedef PitchedVoiceState MaleState;
typedef PitchedVoiceProfile MaleProfile;

MaleState *male_init(int sample_rate);
void male_free(MaleState *state);
int male_process(MaleState *state,
                 const float input[DSP_HOP_SIZE],
                 float output[DSP_HOP_SIZE]);
void male_get_profile(const MaleState *state, MaleProfile *profile);

#endif
