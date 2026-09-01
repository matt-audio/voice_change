#ifndef VC_ROBOT_MONSTER_PITCHED_VOICE_H
#define VC_ROBOT_MONSTER_PITCHED_VOICE_H

#include "dsp.h"

#define PITCHED_VOICE_MAX_PEAKS 3

#define PITCHED_VOICE_PROCESS_ERROR (-1)
#define PITCHED_VOICE_PROCESS_NOT_READY 0
#define PITCHED_VOICE_PROCESS_OUTPUT 1

typedef enum {
    PITCHED_VOICE_RATE_DEFAULT = 0,
    PITCHED_VOICE_RATE_FULL = 1,
    PITCHED_VOICE_RATE_WIDEBAND = 2,
    /* Legacy value retained for binary compatibility; uses wideband DSP. */
    PITCHED_VOICE_RATE_LOW_CPU = 6,
} PitchedVoiceRate;

typedef struct {
    float frequency;
    float q;
    float gain_db;
} PitchedVoicePeak;

typedef struct {
    float pitch_ratio;
    PitchedVoiceRate rate;
    float highpass_hz;
    float lowpass_hz;
    PitchedVoicePeak peaks[PITCHED_VOICE_MAX_PEAKS];
    size_t peak_count;
    float tremolo_hz;
    float tremolo_depth;
    float chorus_hz;
    float chorus_base_delay_ms;
    float chorus_depth_ms;
    float chorus_mix;
    float compressor_threshold;
    float compressor_ratio;
    float attack_ms;
    float release_ms;
    float makeup_gain;
    float softclip_drive;
    float softclip_scale;
    float output_limit;
} PitchedVoiceConfig;

typedef struct {
    double pitch_shift_seconds;
    double chorus_seconds;
    double output_push_seconds;
    double tremolo_seconds;
    double eq_seconds;
    double dynamics_seconds;
    size_t hops;
    size_t shift_blocks;
    double max_hop_seconds;
} PitchedVoiceProfile;

typedef struct PitchedVoiceState PitchedVoiceState;

PitchedVoiceState *pitched_voice_init(int sample_rate,
                                      const PitchedVoiceConfig *config);
void pitched_voice_free(PitchedVoiceState *state);
int pitched_voice_process(PitchedVoiceState *state,
                          const float input[DSP_HOP_SIZE],
                          float output[DSP_HOP_SIZE]);
void pitched_voice_get_profile(const PitchedVoiceState *state,
                               PitchedVoiceProfile *profile);

#endif
