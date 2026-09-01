#ifndef VC_ROBOT_MONSTER_MONSTER_H
#define VC_ROBOT_MONSTER_MONSTER_H

#include "dsp.h"

#define MONSTER_TARGET_HZ 55.0f
#define MONSTER_LPC_UPDATE_HOPS 64
#define MONSTER_LPC_RMS_CHANGE_UP 1.75f
#define MONSTER_LPC_RMS_CHANGE_DOWN 0.57f
#ifndef MONSTER_LPC_RMS_REFRESH_MIN_HOPS
#define MONSTER_LPC_RMS_REFRESH_MIN_HOPS 8
#endif
#define MONSTER_LPC_MIX 0.72f
#define MONSTER_PSOLA_MIX 0.28f
#define MONSTER_SUB_MIX 0.10f
#define MONSTER_LPC_ROLLOFF_START_HZ 6000.0f
#define MONSTER_LPC_ROLLOFF_END_HZ 9000.0f
#define MONSTER_OUTPUT_OFFSET (DSP_FRAME_SIZE / 2 - DSP_HOP_SIZE)

#if MONSTER_LPC_UPDATE_HOPS < 1
#error "MONSTER_LPC_UPDATE_HOPS must be at least 1"
#endif
#if MONSTER_LPC_RMS_REFRESH_MIN_HOPS < 1
#error "MONSTER_LPC_RMS_REFRESH_MIN_HOPS must be at least 1"
#endif

#define MONSTER_PROCESS_ERROR (-1)
#define MONSTER_PROCESS_NOT_READY 0
#define MONSTER_PROCESS_OUTPUT 1

typedef struct MonsterState MonsterState;

typedef struct {
    double rms_seconds;
    double yin_seconds;
    double lpc_seconds;
    double psola_seconds;
    double body_seconds;
    double ola_seconds;
    double output_seconds;
    double window_slide_seconds;
    size_t hops;
    size_t pitch_updates;
    size_t rendered_frames;
    size_t lpc_refreshes;
    size_t lpc_refresh_initial;
    size_t lpc_refresh_periodic;
    size_t lpc_refresh_onset;
    size_t lpc_refresh_silence;
    size_t lpc_refresh_rms_up;
    size_t lpc_refresh_rms_down;
    double max_hop_seconds;
} MonsterProfile;

MonsterState *monster_init(int sample_rate, float target_hz);
void monster_free(MonsterState *state);
int monster_process(MonsterState *state,
                     const float input[DSP_HOP_SIZE],
                     float output[DSP_HOP_SIZE]);
void monster_get_profile(const MonsterState *state, MonsterProfile *profile);

#endif
