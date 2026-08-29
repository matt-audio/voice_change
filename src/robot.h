#ifndef VC_ROBOT_MONSTER_ROBOT_H
#define VC_ROBOT_MONSTER_ROBOT_H

#include "dsp.h"

#define ROBOT_TARGET_HZ 52.0f
#define ROBOT_SILENCE_RMS_THRESHOLD 0.002f
#define ROBOT_FLANGER_DELAY_SIZE 1024

#define ROBOT_PROCESS_ERROR (-1)
#define ROBOT_PROCESS_NOT_READY 0
#define ROBOT_PROCESS_OUTPUT 1

typedef struct RobotState RobotState;

typedef struct {
    double yin_seconds;
    double psola_seconds;
    double ring_seconds;
    double flanger_seconds;
    double eq_seconds;
    double dynamics_seconds;
    double window_slide_seconds;
    size_t hops;
    size_t pitch_updates;
    double max_hop_seconds;
} RobotProfile;

RobotState *robot_init(int sample_rate);
void robot_free(RobotState *state);
int robot_process(RobotState *state,
                  const float input[DSP_HOP_SIZE],
                  float output[DSP_HOP_SIZE]);
void robot_get_profile(const RobotState *state, RobotProfile *profile);

#endif
