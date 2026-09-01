#include "robot.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Original 2.0 makeup gain reduced by 3 dB. */
#define ROBOT_MAKEUP_GAIN 1.41589157f
#ifdef VC_ROBOT_PROFILE
#include <time.h>
#endif

#define ROBOT_FLANGER_MASK (ROBOT_FLANGER_DELAY_SIZE - 1)

typedef struct {
    float delay[ROBOT_FLANGER_DELAY_SIZE];
    int write_index;
    float phase;
    float phase_increment;
    float base_delay;
    float depth;
} RobotFlanger;

typedef struct {
    float envelope;
    float attack;
    float release;
} RobotDynamics;

struct RobotState {
    int sample_rate;
    float window[DSP_FRAME_SIZE];
    size_t input_fill;
    int ready;
    int silence_reset;
    uint64_t global_start;
    DspYinState yin;
    DspPitch pitch;
    DspPsolaConfig psola_config;
    DspPsolaState psola;
    float rendered_frame[DSP_HOP_SIZE];
    DspBiquad filters[5];
    RobotFlanger flanger;
    RobotDynamics dynamics;
    float ring_phase;
    float ring_phase_increment;
    RobotProfile profile;
};

#ifdef VC_ROBOT_PROFILE
static double robot_profile_cpu_now(void) {
    struct timespec now;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &now);
    return (double)now.tv_sec + (double)now.tv_nsec / 1000000000.0;
}
static double robot_profile_wall_now(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)now.tv_sec + (double)now.tv_nsec / 1000000000.0;
}
#define ROBOT_PROFILE_START(name) double name = robot_profile_cpu_now()
#define ROBOT_PROFILE_WALL_START(name) double name = robot_profile_wall_now()
#define ROBOT_PROFILE_ADD(field, start) \
    do { state->profile.field += robot_profile_cpu_now() - (start); } while (0)
#define ROBOT_PROFILE_MAX(field, start) do { \
    double elapsed = robot_profile_wall_now() - (start); \
    if (elapsed > state->profile.field) state->profile.field = elapsed; \
} while (0)
#else
#define ROBOT_PROFILE_START(name) ((void)0)
#define ROBOT_PROFILE_WALL_START(name) ((void)0)
#define ROBOT_PROFILE_ADD(field, start) ((void)0)
#define ROBOT_PROFILE_MAX(field, start) ((void)0)
#endif

static float robot_flanger_sample(RobotState *state, float sample) {
    RobotFlanger *flanger = &state->flanger;
    float lfo = (dsp_sine_cycles_wrapped(flanger->phase) + 1.0f) * 0.5f;
    float delay = flanger->base_delay + flanger->depth * lfo;
    float read = (float)flanger->write_index - delay;
    if (read < 0.0f) read += ROBOT_FLANGER_DELAY_SIZE;
    else if (read >= ROBOT_FLANGER_DELAY_SIZE) read -= ROBOT_FLANGER_DELAY_SIZE;
    int left = (int)read;
    int right = (left + 1) & ROBOT_FLANGER_MASK;
    float fraction = read - (float)left;
    float delayed = (1.0f - fraction) * flanger->delay[left] +
                    fraction * flanger->delay[right];
    float output = sample + delayed * 0.30f;
    flanger->delay[flanger->write_index] = sample + delayed * 0.42f;
    flanger->write_index = (flanger->write_index + 1) & ROBOT_FLANGER_MASK;
    flanger->phase += flanger->phase_increment;
    if (flanger->phase >= 1.0f) flanger->phase -= 1.0f;
    return isfinite(output) ? output : 0.0f;
}

static float robot_dynamics_sample(RobotState *state, float sample) {
    RobotDynamics *dynamics = &state->dynamics;
    float magnitude = fabsf(sample);
    float coefficient = magnitude > dynamics->envelope
        ? dynamics->attack : dynamics->release;
    dynamics->envelope = magnitude + coefficient *
        (dynamics->envelope - magnitude);

    float gain = 1.0f;
    if (dynamics->envelope < 0.005f && dynamics->envelope > 1e-6f) {
        float ratio = dynamics->envelope / 0.005f;
        float ratio_squared = ratio * ratio;
        float ratio_fourth = ratio_squared * ratio_squared;
        gain *= ratio_fourth * ratio_squared * ratio;
    }
    if (dynamics->envelope > 0.10f)
        gain *= powf(0.10f / dynamics->envelope, 0.875f);

    float output = sample * gain * ROBOT_MAKEUP_GAIN;
    if (output > 0.90f) output = 0.90f;
    if (output < -0.90f) output = -0.90f;
    return output;
}

static int robot_render(RobotState *state, float output[DSP_HOP_SIZE]) {
    DspPitch *pitch = &state->pitch;
    float *rendered = state->rendered_frame;
    float frame_rms = dsp_frame_rms(state->window);
    if (frame_rms < ROBOT_SILENCE_RMS_THRESHOLD) {
        if (!state->silence_reset) {
            dsp_yin_reset(&state->yin);
            dsp_psola_reset(&state->psola);
            state->silence_reset = 1;
        }
        memset(output, 0, DSP_HOP_SIZE * sizeof(*output));
        return 1;
    }
    state->silence_reset = 0;

    ROBOT_PROFILE_START(yin_start);
    if (!dsp_yin_process(&state->yin, state->window, frame_rms,
                         state->global_start, pitch)) return 0;
    ROBOT_PROFILE_ADD(yin_seconds, yin_start);
    if (state->yin.last_was_update) ++state->profile.pitch_updates;
    /* Keep the original Robot behavior during a short YIN loss of lock:
     * continue with a neutral source period instead of punching a zero hop. */
    if (!pitch->voiced) {
        pitch->voiced = 1;
        pitch->f0_hz = 120.0f;
        pitch->confidence = 1.0f;
        pitch->period_samples = state->sample_rate / 120;
    }

    ROBOT_PROFILE_START(psola_start);
    if (!dsp_psola_process(&state->psola, &state->psola_config, pitch,
                           state->window, state->global_start, rendered,
                           DSP_HOP_SIZE)) return 0;
    memcpy(output, rendered, DSP_HOP_SIZE * sizeof(*output));
    ROBOT_PROFILE_ADD(psola_seconds, psola_start);
    return 1;
}

static int robot_process_ready(RobotState *state,
                               float output[DSP_HOP_SIZE]) {
    if (!robot_render(state, output)) return ROBOT_PROCESS_ERROR;
    ++state->profile.hops;

    ROBOT_PROFILE_START(ring_start);
    for (size_t i = 0; i < DSP_HOP_SIZE; ++i) {
        float sample = output[i];
        float carrier = dsp_sine_cycles_wrapped(state->ring_phase);
        state->ring_phase += state->ring_phase_increment;
        if (state->ring_phase >= 1.0f) state->ring_phase -= 1.0f;
        output[i] = sample * 0.02f + sample * carrier * 0.98f;
    }
    ROBOT_PROFILE_ADD(ring_seconds, ring_start);

    ROBOT_PROFILE_START(flanger_start);
    for (size_t i = 0; i < DSP_HOP_SIZE; ++i)
        output[i] = robot_flanger_sample(state, output[i]);
    ROBOT_PROFILE_ADD(flanger_seconds, flanger_start);

    ROBOT_PROFILE_START(eq_start);
    for (size_t i = 0; i < DSP_HOP_SIZE; ++i)
        for (size_t filter = 0; filter < 5; ++filter)
            output[i] = dsp_biquad_process_inline(
                &state->filters[filter], output[i]);
    ROBOT_PROFILE_ADD(eq_seconds, eq_start);

    ROBOT_PROFILE_START(dynamics_start);
    for (size_t i = 0; i < DSP_HOP_SIZE; ++i)
        output[i] = robot_dynamics_sample(state, output[i]);
    ROBOT_PROFILE_ADD(dynamics_seconds, dynamics_start);
    state->global_start += DSP_HOP_SIZE;
    return ROBOT_PROCESS_OUTPUT;
}

RobotState *robot_init(int sample_rate) {
    if (sample_rate != DSP_SAMPLE_RATE) return NULL;
    dsp_init();
    RobotState *state = (RobotState *)calloc(1, sizeof(*state));
    if (!state) return NULL;
    state->sample_rate = sample_rate;
    dsp_yin_init(&state->yin, sample_rate);
    state->psola_config.sample_rate = sample_rate;
    state->psola_config.target_hz = ROBOT_TARGET_HZ;
    state->psola_config.target_period =
        (int)((float)sample_rate / ROBOT_TARGET_HZ);
    dsp_psola_init(&state->psola, &state->psola_config);

    float sr = (float)sample_rate;
    state->flanger.base_delay = 5.0f * sr / 1000.0f;
    state->flanger.depth = 3.0f * sr / 1000.0f;
    state->flanger.phase_increment = 0.22f / sr;
    state->ring_phase_increment = ROBOT_TARGET_HZ / sr;
    state->dynamics.attack = expf(-1.0f / (0.003f * sr));
    state->dynamics.release = expf(-1.0f / (0.090f * sr));

    state->filters[0] = dsp_highpass(45.0f, sample_rate);
    state->filters[1] = dsp_peak_eq(180.0f, sample_rate, 0.9f, 3.0f);
    state->filters[2] = dsp_peak_eq(900.0f, sample_rate, 1.2f, 4.0f);
    state->filters[3] = dsp_peak_eq(2200.0f, sample_rate, 1.0f, 5.0f);
    state->filters[4] = dsp_lowpass(9500.0f, sample_rate);
    return state;
}

void robot_free(RobotState *state) {
    free(state);
}

int robot_process(RobotState *state,
                  const float input[DSP_HOP_SIZE],
                  float output[DSP_HOP_SIZE]) {
    if (!state || !input || !output) return ROBOT_PROCESS_ERROR;
    ROBOT_PROFILE_WALL_START(hop_start);
    if (!state->ready) {
        memcpy(state->window + state->input_fill, input,
               DSP_HOP_SIZE * sizeof(*input));
        state->input_fill += DSP_HOP_SIZE;
        if (state->input_fill < DSP_FRAME_SIZE) {
            memset(output, 0, DSP_HOP_SIZE * sizeof(*output));
            ROBOT_PROFILE_MAX(max_hop_seconds, hop_start);
            return ROBOT_PROCESS_NOT_READY;
        }
        state->ready = 1;
    } else {
        ROBOT_PROFILE_START(slide_start);
        memmove(state->window, state->window + DSP_HOP_SIZE,
                (DSP_FRAME_SIZE - DSP_HOP_SIZE) * sizeof(float));
        ROBOT_PROFILE_ADD(window_slide_seconds, slide_start);
        memcpy(state->window + DSP_FRAME_SIZE - DSP_HOP_SIZE,
               input, DSP_HOP_SIZE * sizeof(*input));
    }
    int status = robot_process_ready(state, output);
    ROBOT_PROFILE_MAX(max_hop_seconds, hop_start);
    return status;
}

void robot_get_profile(const RobotState *state, RobotProfile *profile) {
    if (state && profile) *profile = state->profile;
}
