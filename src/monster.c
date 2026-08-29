#include "monster.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#ifdef VC_MONSTER_PROFILE
#include <time.h>
#endif

typedef struct {
    float envelope;
    float attack;
    float release;
} MonsterDynamics;

#define MONSTER_GATE_START_RMS 8e-4f
#define MONSTER_GATE_END_RMS 1.6e-3f

struct MonsterState {
    int sample_rate;
    float target_hz;
    float window[DSP_FRAME_SIZE];
    float ola[DSP_FRAME_SIZE];
    float ola_weights[DSP_FRAME_SIZE];
    size_t ola_cursor;
    float lpc_frame[DSP_FRAME_SIZE];
    float psola_frame[DSP_FRAME_SIZE];
    float hybrid_frame[DSP_FRAME_SIZE];
    uint16_t sub_left[DSP_FRAME_SIZE];
    float sub_fraction[DSP_FRAME_SIZE];
    size_t input_fill;
    int ready;
    uint64_t global_start;

    DspYinState yin;
    DspPitch pitch;
    DspPsolaConfig psola_config;
    DspPsolaState psola;
    DspLpcState *lpc;
    DspBiquad filters[6];
    MonsterDynamics dynamics;

    int lpc_cache_valid;
    int lpc_render_valid;
    int lpc_output_hop_active;
    size_t lpc_hops_since_update;
    float lpc_windowed_rms;
    MonsterProfile profile;
};

#ifdef VC_MONSTER_PROFILE
static double monster_profile_now(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)now.tv_sec + (double)now.tv_nsec / 1000000000.0;
}
#define MONSTER_PROFILE_START(name) double name = monster_profile_now()
#define MONSTER_PROFILE_ADD(field, start) \
    do { state->profile.field += monster_profile_now() - (start); } while (0)
#define MONSTER_PROFILE_MAX(field, start) do { \
    double elapsed = monster_profile_now() - (start); \
    if (elapsed > state->profile.field) state->profile.field = elapsed; \
} while (0)
#else
#define MONSTER_PROFILE_START(name) ((void)0)
#define MONSTER_PROFILE_ADD(field, start) ((void)0)
#define MONSTER_PROFILE_MAX(field, start) ((void)0)
#endif

static float monster_gate(float rms) {
    float value = fminf(1.0f, fmaxf(0.0f,
        (rms - MONSTER_GATE_START_RMS) /
        (MONSTER_GATE_END_RMS - MONSTER_GATE_START_RMS)));
    return value * value * (3.0f - 2.0f * value);
}

static float monster_dynamics_sample(MonsterState *state, float sample) {
    MonsterDynamics *dynamics = &state->dynamics;
    float magnitude = fabsf(sample);
    float coefficient = magnitude > dynamics->envelope
        ? dynamics->attack : dynamics->release;
    dynamics->envelope = magnitude + coefficient *
        (dynamics->envelope - magnitude);
    float gain = dynamics->envelope > 0.16f
        ? powf(0.16f / dynamics->envelope, 5.0f / 6.0f) : 1.0f;
    float value = tanhf(sample * gain * 2.4f) * 0.78f;
    if (value > 0.92f) value = 0.92f;
    if (value < -0.92f) value = -0.92f;
    return value;
}

static void monster_frame_rms(const float frame[DSP_FRAME_SIZE],
                              const float hann[DSP_FRAME_SIZE],
                              float *raw_rms, float *windowed_rms) {
    double raw_energy = 0.0;
    double windowed_energy = 0.0;
    for (size_t i = 0; i < DSP_FRAME_SIZE; ++i) {
        raw_energy += (double)frame[i] * frame[i];
        float sample = frame[i] * hann[i];
        windowed_energy += (double)sample * sample;
    }
    *raw_rms = (float)sqrt(raw_energy / DSP_FRAME_SIZE);
    *windowed_rms = (float)sqrt(windowed_energy / DSP_FRAME_SIZE);
}

static int monster_render_frame(MonsterState *state) {
    float *lpc = state->lpc_frame;
    float *psola = state->psola_frame;
    float *hybrid = state->hybrid_frame;
    const float *hann = dsp_frame_hann();
    DspPitch *pitch = &state->pitch;

    uint64_t global_start = state->global_start;
    float input_rms;
    float windowed_rms;
    MONSTER_PROFILE_START(rms_start);
    monster_frame_rms(state->window, hann, &input_rms, &windowed_rms);
    MONSTER_PROFILE_ADD(rms_seconds, rms_start);
    MONSTER_PROFILE_START(yin_start);
    if (!dsp_yin_process(&state->yin, state->window, input_rms,
                         global_start, pitch)) return 0;
    MONSTER_PROFILE_ADD(yin_seconds, yin_start);
    if (state->yin.last_was_update) ++state->profile.pitch_updates;

    MONSTER_PROFILE_START(lpc_start);
    double output_hop_energy = 0.0;
    for (size_t i = MONSTER_OUTPUT_OFFSET;
         i < MONSTER_OUTPUT_OFFSET + DSP_HOP_SIZE; ++i)
        output_hop_energy += (double)state->window[i] * state->window[i];
    float output_hop_rms =
        (float)sqrt(output_hop_energy / DSP_HOP_SIZE);
    int output_hop_active = state->lpc_output_hop_active
        ? output_hop_rms >= MONSTER_GATE_START_RMS
        : output_hop_rms >= MONSTER_GATE_END_RMS;
    int lpc_activity_changed =
        output_hop_active != state->lpc_output_hop_active;
    state->lpc_output_hop_active = output_hop_active;
    int lpc_low_level_onset =
        state->lpc_windowed_rms <= 1e-5f && windowed_rms > 1e-5f;
    int lpc_rms_refresh_allowed = state->lpc_hops_since_update >=
        MONSTER_LPC_RMS_REFRESH_MIN_HOPS - 1;
    int lpc_rate_limited_onset = lpc_activity_changed &&
        output_hop_active && !lpc_rms_refresh_allowed;
    int lpc_rms_up = lpc_rms_refresh_allowed &&
        state->lpc_windowed_rms > 1e-5f &&
        windowed_rms > state->lpc_windowed_rms * MONSTER_LPC_RMS_CHANGE_UP;
    int lpc_rms_down = lpc_rms_refresh_allowed &&
        state->lpc_windowed_rms > 1e-5f &&
        windowed_rms < state->lpc_windowed_rms * MONSTER_LPC_RMS_CHANGE_DOWN;
    int lpc_threshold_changed =
        (windowed_rms >= DSP_LPC_SILENCE_RMS) != state->lpc_render_valid;
    int lpc_periodic =
        state->lpc_hops_since_update >= MONSTER_LPC_UPDATE_HOPS - 1;
    int lpc_initial = !state->lpc_cache_valid;
    int refresh_lpc = lpc_initial || lpc_periodic ||
        lpc_rate_limited_onset || lpc_low_level_onset ||
        lpc_rms_up || lpc_rms_down || lpc_threshold_changed;
    if (refresh_lpc) {
        DspLpcRenderStatus lpc_status =
            dsp_lpc_render(state->lpc, state->window, lpc);
        if (lpc_status == DSP_LPC_RENDER_ERROR) {
            memset(state->ola, 0, sizeof(state->ola));
            memset(state->ola_weights, 0, sizeof(state->ola_weights));
            return 0;
        }
        state->lpc_cache_valid = 1;
        state->lpc_render_valid = lpc_status == DSP_LPC_RENDER_VALID;
        state->lpc_hops_since_update = 0;
        state->lpc_windowed_rms = windowed_rms;
        ++state->profile.lpc_refreshes;
        if (lpc_initial) {
            ++state->profile.lpc_refresh_initial;
        } else if (lpc_rate_limited_onset) {
            ++state->profile.lpc_refresh_onset;
        } else if (lpc_threshold_changed) {
            if (windowed_rms >= DSP_LPC_SILENCE_RMS)
                ++state->profile.lpc_refresh_onset;
            else
                ++state->profile.lpc_refresh_silence;
        } else if (lpc_low_level_onset) {
            ++state->profile.lpc_refresh_onset;
        } else if (lpc_periodic) {
            ++state->profile.lpc_refresh_periodic;
        } else if (lpc_rms_up) {
            ++state->profile.lpc_refresh_rms_up;
        } else if (lpc_rms_down) {
            ++state->profile.lpc_refresh_rms_down;
        }
    } else {
        if (state->lpc_render_valid &&
            !dsp_lpc_render_cached(state->lpc, lpc)) return 0;
        ++state->lpc_hops_since_update;
    }
    MONSTER_PROFILE_ADD(lpc_seconds, lpc_start);

    MONSTER_PROFILE_START(psola_start);
    if (!dsp_psola_process(&state->psola, &state->psola_config, pitch,
                           state->window, global_start, psola,
                           DSP_FRAME_SIZE)) return 0;
    MONSTER_PROFILE_ADD(psola_seconds, psola_start);

    MONSTER_PROFILE_START(body_start);
    for (size_t i = 0; i < DSP_FRAME_SIZE; ++i) {
        hybrid[i] = lpc[i] * MONSTER_LPC_MIX +
                    psola[i] * MONSTER_PSOLA_MIX;
    }
    float gate = monster_gate(output_hop_rms);
    for (size_t i = 0; i < DSP_FRAME_SIZE; ++i) {
        size_t left = state->sub_left[i];
        size_t right = left + 1 < DSP_FRAME_SIZE ? left + 1 : left;
        float fraction = state->sub_fraction[i];
        float sub = hybrid[left] + (hybrid[right] - hybrid[left]) * fraction;
        hybrid[i] = (hybrid[i] * (1.0f - MONSTER_SUB_MIX) +
                     sub * MONSTER_SUB_MIX) * gate;
        size_t index = (state->ola_cursor + i) & (DSP_FRAME_SIZE - 1);
        state->ola[index] += hybrid[i] * hann[i];
        state->ola_weights[index] += hann[i];
    }
    MONSTER_PROFILE_ADD(body_seconds, body_start);
    state->global_start += DSP_HOP_SIZE;
    ++state->profile.rendered_frames;
    return 1;
}

MonsterState *monster_init(int sample_rate, float target_hz) {
    if (sample_rate != DSP_SAMPLE_RATE || !isfinite(target_hz) || target_hz < 1.0f ||
        target_hz >= sample_rate / 2.0f) return NULL;
    dsp_init();
    MonsterState *state = (MonsterState *)calloc(1, sizeof(*state));
    if (!state) return NULL;
    state->sample_rate = sample_rate;
    state->target_hz = target_hz;
    state->input_fill = MONSTER_OUTPUT_OFFSET;
    for (size_t i = 0; i < DSP_FRAME_SIZE; ++i) {
        double position = ((double)(DSP_FRAME_SIZE / 2) + (double)i) *
                          (double)(DSP_FRAME_SIZE - 1) /
                          (double)(DSP_FRAME_SIZE * 2 - 1);
        state->sub_left[i] = (uint16_t)position;
        state->sub_fraction[i] =
            (float)(position - (double)state->sub_left[i]);
    }
    dsp_yin_init(&state->yin, sample_rate);
    state->psola_config.sample_rate = sample_rate;
    state->psola_config.target_hz = target_hz;
    state->psola_config.target_period = (int)((float)sample_rate / target_hz);
    state->psola_config.full_frame_output = 1;
    dsp_psola_init(&state->psola, &state->psola_config);
    state->lpc = dsp_lpc_create(sample_rate, target_hz);
    if (!state->lpc) {
        free(state);
        return NULL;
    }
    dsp_lpc_set_harmonic_rolloff(state->lpc,
                                 MONSTER_LPC_ROLLOFF_START_HZ,
                                 MONSTER_LPC_ROLLOFF_END_HZ);
    state->dynamics.attack = expf(-1.0f / (0.012f * sample_rate));
    state->dynamics.release = expf(-1.0f / (0.240f * sample_rate));
    state->filters[0] = dsp_highpass(24.0f, sample_rate);
    state->filters[1] = dsp_peak_eq(55.0f, sample_rate, 0.95f, 9.0f);
    state->filters[2] = dsp_peak_eq(110.0f, sample_rate, 0.9f, 5.0f);
    state->filters[3] = dsp_peak_eq(230.0f, sample_rate, 1.0f, -2.5f);
    state->filters[4] = dsp_peak_eq(900.0f, sample_rate, 1.0f, 2.0f);
    state->filters[5] = dsp_lowpass(5600.0f, sample_rate);
    return state;
}

void monster_free(MonsterState *state) {
    if (!state) return;
    dsp_lpc_free(state->lpc);
    free(state);
}

int monster_process(MonsterState *state,
                    const float input[DSP_HOP_SIZE],
                    float output[DSP_HOP_SIZE]) {
    if (!state || !input || !output) return MONSTER_PROCESS_ERROR;
    MONSTER_PROFILE_START(hop_start);
    if (!state->ready) {
        memcpy(state->window + state->input_fill, input,
               DSP_HOP_SIZE * sizeof(*input));
        state->input_fill += DSP_HOP_SIZE;
        if (state->input_fill < DSP_FRAME_SIZE) {
            memset(output, 0, DSP_HOP_SIZE * sizeof(*output));
            return MONSTER_PROCESS_NOT_READY;
        }
        state->ready = 1;
    } else {
        MONSTER_PROFILE_START(slide_start);
        memmove(state->window, state->window + DSP_HOP_SIZE,
                (DSP_FRAME_SIZE - DSP_HOP_SIZE) * sizeof(float));
        MONSTER_PROFILE_ADD(window_slide_seconds, slide_start);
        memcpy(state->window + DSP_FRAME_SIZE - DSP_HOP_SIZE,
               input, DSP_HOP_SIZE * sizeof(*input));
    }

    if (!monster_render_frame(state)) {
        MONSTER_PROFILE_MAX(max_hop_seconds, hop_start);
        return MONSTER_PROCESS_ERROR;
    }
    MONSTER_PROFILE_START(output_start);
    for (size_t i = 0; i < DSP_HOP_SIZE; ++i) {
        size_t index = (state->ola_cursor + MONSTER_OUTPUT_OFFSET + i) &
                       (DSP_FRAME_SIZE - 1);
        float value = state->ola_weights[index] > 1e-5f
            ? state->ola[index] / state->ola_weights[index] : 0.0f;
        for (size_t filter = 0; filter < 6; ++filter)
            value = dsp_biquad_process(&state->filters[filter], value);
        float crushed = roundf(value * 4096.0f) / 4096.0f;
        value = value * 0.86f + crushed * 0.14f;
        output[i] = monster_dynamics_sample(state, value);
    }
    MONSTER_PROFILE_ADD(output_seconds, output_start);

    MONSTER_PROFILE_START(ola_shift_start);
    for (size_t i = 0; i < DSP_HOP_SIZE; ++i) {
        size_t index = (state->ola_cursor + i) & (DSP_FRAME_SIZE - 1);
        state->ola[index] = 0.0f;
        state->ola_weights[index] = 0.0f;
    }
    state->ola_cursor = (state->ola_cursor + DSP_HOP_SIZE) &
                        (DSP_FRAME_SIZE - 1);
    MONSTER_PROFILE_ADD(ola_seconds, ola_shift_start);
    MONSTER_PROFILE_MAX(max_hop_seconds, hop_start);
    ++state->profile.hops;
    return MONSTER_PROCESS_OUTPUT;
}

void monster_get_profile(const MonsterState *state, MonsterProfile *profile) {
    if (state && profile) *profile = state->profile;
}
