#include "pitched_voice.h"

#include <math.h>
#include <rubberband/rubberband-c.h>
#include <stdlib.h>
#include <string.h>
#ifdef VC_PITCHED_VOICE_PROFILE
#include <time.h>
#endif

#define PITCHED_VOICE_MAX_FILTERS (PITCHED_VOICE_MAX_PEAKS + 2)
#define PITCHED_VOICE_SHIFT_BLOCK_SIZE (3 * DSP_HOP_SIZE / 2)
#define PITCHED_VOICE_OUTPUT_FIFO_SIZE 8192
#define PITCHED_VOICE_RESAMPLE_FACTOR 6
#define PITCHED_VOICE_RESAMPLE_TAPS 97
#define PITCHED_VOICE_CHORUS_SIZE 512
#define PITCHED_VOICE_CHORUS_MASK (PITCHED_VOICE_CHORUS_SIZE - 1)

/* 2.8 kHz low-pass at 48 kHz, Kaiser beta 7, -72 dB at 4 kHz. */
static const float k_resample_coefficients[PITCHED_VOICE_RESAMPLE_TAPS] = {
    -3.7402581798e-05f, -6.3074632489e-05f, -8.5434471474e-05f,
    -9.2936259277e-05f, -7.2405210307e-05f, -1.2271633430e-05f,
    9.3361097332e-05f, 2.4061860851e-04f, 4.1278682864e-04f,
    5.7939226500e-04f, 6.9835371702e-04f, 7.2165696223e-04f,
    6.0430423717e-04f, 3.1546924006e-04f, -1.4999618701e-04f,
    -7.6212276670e-04f, -1.4510447477e-03f, -2.1091344381e-03f,
    -2.6014143523e-03f, -2.7842439977e-03f, -2.5306177454e-03f,
    -1.7587333885e-03f, -4.5912728991e-04f, 1.2850548879e-03f,
    3.2899200785e-03f, 5.2789159705e-03f, 6.9091040678e-03f,
    7.8147200780e-03f, 7.6639539054e-03f, 6.2219185418e-03f,
    3.4105742642e-03f, -6.4458070287e-04f, -5.5906230121e-03f,
    -1.0853407961e-02f, -1.5677393714e-02f, -1.9197842204e-02f,
    -2.0539041932e-02f, -1.8927546408e-02f, -1.3806020960e-02f,
    -4.9317894744e-03f, 7.5550396721e-03f, 2.3105867307e-02f,
    4.0787185836e-02f, 5.9353847905e-02f, 7.7365519688e-02f,
    9.3333514784e-02f, 1.0588047342e-01f, 1.1389295654e-01f,
    1.1664739234e-01f, 1.1389295654e-01f, 1.0588047342e-01f,
    9.3333514784e-02f, 7.7365519688e-02f, 5.9353847905e-02f,
    4.0787185836e-02f, 2.3105867307e-02f, 7.5550396721e-03f,
    -4.9317894744e-03f, -1.3806020960e-02f, -1.8927546408e-02f,
    -2.0539041932e-02f, -1.9197842204e-02f, -1.5677393714e-02f,
    -1.0853407961e-02f, -5.5906230121e-03f, -6.4458070287e-04f,
    3.4105742642e-03f, 6.2219185418e-03f, 7.6639539054e-03f,
    7.8147200780e-03f, 6.9091040678e-03f, 5.2789159705e-03f,
    3.2899200785e-03f, 1.2850548879e-03f, -4.5912728991e-04f,
    -1.7587333885e-03f, -2.5306177454e-03f, -2.7842439977e-03f,
    -2.6014143523e-03f, -2.1091344381e-03f, -1.4510447477e-03f,
    -7.6212276670e-04f, -1.4999618701e-04f, 3.1546924006e-04f,
    6.0430423717e-04f, 7.2165696223e-04f, 6.9835371702e-04f,
    5.7939226500e-04f, 4.1278682864e-04f, 2.4061860851e-04f,
    9.3361097332e-05f, -1.2271633430e-05f, -7.2405210307e-05f,
    -9.2936259277e-05f, -8.5434471474e-05f, -6.3074632489e-05f,
    -3.7402581798e-05f,
};

_Static_assert((PITCHED_VOICE_OUTPUT_FIFO_SIZE &
                (PITCHED_VOICE_OUTPUT_FIFO_SIZE - 1)) == 0,
               "pitch output FIFO size must be a power of two");
_Static_assert(PITCHED_VOICE_OUTPUT_FIFO_SIZE >=
               PITCHED_VOICE_SHIFT_BLOCK_SIZE *
               PITCHED_VOICE_RESAMPLE_FACTOR + DSP_HOP_SIZE,
               "pitch output FIFO must hold one expanded block");
_Static_assert(PITCHED_VOICE_SHIFT_BLOCK_SIZE *
               PITCHED_VOICE_RESAMPLE_FACTOR % DSP_HOP_SIZE == 0,
               "pitch block must contain a whole number of process hops");
_Static_assert((PITCHED_VOICE_CHORUS_SIZE &
                (PITCHED_VOICE_CHORUS_SIZE - 1)) == 0,
               "chorus delay size must be a power of two");

typedef struct {
    float envelope;
    float attack;
    float release;
    float threshold;
    float exponent;
    float makeup_gain;
    float softclip_drive;
    float softclip_scale;
    float output_limit;
} PitchedVoiceDynamics;

struct PitchedVoiceState {
    RubberBandState shifter;
    unsigned int pitch_sample_rate_divisor;
    float shift_input[PITCHED_VOICE_SHIFT_BLOCK_SIZE];
    float shift_output[PITCHED_VOICE_SHIFT_BLOCK_SIZE];
    size_t shift_input_fill;
    float downsample_history[2 * PITCHED_VOICE_RESAMPLE_TAPS];
    size_t downsample_write;
    unsigned int downsample_phase;
    float upsample_history[2 * PITCHED_VOICE_RESAMPLE_TAPS];
    size_t upsample_write;
    float output_fifo[PITCHED_VOICE_OUTPUT_FIFO_SIZE];
    size_t output_fifo_read;
    size_t output_fifo_count;
    size_t startup_discard;

    DspBiquad filters[PITCHED_VOICE_MAX_FILTERS];
    size_t filter_count;
    PitchedVoiceDynamics dynamics;
    float tremolo_phase;
    float tremolo_increment;
    float tremolo_depth;
    float chorus_delay[PITCHED_VOICE_CHORUS_SIZE];
    size_t chorus_write;
    float chorus_phase;
    float chorus_phase_increment;
    float chorus_base_delay;
    float chorus_depth;
    float chorus_mix;
    PitchedVoiceProfile profile;
};

#ifdef VC_PITCHED_VOICE_PROFILE
static double pitched_voice_profile_cpu_now(void) {
    struct timespec now;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &now);
    return (double)now.tv_sec + (double)now.tv_nsec / 1000000000.0;
}
static double pitched_voice_profile_wall_now(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)now.tv_sec + (double)now.tv_nsec / 1000000000.0;
}
#define PITCHED_PROFILE_START(name) \
    double name = pitched_voice_profile_cpu_now()
#define PITCHED_PROFILE_WALL_START(name) \
    double name = pitched_voice_profile_wall_now()
#define PITCHED_PROFILE_ADD(field, start) \
    do { state->profile.field += pitched_voice_profile_cpu_now() - (start); } while (0)
#define PITCHED_PROFILE_MAX(field, start) do { \
    double elapsed = pitched_voice_profile_wall_now() - (start); \
    if (elapsed > state->profile.field) state->profile.field = elapsed; \
} while (0)
#else
#define PITCHED_PROFILE_START(name) ((void)0)
#define PITCHED_PROFILE_WALL_START(name) ((void)0)
#define PITCHED_PROFILE_ADD(field, start) ((void)0)
#define PITCHED_PROFILE_MAX(field, start) ((void)0)
#endif

static int pitched_voice_config_valid(const PitchedVoiceConfig *config) {
    if (!config || !isfinite(config->pitch_ratio) ||
        config->pitch_ratio <= 0.0f ||
        (config->rate != PITCHED_VOICE_RATE_DEFAULT &&
         config->rate != PITCHED_VOICE_RATE_FULL &&
         config->rate != PITCHED_VOICE_RATE_LOW_CPU) ||
        !isfinite(config->highpass_hz) || config->highpass_hz < 0.0f ||
        !isfinite(config->lowpass_hz) || config->lowpass_hz < 0.0f ||
        config->peak_count > PITCHED_VOICE_MAX_PEAKS ||
        !isfinite(config->tremolo_hz) || config->tremolo_hz < 0.0f ||
        !isfinite(config->tremolo_depth) || config->tremolo_depth < 0.0f ||
        config->tremolo_depth > 1.0f ||
        !isfinite(config->chorus_hz) || config->chorus_hz < 0.0f ||
        !isfinite(config->chorus_base_delay_ms) ||
        config->chorus_base_delay_ms < 0.0f ||
        !isfinite(config->chorus_depth_ms) || config->chorus_depth_ms < 0.0f ||
        config->chorus_depth_ms > config->chorus_base_delay_ms ||
        !isfinite(config->chorus_mix) || config->chorus_mix < 0.0f ||
        config->chorus_mix > 1.0f ||
        !isfinite(config->compressor_threshold) ||
        config->compressor_threshold <= 0.0f ||
        !isfinite(config->compressor_ratio) ||
        config->compressor_ratio < 1.0f ||
        !isfinite(config->attack_ms) || config->attack_ms <= 0.0f ||
        !isfinite(config->release_ms) || config->release_ms <= 0.0f ||
        !isfinite(config->makeup_gain) || config->makeup_gain <= 0.0f ||
        !isfinite(config->softclip_drive) || config->softclip_drive <= 0.0f ||
        !isfinite(config->softclip_scale) || config->softclip_scale <= 0.0f ||
        !isfinite(config->output_limit) || config->output_limit <= 0.0f ||
        config->output_limit > 1.0f) return 0;
    unsigned int rate_divisor = config->rate == PITCHED_VOICE_RATE_LOW_CPU
        ? PITCHED_VOICE_RESAMPLE_FACTOR : 1;
    float chorus_sample_rate =
        (float)DSP_SAMPLE_RATE / (float)rate_divisor;
    float minimum_chorus_delay =
        (config->chorus_base_delay_ms - config->chorus_depth_ms) *
        chorus_sample_rate / 1000.0f;
    float maximum_chorus_delay =
        (config->chorus_base_delay_ms + config->chorus_depth_ms) *
        chorus_sample_rate / 1000.0f;
    if ((config->chorus_mix > 0.0f && minimum_chorus_delay < 1.0f) ||
        maximum_chorus_delay >=
        PITCHED_VOICE_CHORUS_SIZE - 2)
        return 0;
    if (config->highpass_hz >= DSP_SAMPLE_RATE * 0.5f ||
        config->lowpass_hz >= DSP_SAMPLE_RATE * 0.5f) return 0;
    for (size_t i = 0; i < config->peak_count; ++i) {
        const PitchedVoicePeak *peak = &config->peaks[i];
        if (!isfinite(peak->frequency) || peak->frequency <= 0.0f ||
            peak->frequency >= DSP_SAMPLE_RATE * 0.5f ||
            !isfinite(peak->q) || peak->q <= 0.0f ||
            !isfinite(peak->gain_db)) return 0;
    }
    return config->highpass_hz == 0.0f ||
        config->lowpass_hz == 0.0f ||
        config->highpass_hz < config->lowpass_hz;
}

static float pitched_voice_resample_convolution(const float history[],
                                                 size_t latest) {
    float sum = 0.0f;
    for (size_t tap = 0; tap < PITCHED_VOICE_RESAMPLE_TAPS; ++tap)
        sum += k_resample_coefficients[tap] * history[latest - tap];
    return sum;
}

static int pitched_voice_fifo_push(PitchedVoiceState *state, float sample) {
    if (state->output_fifo_count == PITCHED_VOICE_OUTPUT_FIFO_SIZE) return 0;
    size_t write = (state->output_fifo_read + state->output_fifo_count) &
        (PITCHED_VOICE_OUTPUT_FIFO_SIZE - 1);
    state->output_fifo[write] = sample;
    ++state->output_fifo_count;
    return 1;
}

static int pitched_voice_upsample_push(PitchedVoiceState *state,
                                       float sample) {
    size_t write = state->upsample_write;
    state->upsample_history[write] = sample;
    state->upsample_history[write + PITCHED_VOICE_RESAMPLE_TAPS] = sample;
    size_t latest = write + PITCHED_VOICE_RESAMPLE_TAPS;
    state->upsample_write = (write + 1) % PITCHED_VOICE_RESAMPLE_TAPS;

    for (size_t phase = 0; phase < PITCHED_VOICE_RESAMPLE_FACTOR; ++phase) {
        float sum = 0.0f;
        size_t history_offset = 0;
        for (size_t tap = phase; tap < PITCHED_VOICE_RESAMPLE_TAPS;
             tap += PITCHED_VOICE_RESAMPLE_FACTOR) {
            sum += k_resample_coefficients[tap] *
                state->upsample_history[latest - history_offset];
            ++history_offset;
        }
        if (!pitched_voice_fifo_push(
                state, sum * (float)PITCHED_VOICE_RESAMPLE_FACTOR)) return 0;
    }
    return 1;
}

static void pitched_voice_downsample_push(PitchedVoiceState *state,
                                          float sample) {
    size_t write = state->downsample_write;
    state->downsample_history[write] = sample;
    state->downsample_history[write + PITCHED_VOICE_RESAMPLE_TAPS] = sample;
    size_t latest = write + PITCHED_VOICE_RESAMPLE_TAPS;
    state->downsample_write = (write + 1) % PITCHED_VOICE_RESAMPLE_TAPS;

    if (++state->downsample_phase < PITCHED_VOICE_RESAMPLE_FACTOR) return;
    state->downsample_phase = 0;
    state->shift_input[state->shift_input_fill++] =
        pitched_voice_resample_convolution(state->downsample_history, latest);
}

static float pitched_voice_dynamics_sample(PitchedVoiceState *state,
                                            float sample) {
    PitchedVoiceDynamics *dynamics = &state->dynamics;
    float magnitude = fabsf(sample);
    float coefficient = magnitude > dynamics->envelope
        ? dynamics->attack : dynamics->release;
    dynamics->envelope = magnitude + coefficient *
        (dynamics->envelope - magnitude);
    float gain = dynamics->envelope > dynamics->threshold
        ? powf(dynamics->threshold / dynamics->envelope,
               dynamics->exponent) : 1.0f;
    float output = tanhf(sample * gain * dynamics->makeup_gain *
                         dynamics->softclip_drive) *
                   dynamics->softclip_scale;
    if (output > dynamics->output_limit) output = dynamics->output_limit;
    if (output < -dynamics->output_limit) output = -dynamics->output_limit;
    return output;
}

static void pitched_voice_chorus_block(PitchedVoiceState *state,
                                        float samples[], size_t start,
                                        size_t end) {
    const size_t modulation_block = 64;
    while (start < end) {
        size_t count = end - start < modulation_block
            ? end - start : modulation_block;
        float end_phase = state->chorus_phase +
            state->chorus_phase_increment * (float)count;
        end_phase -= floorf(end_phase);
        float delay = state->chorus_base_delay + state->chorus_depth *
            dsp_sine_cycles_wrapped(state->chorus_phase);
        float end_delay = state->chorus_base_delay + state->chorus_depth *
            dsp_sine_cycles_wrapped(end_phase);
        float delay_step = (end_delay - delay) / (float)count;
        for (size_t i = 0; i < count; ++i) {
            float read = (float)state->chorus_write - delay;
            if (read < 0.0f) read += PITCHED_VOICE_CHORUS_SIZE;
            size_t left = (size_t)read;
            size_t right = (left + 1) & PITCHED_VOICE_CHORUS_MASK;
            float fraction = read - (float)left;
            float delayed = state->chorus_delay[left] +
                (state->chorus_delay[right] - state->chorus_delay[left]) *
                fraction;
            float sample = samples[start + i];
            state->chorus_delay[state->chorus_write] = sample;
            state->chorus_write =
                (state->chorus_write + 1) & PITCHED_VOICE_CHORUS_MASK;
            samples[start + i] = sample * (1.0f - state->chorus_mix) +
                delayed * state->chorus_mix;
            delay += delay_step;
        }
        state->chorus_phase = end_phase;
        start += count;
    }
}

static int pitched_voice_drain_shifter(PitchedVoiceState *state) {
    float *outputs[1] = { state->shift_output };
    for (;;) {
        PITCHED_PROFILE_START(retrieve_start);
        int available = rubberband_available(state->shifter);
        if (available <= 0) {
            PITCHED_PROFILE_ADD(pitch_shift_seconds, retrieve_start);
            return available == 0;
        }
        size_t requested = (size_t)available < PITCHED_VOICE_SHIFT_BLOCK_SIZE
            ? (size_t)available : PITCHED_VOICE_SHIFT_BLOCK_SIZE;
        size_t retrieved = rubberband_retrieve(state->shifter, outputs,
                                                (unsigned int)requested);
        PITCHED_PROFILE_ADD(pitch_shift_seconds, retrieve_start);
        if (retrieved == 0) return 0;
        size_t start = 0;
        if (state->startup_discard > 0) {
            size_t discard = state->startup_discard < retrieved
                ? state->startup_discard : retrieved;
            state->startup_discard -= discard;
            start = discard;
        }
        size_t count = retrieved - start;
        size_t output_count = count * state->pitch_sample_rate_divisor;
        if (output_count >
            PITCHED_VOICE_OUTPUT_FIFO_SIZE - state->output_fifo_count) return 0;
        if (state->chorus_mix > 0.0f) {
            PITCHED_PROFILE_START(chorus_start);
            pitched_voice_chorus_block(
                state, state->shift_output, start, retrieved);
            PITCHED_PROFILE_ADD(chorus_seconds, chorus_start);
        }
        PITCHED_PROFILE_START(output_push_start);
        for (size_t i = start; i < retrieved; ++i) {
            float sample = state->shift_output[i];
            if (state->pitch_sample_rate_divisor == 1) {
                if (!pitched_voice_fifo_push(state, sample))
                    return 0;
            } else if (!pitched_voice_upsample_push(
                           state, sample)) {
                return 0;
            }
        }
        PITCHED_PROFILE_ADD(output_push_seconds, output_push_start);
    }
}

static int pitched_voice_process_block(PitchedVoiceState *state,
                                       const float input[], size_t count) {
    const float *inputs[1] = { input };
    PITCHED_PROFILE_START(process_start);
    rubberband_process(state->shifter, inputs, (unsigned int)count, 0);
    PITCHED_PROFILE_ADD(pitch_shift_seconds, process_start);
    return pitched_voice_drain_shifter(state);
}

static int pitched_voice_shift_block(PitchedVoiceState *state) {
    int ok = pitched_voice_process_block(
        state, state->shift_input, PITCHED_VOICE_SHIFT_BLOCK_SIZE);
    ++state->profile.shift_blocks;
    return ok;
}

PitchedVoiceState *pitched_voice_init(int sample_rate,
                                      const PitchedVoiceConfig *config) {
    if (sample_rate != DSP_SAMPLE_RATE || !pitched_voice_config_valid(config))
        return NULL;
    dsp_init();
    PitchedVoiceState *state =
        (PitchedVoiceState *)calloc(1, sizeof(*state));
    if (!state) return NULL;
    state->pitch_sample_rate_divisor = config->rate ==
        PITCHED_VOICE_RATE_LOW_CPU ? PITCHED_VOICE_RATE_LOW_CPU :
        PITCHED_VOICE_RATE_FULL;
    RubberBandOptions options = RubberBandOptionProcessRealTime |
        RubberBandOptionEngineFaster |
        RubberBandOptionPitchHighSpeed |
        RubberBandOptionTransientsSmooth |
        RubberBandOptionDetectorSoft |
        RubberBandOptionThreadingNever |
        RubberBandOptionWindowStandard |
        RubberBandOptionFormantShifted;
    unsigned int pitch_sample_rate = (unsigned int)sample_rate /
        state->pitch_sample_rate_divisor;
    state->shifter = rubberband_new(pitch_sample_rate, 1, options, 1.0,
                                    config->pitch_ratio);
    if (!state->shifter) {
        free(state);
        return NULL;
    }
    rubberband_set_max_process_size(state->shifter,
                                    PITCHED_VOICE_SHIFT_BLOCK_SIZE);
    state->startup_discard = rubberband_get_start_delay(state->shifter);
    size_t startup_padding = rubberband_get_preferred_start_pad(state->shifter);
    float silence[PITCHED_VOICE_SHIFT_BLOCK_SIZE] = {0.0f};
    while (startup_padding > 0) {
        size_t count = startup_padding < PITCHED_VOICE_SHIFT_BLOCK_SIZE
            ? startup_padding : PITCHED_VOICE_SHIFT_BLOCK_SIZE;
        if (!pitched_voice_process_block(state, silence, count)) {
            pitched_voice_free(state);
            return NULL;
        }
        startup_padding -= count;
    }
    state->tremolo_increment = config->tremolo_hz / sample_rate;
    state->tremolo_depth = config->tremolo_depth;
    state->chorus_phase_increment =
        config->chorus_hz / (float)pitch_sample_rate;
    state->chorus_base_delay =
        config->chorus_base_delay_ms * (float)pitch_sample_rate / 1000.0f;
    state->chorus_depth =
        config->chorus_depth_ms * (float)pitch_sample_rate / 1000.0f;
    state->chorus_mix = config->chorus_mix;

    if (config->highpass_hz > 0.0f)
        state->filters[state->filter_count++] =
            dsp_highpass(config->highpass_hz, sample_rate);
    for (size_t i = 0; i < config->peak_count; ++i) {
        const PitchedVoicePeak *peak = &config->peaks[i];
        state->filters[state->filter_count++] =
            dsp_peak_eq(peak->frequency, sample_rate,
                        peak->q, peak->gain_db);
    }
    if (config->lowpass_hz > 0.0f)
        state->filters[state->filter_count++] =
            dsp_lowpass(config->lowpass_hz, sample_rate);

    float sr = (float)sample_rate;
    state->dynamics.attack =
        expf(-1.0f / (config->attack_ms * 0.001f * sr));
    state->dynamics.release =
        expf(-1.0f / (config->release_ms * 0.001f * sr));
    state->dynamics.threshold = config->compressor_threshold;
    state->dynamics.exponent = 1.0f - 1.0f / config->compressor_ratio;
    state->dynamics.makeup_gain = config->makeup_gain;
    state->dynamics.softclip_drive = config->softclip_drive;
    state->dynamics.softclip_scale = config->softclip_scale;
    state->dynamics.output_limit = config->output_limit;
    return state;
}

void pitched_voice_free(PitchedVoiceState *state) {
    if (!state) return;
    if (state->shifter) rubberband_delete(state->shifter);
    free(state);
}

int pitched_voice_process(PitchedVoiceState *state,
                          const float input[DSP_HOP_SIZE],
                          float output[DSP_HOP_SIZE]) {
    if (!state || !input || !output) return PITCHED_VOICE_PROCESS_ERROR;
    PITCHED_PROFILE_WALL_START(hop_start);
    if (state->pitch_sample_rate_divisor == 1) {
        size_t input_offset = 0;
        while (input_offset < DSP_HOP_SIZE) {
            size_t available = PITCHED_VOICE_SHIFT_BLOCK_SIZE -
                state->shift_input_fill;
            size_t count = DSP_HOP_SIZE - input_offset < available
                ? DSP_HOP_SIZE - input_offset : available;
            memcpy(state->shift_input + state->shift_input_fill,
                   input + input_offset, count * sizeof(*input));
            state->shift_input_fill += count;
            input_offset += count;
            if (state->shift_input_fill == PITCHED_VOICE_SHIFT_BLOCK_SIZE) {
                if (!pitched_voice_shift_block(state)) {
                    PITCHED_PROFILE_MAX(max_hop_seconds, hop_start);
                    return PITCHED_VOICE_PROCESS_ERROR;
                }
                state->shift_input_fill = 0;
            }
        }
    } else {
        for (size_t i = 0; i < DSP_HOP_SIZE; ++i)
            pitched_voice_downsample_push(state, input[i]);
        if (state->shift_input_fill == PITCHED_VOICE_SHIFT_BLOCK_SIZE) {
            if (!pitched_voice_shift_block(state)) {
                PITCHED_PROFILE_MAX(max_hop_seconds, hop_start);
                return PITCHED_VOICE_PROCESS_ERROR;
            }
            state->shift_input_fill = 0;
        }
    }

    if (state->output_fifo_count < DSP_HOP_SIZE) {
        memset(output, 0, DSP_HOP_SIZE * sizeof(*output));
        PITCHED_PROFILE_MAX(max_hop_seconds, hop_start);
        return PITCHED_VOICE_PROCESS_NOT_READY;
    }
    for (size_t i = 0; i < DSP_HOP_SIZE; ++i) {
        output[i] = state->output_fifo[state->output_fifo_read];
        state->output_fifo_read = (state->output_fifo_read + 1) &
            (PITCHED_VOICE_OUTPUT_FIFO_SIZE - 1);
    }
    state->output_fifo_count -= DSP_HOP_SIZE;

    PITCHED_PROFILE_START(eq_start);
    for (size_t i = 0; i < DSP_HOP_SIZE; ++i)
        for (size_t filter = 0; filter < state->filter_count; ++filter)
            output[i] = dsp_biquad_process(&state->filters[filter], output[i]);
    PITCHED_PROFILE_ADD(eq_seconds, eq_start);

    if (state->tremolo_depth > 0.0f) {
        PITCHED_PROFILE_START(tremolo_start);
        float end_phase = state->tremolo_phase +
            state->tremolo_increment * (float)DSP_HOP_SIZE;
        if (end_phase >= 1.0f) end_phase -= floorf(end_phase);
        float modulation = dsp_sine_cycles_wrapped(state->tremolo_phase);
        float end_modulation = dsp_sine_cycles_wrapped(end_phase);
        float gain = 1.0f - state->tremolo_depth *
            0.5f * (1.0f - modulation);
        float end_gain = 1.0f - state->tremolo_depth *
            0.5f * (1.0f - end_modulation);
        float gain_step = (end_gain - gain) / (float)DSP_HOP_SIZE;
        for (size_t i = 0; i < DSP_HOP_SIZE; ++i) {
            output[i] *= gain;
            gain += gain_step;
        }
        state->tremolo_phase = end_phase;
        PITCHED_PROFILE_ADD(tremolo_seconds, tremolo_start);
    }

    PITCHED_PROFILE_START(dynamics_start);
    for (size_t i = 0; i < DSP_HOP_SIZE; ++i)
        output[i] = pitched_voice_dynamics_sample(state, output[i]);
    PITCHED_PROFILE_ADD(dynamics_seconds, dynamics_start);
    ++state->profile.hops;
    PITCHED_PROFILE_MAX(max_hop_seconds, hop_start);
    return PITCHED_VOICE_PROCESS_OUTPUT;
}

void pitched_voice_get_profile(const PitchedVoiceState *state,
                               PitchedVoiceProfile *profile) {
    if (state && profile) *profile = state->profile;
}
