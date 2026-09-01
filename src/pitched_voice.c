#include "pitched_voice.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#ifdef VC_PITCHED_VOICE_PROFILE
#include <time.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PITCHED_VOICE_MAX_FILTERS (PITCHED_VOICE_MAX_PEAKS + 2)
#define PITCHED_VOICE_SHIFT_BLOCK_SIZE (2 * DSP_HOP_SIZE)
#define PITCHED_VOICE_OUTPUT_FIFO_SIZE 8192
#define PITCHED_VOICE_RESAMPLE_FACTOR 2
#define PITCHED_VOICE_RESAMPLE_TAPS 65
#define PITCHED_VOICE_CHORUS_SIZE 512
#define PITCHED_VOICE_CHORUS_MASK (PITCHED_VOICE_CHORUS_SIZE - 1)
#define PITCHED_VOICE_PV_FRAME_SIZE 1024
#define PITCHED_VOICE_PV_HOP_SIZE 256
#define PITCHED_VOICE_PV_BIN_COUNT (PITCHED_VOICE_PV_FRAME_SIZE / 2 + 1)

/* 10 kHz low-pass at 48 kHz, Kaiser beta 7, -82 dB at 12 kHz. */
static const float k_resample_coefficients[PITCHED_VOICE_RESAMPLE_TAPS] = {
    -5.1094626430e-05f, 3.0077214380e-05f, 1.9717394532e-04f,
    7.9571244414e-05f, -3.9252159588e-04f, -4.5364061592e-04f,
    4.4001969021e-04f, 1.1370968109e-03f, -1.8888485908e-18f,
    -1.9180384432e-03f, -1.2592081199e-03f, 2.2293951451e-03f,
    3.3797883478e-03f, -1.2379661752e-03f, -5.8116919226e-03f,
    -1.8139027909e-03f, 7.2718788502e-03f, 7.0750040598e-03f,
    -5.9348923269e-03f, -1.3556047624e-02f, 1.0138538233e-17f,
    1.8853283851e-02f, 1.1513317216e-02f, -1.9262073839e-02f,
    -2.8057813774e-02f, 1.0061460332e-02f, 4.7295675071e-02f,
    1.5216750508e-02f, -6.5500719328e-02f, -7.2913919411e-02f,
    7.8573496919e-02f, 3.0648276717e-01f, 4.1665354844e-01f,
    3.0648276717e-01f, 7.8573496919e-02f, -7.2913919411e-02f,
    -6.5500719328e-02f, 1.5216750508e-02f, 4.7295675071e-02f,
    1.0061460332e-02f, -2.8057813774e-02f, -1.9262073839e-02f,
    1.1513317216e-02f, 1.8853283851e-02f, 1.0138538233e-17f,
    -1.3556047624e-02f, -5.9348923269e-03f, 7.0750040598e-03f,
    7.2718788502e-03f, -1.8139027909e-03f, -5.8116919226e-03f,
    -1.2379661752e-03f, 3.3797883478e-03f, 2.2293951451e-03f,
    -1.2592081199e-03f, -1.9180384432e-03f, -1.8888485908e-18f,
    1.1370968109e-03f, 4.4001969021e-04f, -4.5364061592e-04f,
    -3.9252159588e-04f, 7.9571244414e-05f, 1.9717394532e-04f,
    3.0077214380e-05f, -5.1094626430e-05f,
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
_Static_assert(PITCHED_VOICE_SHIFT_BLOCK_SIZE % PITCHED_VOICE_PV_HOP_SIZE == 0,
               "pitch block must contain whole phase-vocoder hops");
_Static_assert(PITCHED_VOICE_RESAMPLE_FACTOR == 2,
               "symmetric polyphase resampler requires a factor of two");
_Static_assert(PITCHED_VOICE_RESAMPLE_TAPS % 2 == 1,
               "symmetric resampler requires an odd tap count");

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
    unsigned int pitch_sample_rate_divisor;
    float pitch_ratio;
    float shift_input[PITCHED_VOICE_SHIFT_BLOCK_SIZE];
    float shift_output[PITCHED_VOICE_SHIFT_BLOCK_SIZE];
    size_t shift_input_fill;
    float pv_input[PITCHED_VOICE_PV_FRAME_SIZE];
    float pv_window[PITCHED_VOICE_PV_FRAME_SIZE];
    float pv_ola[PITCHED_VOICE_PV_FRAME_SIZE];
    float pv_weights[PITCHED_VOICE_PV_FRAME_SIZE];
    DspComplex pv_fft[PITCHED_VOICE_PV_FRAME_SIZE];
    float pv_magnitudes[PITCHED_VOICE_PV_BIN_COUNT];
    float pv_input_phases[PITCHED_VOICE_PV_BIN_COUNT];
    float pv_frequencies[PITCHED_VOICE_PV_BIN_COUNT];
    float pv_previous_phases[PITCHED_VOICE_PV_BIN_COUNT];
    float pv_output_phases[PITCHED_VOICE_PV_BIN_COUNT];
    float pv_output_magnitudes[PITCHED_VOICE_PV_BIN_COUNT];
    float pv_base_phases[PITCHED_VOICE_PV_BIN_COUNT];
    uint16_t pv_source_bins[PITCHED_VOICE_PV_BIN_COUNT];
    float pv_source_fractions[PITCHED_VOICE_PV_BIN_COUNT];
    size_t pv_mapped_bins;
    size_t pv_analysis_bins;
    const float *sine_lut;
    size_t pv_frames;
    int pv_phase_ready;
    float downsample_history[2 * PITCHED_VOICE_RESAMPLE_TAPS];
    size_t downsample_write;
    unsigned int downsample_phase;
    float upsample_history[2 * PITCHED_VOICE_RESAMPLE_TAPS];
    size_t upsample_write;
    float output_fifo[PITCHED_VOICE_OUTPUT_FIFO_SIZE];
    size_t output_fifo_read;
    size_t output_fifo_count;

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
         config->rate != PITCHED_VOICE_RATE_WIDEBAND &&
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
    unsigned int rate_divisor =
        (config->rate == PITCHED_VOICE_RATE_WIDEBAND ||
         config->rate == PITCHED_VOICE_RATE_LOW_CPU)
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
    size_t half = PITCHED_VOICE_RESAMPLE_TAPS / 2;
    for (size_t tap = 0; tap < half; ++tap) {
        sum += k_resample_coefficients[tap] *
            (history[latest - tap] +
             history[latest - (PITCHED_VOICE_RESAMPLE_TAPS - 1 - tap)]);
    }
    sum += k_resample_coefficients[half] * history[latest - half];
    return sum;
}

static float pitched_voice_resample_phase(const float history[],
                                           size_t latest, size_t phase) {
    size_t count = (PITCHED_VOICE_RESAMPLE_TAPS - phase +
                    PITCHED_VOICE_RESAMPLE_FACTOR - 1) /
        PITCHED_VOICE_RESAMPLE_FACTOR;
    float sum = 0.0f;
    for (size_t offset = 0; offset < count / 2; ++offset) {
        size_t tap = phase + offset * PITCHED_VOICE_RESAMPLE_FACTOR;
        sum += k_resample_coefficients[tap] *
            (history[latest - offset] +
             history[latest - (count - 1 - offset)]);
    }
    if (count & 1) {
        size_t offset = count / 2;
        size_t tap = phase + offset * PITCHED_VOICE_RESAMPLE_FACTOR;
        sum += k_resample_coefficients[tap] * history[latest - offset];
    }
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
    state->upsample_write = write + 1 == PITCHED_VOICE_RESAMPLE_TAPS
        ? 0 : write + 1;

    for (size_t phase = 0; phase < PITCHED_VOICE_RESAMPLE_FACTOR; ++phase) {
        float sum = pitched_voice_resample_phase(
            state->upsample_history, latest, phase);
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
    state->downsample_write = write + 1 == PITCHED_VOICE_RESAMPLE_TAPS
        ? 0 : write + 1;

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

static float pitched_voice_wrap_phase(float phase) {
    const float two_pi = 2.0f * (float)M_PI;
    return phase - two_pi * roundf(phase / two_pi);
}

static DspComplex pitched_voice_phase_vector(const PitchedVoiceState *state,
                                              float cycles) {
    if (cycles >= 1.0f) cycles -= 1.0f;
    if (cycles < 0.0f) cycles += 1.0f;
    if (cycles < 0.0f) cycles += 1.0f;
    float position = cycles * 1024.0f;
    size_t sine_index = (size_t)position;
    float fraction = position - (float)sine_index;
    size_t cosine_index = (sine_index + 256) & 1023;
    float sine = state->sine_lut[sine_index] + fraction *
        (state->sine_lut[sine_index + 1] - state->sine_lut[sine_index]);
    float cosine = state->sine_lut[cosine_index] + fraction *
        (state->sine_lut[cosine_index + 1] - state->sine_lut[cosine_index]);
    return (DspComplex){cosine, sine};
}

static void pitched_voice_vocoder_frame(PitchedVoiceState *state,
                                         const float input[], float output[]) {
    const float two_pi = 2.0f * (float)M_PI;
    const float bin_scale = two_pi / (float)PITCHED_VOICE_PV_FRAME_SIZE;
    const float hop = (float)PITCHED_VOICE_PV_HOP_SIZE;
    memmove(state->pv_input,
            state->pv_input + PITCHED_VOICE_PV_HOP_SIZE,
            (PITCHED_VOICE_PV_FRAME_SIZE - PITCHED_VOICE_PV_HOP_SIZE) *
                sizeof(*state->pv_input));
    memcpy(state->pv_input +
               (PITCHED_VOICE_PV_FRAME_SIZE - PITCHED_VOICE_PV_HOP_SIZE),
           input, PITCHED_VOICE_PV_HOP_SIZE * sizeof(*input));

    double energy = 0.0;
    for (size_t i = 0; i < PITCHED_VOICE_PV_FRAME_SIZE; ++i) {
        float sample = state->pv_input[i];
        energy += (double)sample * sample;
        state->pv_fft[i] = (DspComplex){sample * state->pv_window[i], 0.0f};
    }

    if (energy > 1e-12) {
        dsp_rfft(state->pv_fft, PITCHED_VOICE_PV_FRAME_SIZE);
        for (size_t bin = 0; bin < state->pv_analysis_bins; ++bin) {
            float real = state->pv_fft[bin].r;
            float imaginary = state->pv_fft[bin].i;
            float phase = atan2f(imaginary, real);
            state->pv_magnitudes[bin] =
                sqrtf(real * real + imaginary * imaginary);
            state->pv_input_phases[bin] = phase;
            float omega = bin_scale * (float)bin;
            if (state->pv_phase_ready) {
                float delta = phase - state->pv_previous_phases[bin] -
                    omega * hop;
                omega += pitched_voice_wrap_phase(delta) / hop;
            }
            state->pv_previous_phases[bin] = phase;
            state->pv_frequencies[bin] = omega;
        }

        memset(state->pv_fft, 0, sizeof(state->pv_fft));
        memset(state->pv_output_magnitudes, 0,
               sizeof(state->pv_output_magnitudes));
        size_t mapped_bins = state->pv_mapped_bins;
        for (size_t target = 0; target < mapped_bins; ++target) {
            size_t left = state->pv_source_bins[target];
            size_t right = left + 1 < PITCHED_VOICE_PV_BIN_COUNT
                ? left + 1 : left;
            float fraction = state->pv_source_fractions[target];
            float magnitude = state->pv_magnitudes[left] +
                (state->pv_magnitudes[right] -
                 state->pv_magnitudes[left]) * fraction;
            float frequency = state->pv_frequencies[left] +
                (state->pv_frequencies[right] -
                 state->pv_frequencies[left]) * fraction;
            float phase_delta = pitched_voice_wrap_phase(
                state->pv_input_phases[right] -
                state->pv_input_phases[left]);
            state->pv_base_phases[target] =
                state->pv_input_phases[left] + phase_delta * fraction;
            state->pv_output_magnitudes[target] = magnitude;
            if (!state->pv_phase_ready) {
                state->pv_output_phases[target] =
                    state->pv_base_phases[target];
            } else {
                state->pv_output_phases[target] = pitched_voice_wrap_phase(
                    state->pv_output_phases[target] +
                    frequency * state->pitch_ratio * hop);
            }
        }

        size_t peaks[PITCHED_VOICE_PV_BIN_COUNT];
        size_t peak_count = 0;
        if (state->pitch_ratio < 1.75f) {
            for (size_t bin = 1; bin + 1 < mapped_bins; ++bin) {
                if (state->pv_output_magnitudes[bin] >=
                        state->pv_output_magnitudes[bin - 1] &&
                    state->pv_output_magnitudes[bin] >
                        state->pv_output_magnitudes[bin + 1]) {
                    peaks[peak_count++] = bin;
                }
            }
        }
        size_t peak_cursor = 0;
        for (size_t target = 0; target < mapped_bins; ++target) {
            float phase = state->pv_output_phases[target];
            if (state->pitch_ratio < 1.75f && peak_count > 0 && target > 0 &&
                target < PITCHED_VOICE_PV_FRAME_SIZE / 2) {
                while (peak_cursor + 1 < peak_count &&
                       peaks[peak_cursor + 1] <= target) {
                    ++peak_cursor;
                }
                size_t nearest = peaks[peak_cursor];
                if (peak_cursor + 1 < peak_count &&
                    peaks[peak_cursor + 1] - target <
                    (target > nearest ? target - nearest : nearest - target)) {
                    nearest = peaks[peak_cursor + 1];
                }
                size_t distance = target > nearest
                    ? target - nearest : nearest - target;
                if (distance <= 4) {
                    phase = state->pv_output_phases[nearest] +
                        pitched_voice_wrap_phase(
                            state->pv_base_phases[target] -
                            state->pv_base_phases[nearest]);
                }
            }
            float magnitude = state->pv_output_magnitudes[target];
            float phase_cycles = phase / two_pi;
            DspComplex phase_vector =
                pitched_voice_phase_vector(state, phase_cycles);
            DspComplex value = {
                magnitude * phase_vector.r, magnitude * phase_vector.i};
            if (target == 0 ||
                target == PITCHED_VOICE_PV_FRAME_SIZE / 2) {
                value.i = 0.0f;
            }
            state->pv_fft[target] = value;
        }
        state->pv_phase_ready = 1;
        dsp_irfft(state->pv_fft, PITCHED_VOICE_PV_FRAME_SIZE);
    } else {
        memset(state->pv_fft, 0, sizeof(state->pv_fft));
        state->pv_phase_ready = 0;
    }

    for (size_t i = 0; i < PITCHED_VOICE_PV_FRAME_SIZE; ++i) {
        float window = state->pv_window[i];
        state->pv_ola[i] += state->pv_fft[i].r * window;
        state->pv_weights[i] += window * window;
    }
    ++state->pv_frames;
    for (size_t i = 0; i < PITCHED_VOICE_PV_HOP_SIZE; ++i) {
        output[i] = state->pv_frames >= 4 && state->pv_weights[i] > 1e-3f
            ? state->pv_ola[i] / state->pv_weights[i] : 0.0f;
    }
    memmove(state->pv_ola,
            state->pv_ola + PITCHED_VOICE_PV_HOP_SIZE,
            (PITCHED_VOICE_PV_FRAME_SIZE - PITCHED_VOICE_PV_HOP_SIZE) *
                sizeof(*state->pv_ola));
    memmove(state->pv_weights,
            state->pv_weights + PITCHED_VOICE_PV_HOP_SIZE,
            (PITCHED_VOICE_PV_FRAME_SIZE - PITCHED_VOICE_PV_HOP_SIZE) *
                sizeof(*state->pv_weights));
    memset(state->pv_ola +
               (PITCHED_VOICE_PV_FRAME_SIZE - PITCHED_VOICE_PV_HOP_SIZE),
           0, PITCHED_VOICE_PV_HOP_SIZE * sizeof(*state->pv_ola));
    memset(state->pv_weights +
               (PITCHED_VOICE_PV_FRAME_SIZE - PITCHED_VOICE_PV_HOP_SIZE),
           0, PITCHED_VOICE_PV_HOP_SIZE * sizeof(*state->pv_weights));
}

static int pitched_voice_shift_block(PitchedVoiceState *state) {
    if (PITCHED_VOICE_SHIFT_BLOCK_SIZE * state->pitch_sample_rate_divisor >
        PITCHED_VOICE_OUTPUT_FIFO_SIZE - state->output_fifo_count) return 0;
    PITCHED_PROFILE_START(process_start);
    for (size_t offset = 0; offset < PITCHED_VOICE_SHIFT_BLOCK_SIZE;
         offset += PITCHED_VOICE_PV_HOP_SIZE) {
        pitched_voice_vocoder_frame(
            state, state->shift_input + offset, state->shift_output + offset);
    }
    PITCHED_PROFILE_ADD(pitch_shift_seconds, process_start);

    if (state->chorus_mix > 0.0f) {
        PITCHED_PROFILE_START(chorus_start);
        pitched_voice_chorus_block(
            state, state->shift_output, 0, PITCHED_VOICE_SHIFT_BLOCK_SIZE);
        PITCHED_PROFILE_ADD(chorus_seconds, chorus_start);
    }
    PITCHED_PROFILE_START(output_push_start);
    for (size_t i = 0; i < PITCHED_VOICE_SHIFT_BLOCK_SIZE; ++i) {
        float sample = state->shift_output[i];
        if (state->pitch_sample_rate_divisor == 1) {
            if (!pitched_voice_fifo_push(state, sample)) return 0;
        } else if (!pitched_voice_upsample_push(state, sample)) {
            return 0;
        }
    }
    PITCHED_PROFILE_ADD(output_push_seconds, output_push_start);
    ++state->profile.shift_blocks;
    return 1;
}

PitchedVoiceState *pitched_voice_init(int sample_rate,
                                      const PitchedVoiceConfig *config) {
    if (sample_rate != DSP_SAMPLE_RATE || !pitched_voice_config_valid(config))
        return NULL;
    dsp_init();
    PitchedVoiceState *state =
        (PitchedVoiceState *)calloc(1, sizeof(*state));
    if (!state) return NULL;
    state->pitch_sample_rate_divisor =
        (config->rate == PITCHED_VOICE_RATE_WIDEBAND ||
         config->rate == PITCHED_VOICE_RATE_LOW_CPU)
        ? PITCHED_VOICE_RESAMPLE_FACTOR : PITCHED_VOICE_RATE_FULL;
    state->pitch_ratio = config->pitch_ratio;
    state->sine_lut = dsp_sine_lut();
    for (size_t target = 0; target < PITCHED_VOICE_PV_BIN_COUNT; ++target) {
        float source = (float)target / state->pitch_ratio;
        if (source > PITCHED_VOICE_PV_BIN_COUNT - 1) break;
        size_t left = (size_t)source;
        state->pv_source_bins[state->pv_mapped_bins] = (uint16_t)left;
        state->pv_source_fractions[state->pv_mapped_bins] =
            source - (float)left;
        ++state->pv_mapped_bins;
        size_t required = left + 2 < PITCHED_VOICE_PV_BIN_COUNT
            ? left + 2 : PITCHED_VOICE_PV_BIN_COUNT;
        if (required > state->pv_analysis_bins)
            state->pv_analysis_bins = required;
    }
    unsigned int pitch_sample_rate = (unsigned int)sample_rate /
        state->pitch_sample_rate_divisor;
    for (size_t i = 0; i < PITCHED_VOICE_PV_FRAME_SIZE; ++i)
        state->pv_window[i] = dsp_hann(i, PITCHED_VOICE_PV_FRAME_SIZE);
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
    for (size_t filter = 0; filter < state->filter_count; ++filter)
        for (size_t i = 0; i < DSP_HOP_SIZE; ++i)
            output[i] = dsp_biquad_process_inline(
                &state->filters[filter], output[i]);
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
