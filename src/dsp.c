#include "dsp.h"

#include <math.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DSP_YIN_CONFIDENCE_LIMIT 0.45f
#define DSP_YIN_THRESHOLD 0.15f
#define DSP_LPC_FFT_SIZE 2048
#define DSP_LPC_ORDER_MAX 24
#define DSP_FFT_MAX_STAGES 11
#define DSP_LPC_HARMONIC_LIMIT 160

static float g_trig_lut[1025];
static float g_frame_hann[DSP_FRAME_SIZE];
static uint16_t g_fft_bit_reverse_512[DSP_YIN_FFT_SIZE];
static uint16_t g_fft_bit_reverse_2048[DSP_LPC_FFT_SIZE];
static DspComplex g_fft_steps[DSP_FFT_MAX_STAGES][2];
static atomic_int g_tables_state;

_Static_assert((DSP_PSOLA_RING_SIZE & (DSP_PSOLA_RING_SIZE - 1)) == 0,
               "PSOLA ring size must be a power of two");
_Static_assert((DSP_FRAME_SIZE & (DSP_FRAME_SIZE - 1)) == 0,
               "DSP frame size must be a power of two");
_Static_assert((DSP_LPC_FFT_SIZE & (DSP_LPC_FFT_SIZE - 1)) == 0,
               "LPC FFT size must be a power of two");
_Static_assert(DSP_LPC_FFT_SIZE == DSP_FRAME_SIZE,
               "LPC FFT size must match the DSP frame size");
_Static_assert(DSP_FRAME_SIZE % DSP_HOP_SIZE == 0,
               "DSP frame size must be an integer number of hops");
_Static_assert(DSP_YIN_ANALYSIS_SIZE * (DSP_SAMPLE_RATE /
               DSP_YIN_ANALYSIS_RATE) <= DSP_FRAME_SIZE,
               "YIN analysis window must fit in the DSP frame");
_Static_assert(DSP_LPC_FFT_SIZE <= UINT16_MAX,
               "FFT bit-reverse indices must fit in uint16_t");

static float dsp_sine_cycles_wrapped_ready(float cycles) {
    float position = cycles * 1024.0f;
    int index = (int)position;
    float fraction = position - (float)index;
    return g_trig_lut[index] + fraction *
        (g_trig_lut[index + 1] - g_trig_lut[index]);
}

static float dsp_sine_cycles_ready(float cycles) {
    cycles -= floorf(cycles);
    return dsp_sine_cycles_wrapped_ready(cycles);
}

static float dsp_hann_ready(size_t index, size_t size) {
    float phase = (float)index / (float)(size - 1);
    return 0.5f - 0.5f * dsp_sine_cycles_ready(phase + 0.25f);
}

static void dsp_build_bit_reverse(uint16_t *indices, size_t size) {
    indices[0] = 0;
    for (size_t i = 1, j = 0; i < size; ++i) {
        size_t bit = size >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        indices[i] = (uint16_t)j;
    }
}

void dsp_init(void) {
    if (atomic_load_explicit(&g_tables_state, memory_order_acquire) == 2)
        return;
    int expected = 0;
    if (atomic_compare_exchange_strong_explicit(
            &g_tables_state, &expected, 1,
            memory_order_acq_rel, memory_order_acquire)) {
        for (int i = 0; i <= 1024; ++i)
            g_trig_lut[i] = sinf(2.0f * (float)M_PI * (float)i / 1024.0f);
        for (size_t i = 0; i < DSP_FRAME_SIZE; ++i) {
            g_frame_hann[i] = dsp_hann_ready(i, DSP_FRAME_SIZE);
        }
        dsp_build_bit_reverse(g_fft_bit_reverse_512, DSP_YIN_FFT_SIZE);
        dsp_build_bit_reverse(g_fft_bit_reverse_2048, DSP_LPC_FFT_SIZE);
        size_t length = 2;
        for (size_t stage = 0; stage < DSP_FFT_MAX_STAGES; ++stage) {
            float angle = 2.0f * (float)M_PI / (float)length;
            g_fft_steps[stage][0] = (DspComplex){cosf(angle), -sinf(angle)};
            g_fft_steps[stage][1] = (DspComplex){cosf(angle), sinf(angle)};
            length <<= 1;
        }
        atomic_store_explicit(&g_tables_state, 2, memory_order_release);
        return;
    }
    while (atomic_load_explicit(&g_tables_state, memory_order_acquire) != 2) {
    }
}

float dsp_sine_cycles(float cycles) {
    dsp_init();
    return dsp_sine_cycles_ready(cycles);
}

float dsp_sine_cycles_wrapped(float cycles) {
    if (cycles >= 1.0f) cycles -= 1.0f;
    else if (cycles < 0.0f) cycles += 1.0f;
    return dsp_sine_cycles_wrapped_ready(cycles);
}

float dsp_hann(size_t index, size_t size) {
    if (size <= 1 || index >= size) return 1.0f;
    dsp_init();
    return dsp_hann_ready(index, size);
}

const float *dsp_frame_hann(void) {
    dsp_init();
    return g_frame_hann;
}

static void dsp_fft_ready(DspComplex *values, size_t size, int inverse) {
    const uint16_t *bit_reverse = size == DSP_YIN_FFT_SIZE
        ? g_fft_bit_reverse_512
        : (size == DSP_LPC_FFT_SIZE ? g_fft_bit_reverse_2048 : NULL);
    for (size_t i = 1, j = 0; i < size; ++i) {
        if (bit_reverse) {
            j = bit_reverse[i];
        } else {
            size_t bit = size >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
        }
        if (i < j) {
            DspComplex temp = values[i];
            values[i] = values[j];
            values[j] = temp;
        }
    }

    size_t stage = 0;
    for (size_t length = 2; length <= size; length <<= 1, ++stage) {
        size_t half = length >> 1;
        DspComplex step;
        if (stage < DSP_FFT_MAX_STAGES) {
            step = g_fft_steps[stage][inverse != 0];
        } else {
            float angle = (inverse ? 2.0f : -2.0f) *
                (float)M_PI / (float)length;
            step = (DspComplex){cosf(angle), sinf(angle)};
        }
        for (size_t start = 0; start < size; start += length) {
            DspComplex rotation = {1.0f, 0.0f};
            for (size_t j = 0; j < half; ++j) {
                DspComplex even = values[start + j];
                DspComplex odd = values[start + j + half];
                DspComplex rotated = {
                    odd.r * rotation.r - odd.i * rotation.i,
                    odd.r * rotation.i + odd.i * rotation.r,
                };
                values[start + j] = (DspComplex){
                    even.r + rotated.r, even.i + rotated.i};
                values[start + j + half] = (DspComplex){
                    even.r - rotated.r, even.i - rotated.i};
                DspComplex next = {
                    rotation.r * step.r - rotation.i * step.i,
                    rotation.r * step.i + rotation.i * step.r,
                };
                rotation = next;
            }
        }
    }

    if (inverse) {
        for (size_t i = 0; i < size; ++i) {
            values[i].r /= (float)size;
            values[i].i /= (float)size;
        }
    }
}

void dsp_fft(DspComplex *values, size_t size, int inverse) {
    dsp_init();
    dsp_fft_ready(values, size, inverse);
}

static DspBiquad dsp_biquad_make(float b0, float b1, float b2,
                                 float a0, float a1, float a2) {
    DspBiquad filter = {
        b0 / a0, b1 / a0, b2 / a0,
        a1 / a0, a2 / a0, 0.0f, 0.0f};
    return filter;
}

DspBiquad dsp_highpass(float frequency, int sample_rate) {
    float w = 2.0f * (float)M_PI * frequency / (float)sample_rate;
    float alpha = sinf(w) / (2.0f * 0.7071f);
    float c = cosf(w);
    return dsp_biquad_make((1.0f + c) / 2.0f, -(1.0f + c),
                           (1.0f + c) / 2.0f,
                           1.0f + alpha, -2.0f * c, 1.0f - alpha);
}

DspBiquad dsp_lowpass(float frequency, int sample_rate) {
    float w = 2.0f * (float)M_PI * frequency / (float)sample_rate;
    float alpha = sinf(w) / (2.0f * 0.7071f);
    float c = cosf(w);
    return dsp_biquad_make((1.0f - c) / 2.0f, 1.0f - c,
                           (1.0f - c) / 2.0f,
                           1.0f + alpha, -2.0f * c, 1.0f - alpha);
}

DspBiquad dsp_peak_eq(float frequency, int sample_rate,
                      float q, float gain_db) {
    float w = 2.0f * (float)M_PI * frequency / (float)sample_rate;
    float alpha = sinf(w) / (2.0f * q);
    float c = cosf(w);
    float gain = powf(10.0f, gain_db / 40.0f);
    return dsp_biquad_make(1.0f + alpha * gain, -2.0f * c,
                           1.0f - alpha * gain,
                           1.0f + alpha / gain, -2.0f * c,
                           1.0f - alpha / gain);
}

float dsp_biquad_process(DspBiquad *filter, float sample) {
    float output = filter->b0 * sample + filter->z1;
    filter->z1 = filter->b1 * sample - filter->a1 * output + filter->z2;
    filter->z2 = filter->b2 * sample - filter->a2 * output;
    return isfinite(output) ? output : 0.0f;
}

float dsp_frame_rms(const float frame[DSP_FRAME_SIZE]) {
    double energy = 0.0;
    for (size_t i = 0; i < DSP_FRAME_SIZE; ++i)
        energy += (double)frame[i] * frame[i];
    return (float)sqrt(energy / DSP_FRAME_SIZE);
}

void dsp_yin_init(DspYinState *state, int sample_rate) {
    dsp_init();
    memset(state, 0, sizeof(*state));
    state->sample_rate = sample_rate;
    state->decimation = sample_rate / DSP_YIN_ANALYSIS_RATE;
    if (state->decimation < 1) state->decimation = 1;
    state->pitch_sample_rate = sample_rate / state->decimation;
    state->decimation_reciprocal = 1.0f / (float)state->decimation;
    state->max_lag = state->pitch_sample_rate / (int)DSP_YIN_MIN_HZ;
    if (state->max_lag > DSP_YIN_ANALYSIS_SIZE - 1)
        state->max_lag = DSP_YIN_ANALYSIS_SIZE - 1;
    if (state->max_lag < 1) state->max_lag = 1;
}

void dsp_yin_reset(DspYinState *state) {
    int sample_rate = state->sample_rate;
    dsp_yin_init(state, sample_rate);
}

static void dsp_yin_output(const DspYinState *state, DspPitch *pitch) {
    pitch->f0_hz = state->cached_f0;
    pitch->confidence = state->cached_confidence;
    pitch->voiced = state->cached_f0 > 0.0f;
    pitch->period_samples = pitch->voiced
        ? (int)lroundf((float)state->sample_rate / pitch->f0_hz) : 0;
}

int dsp_yin_process(DspYinState *state, const float frame[DSP_FRAME_SIZE],
                    float frame_rms, uint64_t global_start, DspPitch *pitch) {
    if (!state || !frame || !pitch) return 0;
    memset(pitch, 0, sizeof(*pitch));
    state->last_was_update = 0;

    /* Reject silence on every hop, even while the pitch estimate is cached. */
    if (frame_rms < 8e-4f) {
        state->cached_f0 = 0.0f;
        state->cached_confidence = 1.0f;
        state->cache_ready = 0;
        state->hops_since_update = 0;
        return 1;
    }

    if (state->cache_ready && state->hops_since_update < DSP_YIN_UPDATE_HOPS) {
        ++state->hops_since_update;
        dsp_yin_output(state, pitch);
        return 1;
    }

    int decimation = state->decimation;
    size_t phase = (size_t)decimation -
        (size_t)(global_start % (uint64_t)decimation);
    if (phase == (size_t)decimation) phase = 0;

    state->energy_prefix[0] = 0.0f;
    for (int i = 0; i < DSP_YIN_ANALYSIS_SIZE; ++i) {
        float sum = 0.0f;
        for (int j = 0; j < decimation; ++j)
            sum += frame[phase + (size_t)i * decimation + (size_t)j];
        float sample = sum * state->decimation_reciprocal;
        state->energy_prefix[i + 1] = state->energy_prefix[i] + sample * sample;
        state->spectrum[i] = (DspComplex){sample, 0.0f};
    }
    for (int i = DSP_YIN_ANALYSIS_SIZE; i < DSP_YIN_FFT_SIZE; ++i)
        state->spectrum[i] = (DspComplex){0.0f, 0.0f};

    float energy = state->energy_prefix[DSP_YIN_ANALYSIS_SIZE];
    if (energy < 1e-5f / (float)decimation) {
        state->cached_f0 = 0.0f;
        state->cached_confidence = 1.0f;
        state->cache_ready = 1;
        state->hops_since_update = 1;
        return 1;
    }

    DspComplex *spectrum = state->spectrum;
    dsp_fft_ready(spectrum, DSP_YIN_FFT_SIZE, 0);
    for (int i = 0; i < DSP_YIN_FFT_SIZE; ++i) {
        spectrum[i].r = spectrum[i].r * spectrum[i].r +
                        spectrum[i].i * spectrum[i].i;
        spectrum[i].i = 0.0f;
    }
    dsp_fft_ready(spectrum, DSP_YIN_FFT_SIZE, 1);

    int min_lag = state->pitch_sample_rate / (int)DSP_YIN_MAX_HZ;
    int best = 0;
    for (int tau = 1; tau <= state->max_lag; ++tau) {
        float left = state->energy_prefix[DSP_YIN_ANALYSIS_SIZE - tau];
        float right = energy - state->energy_prefix[tau];
        float difference = left + right - 2.0f * spectrum[tau].r;
        state->difference[tau] = difference > 0.0f ? difference : 0.0f;
    }

    state->cmndf[0] = 1.0f;
    float running = 0.0f;
    for (int tau = 1; tau <= state->max_lag; ++tau) {
        running += state->difference[tau];
        state->cmndf[tau] = running > 0.0f
            ? state->difference[tau] * tau / running : 1.0f;
    }

    for (int tau = min_lag; tau <= state->max_lag; ++tau) {
        if (state->cmndf[tau] < DSP_YIN_THRESHOLD) {
            while (tau < state->max_lag &&
                   state->cmndf[tau + 1] < state->cmndf[tau]) ++tau;
            best = tau;
            break;
        }
    }
    if (!best) {
        best = min_lag;
        for (int tau = min_lag + 1; tau <= state->max_lag; ++tau)
            if (state->cmndf[tau] < state->cmndf[best]) best = tau;
        if (state->cmndf[best] > DSP_YIN_CONFIDENCE_LIMIT) best = 0;
    }

    state->cached_f0 = 0.0f;
    state->cached_confidence = best ? state->cmndf[best] : 1.0f;
    if (best) {
        state->cached_f0 = (float)state->pitch_sample_rate /
            (float)best;
    }
    state->cache_ready = 1;
    state->hops_since_update = 1;
    state->last_was_update = 1;
    dsp_yin_output(state, pitch);
    return 1;
}

static int64_t dsp_nearest_mark(DspPsolaState *state, int64_t target) {
    if (state->mark_count <= 0) return -1;
    if (state->mark_cursor >= state->mark_count)
        state->mark_cursor = state->mark_count - 1;
    while (state->mark_cursor + 1 < state->mark_count) {
        int64_t current = state->marks[state->mark_cursor];
        int64_t next = state->marks[state->mark_cursor + 1];
        if (llabs(next - target) > llabs(current - target)) break;
        ++state->mark_cursor;
    }
    while (state->mark_cursor > 0) {
        int64_t current = state->marks[state->mark_cursor];
        int64_t previous = state->marks[state->mark_cursor - 1];
        if (llabs(previous - target) >= llabs(current - target)) break;
        --state->mark_cursor;
    }
    return state->marks[state->mark_cursor];
}

static void dsp_rebuild_marks(DspPsolaState *state,
                              const float frame[DSP_FRAME_SIZE],
                              uint64_t global_start, int source_period) {
    int64_t frame_start = (int64_t)global_start;
    int64_t frame_end = frame_start + DSP_FRAME_SIZE;
    int64_t current = frame_start;
    state->mark_count = 0;
    while (current < frame_end && state->mark_count < DSP_PSOLA_MAX_MARKS) {
        int local_start = (int)(current - frame_start);
        int local_end = current + source_period < frame_end
            ? (int)(current + source_period - frame_start) : DSP_FRAME_SIZE;
        int peak = local_start;
        float peak_abs = fabsf(frame[peak]);
        for (int i = local_start + 1; i < local_end; ++i) {
            float sample_abs = fabsf(frame[i]);
            if (sample_abs > peak_abs) {
                peak = i;
                peak_abs = sample_abs;
            }
        }
        state->marks[state->mark_count++] = frame_start + peak;
        current = frame_start + peak + source_period;
    }
    state->mark_period = source_period;
    state->mark_cursor = 0;
    state->marks_ready = 1;
}

static void dsp_update_marks(DspPsolaState *state,
                             const float frame[DSP_FRAME_SIZE],
                             uint64_t global_start, int source_period) {
    int64_t frame_start = (int64_t)global_start;
    int64_t frame_end = frame_start + DSP_FRAME_SIZE;
    if (!state->marks_ready || state->mark_period != source_period) {
        dsp_rebuild_marks(state, frame, global_start, source_period);
        return;
    }

    int kept = 0;
    for (int i = 0; i < state->mark_count; ++i)
        if (state->marks[i] >= frame_start) state->marks[kept++] = state->marks[i];
    state->mark_count = kept;
    if (state->mark_cursor >= state->mark_count)
        state->mark_cursor = state->mark_count > 0 ? state->mark_count - 1 : 0;

    int64_t current = state->mark_count > 0
        ? state->marks[state->mark_count - 1] + source_period : frame_start;
    while (current < frame_end && state->mark_count < DSP_PSOLA_MAX_MARKS) {
        int local_start = (int)(current - frame_start);
        int local_end = current + source_period < frame_end
            ? (int)(current + source_period - frame_start) : DSP_FRAME_SIZE;
        int peak = local_start;
        float peak_abs = fabsf(frame[peak]);
        for (int i = local_start + 1; i < local_end; ++i) {
            float sample_abs = fabsf(frame[i]);
            if (sample_abs > peak_abs) {
                peak = i;
                peak_abs = sample_abs;
            }
        }
        state->marks[state->mark_count++] = frame_start + peak;
        current = frame_start + peak + source_period;
    }
}

void dsp_psola_init(DspPsolaState *state, const DspPsolaConfig *config) {
    dsp_init();
    memset(state, 0, sizeof(*state));
    (void)config;
}

void dsp_psola_reset(DspPsolaState *state) {
    if (!state) return;
    memset(state->sums, 0, sizeof(state->sums));
    memset(state->weights, 0, sizeof(state->weights));
    state->mark_count = 0;
    state->mark_cursor = 0;
    state->mark_period = 0;
    state->marks_ready = 0;
    state->next_target = 0;
    state->target_initialized = 0;
    state->grain_window_period = 0;
}

int dsp_psola_process(DspPsolaState *state, const DspPsolaConfig *config,
                      const DspPitch *pitch,
                      const float frame[DSP_FRAME_SIZE],
                      uint64_t global_start,
                      float output[], size_t output_capacity) {
    if (!state || !config || !pitch || !frame || !output) return 0;
    size_t output_count = config->full_frame_output
        ? DSP_FRAME_SIZE : DSP_HOP_SIZE;
    if (output_capacity < output_count) return 0;
    if (!pitch->voiced || pitch->period_samples < 2) {
        memset(output, 0, output_count * sizeof(*output));
        dsp_psola_reset(state);
        return 1;
    }

    int source_period = pitch->period_samples;
    if (source_period < 2) source_period = 2;
    if (source_period > DSP_PSOLA_MAX_PERIOD)
        source_period = DSP_PSOLA_MAX_PERIOD;
    if (state->marks_ready) {
        int delta = source_period - state->mark_period;
        if (delta >= -1 && delta <= 1) source_period = state->mark_period;
    }
    dsp_update_marks(state, frame, global_start, source_period);
    if (state->mark_count <= 0) {
        memset(output, 0, output_count * sizeof(*output));
        return 1;
    }

    int target_period = config->target_period;
    if (target_period < 1)
        target_period = (int)((float)config->sample_rate / config->target_hz);
    if (target_period < 1) target_period = 1;

    int64_t frame_start = (int64_t)global_start;
    int64_t frame_end = frame_start + DSP_FRAME_SIZE;
    if (!state->target_initialized) {
        int64_t target = state->marks[0];
        int64_t minimum = frame_start -
            config->sample_rate / (int)DSP_YIN_MIN_HZ;
        while (target > minimum) target -= target_period;
        while (target < 0) target += target_period;
        state->next_target = target;
        state->target_initialized = 1;
    }

    int radius = source_period;
    if (radius > DSP_PSOLA_MAX_PERIOD) radius = DSP_PSOLA_MAX_PERIOD;
    if (state->grain_window_period != radius) {
        for (int i = 0; i < 2 * radius; ++i)
            state->grain_window[i] =
                dsp_hann_ready((size_t)i, (size_t)(2 * radius));
        state->grain_window_period = radius;
    }

    int64_t schedule_end = config->full_frame_output
        ? frame_end : frame_start + DSP_HOP_SIZE + radius;
    while (state->next_target < schedule_end) {
        int64_t synth = state->next_target;
        int64_t nearest = dsp_nearest_mark(state, synth);
        if (nearest < 0) break;
        int offset_start = -radius;
        int offset_end = radius;
        int64_t bound = frame_start - synth;
        if (bound > offset_start) offset_start = bound;
        bound = frame_start - nearest;
        if (bound > offset_start) offset_start = bound;
        bound = frame_start + DSP_PSOLA_RING_SIZE - synth;
        if (bound < offset_end) offset_end = bound;
        bound = frame_end - nearest;
        if (bound < offset_end) offset_end = bound;
        for (int offset = offset_start; offset < offset_end; ++offset) {
            int64_t destination = synth + offset;
            int64_t source = nearest + offset;
            size_t ring_index = (size_t)destination &
                (DSP_PSOLA_RING_SIZE - 1);
            float window = state->grain_window[offset + radius];
            state->sums[ring_index] +=
                frame[(size_t)(source - frame_start)] * window;
            state->weights[ring_index] += window;
        }
        state->next_target += target_period;
    }

    for (size_t i = 0; i < output_count; ++i) {
        size_t ring_index = (size_t)(frame_start + (int64_t)i) &
            (DSP_PSOLA_RING_SIZE - 1);
        output[i] = state->weights[ring_index] > 1e-4f
            ? state->sums[ring_index] / state->weights[ring_index] : 0.0f;
        if (i < DSP_HOP_SIZE) {
            state->sums[ring_index] = 0.0f;
            state->weights[ring_index] = 0.0f;
        }
    }
    return 1;
}

typedef struct {
    uint16_t bin;
    uint16_t harmonic;
    float magnitude;
    float envelope;
    DspComplex response_step;
} DspLpcHarmonic;

struct DspLpcState {
    int sample_rate;
    float frame[DSP_LPC_FFT_SIZE];
    DspComplex spectrum[DSP_LPC_FFT_SIZE];
    DspLpcHarmonic harmonics[DSP_LPC_HARMONIC_LIMIT];
    float rendered_frame[DSP_LPC_FFT_SIZE];
    double autocorr[DSP_LPC_ORDER_MAX + 1];
    double coefficients[DSP_LPC_ORDER_MAX + 1];
    double levinson_previous[DSP_LPC_ORDER_MAX + 1];
    size_t harmonic_count;
    double phase;
    double phase_step;
    size_t rendered_shift;
    int valid;
    int rendered_valid;
};

static void dsp_levinson(const double *autocorr, int order,
                         double *coefficients, double *gain,
                         double *previous) {
    double r0 = autocorr[0];
    double error = fmax(r0, 1e-10);
    for (int i = 0; i <= order; ++i) coefficients[i] = 0.0;
    coefficients[0] = 1.0;
    for (int i = 1; i <= order; ++i) {
        if (error <= fmax(r0 * 1e-7, 1e-10)) break;
        double value = autocorr[i];
        for (int j = 1; j < i; ++j)
            value += coefficients[j] * autocorr[i - j];
        double reflection = -value / error;
        if (!isfinite(reflection)) reflection = 0.0;
        if (reflection < -0.96) reflection = -0.96;
        if (reflection > 0.96) reflection = 0.96;
        memcpy(previous, coefficients, (size_t)(i + 1) * sizeof(*previous));
        coefficients[i] = reflection;
        for (int j = 1; j < i; ++j)
            coefficients[j] = previous[j] + reflection * previous[i - j];
        error = fmax(error * (1.0 - reflection * reflection), 1e-10);
    }
    *gain = sqrt(error);
}

DspLpcState *dsp_lpc_create(int sample_rate, float target_hz) {
    DspLpcState *state = (DspLpcState *)calloc(1, sizeof(*state));
    if (!state) return NULL;
    dsp_init();
    state->sample_rate = sample_rate;
    state->phase_step = 2.0 * M_PI * target_hz *
        (DSP_LPC_HOP_SIZE / 2.0) / sample_rate;

    double bin_resolution = (double)sample_rate / DSP_LPC_FFT_SIZE;
    for (size_t bin = 0; bin <= DSP_LPC_FFT_SIZE / 2; ++bin) {
        float magnitude = 0.0f;
        for (int harmonic = 1; harmonic < DSP_LPC_HARMONIC_LIMIT; ++harmonic) {
            if (target_hz * harmonic >= sample_rate / 2.0f) break;
            int target_bin = (int)lround(target_hz * harmonic / bin_resolution);
            if ((size_t)target_bin == bin)
                magnitude = 1.0f / sqrtf((float)harmonic);
        }
        if (magnitude != 0.0f &&
            state->harmonic_count < DSP_LPC_HARMONIC_LIMIT) {
            DspLpcHarmonic *entry = &state->harmonics[state->harmonic_count++];
            entry->bin = (uint16_t)bin;
            entry->harmonic = (uint16_t)fmax(1.0,
                round(bin * bin_resolution / target_hz));
            entry->magnitude = magnitude;
            float angle = 2.0f * (float)M_PI * (float)bin /
                          (float)DSP_LPC_FFT_SIZE;
            entry->response_step =
                (DspComplex){cosf(angle), -sinf(angle)};
        }
    }
    return state;
}

void dsp_lpc_free(DspLpcState *state) {
    free(state);
}

void dsp_lpc_reset(DspLpcState *state) {
    if (!state) return;
    state->phase = 0.0;
    state->rendered_shift = 0;
    state->valid = 0;
    state->rendered_valid = 0;
}

void dsp_lpc_set_harmonic_rolloff(DspLpcState *state,
                                  float start_hz, float end_hz) {
    if (!state || !isfinite(start_hz) || !isfinite(end_hz) ||
        end_hz <= start_hz || start_hz < 0.0f)
        return;
    double bin_resolution = (double)state->sample_rate / DSP_LPC_FFT_SIZE;
    size_t kept = 0;
    for (size_t i = 0; i < state->harmonic_count; ++i) {
        DspLpcHarmonic entry = state->harmonics[i];
        float frequency = (float)(entry.bin * bin_resolution);
        float gain = 1.0f;
        if (frequency >= end_hz) {
            gain = 0.0f;
        } else if (frequency > start_hz) {
            float position = (end_hz - frequency) / (end_hz - start_hz);
            gain = position * position * (3.0f - 2.0f * position);
        }
        entry.magnitude *= gain;
        if (entry.magnitude != 0.0f) state->harmonics[kept++] = entry;
    }
    state->harmonic_count = kept;
}

DspLpcRenderStatus dsp_lpc_render(
    DspLpcState *state, const float frame[DSP_FRAME_SIZE],
    float output[DSP_FRAME_SIZE]) {
    if (!state || !frame || !output) return DSP_LPC_RENDER_ERROR;
    int order = state->sample_rate / 2000 + 4;
    if (order < 12) order = 12;
    if (order > DSP_LPC_ORDER_MAX) order = DSP_LPC_ORDER_MAX;

    double energy = 0.0;
    const float *hann = dsp_frame_hann();
    for (size_t i = 0; i < DSP_LPC_FFT_SIZE; ++i) {
        state->frame[i] = frame[i] * hann[i];
        energy += (double)state->frame[i] * state->frame[i];
    }
    state->valid = 0;
    state->rendered_valid = 0;
    if (sqrt(energy / DSP_LPC_FFT_SIZE) < DSP_LPC_SILENCE_RMS) {
        memset(output, 0, DSP_FRAME_SIZE * sizeof(*output));
        return DSP_LPC_RENDER_SILENT;
    }

    memset(state->autocorr, 0, sizeof(state->autocorr));
    state->autocorr[0] = energy;
    for (int lag = 1; lag <= order; ++lag) {
        /* Independent lanes remove the serial dependency in this hot loop. */
        double sum0 = 0.0;
        double sum1 = 0.0;
        double sum2 = 0.0;
        double sum3 = 0.0;
        int i = lag;
        for (; i + 3 < DSP_LPC_FFT_SIZE; i += 4) {
            sum0 += (double)state->frame[i] * state->frame[i - lag];
            sum1 += (double)state->frame[i + 1] *
                    state->frame[i + 1 - lag];
            sum2 += (double)state->frame[i + 2] *
                    state->frame[i + 2 - lag];
            sum3 += (double)state->frame[i + 3] *
                    state->frame[i + 3 - lag];
        }
        for (; i < DSP_LPC_FFT_SIZE; ++i)
            sum0 += (double)state->frame[i] * state->frame[i - lag];
        state->autocorr[lag] = (sum0 + sum1) + (sum2 + sum3);
    }

    double gain;
    dsp_levinson(state->autocorr, order, state->coefficients, &gain,
                 state->levinson_previous);
    for (size_t i = 0; i < state->harmonic_count; ++i) {
        DspLpcHarmonic *entry = &state->harmonics[i];
        DspComplex response = {
            (float)state->coefficients[order], 0.0f};
        for (int coefficient = order - 1; coefficient >= 0; --coefficient) {
            float real = response.r * entry->response_step.r -
                         response.i * entry->response_step.i;
            float imaginary = response.r * entry->response_step.i +
                              response.i * entry->response_step.r;
            response = (DspComplex){
                real + (float)state->coefficients[coefficient], imaginary};
        }
        double magnitude = hypot(response.r, response.i);
        entry->envelope = (float)(gain / (magnitude + 1e-5));
    }
    state->valid = 1;

    memset(state->spectrum, 0, sizeof(state->spectrum));
    for (size_t i = 0; i < state->harmonic_count; ++i) {
        const DspLpcHarmonic *entry = &state->harmonics[i];
        size_t bin = entry->bin;
        float shaped = entry->magnitude * entry->envelope;
        double harmonic = entry->harmonic;
        double angle = state->phase * harmonic;
        state->spectrum[bin] = (DspComplex){
            (float)(shaped * cos(angle)), (float)(shaped * sin(angle))};
        if (bin > 0 && bin + 1 < DSP_LPC_FFT_SIZE / 2 + 1)
            state->spectrum[DSP_LPC_FFT_SIZE - bin] =
                (DspComplex){state->spectrum[bin].r,
                             -state->spectrum[bin].i};
    }

    double shaped_energy = 0.0;
    for (size_t i = 0; i < state->harmonic_count; ++i) {
        size_t bin = state->harmonics[i].bin;
        shaped_energy +=
            (double)state->spectrum[bin].r * state->spectrum[bin].r +
            (double)state->spectrum[bin].i * state->spectrum[bin].i;
    }
    shaped_energy = sqrt(shaped_energy / (DSP_LPC_FFT_SIZE / 2 + 1));
    double source_energy = sqrt(energy);
    if (source_energy > 1e-8 && shaped_energy > 1e-8) {
        float scale = (float)(source_energy / shaped_energy);
        for (size_t i = 0; i < state->harmonic_count; ++i) {
            size_t bin = state->harmonics[i].bin;
            state->spectrum[bin].r *= scale;
            state->spectrum[bin].i *= scale;
            if (bin > 0 && bin + 1 < DSP_LPC_FFT_SIZE / 2 + 1) {
                size_t mirror = DSP_LPC_FFT_SIZE - bin;
                state->spectrum[mirror].r *= scale;
                state->spectrum[mirror].i *= scale;
            }
        }
    }

    dsp_fft_ready(state->spectrum, DSP_LPC_FFT_SIZE, 1);
    for (size_t i = 0; i < DSP_LPC_FFT_SIZE; ++i) {
        float sample = state->spectrum[i].r;
        output[i] = sample;
        state->rendered_frame[i] = sample;
    }
    state->rendered_valid = 1;
    state->rendered_shift = 0;
    state->phase = fmod(state->phase + state->phase_step, 2.0 * M_PI);
    return DSP_LPC_RENDER_VALID;
}

int dsp_lpc_render_cached(DspLpcState *state, float output[DSP_FRAME_SIZE]) {
    if (!state || !output) return 0;
    if (!state->valid || !state->rendered_valid) {
        memset(output, 0, DSP_FRAME_SIZE * sizeof(*output));
        return 1;
    }
    state->rendered_shift =
        (state->rendered_shift + DSP_LPC_HOP_SIZE / 2) &
        (DSP_LPC_FFT_SIZE - 1);
    size_t first_count = DSP_LPC_FFT_SIZE - state->rendered_shift;
    memcpy(output, state->rendered_frame + state->rendered_shift,
           first_count * sizeof(*output));
    memcpy(output + first_count, state->rendered_frame,
           state->rendered_shift * sizeof(*output));
    state->phase = fmod(state->phase + state->phase_step, 2.0 * M_PI);
    return 1;
}
