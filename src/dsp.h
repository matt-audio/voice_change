#ifndef VC_ROBOT_MONSTER_DSP_H
#define VC_ROBOT_MONSTER_DSP_H

#include <stddef.h>
#include <stdint.h>

#define DSP_SAMPLE_RATE 48000
#define DSP_FRAME_SIZE 2048
#define DSP_HOP_SIZE 256

/* The shared pitch tracker is deliberately the XMOS-friendly path. */
#define DSP_YIN_ANALYSIS_RATE 8000
#define DSP_YIN_ANALYSIS_SIZE 171
#define DSP_YIN_FFT_SIZE 512
#define DSP_YIN_UPDATE_HOPS 4
#define DSP_YIN_MIN_HZ 60.0f
#define DSP_YIN_MAX_HZ 400.0f

#define DSP_PSOLA_RING_SIZE 4096
#define DSP_PSOLA_MAX_PERIOD 1024
#define DSP_PSOLA_MAX_MARKS (DSP_FRAME_SIZE / 2 + 1)
#define DSP_LPC_HOP_SIZE 512
#define DSP_LPC_SILENCE_RMS 8e-4f

typedef struct {
    float r;
    float i;
} DspComplex;

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float z1;
    float z2;
} DspBiquad;

typedef struct {
    float f0_hz;
    float confidence;
    int voiced;
    int period_samples;
} DspPitch;

typedef struct {
    int sample_rate;
    int decimation;
    int pitch_sample_rate;
    int max_lag;
    float decimation_reciprocal;
    float difference[DSP_YIN_ANALYSIS_SIZE];
    float cmndf[DSP_YIN_ANALYSIS_SIZE];
    DspComplex spectrum[DSP_YIN_FFT_SIZE];
    float energy_prefix[DSP_YIN_ANALYSIS_SIZE + 1];
    float cached_f0;
    float cached_confidence;
    unsigned hops_since_update;
    int cache_ready;
    int last_was_update;
} DspYinState;

typedef struct {
    int sample_rate;
    float target_hz;
    int target_period;
    int full_frame_output;
} DspPsolaConfig;

typedef struct {
    int mark_count;
    int mark_cursor;
    int mark_period;
    int marks_ready;
    int64_t next_target;
    int target_initialized;
    int grain_window_period;
    float grain_window[DSP_PSOLA_MAX_PERIOD * 2];
    float sums[DSP_PSOLA_RING_SIZE];
    float weights[DSP_PSOLA_RING_SIZE];
    int64_t marks[DSP_PSOLA_MAX_MARKS];
} DspPsolaState;

typedef struct DspLpcState DspLpcState;

typedef enum {
    DSP_LPC_RENDER_ERROR = -1,
    DSP_LPC_RENDER_SILENT = 0,
    DSP_LPC_RENDER_VALID = 1
} DspLpcRenderStatus;

void dsp_init(void);
void dsp_fft(DspComplex *values, size_t size, int inverse);
float dsp_hann(size_t index, size_t size);
float dsp_sine_cycles(float cycles);
/* Fast path after dsp_init(); cycles must stay within one wrap of [0, 1). */
float dsp_sine_cycles_wrapped(float cycles);
const float *dsp_frame_hann(void);

DspBiquad dsp_highpass(float frequency, int sample_rate);
DspBiquad dsp_lowpass(float frequency, int sample_rate);
DspBiquad dsp_peak_eq(float frequency, int sample_rate,
                     float q, float gain_db);
float dsp_biquad_process(DspBiquad *filter, float sample);
float dsp_frame_rms(const float frame[DSP_FRAME_SIZE]);

void dsp_yin_init(DspYinState *state, int sample_rate);
void dsp_yin_reset(DspYinState *state);
int dsp_yin_process(DspYinState *state, const float frame[DSP_FRAME_SIZE],
                    float frame_rms, uint64_t global_start, DspPitch *pitch);

void dsp_psola_init(DspPsolaState *state, const DspPsolaConfig *config);
void dsp_psola_reset(DspPsolaState *state);
int dsp_psola_process(DspPsolaState *state, const DspPsolaConfig *config,
                      const DspPitch *pitch,
                      const float frame[DSP_FRAME_SIZE],
                      uint64_t global_start,
                      float output[], size_t output_capacity);

DspLpcState *dsp_lpc_create(int sample_rate, float target_hz);
void dsp_lpc_free(DspLpcState *state);
void dsp_lpc_reset(DspLpcState *state);
void dsp_lpc_set_harmonic_rolloff(DspLpcState *state,
                                  float start_hz, float end_hz);
DspLpcRenderStatus dsp_lpc_render(
    DspLpcState *state, const float frame[DSP_FRAME_SIZE],
    float output[DSP_FRAME_SIZE]);
int dsp_lpc_render_cached(DspLpcState *state,
                          float output[DSP_FRAME_SIZE]);

#endif
