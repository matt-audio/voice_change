#include "pitched_voice.h"

#include <math.h>
#include <stddef.h>

static int test_rate(PitchedVoiceRate rate) {
    const PitchedVoiceConfig wet_config = {
        .pitch_ratio = 1.25f,
        .rate = rate,
        .chorus_hz = 3.2f,
        .chorus_base_delay_ms = 4.0f,
        .chorus_depth_ms = 0.8f,
        .chorus_mix = 0.18f,
        .compressor_threshold = 0.15f,
        .compressor_ratio = 4.0f,
        .attack_ms = 5.0f,
        .release_ms = 80.0f,
        .makeup_gain = 1.0f,
        .softclip_drive = 1.0f,
        .softclip_scale = 1.0f,
        .output_limit = 0.95f,
    };
    PitchedVoiceConfig dry_config = wet_config;
    dry_config.chorus_hz = 0.0f;
    dry_config.chorus_base_delay_ms = 0.0f;
    dry_config.chorus_depth_ms = 0.0f;
    dry_config.chorus_mix = 0.0f;
    PitchedVoiceState *wet = pitched_voice_init(DSP_SAMPLE_RATE, &wet_config);
    PitchedVoiceState *dry = pitched_voice_init(DSP_SAMPLE_RATE, &dry_config);
    if (!wet || !dry) {
        pitched_voice_free(wet);
        pitched_voice_free(dry);
        return 1;
    }

    float input[DSP_HOP_SIZE];
    float wet_output[DSP_HOP_SIZE];
    float dry_output[DSP_HOP_SIZE];
    size_t output_hops = 0;
    size_t sample_index = 0;
    double difference_energy = 0.0;
    for (size_t hop = 0; hop < 128; ++hop) {
        for (size_t i = 0; i < DSP_HOP_SIZE; ++i, ++sample_index)
            input[i] = 0.1f * sinf(6.2831853071795864769f * 140.0f *
                                   (float)sample_index / DSP_SAMPLE_RATE);
        int wet_status = pitched_voice_process(wet, input, wet_output);
        int dry_status = pitched_voice_process(dry, input, dry_output);
        if (wet_status != dry_status ||
            wet_status == PITCHED_VOICE_PROCESS_ERROR) {
            pitched_voice_free(wet);
            pitched_voice_free(dry);
            return 1;
        }
        if (wet_status == PITCHED_VOICE_PROCESS_OUTPUT) {
            ++output_hops;
            for (size_t i = 0; i < DSP_HOP_SIZE; ++i) {
                if (!isfinite(wet_output[i]) || !isfinite(dry_output[i])) {
                    pitched_voice_free(wet);
                    pitched_voice_free(dry);
                    return 1;
                }
                float difference = wet_output[i] - dry_output[i];
                difference_energy += (double)difference * difference;
            }
        }
    }
    pitched_voice_free(wet);
    pitched_voice_free(dry);
    return output_hops > 0 && difference_energy > 1e-6 ? 0 : 1;
}

int main(void) {
    return test_rate(PITCHED_VOICE_RATE_FULL) ||
        test_rate(PITCHED_VOICE_RATE_LOW_CPU);
}
