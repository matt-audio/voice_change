#include "donald.h"

DonaldState *donald_init(int sample_rate) {
    static const PitchedVoiceConfig config = {
        .pitch_ratio = 2.0f,
        .rate = PITCHED_VOICE_RATE_LOW_CPU,
        .highpass_hz = 180.0f,
        .peaks = {
            {760.0f, 1.25f, 8.0f},
            {1900.0f, 1.5f, 9.0f},
        },
        .peak_count = 2,
        .tremolo_hz = 9.5f,
        .tremolo_depth = 0.12f,
        .chorus_hz = 3.8f,
        .chorus_base_delay_ms = 4.2f,
        .chorus_depth_ms = 1.0f,
        .chorus_mix = 0.18f,
        .compressor_threshold = 0.105f,
        .compressor_ratio = 6.0f,
        .attack_ms = 2.5f,
        .release_ms = 55.0f,
        .makeup_gain = 1.5f,
        .softclip_drive = 1.5f,
        .softclip_scale = 0.70f,
        .output_limit = 0.92f,
    };
    return pitched_voice_init(sample_rate, &config);
}

void donald_free(DonaldState *state) {
    pitched_voice_free(state);
}

int donald_process(DonaldState *state,
                   const float input[DSP_HOP_SIZE],
                   float output[DSP_HOP_SIZE]) {
    return pitched_voice_process(state, input, output);
}

void donald_get_profile(const DonaldState *state, DonaldProfile *profile) {
    pitched_voice_get_profile(state, profile);
}
