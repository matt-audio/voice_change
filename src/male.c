#include "male.h"

MaleState *male_init(int sample_rate) {
    static const PitchedVoiceConfig config = {
        .pitch_ratio = 0.707106781f,
        .rate = PITCHED_VOICE_RATE_WIDEBAND,
        .highpass_hz = 55.0f,
        .peaks = {
            {130.0f, 0.9f, 5.5f},
            {300.0f, 1.0f, 2.0f},
        },
        .peak_count = 2,
        .compressor_threshold = 0.15f,
        .compressor_ratio = 5.0f,
        .attack_ms = 10.0f,
        .release_ms = 120.0f,
        .makeup_gain = 1.5f,
        .softclip_drive = 1.3f,
        .softclip_scale = 0.75f,
        .output_limit = 0.92f,
    };
    return pitched_voice_init(sample_rate, &config);
}

void male_free(MaleState *state) {
    pitched_voice_free(state);
}

int male_process(MaleState *state,
                 const float input[DSP_HOP_SIZE],
                 float output[DSP_HOP_SIZE]) {
    return pitched_voice_process(state, input, output);
}

void male_get_profile(const MaleState *state, MaleProfile *profile) {
    pitched_voice_get_profile(state, profile);
}
