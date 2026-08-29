#include "female.h"

FemaleState *female_init(int sample_rate) {
    static const PitchedVoiceConfig config = {
        .pitch_ratio = 1.414213562f,
        .rate = PITCHED_VOICE_RATE_LOW_CPU,
        .highpass_hz = 120.0f,
        .peaks = {
            {250.0f, 1.0f, -3.0f},
            {2800.0f, 1.2f, 4.0f},
        },
        .peak_count = 2,
        .compressor_threshold = 0.12f,
        .compressor_ratio = 4.0f,
        .attack_ms = 5.0f,
        .release_ms = 80.0f,
        .makeup_gain = 1.5f,
        .softclip_drive = 1.2f,
        .softclip_scale = 0.80f,
        .output_limit = 0.92f,
    };
    return pitched_voice_init(sample_rate, &config);
}

void female_free(FemaleState *state) {
    pitched_voice_free(state);
}

int female_process(FemaleState *state,
                   const float input[DSP_HOP_SIZE],
                   float output[DSP_HOP_SIZE]) {
    return pitched_voice_process(state, input, output);
}

void female_get_profile(const FemaleState *state, FemaleProfile *profile) {
    pitched_voice_get_profile(state, profile);
}
