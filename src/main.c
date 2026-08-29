#include "donald.h"
#include "female.h"
#include "male.h"
#include "monster.h"
#include "robot.h"
#include "wav.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#ifdef VC_UNIFIED_PROFILE
#include <time.h>
#endif

static int paths_refer_to_same_file(const char *first, const char *second) {
    struct stat first_stat;
    struct stat second_stat;
    if (stat(first, &first_stat) != 0 || stat(second, &second_stat) != 0)
        return strcmp(first, second) == 0;
    return first_stat.st_dev == second_stat.st_dev &&
           first_stat.st_ino == second_stat.st_ino;
}

#ifdef VC_UNIFIED_PROFILE
static double main_profile_now(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)now.tv_sec + (double)now.tv_nsec / 1000000000.0;
}
static double main_profile_cpu_now(void) {
    struct timespec now;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &now);
    return (double)now.tv_sec + (double)now.tv_nsec / 1000000000.0;
}
#define MAIN_PROFILE_START(name) double name = main_profile_now()
#define MAIN_PROFILE_MARK(name) do { (name) = main_profile_now(); } while (0)
#define MAIN_PROFILE_ADD(total, start) \
    do { (total) += main_profile_now() - (start); } while (0)
#define MAIN_DSP_PROFILE_START() \
    double dsp_cpu_start = main_profile_cpu_now(); \
    double dsp_wall_start = main_profile_now()
#define MAIN_DSP_PROFILE_DONE(total, max_cpu, max_wall) do { \
    double cpu_elapsed = main_profile_cpu_now() - dsp_cpu_start; \
    double wall_elapsed = main_profile_now() - dsp_wall_start; \
    (total) += cpu_elapsed; \
    if (cpu_elapsed > (max_cpu)) (max_cpu) = cpu_elapsed; \
    if (wall_elapsed > (max_wall)) (max_wall) = wall_elapsed; \
} while (0)
#else
#define MAIN_PROFILE_START(name) ((void)0)
#define MAIN_PROFILE_MARK(name) ((void)0)
#define MAIN_PROFILE_ADD(total, start) ((void)0)
#define MAIN_DSP_PROFILE_START() ((void)0)
#define MAIN_DSP_PROFILE_DONE(total, max_cpu, max_wall) ((void)0)
#endif

#ifdef VC_UNIFIED_PROFILE
static void print_profile_stage(const char *name, double seconds,
                                double total) {
    double percentage = total > 0.0 ? seconds * 100.0 / total : 0.0;
    printf("  %-24s %9.6f s (%5.1f%%)\n", name, seconds, percentage);
}
#endif

static int run_robot(const char *input_path, const char *output_path) {
    MAIN_PROFILE_START(program_start);
    WavReader reader;
    MAIN_PROFILE_START(setup_start);
    if (!wav_reader_open(&reader, input_path, DSP_HOP_SIZE)) return 1;
    RobotState *state = robot_init((int)reader.sample_rate);
    if (!state) {
        wav_reader_close(&reader);
        return 1;
    }
    WavWriter writer;
    if (!wav_writer_open(&writer, output_path, (int)reader.sample_rate,
                         reader.total_frames, DSP_HOP_SIZE)) {
        robot_free(state);
        wav_reader_close(&reader);
        return 1;
    }
#ifdef VC_UNIFIED_PROFILE
    double setup_seconds = 0.0;
    double wav_read_seconds = 0.0;
    double wav_write_seconds = 0.0;
    double dsp_cpu_seconds = 0.0;
    double max_dsp_cpu_seconds = 0.0;
    double max_dsp_wall_seconds = 0.0;
#endif
    MAIN_PROFILE_ADD(setup_seconds, setup_start);

    float frame_in[DSP_HOP_SIZE] = {0.0f};
    float frame_out[DSP_HOP_SIZE] = {0.0f};
    size_t input_pos = 0;
    size_t output_pos = 0;
    MAIN_PROFILE_START(io_start);
    while (input_pos < reader.total_frames) {
        size_t count = reader.total_frames - input_pos < DSP_HOP_SIZE
            ? reader.total_frames - input_pos : DSP_HOP_SIZE;
        memset(frame_in, 0, sizeof(frame_in));
        MAIN_PROFILE_MARK(io_start);
        if (!wav_read_frames(&reader, frame_in, count)) {
            robot_free(state);
            wav_reader_close(&reader);
            wav_writer_close(&writer);
            return 1;
        }
        MAIN_PROFILE_ADD(wav_read_seconds, io_start);
        input_pos += count;
        MAIN_DSP_PROFILE_START();
        int status = robot_process(state, frame_in, frame_out);
        MAIN_DSP_PROFILE_DONE(dsp_cpu_seconds, max_dsp_cpu_seconds,
                              max_dsp_wall_seconds);
        if (status == ROBOT_PROCESS_ERROR) {
            robot_free(state);
            wav_reader_close(&reader);
            wav_writer_close(&writer);
            return 1;
        }
        if (status == ROBOT_PROCESS_OUTPUT) {
            size_t write_count = reader.total_frames - output_pos < DSP_HOP_SIZE
                ? reader.total_frames - output_pos : DSP_HOP_SIZE;
            MAIN_PROFILE_MARK(io_start);
            if (!wav_write_frames(&writer, frame_out, write_count)) {
                robot_free(state);
                wav_reader_close(&reader);
                wav_writer_close(&writer);
                return 1;
            }
            MAIN_PROFILE_ADD(wav_write_seconds, io_start);
            output_pos += write_count;
        }
    }

    memset(frame_in, 0, sizeof(frame_in));
    while (output_pos < reader.total_frames) {
        MAIN_DSP_PROFILE_START();
        int status = robot_process(state, frame_in, frame_out);
        MAIN_DSP_PROFILE_DONE(dsp_cpu_seconds, max_dsp_cpu_seconds,
                              max_dsp_wall_seconds);
        if (status == ROBOT_PROCESS_ERROR) {
            robot_free(state);
            wav_reader_close(&reader);
            wav_writer_close(&writer);
            return 1;
        }
        if (status == ROBOT_PROCESS_NOT_READY) continue;
        size_t write_count = reader.total_frames - output_pos < DSP_HOP_SIZE
            ? reader.total_frames - output_pos : DSP_HOP_SIZE;
        MAIN_PROFILE_MARK(io_start);
        if (!wav_write_frames(&writer, frame_out, write_count)) {
            robot_free(state);
            wav_reader_close(&reader);
            wav_writer_close(&writer);
            return 1;
        }
        MAIN_PROFILE_ADD(wav_write_seconds, io_start);
        output_pos += write_count;
    }
    MAIN_PROFILE_MARK(io_start);
    int ok = fflush(writer.file) == 0;
    size_t expected_frames = reader.total_frames;
    MAIN_PROFILE_ADD(wav_write_seconds, io_start);

    printf("Mode: robot\nProcessed %zu samples (%.3f seconds)\n",
           output_pos, (double)output_pos / reader.sample_rate);
    printf("Saved: %s\n", output_path);
#ifdef VC_UNIFIED_PROFILE
    double total_seconds = 0.0;
    MAIN_PROFILE_ADD(total_seconds, program_start);
    RobotProfile profile;
    robot_get_profile(state, &profile);
    double dsp_seconds = dsp_cpu_seconds;
    double accounted = setup_seconds + wav_read_seconds + wav_write_seconds;
    double other = total_seconds - accounted;
    if (other < 0.0) other = 0.0;
    printf("Profile (CPU stages, %zu hops, %zu pitch updates):\n",
           profile.hops, profile.pitch_updates);
    printf("  %-24s %9.6f s\n", "DSP CPU total", dsp_seconds);
    print_profile_stage("setup", setup_seconds, total_seconds);
    print_profile_stage("WAV read", wav_read_seconds, total_seconds);
#ifdef VC_ROBOT_PROFILE
    print_profile_stage("YIN", profile.yin_seconds, dsp_seconds);
    print_profile_stage("PSOLA", profile.psola_seconds, dsp_seconds);
    print_profile_stage("ring modulation", profile.ring_seconds, dsp_seconds);
    print_profile_stage("flanger", profile.flanger_seconds, dsp_seconds);
    print_profile_stage("EQ", profile.eq_seconds, dsp_seconds);
    print_profile_stage("dynamics", profile.dynamics_seconds, dsp_seconds);
    print_profile_stage("window slide", profile.window_slide_seconds, dsp_seconds);
#endif
    printf("  %-24s %9.3f us\n", "max DSP CPU hop",
           max_dsp_cpu_seconds * 1000000.0);
    printf("  %-24s %9.3f us\n", "max DSP wall hop",
           max_dsp_wall_seconds * 1000000.0);
    print_profile_stage("WAV write + flush", wav_write_seconds, total_seconds);
    print_profile_stage("processing + other wall", other, total_seconds);
    printf("  %-24s %9.6f s\n", "total", total_seconds);
#endif
    robot_free(state);
    wav_reader_close(&reader);
    wav_writer_close(&writer);
    return ok && output_pos == expected_frames ? 0 : 1;
}

static int run_monster(const char *input_path, const char *output_path,
                       float target_hz) {
    MAIN_PROFILE_START(program_start);
    WavReader reader;
    MAIN_PROFILE_START(setup_start);
    if (!wav_reader_open(&reader, input_path, DSP_HOP_SIZE)) return 1;
    MonsterState *state = monster_init((int)reader.sample_rate, target_hz);
    if (!state) {
        wav_reader_close(&reader);
        return 1;
    }
    WavWriter writer;
    if (!wav_writer_open(&writer, output_path, (int)reader.sample_rate,
                         reader.total_frames, DSP_HOP_SIZE)) {
        monster_free(state);
        wav_reader_close(&reader);
        return 1;
    }
#ifdef VC_UNIFIED_PROFILE
    double setup_seconds = 0.0;
    double wav_read_seconds = 0.0;
    double wav_write_seconds = 0.0;
    double dsp_cpu_seconds = 0.0;
    double max_dsp_cpu_seconds = 0.0;
    double max_dsp_wall_seconds = 0.0;
#endif
    MAIN_PROFILE_ADD(setup_seconds, setup_start);

    float frame_in[DSP_HOP_SIZE] = {0.0f};
    float frame_out[DSP_HOP_SIZE] = {0.0f};
    size_t input_pos = 0;
    size_t output_pos = 0;
    MAIN_PROFILE_START(io_start);
    while (input_pos < reader.total_frames) {
        size_t count = reader.total_frames - input_pos < DSP_HOP_SIZE
            ? reader.total_frames - input_pos : DSP_HOP_SIZE;
        memset(frame_in, 0, sizeof(frame_in));
        MAIN_PROFILE_MARK(io_start);
        if (!wav_read_frames(&reader, frame_in, count)) {
            monster_free(state);
            wav_reader_close(&reader);
            wav_writer_close(&writer);
            return 1;
        }
        MAIN_PROFILE_ADD(wav_read_seconds, io_start);
        input_pos += count;
        MAIN_DSP_PROFILE_START();
        int status = monster_process(state, frame_in, frame_out);
        MAIN_DSP_PROFILE_DONE(dsp_cpu_seconds, max_dsp_cpu_seconds,
                              max_dsp_wall_seconds);
        if (status == MONSTER_PROCESS_ERROR) {
            monster_free(state);
            wav_reader_close(&reader);
            wav_writer_close(&writer);
            return 1;
        }
        if (status == MONSTER_PROCESS_OUTPUT) {
            size_t write_count = reader.total_frames - output_pos < DSP_HOP_SIZE
                ? reader.total_frames - output_pos : DSP_HOP_SIZE;
            MAIN_PROFILE_MARK(io_start);
            if (!wav_write_frames(&writer, frame_out, write_count)) {
                monster_free(state);
                wav_reader_close(&reader);
                wav_writer_close(&writer);
                return 1;
            }
            MAIN_PROFILE_ADD(wav_write_seconds, io_start);
            output_pos += write_count;
        }
    }

    memset(frame_in, 0, sizeof(frame_in));
    while (output_pos < reader.total_frames) {
        MAIN_DSP_PROFILE_START();
        int status = monster_process(state, frame_in, frame_out);
        MAIN_DSP_PROFILE_DONE(dsp_cpu_seconds, max_dsp_cpu_seconds,
                              max_dsp_wall_seconds);
        if (status == MONSTER_PROCESS_ERROR) {
            monster_free(state);
            wav_reader_close(&reader);
            wav_writer_close(&writer);
            return 1;
        }
        if (status == MONSTER_PROCESS_NOT_READY) continue;
        size_t write_count = reader.total_frames - output_pos < DSP_HOP_SIZE
            ? reader.total_frames - output_pos : DSP_HOP_SIZE;
        MAIN_PROFILE_MARK(io_start);
        if (!wav_write_frames(&writer, frame_out, write_count)) {
            monster_free(state);
            wav_reader_close(&reader);
            wav_writer_close(&writer);
            return 1;
        }
        MAIN_PROFILE_ADD(wav_write_seconds, io_start);
        output_pos += write_count;
    }
    MAIN_PROFILE_MARK(io_start);
    int ok = fflush(writer.file) == 0;
    size_t expected_frames = reader.total_frames;
    MAIN_PROFILE_ADD(wav_write_seconds, io_start);

    printf("Mode: monster (target %.2f Hz)\n", target_hz);
    printf("Processed %zu samples (%.3f seconds)\n",
           output_pos, (double)output_pos / reader.sample_rate);
    printf("Saved: %s\n", output_path);
#ifdef VC_UNIFIED_PROFILE
    double total_seconds = 0.0;
    MAIN_PROFILE_ADD(total_seconds, program_start);
    MonsterProfile profile;
    monster_get_profile(state, &profile);
    double dsp_seconds = dsp_cpu_seconds;
    double accounted = setup_seconds + wav_read_seconds + wav_write_seconds;
    double other = total_seconds - accounted;
    if (other < 0.0) other = 0.0;
    printf("Profile (CPU stages, %zu hops, %zu pitch updates, "
           "%zu LPC refreshes, %zu rendered frames):\n",
           profile.hops, profile.pitch_updates, profile.lpc_refreshes,
           profile.rendered_frames);
#ifdef VC_MONSTER_PROFILE
    printf("  LPC primary reasons      initial=%zu periodic=%zu onset=%zu "
           "silence=%zu rms-up=%zu rms-down=%zu\n",
           profile.lpc_refresh_initial, profile.lpc_refresh_periodic,
           profile.lpc_refresh_onset, profile.lpc_refresh_silence,
           profile.lpc_refresh_rms_up, profile.lpc_refresh_rms_down);
#endif
    printf("  %-24s %9.6f s\n", "DSP CPU total", dsp_seconds);
    print_profile_stage("setup", setup_seconds, total_seconds);
    print_profile_stage("WAV read", wav_read_seconds, total_seconds);
#ifdef VC_MONSTER_PROFILE
    print_profile_stage("frame RMS", profile.rms_seconds, dsp_seconds);
    print_profile_stage("YIN", profile.yin_seconds, dsp_seconds);
    print_profile_stage("LPC", profile.lpc_seconds, dsp_seconds);
    print_profile_stage("PSOLA", profile.psola_seconds, dsp_seconds);
    print_profile_stage("body + gate + OLA", profile.body_seconds, dsp_seconds);
    print_profile_stage("OLA cleanup", profile.ola_seconds, dsp_seconds);
    print_profile_stage("output post", profile.output_seconds, dsp_seconds);
    print_profile_stage("window slide", profile.window_slide_seconds, dsp_seconds);
#endif
    printf("  %-24s %9.3f us\n", "max DSP CPU hop",
           max_dsp_cpu_seconds * 1000000.0);
    printf("  %-24s %9.3f us\n", "max DSP wall hop",
           max_dsp_wall_seconds * 1000000.0);
    print_profile_stage("WAV write + flush", wav_write_seconds, total_seconds);
    print_profile_stage("processing + other wall", other, total_seconds);
    printf("  %-24s %9.6f s\n", "total", total_seconds);
#endif
    monster_free(state);
    wav_reader_close(&reader);
    wav_writer_close(&writer);
    return ok && output_pos == expected_frames ? 0 : 1;
}

typedef PitchedVoiceState *(*PitchedModeInit)(int sample_rate);
typedef void (*PitchedModeFree)(PitchedVoiceState *state);
typedef int (*PitchedModeProcess)(
    PitchedVoiceState *state, const float input[DSP_HOP_SIZE],
    float output[DSP_HOP_SIZE]);
typedef void (*PitchedModeGetProfile)(
    const PitchedVoiceState *state, PitchedVoiceProfile *profile);

static int run_pitched_mode(const char *mode_name,
                            const char *input_path,
                            const char *output_path,
                            PitchedModeInit init,
                            PitchedModeFree destroy,
                            PitchedModeProcess process,
                            PitchedModeGetProfile get_profile) {
    MAIN_PROFILE_START(program_start);
    WavReader reader;
    MAIN_PROFILE_START(setup_start);
    if (!wav_reader_open(&reader, input_path, DSP_HOP_SIZE)) return 1;
    PitchedVoiceState *state = init((int)reader.sample_rate);
    if (!state) {
        wav_reader_close(&reader);
        return 1;
    }
    WavWriter writer;
    if (!wav_writer_open(&writer, output_path, (int)reader.sample_rate,
                         reader.total_frames, DSP_HOP_SIZE)) {
        destroy(state);
        wav_reader_close(&reader);
        return 1;
    }
#ifdef VC_UNIFIED_PROFILE
    double setup_seconds = 0.0;
    double wav_read_seconds = 0.0;
    double wav_write_seconds = 0.0;
    double dsp_cpu_seconds = 0.0;
    double max_dsp_cpu_seconds = 0.0;
    double max_dsp_wall_seconds = 0.0;
#endif
    MAIN_PROFILE_ADD(setup_seconds, setup_start);

    float frame_in[DSP_HOP_SIZE] = {0.0f};
    float frame_out[DSP_HOP_SIZE] = {0.0f};
    size_t input_pos = 0;
    size_t output_pos = 0;
    MAIN_PROFILE_START(io_start);
    while (input_pos < reader.total_frames) {
        size_t count = reader.total_frames - input_pos < DSP_HOP_SIZE
            ? reader.total_frames - input_pos : DSP_HOP_SIZE;
        memset(frame_in, 0, sizeof(frame_in));
        MAIN_PROFILE_MARK(io_start);
        if (!wav_read_frames(&reader, frame_in, count)) {
            destroy(state);
            wav_reader_close(&reader);
            wav_writer_close(&writer);
            return 1;
        }
        MAIN_PROFILE_ADD(wav_read_seconds, io_start);
        input_pos += count;
        MAIN_DSP_PROFILE_START();
        int status = process(state, frame_in, frame_out);
        MAIN_DSP_PROFILE_DONE(dsp_cpu_seconds, max_dsp_cpu_seconds,
                              max_dsp_wall_seconds);
        if (status == PITCHED_VOICE_PROCESS_ERROR) {
            destroy(state);
            wav_reader_close(&reader);
            wav_writer_close(&writer);
            return 1;
        }
        if (status == PITCHED_VOICE_PROCESS_OUTPUT) {
            size_t write_count = reader.total_frames - output_pos < DSP_HOP_SIZE
                ? reader.total_frames - output_pos : DSP_HOP_SIZE;
            MAIN_PROFILE_MARK(io_start);
            if (!wav_write_frames(&writer, frame_out, write_count)) {
                destroy(state);
                wav_reader_close(&reader);
                wav_writer_close(&writer);
                return 1;
            }
            MAIN_PROFILE_ADD(wav_write_seconds, io_start);
            output_pos += write_count;
        }
    }

    memset(frame_in, 0, sizeof(frame_in));
    while (output_pos < reader.total_frames) {
        MAIN_DSP_PROFILE_START();
        int status = process(state, frame_in, frame_out);
        MAIN_DSP_PROFILE_DONE(dsp_cpu_seconds, max_dsp_cpu_seconds,
                              max_dsp_wall_seconds);
        if (status == PITCHED_VOICE_PROCESS_ERROR) {
            destroy(state);
            wav_reader_close(&reader);
            wav_writer_close(&writer);
            return 1;
        }
        if (status == PITCHED_VOICE_PROCESS_NOT_READY) continue;
        size_t write_count = reader.total_frames - output_pos < DSP_HOP_SIZE
            ? reader.total_frames - output_pos : DSP_HOP_SIZE;
        MAIN_PROFILE_MARK(io_start);
        if (!wav_write_frames(&writer, frame_out, write_count)) {
            destroy(state);
            wav_reader_close(&reader);
            wav_writer_close(&writer);
            return 1;
        }
        MAIN_PROFILE_ADD(wav_write_seconds, io_start);
        output_pos += write_count;
    }
    MAIN_PROFILE_MARK(io_start);
    int ok = fflush(writer.file) == 0;
    size_t expected_frames = reader.total_frames;
    MAIN_PROFILE_ADD(wav_write_seconds, io_start);

    printf("Mode: %s\n", mode_name);
    printf("Processed %zu samples (%.3f seconds)\n",
           output_pos, (double)output_pos / reader.sample_rate);
    printf("Saved: %s\n", output_path);
#ifdef VC_UNIFIED_PROFILE
    double total_seconds = 0.0;
    MAIN_PROFILE_ADD(total_seconds, program_start);
    PitchedVoiceProfile profile;
    get_profile(state, &profile);
    double dsp_seconds = dsp_cpu_seconds;
    double accounted = setup_seconds + wav_read_seconds + wav_write_seconds;
    double other = total_seconds - accounted;
    if (other < 0.0) other = 0.0;
    printf("Profile (CPU stages, %zu hops, %zu shift blocks):\n",
           profile.hops, profile.shift_blocks);
    printf("  %-24s %9.6f s\n", "DSP CPU total", dsp_seconds);
    print_profile_stage("setup", setup_seconds, total_seconds);
    print_profile_stage("WAV read", wav_read_seconds, total_seconds);
#ifdef VC_PITCHED_VOICE_PROFILE
    print_profile_stage("Rubber Band R2", profile.pitch_shift_seconds,
                        dsp_seconds);
    print_profile_stage("chorus", profile.chorus_seconds, dsp_seconds);
    print_profile_stage("upsample + FIFO push", profile.output_push_seconds,
                        dsp_seconds);
    print_profile_stage("tremolo", profile.tremolo_seconds, dsp_seconds);
    print_profile_stage("EQ", profile.eq_seconds, dsp_seconds);
    print_profile_stage("dynamics", profile.dynamics_seconds, dsp_seconds);
#endif
    printf("  %-24s %9.3f us\n", "max DSP CPU hop",
           max_dsp_cpu_seconds * 1000000.0);
    printf("  %-24s %9.3f us\n", "max DSP wall hop",
           max_dsp_wall_seconds * 1000000.0);
    print_profile_stage("WAV write + flush", wav_write_seconds, total_seconds);
    print_profile_stage("processing + other wall", other, total_seconds);
    printf("  %-24s %9.6f s\n", "total", total_seconds);
#else
    (void)get_profile;
#endif
    destroy(state);
    wav_reader_close(&reader);
    wav_writer_close(&writer);
    return ok && output_pos == expected_frames ? 0 : 1;
}

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr,
                "Usage: %s <robot|monster|male|female|donald> "
                "<input.wav> <output.wav> [target_hz]\n",
                argv[0]);
        return 1;
    }
    if (paths_refer_to_same_file(argv[2], argv[3])) {
        fprintf(stderr, "Input and output must be different files\n");
        return 1;
    }
    if (strcmp(argv[1], "robot") == 0)
        return run_robot(argv[2], argv[3]);
    if (strcmp(argv[1], "monster") == 0) {
        float target_hz = argc > 4 ? strtof(argv[4], NULL) : MONSTER_TARGET_HZ;
        return run_monster(argv[2], argv[3], target_hz);
    }
    if (strcmp(argv[1], "male") == 0)
        return run_pitched_mode("male", argv[2], argv[3],
                                male_init, male_free, male_process,
                                male_get_profile);
    if (strcmp(argv[1], "female") == 0)
        return run_pitched_mode("female", argv[2], argv[3],
                                female_init, female_free, female_process,
                                female_get_profile);
    if (strcmp(argv[1], "donald") == 0)
        return run_pitched_mode("donald", argv[2], argv[3],
                                donald_init, donald_free, donald_process,
                                donald_get_profile);
    fprintf(stderr, "Unknown mode '%s'\n", argv[1]);
    return 1;
}
