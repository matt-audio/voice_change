#ifndef VC_ROBOT_MONSTER_WAV_H
#define VC_ROBOT_MONSTER_WAV_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
    FILE *file;
    uint16_t channels;
    uint32_t sample_rate;
    size_t total_frames;
    unsigned char *pcm_buffer;
    size_t pcm_buffer_capacity;
} WavReader;

typedef struct {
    FILE *file;
    uint32_t data_bytes;
    unsigned char *pcm_buffer;
    size_t pcm_buffer_capacity;
} WavWriter;

int wav_reader_open(WavReader *reader, const char *path,
                    size_t max_block_frames);
int wav_read_frames(WavReader *reader, float *buffer, size_t count);
void wav_reader_close(WavReader *reader);

int wav_writer_open(WavWriter *writer, const char *path,
                    int sample_rate, size_t frames,
                    size_t max_block_frames);
int wav_write_frames(WavWriter *writer, const float *samples, size_t count);
void wav_writer_close(WavWriter *writer);

#endif
