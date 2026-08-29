#include "wav.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static int read_u16(FILE *file, uint16_t *value) {
    unsigned char bytes[2];
    if (fread(bytes, 1, 2, file) != 2) return 0;
    *value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
    return 1;
}

static int read_u32(FILE *file, uint32_t *value) {
    unsigned char bytes[4];
    if (fread(bytes, 1, 4, file) != 4) return 0;
    *value = (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
             ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
    return 1;
}

static int write_u16(FILE *file, uint16_t value) {
    unsigned char bytes[2] = {(unsigned char)value,
                              (unsigned char)(value >> 8)};
    return fwrite(bytes, 1, 2, file) == 2;
}

static int write_u32(FILE *file, uint32_t value) {
    unsigned char bytes[4] = {(unsigned char)value,
                              (unsigned char)(value >> 8),
                              (unsigned char)(value >> 16),
                              (unsigned char)(value >> 24)};
    return fwrite(bytes, 1, 4, file) == 4;
}

static int skip_bytes(FILE *file, uint32_t count) {
    return fseek(file, (long)count, SEEK_CUR) == 0;
}

int wav_reader_open(WavReader *reader, const char *path,
                    size_t max_block_frames) {
    char riff[4], wave[4];
    uint16_t format = 0, bits = 0;
    uint32_t ignored = 0, data_size = 0;
    long data_offset = 0;
    int have_fmt = 0, have_data = 0;

    memset(reader, 0, sizeof(*reader));
    reader->file = fopen(path, "rb");
    if (!reader->file) {
        fprintf(stderr, "Cannot open input '%s': %s\n", path, strerror(errno));
        return 0;
    }
    if (fread(riff, 1, 4, reader->file) != 4 ||
        !read_u32(reader->file, &ignored) ||
        fread(wave, 1, 4, reader->file) != 4 ||
        memcmp(riff, "RIFF", 4) != 0 || memcmp(wave, "WAVE", 4) != 0) {
        wav_reader_close(reader);
        return 0;
    }

    while (!feof(reader->file)) {
        char id[4];
        uint32_t size;
        if (fread(id, 1, 4, reader->file) != 4 ||
            !read_u32(reader->file, &size)) break;
        if (memcmp(id, "fmt ", 4) == 0) {
            uint32_t byte_rate;
            uint16_t block_align;
            if (size < 16 || !read_u16(reader->file, &format) ||
                !read_u16(reader->file, &reader->channels) ||
                !read_u32(reader->file, &reader->sample_rate) ||
                !read_u32(reader->file, &byte_rate) ||
                !read_u16(reader->file, &block_align) ||
                !read_u16(reader->file, &bits)) {
                wav_reader_close(reader);
                return 0;
            }
            if (size > 16 && !skip_bytes(reader->file, size - 16)) {
                wav_reader_close(reader);
                return 0;
            }
            have_fmt = 1;
        } else if (memcmp(id, "data", 4) == 0) {
            data_offset = ftell(reader->file);
            data_size = size;
            if (!skip_bytes(reader->file, size)) {
                wav_reader_close(reader);
                return 0;
            }
            have_data = 1;
        } else if (!skip_bytes(reader->file, size)) {
            wav_reader_close(reader);
            return 0;
        }
        if ((size & 1) && !skip_bytes(reader->file, 1)) {
            wav_reader_close(reader);
            return 0;
        }
    }

    if (!have_fmt || !have_data || format != 1 || bits != 16 ||
        reader->channels == 0 || reader->sample_rate == 0) {
        wav_reader_close(reader);
        return 0;
    }
    size_t frame_bytes = (size_t)reader->channels * 2;
    reader->total_frames = data_size / frame_bytes;
    if (max_block_frames > SIZE_MAX / frame_bytes) {
        wav_reader_close(reader);
        return 0;
    }
    reader->pcm_buffer_capacity = max_block_frames * frame_bytes;
    reader->pcm_buffer = (unsigned char *)malloc(reader->pcm_buffer_capacity);
    if (reader->pcm_buffer_capacity > 0 && !reader->pcm_buffer) {
        wav_reader_close(reader);
        return 0;
    }
    if (fseek(reader->file, data_offset, SEEK_SET) != 0) {
        wav_reader_close(reader);
        return 0;
    }
    return 1;
}

int wav_read_frames(WavReader *reader, float *buffer, size_t count) {
    size_t bytes = count * (size_t)reader->channels * sizeof(uint16_t);
    if (bytes > reader->pcm_buffer_capacity ||
        fread(reader->pcm_buffer, 1, bytes, reader->file) != bytes) return 0;

    const unsigned char *raw = reader->pcm_buffer;
    for (size_t i = 0; i < count; ++i) {
        float sum = 0.0f;
        for (uint16_t channel = 0; channel < reader->channels; ++channel) {
            uint16_t value = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
            raw += 2;
            sum += (float)(int16_t)value / 32768.0f;
        }
        buffer[i] = sum / reader->channels;
    }
    return 1;
}

void wav_reader_close(WavReader *reader) {
    if (reader->file) fclose(reader->file);
    free(reader->pcm_buffer);
    memset(reader, 0, sizeof(*reader));
}

int wav_writer_open(WavWriter *writer, const char *path,
                    int sample_rate, size_t frames,
                    size_t max_block_frames) {
    memset(writer, 0, sizeof(*writer));
    if (frames > (UINT32_MAX - 36) / 2) return 0;
    writer->file = fopen(path, "wb");
    if (!writer->file) return 0;
    writer->data_bytes = (uint32_t)(frames * 2);
    int ok = fwrite("RIFF", 1, 4, writer->file) == 4 &&
        write_u32(writer->file, 36 + writer->data_bytes) &&
        fwrite("WAVEfmt ", 1, 8, writer->file) == 8 &&
        write_u32(writer->file, 16) && write_u16(writer->file, 1) &&
        write_u16(writer->file, 1) &&
        write_u32(writer->file, (uint32_t)sample_rate) &&
        write_u32(writer->file, (uint32_t)sample_rate * 2) &&
        write_u16(writer->file, 2) && write_u16(writer->file, 16) &&
        fwrite("data", 1, 4, writer->file) == 4 &&
        write_u32(writer->file, writer->data_bytes);
    if (!ok) {
        wav_writer_close(writer);
        return 0;
    }
    if (max_block_frames > SIZE_MAX / sizeof(uint16_t)) {
        wav_writer_close(writer);
        return 0;
    }
    writer->pcm_buffer_capacity = max_block_frames * sizeof(uint16_t);
    writer->pcm_buffer = (unsigned char *)malloc(writer->pcm_buffer_capacity);
    if (writer->pcm_buffer_capacity > 0 && !writer->pcm_buffer) {
        wav_writer_close(writer);
        return 0;
    }
    return 1;
}

int wav_write_frames(WavWriter *writer, const float *samples, size_t count) {
    size_t bytes = count * sizeof(uint16_t);
    if (bytes > writer->pcm_buffer_capacity) return 0;
    unsigned char *raw = writer->pcm_buffer;
    for (size_t i = 0; i < count; ++i) {
        float value = isfinite(samples[i]) ? samples[i] : 0.0f;
        if (value > 1.0f) value = 1.0f;
        if (value < -1.0f) value = -1.0f;
        uint16_t pcm = (uint16_t)(int16_t)lrintf(value * 32767.0f);
        raw[0] = (unsigned char)pcm;
        raw[1] = (unsigned char)(pcm >> 8);
        raw += 2;
    }
    return fwrite(writer->pcm_buffer, 1, bytes, writer->file) == bytes;
}

void wav_writer_close(WavWriter *writer) {
    if (writer->file) fclose(writer->file);
    free(writer->pcm_buffer);
    memset(writer, 0, sizeof(*writer));
}
