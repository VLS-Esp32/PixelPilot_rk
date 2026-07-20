#ifndef DVR_MP4_WRITER_H
#define DVR_MP4_WRITER_H

#include <cstdint>
#include <cstdio>
#include <string>

struct MP4E_mux_tag;
struct mp4_h26x_writer_tag;

class Mp4Writer {
public:
    Mp4Writer();
    ~Mp4Writer();

    bool open(const std::string &path, int frag_mode);
    bool begin_video(int width, int height);            // CODEC H265
    bool write_nal(const uint8_t *data, int len, int duration_90k); // true on success
    void close();
    bool is_open() const { return file != nullptr; }

    // Highest byte offset written so far = the current on-disk file size. Used to enforce
    // the FAT32 4GB per-file limit without a per-frame fstat.
    uint64_t size() const { return file_size_bytes; }

    // Muxer write sink (invoked via mp4_write_callback). Returns nonzero on short write.
    int write_at(int64_t offset, const void *buf, size_t n);

private:
    FILE *file = nullptr;
    MP4E_mux_tag *mux = nullptr;
    mp4_h26x_writer_tag *writer = nullptr;
    uint64_t file_size_bytes = 0;
};

#endif
