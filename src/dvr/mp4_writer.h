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

private:
    FILE *file = nullptr;
    MP4E_mux_tag *mux = nullptr;
    mp4_h26x_writer_tag *writer = nullptr;
};

#endif
