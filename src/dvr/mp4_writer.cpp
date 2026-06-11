#include <cstdlib>

#include "spdlog/spdlog.h"

#include "mp4_writer.h"
#include "../minimp4.h"

static int mp4_write_callback(int64_t offset, const void *buffer, size_t size, void *token) {
    FILE *f = (FILE *)token;
    fseek(f, offset, SEEK_SET);
    return fwrite(buffer, 1, size, f) != size;
}

Mp4Writer::Mp4Writer() {
    writer = (mp4_h26x_writer_tag *)malloc(sizeof(mp4_h26x_writer_t));
}

Mp4Writer::~Mp4Writer() {
    free(writer);
}

bool Mp4Writer::open(const std::string &path, int frag_mode) {
    file = fopen(path.c_str(), "w");
    if (!file) {
        spdlog::error("[ DVR Mp4Writer ] unable to open DVR file {}", path);
        return false;
    }
    mux = MP4E_open(0 /*sequential_mode*/, frag_mode, file, mp4_write_callback);
    return true;
}

bool Mp4Writer::begin_video(int width, int height) {
    if (MP4E_STATUS_OK != mp4_h26x_write_init(writer, mux, width, height, true /* H265 */)) {
        spdlog::error("[ DVR Mp4Writer ] mp4_h26x_write_init failed");
        return false;
    }
    return true;
}

bool Mp4Writer::write_nal(const uint8_t *data, int len, int duration_90k) {
    auto res = mp4_h26x_write_nal(writer, data, len, duration_90k);
    if (res != MP4E_STATUS_OK && res != MP4E_STATUS_BAD_ARGUMENTS) {
        spdlog::warn("[ DVR Mp4Writer ] mp4_h26x_write_nal failed {}", (int)res);
        return false;
    }
    return true;
}

void Mp4Writer::close() {
    if (mux) {
        MP4E_close(mux);
        mux = nullptr;
    }
    mp4_h26x_write_close(writer);
    if (file) {
        fclose(file);
        file = nullptr;
    }
}
