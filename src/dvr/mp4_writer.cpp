#include <cstdlib>

#include "spdlog/spdlog.h"

#include "mp4_writer.h"
#include "../minimp4.h"

static int mp4_write_callback(int64_t offset, const void *buffer, size_t size, void *token) {
    return ((Mp4Writer *)token)->write_at(offset, buffer, size);
}

int Mp4Writer::write_at(int64_t offset, const void *buf, size_t n) {
    fseek(file, offset, SEEK_SET);
    if (fwrite(buf, 1, n, file) != n) {
        return 1; // short write (e.g. disk full)
    }
    uint64_t end = (uint64_t)offset + n;
    if (end > file_size_bytes) {
        file_size_bytes = end;
    }
    return 0;
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
    file_size_bytes = 0;
    mux = MP4E_open(0 /*sequential_mode*/, frag_mode, this, mp4_write_callback);
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
        if (write_fail_count % WRITE_FAIL_WARN_INTERVAL == 0) {
            spdlog::warn("[ DVR Mp4Writer ] mp4_h26x_write_nal failed {} ({} times)",
                         (int)res, write_fail_count + 1);
        }
        write_fail_count++;
        return false;
    }
    write_fail_count = 0;
    return true;
}

bool Mp4Writer::close() {
    int mux_status = MP4E_STATUS_OK;
    if (mux) {
        mux_status = MP4E_close(mux); // writes the moov/index in non-sequential mode
        mux = nullptr;
    }
    mp4_h26x_write_close(writer);
    int fclose_ret = 0;
    if (file) {
        fclose_ret = fclose(file);
        file = nullptr;
    }
    return mux_status == MP4E_STATUS_OK && fclose_ret == 0;
}
