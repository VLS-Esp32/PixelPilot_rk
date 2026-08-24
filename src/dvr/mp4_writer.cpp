#include <cstdlib>
#include <pthread.h>

#include "spdlog/spdlog.h"

#include "mp4_writer.h"
#include "../minimp4.h"

static int mp4_write_callback(int64_t offset, const void *buffer, size_t size, void *token) {
    return ((Mp4Writer *)token)->write_at(offset, buffer, size);
}

int Mp4Writer::write_at(int64_t offset, const void *buf, size_t n) {
    if (fseek(file, offset, SEEK_SET) != 0) {
        return 1; // cannot position - writing here would land at the wrong offset and corrupt the file
    }
    if (fwrite(buf, 1, n, file) != n) {
        return 1; // short write (e.g. disk full)
    }
    uint64_t end = (uint64_t)offset + n;
    if (end > file_size_bytes.load(std::memory_order_relaxed)) {
        file_size_bytes.store(end, std::memory_order_relaxed);
    }
    return 0;
}

Mp4Writer::Mp4Writer() {
    writer = (mp4_h26x_writer_tag *)calloc(1, sizeof(mp4_h26x_writer_t));
    if (!writer) {
        spdlog::error("[ DVR Mp4Writer ] out of memory allocating the mp4 writer");
    }
    writer_thread_ = std::thread(&Mp4Writer::writer_loop, this);
}

Mp4Writer::~Mp4Writer() {
    bool abandoned;
    {
        std::lock_guard<std::mutex> lock(qm_);
        quit_ = true;
        abandoned = abandoned_;
    }
    qcv_.notify_all();
    if (abandoned) {
        // The writer thread is stuck in an uninterruptible write and may still reference `writer`.
        // Joining would hang and freeing would be a use-after-free; let the process teardown reap it.
        if (writer_thread_.joinable()) {
            writer_thread_.detach();
        }
        return;
    }
    if (writer_thread_.joinable()) {
        writer_thread_.join();
    }
    free(writer);
}

void Mp4Writer::writer_loop() {
    pthread_setname_np(pthread_self(), "__DVR_MP4");
    while (true) {
        NalJob job;
        bool abandoned;
        {
            std::unique_lock<std::mutex> lock(qm_);
            qcv_.wait(lock, [this] { return !q_.empty() || quit_; });
            if (q_.empty()) {
                if (quit_) {
                    return;
                }
                continue;
            }
            job = std::move(q_.front());
            q_.pop();
            q_bytes_ -= job.data.size();
            abandoned = abandoned_;
            processing_ = true;
        }

        if (writer && mux && !abandoned) {
            auto res = mp4_h26x_write_nal(writer, job.data.data(), (int)job.data.size(), job.duration);
            if (res != MP4E_STATUS_OK && res != MP4E_STATUS_BAD_ARGUMENTS) {
                if (write_fail_count % WRITE_FAIL_WARN_INTERVAL == 0) {
                    spdlog::warn("[ DVR Mp4Writer ] mp4_h26x_write_nal failed {} ({} times)",
                                 (int)res, write_fail_count + 1);
                }
                write_fail_count++;
                write_fail_streak.fetch_add(1, std::memory_order_relaxed);
            } else {
                write_fail_count = 0;
                write_fail_streak.store(0, std::memory_order_relaxed);
            }
        }

        {
            std::lock_guard<std::mutex> lock(qm_);
            processing_ = false;
        }
        qidle_.notify_all();
    }
}

bool Mp4Writer::drain() {
    std::unique_lock<std::mutex> lock(qm_);
    return qidle_.wait_for(lock, DRAIN_TIMEOUT, [this] { return q_.empty() && !processing_; });
}

bool Mp4Writer::open(const std::string &path, int frag_mode) {
    if (abandoned_ || !writer) {
        spdlog::error("[ DVR Mp4Writer ] writer is unusable, not opening {}", path);
        return false;
    }
    file = fopen(path.c_str(), "w");
    if (!file) {
        spdlog::error("[ DVR Mp4Writer ] unable to open DVR file {}", path);
        return false;
    }
    file_size_bytes.store(0, std::memory_order_relaxed);
    write_fail_count = 0;
    write_fail_streak.store(0, std::memory_order_relaxed);
    mux = MP4E_open(0 /*sequential_mode*/, frag_mode, this, mp4_write_callback);
    if (!mux) {
        spdlog::error("[ DVR Mp4Writer ] MP4E_open failed for {} (disk full or read-only?)", path);
        fclose(file);
        file = nullptr;
        return false;
    }
    return true;
}

bool Mp4Writer::begin_video(int width, int height) {
    if (!writer || !mux) {
        spdlog::error("[ DVR Mp4Writer ] begin_video called without an open file");
        return false;
    }
    if (MP4E_STATUS_OK != mp4_h26x_write_init(writer, mux, width, height, true /* H265 */)) {
        spdlog::error("[ DVR Mp4Writer ] mp4_h26x_write_init failed");
        return false;
    }
    if (writer->mux_track_id < 0) {
        spdlog::error("[ DVR Mp4Writer ] MP4E_add_track failed");
        return false;
    }
    return true;
}

bool Mp4Writer::write_nal(const uint8_t *data, int len, int duration_90k) {
    if (len <= 0) {
        return true;
    }
    std::lock_guard<std::mutex> lock(qm_);
    if (abandoned_) {
        write_fail_streak.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    if (q_bytes_ + (size_t)len > MAX_QUEUE_BYTES) {
        // The SD card has been stalled long enough to fill the buffer - treat as a write failure so
        // the DVR fail-stops (dropping the NAL here would corrupt the stream anyway).
        write_fail_streak.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    NalJob job;
    job.data.assign(data, data + len);
    job.duration = duration_90k;
    q_bytes_ += (size_t)len;
    q_.push(std::move(job));
    qcv_.notify_one();
    return true;
}

bool Mp4Writer::close() {
    if (!drain()) {
        spdlog::error("[ DVR Mp4Writer ] write queue did not drain in {}s (storage wedged) — "
                      "abandoning file without finalizing",
                      (long long)DRAIN_TIMEOUT.count());
        std::lock_guard<std::mutex> lock(qm_);
        abandoned_ = true;
        return false;
    }

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
