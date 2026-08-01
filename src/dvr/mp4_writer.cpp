#include <cstdlib>
#include <pthread.h>

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
    if (end > file_size_bytes.load(std::memory_order_relaxed)) {
        file_size_bytes.store(end, std::memory_order_relaxed);
    }
    return 0;
}

Mp4Writer::Mp4Writer() {
    writer = (mp4_h26x_writer_tag *)malloc(sizeof(mp4_h26x_writer_t));
    writer_thread_ = std::thread(&Mp4Writer::writer_loop, this);
}

Mp4Writer::~Mp4Writer() {
    {
        std::lock_guard<std::mutex> lock(qm_);
        quit_ = true;
    }
    qcv_.notify_all();
    if (writer_thread_.joinable()) {
        writer_thread_.join();
    }
    free(writer);
}

// Consumer: pulls NAL jobs and does the actual mp4 write (which touches the SD card). Runs for the
// Mp4Writer's whole lifetime; between files the queue is simply empty. `writer`/`mux`/`file` are set
// by open()/begin_video() and cleared by close(), and close() drain()s first, so this thread never
// touches them concurrently with those calls.
void Mp4Writer::writer_loop() {
    pthread_setname_np(pthread_self(), "__DVR_MP4");
    while (true) {
        NalJob job;
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
            processing_ = true;
        }

        if (writer && mux) {
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

void Mp4Writer::drain() {
    std::unique_lock<std::mutex> lock(qm_);
    qidle_.wait(lock, [this] { return q_.empty() && !processing_; });
}

bool Mp4Writer::open(const std::string &path, int frag_mode) {
    file = fopen(path.c_str(), "w");
    if (!file) {
        spdlog::error("[ DVR Mp4Writer ] unable to open DVR file {}", path);
        return false;
    }
    file_size_bytes.store(0, std::memory_order_relaxed);
    write_fail_count = 0;
    write_fail_streak.store(0, std::memory_order_relaxed);
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
    if (len <= 0) {
        return true;
    }
    std::lock_guard<std::mutex> lock(qm_);
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
    drain();   // flush every queued NAL to disk and wait for the writer thread to go idle

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
