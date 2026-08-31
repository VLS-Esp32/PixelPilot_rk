#ifndef DVR_MP4_WRITER_H
#define DVR_MP4_WRITER_H

#include <cstdint>
#include <cstdio>
#include <string>
#include <atomic>
#include <chrono>
#include <queue>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>

struct MP4E_mux_tag;
struct mp4_h26x_writer_tag;

// Muxes H265 NALs to an mp4 file. Per-frame writes are offloaded to an internal writer thread so an
// SD-card write stall buffers encoded NALs (a few KB each) instead of blocking the caller (the DVR
// thread) and dropping live capture frames. open()/begin_video()/close() are synchronous and fast
// (no per-frame SD stalls); close() drains the write queue before finalizing the moov.
class Mp4Writer {
public:
    Mp4Writer();
    ~Mp4Writer();

    bool open(const std::string &path, int frag_mode);
    bool begin_video(int width, int height);            // CODEC H265
    bool write_nal(const uint8_t *data, int len, int duration_90k); // enqueues; false if queue is full
    bool close();                                       // drains the queue, then finalizes (moov)
    bool is_open() const { return file != nullptr && !abandoned_; }

    // Highest byte offset written so far = the current on-disk file size (updated by the writer
    // thread via write_at). Used to enforce the FAT32 4GB per-file limit without a per-frame fstat.
    uint64_t size() const { return file_size_bytes.load(std::memory_order_relaxed); }

    // Consecutive failed NAL writes (disk full / I/O error), updated by the writer thread; the DVR
    // thread reads this to fail-stop.
    uint32_t consecutive_write_failures() const { return write_fail_streak.load(std::memory_order_relaxed); }

    // Muxer write sink (invoked via mp4_write_callback on the writer thread). Nonzero on short write.
    int write_at(int64_t offset, const void *buf, size_t n);

private:
    static const uint32_t WRITE_FAIL_WARN_INTERVAL = 300;
    static const uint64_t SYNC_BYTES = 4 * 1024 * 1024;
    static const int64_t  SYNC_INTERVAL_MS = 2000;
    // Cap on buffered NAL bytes. Encoded NALs are small (~KB), so this absorbs a multi-second SD
    // stall; if exceeded, the SD has been dead far too long - drop and count a failure so the DVR
    // fail-stops rather than growing memory without bound.
    static const size_t MAX_QUEUE_BYTES = 32 * 1024 * 1024;
    // Longest close() waits for the writer thread to finish the queue. Only exceeded if the card is
    // wedged (uninterruptible I/O); bounded so shutdown can't hang until systemd SIGKILLs us.
    static constexpr std::chrono::seconds DRAIN_TIMEOUT{5};

    void writer_loop();
    bool drain();   // block until the queue is empty and the writer thread is idle; false on timeout
    bool sync_now();          // fflush + fdatasync; false if either failed
    void sync_if_due();       // sync once SYNC_BYTES or SYNC_INTERVAL_MS has passed (writer thread)
    static void sync_dir_of(const std::string &path); // make a freshly created file's dirent durable

    FILE *file = nullptr;
    MP4E_mux_tag *mux = nullptr;
    mp4_h26x_writer_tag *writer = nullptr;
    // Set when a drain timed out: the writer thread may still be stuck inside a write, so file/mux
    // must not be finalized or reused. The Mp4Writer is dead for the rest of the process; the
    // destructor joins the writer thread and frees once it does return.
    bool abandoned_ = false;
    std::atomic<uint64_t> file_size_bytes{0};
    uint32_t write_fail_count = 0;                  // writer thread only (warn throttle)
    std::atomic<uint32_t> write_fail_streak{0};     // writer thread writes, DVR reads
    uint64_t bytes_since_sync_ = 0;
    int64_t  last_sync_ms_ = 0;

    struct NalJob { std::vector<uint8_t> data; int duration; };
    std::queue<NalJob> q_;
    size_t q_bytes_ = 0;
    bool   processing_ = false;
    bool   quit_ = false;
    std::mutex qm_;
    std::condition_variable qcv_;     // wakes the writer thread (new job / quit)
    std::condition_variable qidle_;   // wakes drain() when the writer goes idle
    std::thread writer_thread_;
};

#endif
