#ifndef DVR_TS_WRITER_H
#define DVR_TS_WRITER_H

#include <cstdint>
#include <cstdio>
#include <string>
#include <atomic>
#include <queue>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>

// Muxes H265 access units into an MPEG-TS file. Per-frame writes are offloaded to an internal
// writer thread so an SD-card write stall buffers encoded access units (a few KB each) instead of
// blocking the caller (the DVR thread) and dropping live capture frames. open()/begin_video()/
// close() are synchronous and fast (no per-frame SD stalls); close() drains the write queue.
//
// TS is append-only: there is no index and no finalization step, so a file cut short by a power
// loss or a wedged card stays playable up to the last flushed byte. That is the reason this
// container replaced MP4 - an MP4 without its moov is unplayable without a repair pass.
class TsWriter {
public:
    TsWriter();
    ~TsWriter();

    bool open(const std::string &path);
    bool begin_video(int width, int height);            // CODEC H265; resets the per-file mux state
    // Enqueues one complete Annex-B access unit (the MPP encoder emits exactly one per packet, with
    // VPS/SPS/PPS prepended on every IDR). False if the queue is full.
    bool write_nal(const uint8_t *data, int len, int duration_90k);
    bool close();                                       // drains the queue, then closes the file
    bool is_open() const { return file != nullptr && !abandoned_; }

    // Bytes written so far = the current on-disk file size. Used to enforce the FAT32 4GB per-file
    // limit without a per-frame fstat.
    uint64_t size() const { return file_size_bytes.load(std::memory_order_relaxed); }

    // Consecutive failed writes (disk full / I/O error), updated by the writer thread; the DVR
    // thread reads this to fail-stop.
    uint32_t consecutive_write_failures() const { return write_fail_streak.load(std::memory_order_relaxed); }

private:
    void writer_loop();
    bool drain();   // block until the queue is empty and the writer thread is idle; false on timeout
    bool sync_now();          // fflush + fdatasync; false if either failed
    void sync_if_due();       // sync once SYNC_BYTES or SYNC_INTERVAL_MS has passed (writer thread)
    static void sync_dir_of(const std::string &path); // make a freshly created file's dirent durable

    // --- muxing, writer thread only ---
    bool mux_access_unit(const uint8_t *data, int len, int duration_90k);
    void emit_psi(std::vector<uint8_t> &out);
    // Wraps one PSI section in a TS packet (pointer_field + section + CRC32, stuffed to 188).
    void emit_section(std::vector<uint8_t> &out, uint16_t pid, uint8_t &cc,
                      const uint8_t *section, size_t section_len);
    void emit_pes(std::vector<uint8_t> &out, const uint8_t *au, size_t au_len,
                  int64_t pts, int64_t pcr, bool keyframe);
    bool write_block(const uint8_t *data, size_t len);   // sequential append

    FILE *file = nullptr;
    bool  video_started_ = false;
    // Set when a drain timed out: the writer thread may still be stuck inside a write, so file must
    // not be reused. The TsWriter is dead for the rest of the process; the destructor joins the
    // writer thread and closes once it does return.
    bool abandoned_ = false;
    std::atomic<uint64_t> file_size_bytes{0};
    uint32_t write_fail_count = 0;                  // writer thread only (warn throttle)
    std::atomic<uint32_t> write_fail_streak{0};     // writer thread writes, DVR reads
    uint64_t bytes_since_sync_ = 0;
    int64_t  last_sync_ms_ = 0;

    // Per-file mux state, reset by begin_video() so every file is independently playable.
    int64_t media_ticks_ = 0;        // 90kHz ticks elapsed; also the next access unit's PCR
    int64_t last_psi_ticks_ = 0;
    bool    psi_pending_ = true;
    uint8_t cc_video_ = 0;
    uint8_t cc_pat_ = 0;
    uint8_t cc_pmt_ = 0;
    std::vector<uint8_t> pkt_buf_;   // reused per AU so each frame costs one fwrite

    struct AuJob { std::vector<uint8_t> data; int duration; };
    std::queue<AuJob> q_;
    size_t q_bytes_ = 0;
    bool   processing_ = false;
    bool   quit_ = false;
    std::mutex qm_;
    std::condition_variable qcv_;     // wakes the writer thread (new job / quit)
    std::condition_variable qidle_;   // wakes drain() when the writer goes idle
    std::thread writer_thread_;
};

#endif
