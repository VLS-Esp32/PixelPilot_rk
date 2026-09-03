#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>
#include <filesystem>

#include "spdlog/spdlog.h"

#include "ts_writer.h"

static const uint32_t WRITE_FAIL_WARN_INTERVAL = 300;
static const uint64_t SYNC_BYTES = 4 * 1024 * 1024;
static const int64_t  SYNC_INTERVAL_MS = 2000;

// Cap on buffered access-unit bytes. Encoded frames are small (~KB), so this absorbs a multi-second
// SD stall; if exceeded, the SD has been dead far too long - drop and count a failure so the DVR
// fail-stops rather than growing memory without bound.
static const size_t MAX_QUEUE_BYTES = 32 * 1024 * 1024;

// Longest close() waits for the writer thread to finish the queue. Only exceeded if the card is
// wedged (uninterruptible I/O); bounded so shutdown can't hang until supervised teardown kills us.
static const std::chrono::seconds DRAIN_TIMEOUT{5};

// --- MPEG-TS layout ---
static const size_t   TS_PACKET_SIZE = 188;
static const uint16_t PID_PAT   = 0x0000;
static const uint16_t PID_PMT   = 0x1000;
static const uint16_t PID_VIDEO = 0x0100;
static const uint16_t PROGRAM_NUMBER      = 0x0001;
static const uint16_t TRANSPORT_STREAM_ID = 0x0001;
static const uint8_t  STREAM_TYPE_HEVC = 0x24;  // the DVR always re-encodes to H265
static const uint8_t  PES_STREAM_ID    = 0xE0;

// Decoder preroll: PTS runs one second ahead of the PCR, so a player always receives a frame before
// its presentation time. Also keeps the first PTS off zero.
static const int64_t PTS_PREROLL_90K = 90000;

// PAT/PMT cadence. The encoder GOP is 2s, so emitting the tables only on keyframes would be far
// sparser than the ~100ms the spec expects; this interval is what actually carries them.
static const int64_t PSI_INTERVAL_90K = 9000;

// PTS/PCR are 33-bit fields; at 90kHz they wrap after ~26.5 hours.
static const int64_t TS_TIMESTAMP_MASK = (int64_t)0x1FFFFFFFF;

static int64_t monotonic_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

// CRC-32/MPEG-2 (poly 0x04C11DB7, init 0xFFFFFFFF, MSB-first, no final xor) over a PSI section.
// Sections go out ~10 times a second, so the bitwise form costs nothing worth a lookup table.
static uint32_t crc32_mpeg(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint32_t)data[i] << 24;
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80000000u) ? ((crc << 1) ^ 0x04C11DB7u) : (crc << 1);
        }
    }
    return crc;
}

// True if the access unit contains an HEVC IRAP NAL (types 16..21), i.e. it is a random access
// point. Scans Annex-B start codes; a 4-byte start code contains the 3-byte one at offset 1, so
// matching on 00 00 01 covers both.
static bool au_is_keyframe(const uint8_t *d, size_t len) {
    if (len < 4) {
        return false;
    }
    for (size_t i = 0; i + 3 < len; i++) {
        if (d[i] == 0x00 && d[i + 1] == 0x00 && d[i + 2] == 0x01) {
            int nal_type = (d[i + 3] >> 1) & 0x3f;
            if (nal_type >= 16 && nal_type <= 21) {
                return true;
            }
            i += 2;
        }
    }
    return false;
}

// Writes a 5-byte PTS/DTS field: 4-bit prefix, 33-bit value split across three groups, each group
// terminated by a marker bit.
static void write_timestamp(uint8_t *p, uint8_t prefix, int64_t ts) {
    p[0] = (uint8_t)((prefix << 4) | (((ts >> 30) & 0x07) << 1) | 0x01);
    p[1] = (uint8_t)((ts >> 22) & 0xFF);
    p[2] = (uint8_t)((((ts >> 15) & 0x7F) << 1) | 0x01);
    p[3] = (uint8_t)((ts >> 7) & 0xFF);
    p[4] = (uint8_t)(((ts & 0x7F) << 1) | 0x01);
}

// Writes the 6-byte adaptation-field PCR: 33-bit base at 90kHz, 6 reserved bits, 9-bit extension at
// 27MHz. The extension is always 0 - our timing resolution is the 90kHz base.
static void write_pcr(uint8_t *p, int64_t base) {
    p[0] = (uint8_t)((base >> 25) & 0xFF);
    p[1] = (uint8_t)((base >> 17) & 0xFF);
    p[2] = (uint8_t)((base >> 9) & 0xFF);
    p[3] = (uint8_t)((base >> 1) & 0xFF);
    p[4] = (uint8_t)(((base & 0x01) << 7) | 0x7E);
    p[5] = 0x00;
}

TsWriter::TsWriter() {
    writer_thread_ = std::thread(&TsWriter::writer_loop, this);
}

TsWriter::~TsWriter() {
    {
        std::lock_guard<std::mutex> lock(qm_);
        quit_ = true;
    }
    qcv_.notify_all();
    // The writer thread holds a raw pointer to this object, so detaching it would leave
    // it reading freed memory - its own mutex, condvars and queue - the moment a stuck write
    // returned. If the card is wedged the thread is parked in uninterruptible I/O and this blocks
    // until supervised teardown kills the process, which costs nothing extra: everything already
    // written to that recording is on disk and playable. Only reachable at process exit; close()
    // stays bounded, so segment rotation and stop are unaffected.
    if (writer_thread_.joinable()) {
        writer_thread_.join();
    }
}

bool TsWriter::write_block(const uint8_t *data, size_t len) {
    // TS is append-only - no seeking, so a short write can only truncate the tail, never corrupt
    // what is already on disk.
    if (fwrite(data, 1, len, file) != len) {
        return false;   // short write (e.g. disk full)
    }
    file_size_bytes.fetch_add(len, std::memory_order_relaxed);
    return true;
}

void TsWriter::emit_section(std::vector<uint8_t> &out, uint16_t pid, uint8_t &cc,
                            const uint8_t *section, size_t section_len) {
    size_t start = out.size();
    out.resize(start + TS_PACKET_SIZE, 0xFF);
    uint8_t *pkt = out.data() + start;

    pkt[0] = 0x47;
    pkt[1] = (uint8_t)(0x40 | ((pid >> 8) & 0x1F));   // payload_unit_start_indicator
    pkt[2] = (uint8_t)(pid & 0xFF);
    pkt[3] = (uint8_t)(0x10 | (cc & 0x0F));           // payload only
    cc = (uint8_t)((cc + 1) & 0x0F);

    pkt[4] = 0x00;                                    // pointer_field: section starts immediately
    memcpy(pkt + 5, section, section_len);

    uint32_t crc = crc32_mpeg(section, section_len);
    uint8_t *p = pkt + 5 + section_len;
    p[0] = (uint8_t)(crc >> 24);
    p[1] = (uint8_t)(crc >> 16);
    p[2] = (uint8_t)(crc >> 8);
    p[3] = (uint8_t)crc;
    // Everything past the CRC stays 0xFF stuffing from the resize above.
}

void TsWriter::emit_psi(std::vector<uint8_t> &out) {
    // PAT: one program pointing at the PMT PID. section_length counts everything after that field
    // including the CRC: 5 header bytes + 4 program bytes + 4 CRC = 13.
    uint8_t pat[12];
    pat[0]  = 0x00;                                       // table_id
    pat[1]  = 0xB0;                                       // section_syntax_indicator, length hi
    pat[2]  = 0x0D;                                       // section_length lo
    pat[3]  = (uint8_t)(TRANSPORT_STREAM_ID >> 8);
    pat[4]  = (uint8_t)(TRANSPORT_STREAM_ID & 0xFF);
    pat[5]  = 0xC1;                                       // version 0, current_next_indicator
    pat[6]  = 0x00;                                       // section_number
    pat[7]  = 0x00;                                       // last_section_number
    pat[8]  = (uint8_t)(PROGRAM_NUMBER >> 8);
    pat[9]  = (uint8_t)(PROGRAM_NUMBER & 0xFF);
    pat[10] = (uint8_t)(0xE0 | ((PID_PMT >> 8) & 0x1F));
    pat[11] = (uint8_t)(PID_PMT & 0xFF);
    emit_section(out, PID_PAT, cc_pat_, pat, sizeof(pat));

    // PMT: one HEVC elementary stream, carrying the PCR on the video PID. section_length is
    // 9 header bytes + 5 stream bytes + 4 CRC = 18.
    uint8_t pmt[17];
    pmt[0]  = 0x02;                                       // table_id
    pmt[1]  = 0xB0;
    pmt[2]  = 0x12;                                       // section_length
    pmt[3]  = (uint8_t)(PROGRAM_NUMBER >> 8);
    pmt[4]  = (uint8_t)(PROGRAM_NUMBER & 0xFF);
    pmt[5]  = 0xC1;                                       // version 0, current_next_indicator
    pmt[6]  = 0x00;                                       // section_number
    pmt[7]  = 0x00;                                       // last_section_number
    pmt[8]  = (uint8_t)(0xE0 | ((PID_VIDEO >> 8) & 0x1F));// PCR_PID
    pmt[9]  = (uint8_t)(PID_VIDEO & 0xFF);
    pmt[10] = 0xF0;                                       // program_info_length = 0
    pmt[11] = 0x00;
    pmt[12] = STREAM_TYPE_HEVC;                           // stream loop: one elementary stream
    pmt[13] = (uint8_t)(0xE0 | ((PID_VIDEO >> 8) & 0x1F));
    pmt[14] = (uint8_t)(PID_VIDEO & 0xFF);
    pmt[15] = 0xF0;                                       // ES_info_length = 0
    pmt[16] = 0x00;
    emit_section(out, PID_PMT, cc_pmt_, pmt, sizeof(pmt));
}

void TsWriter::emit_pes(std::vector<uint8_t> &out, const uint8_t *au, size_t au_len,
                        int64_t pts, int64_t pcr, bool keyframe) {
    // PES header: unbounded length (the standard choice for video), PTS only. The encoder runs
    // IPPP with no B-frames, so DTS always equals PTS and is correctly omitted.
    uint8_t pes[14];
    pes[0] = 0x00;
    pes[1] = 0x00;
    pes[2] = 0x01;
    pes[3] = PES_STREAM_ID;
    pes[4] = 0x00;                  // PES_packet_length = 0 (unbounded)
    pes[5] = 0x00;
    pes[6] = 0x84;                  // '10' marker, data_alignment_indicator = 1
    pes[7] = 0x80;                  // PTS_DTS_flags = '10'
    pes[8] = 0x05;                  // PES_header_data_length
    write_timestamp(pes + 9, 0x02, pts);

    // The PES header and the access unit form one logical byte stream; walk it without joining
    // them into a temporary buffer.
    size_t pos = 0;
    const size_t total = sizeof(pes) + au_len;
    bool first = true;

    while (pos < total) {
        size_t start = out.size();
        out.resize(start + TS_PACKET_SIZE);
        uint8_t *pkt = out.data() + start;

        pkt[0] = 0x47;
        pkt[1] = (uint8_t)((first ? 0x40 : 0x00) | ((PID_VIDEO >> 8) & 0x1F));
        pkt[2] = (uint8_t)(PID_VIDEO & 0xFF);

        // The first packet of every access unit carries the PCR, and the random access indicator
        // when this AU is a keyframe - that pair is what makes a TS file seekable.
        const bool want_pcr = first;
        const bool want_rai = first && keyframe;
        size_t af_len = 0;   // adaptation field bytes after the length byte
        if (want_pcr || want_rai) {
            af_len = 1 + (want_pcr ? 6 : 0);
        }
        size_t af_total = af_len ? af_len + 1 : 0;
        size_t room = TS_PACKET_SIZE - 4 - af_total;
        size_t remaining = total - pos;

        // A short tail must be padded out to exactly 188 bytes; the padding lives in the
        // adaptation field, which may have to be created or grown to hold it.
        if (remaining < room) {
            size_t pad = room - remaining;
            if (af_total == 0) {
                // A single spare byte is expressible as an adaptation field of length 0.
                af_len   = (pad == 1) ? 0 : pad - 1;
                af_total = pad;
            } else {
                af_len   += pad;
                af_total += pad;
            }
            room = remaining;
        }

        pkt[3] = (uint8_t)((af_total ? 0x30 : 0x10) | (cc_video_ & 0x0F));
        cc_video_ = (uint8_t)((cc_video_ + 1) & 0x0F);

        uint8_t *p = pkt + 4;
        if (af_total) {
            *p++ = (uint8_t)af_len;
            if (af_len) {
                uint8_t *flags = p++;
                *flags = (uint8_t)(want_rai ? 0x40 : 0x00);
                if (want_pcr) {
                    *flags |= 0x10;
                    write_pcr(p, pcr);
                    p += 6;
                }
                // Whatever is left of the adaptation field is stuffing.
                size_t stuffing = (size_t)(pkt + 4 + af_total - p);
                memset(p, 0xFF, stuffing);
                p += stuffing;
            }
        }

        // Copy this packet's slice of [PES header][access unit].
        size_t copied = 0;
        while (copied < room) {
            size_t at = pos + copied;
            const uint8_t *src;
            size_t avail;
            if (at < sizeof(pes)) {
                src   = pes + at;
                avail = sizeof(pes) - at;
            } else {
                src   = au + (at - sizeof(pes));
                avail = au_len - (at - sizeof(pes));
            }
            size_t n = (room - copied < avail) ? room - copied : avail;
            memcpy(p + copied, src, n);
            copied += n;
        }

        pos += room;
        first = false;
    }
}

bool TsWriter::mux_access_unit(const uint8_t *data, int len, int duration_90k) {
    const bool keyframe = au_is_keyframe(data, (size_t)len);
    const int64_t pcr = media_ticks_ & TS_TIMESTAMP_MASK;
    const int64_t pts = (media_ticks_ + PTS_PREROLL_90K) & TS_TIMESTAMP_MASK;

    pkt_buf_.clear();

    if (psi_pending_ || keyframe || media_ticks_ - last_psi_ticks_ >= PSI_INTERVAL_90K) {
        emit_psi(pkt_buf_);
        last_psi_ticks_ = media_ticks_;
        psi_pending_ = false;
    }
    emit_pes(pkt_buf_, data, (size_t)len, pts, pcr, keyframe);

    // The whole access unit goes out in a single write rather than one per 188-byte packet.
    bool ok = write_block(pkt_buf_.data(), pkt_buf_.size());

    // Advance the timeline even if the write failed: the DVR fail-stops on the failure streak, and
    // keeping the clock aligned with the frames it submitted avoids a timestamp jump if it recovers.
    media_ticks_ += duration_90k;
    return ok;
}

void TsWriter::writer_loop() {
    pthread_setname_np(pthread_self(), "__DVR_TS");
    while (true) {
        AuJob job;
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

        if (file && video_started_ && !abandoned) {
            if (!mux_access_unit(job.data.data(), (int)job.data.size(), job.duration)) {
                if (write_fail_count % WRITE_FAIL_WARN_INTERVAL == 0) {
                    spdlog::warn("[ DVR TsWriter ] write failed ({} times)", write_fail_count + 1);
                }
                write_fail_count++;
                write_fail_streak.fetch_add(1, std::memory_order_relaxed);
            } else {
                write_fail_count = 0;
                write_fail_streak.store(0, std::memory_order_relaxed);
            }
            bytes_since_sync_ += job.data.size();
            sync_if_due();
        }

        {
            std::lock_guard<std::mutex> lock(qm_);
            processing_ = false;
        }
        qidle_.notify_all();
    }
}

bool TsWriter::sync_now() {
    if (!file) {
        return true;
    }
    bool ok = (fflush(file) == 0);
    if (ok && fdatasync(fileno(file)) != 0) {
        ok = false;
    }
    bytes_since_sync_ = 0;
    last_sync_ms_ = monotonic_ms();
    return ok;
}

void TsWriter::sync_if_due() {
    int64_t now = monotonic_ms();
    if (bytes_since_sync_ < SYNC_BYTES && now - last_sync_ms_ < SYNC_INTERVAL_MS) {
        return;
    }
    if (!sync_now()) {
        if (write_fail_count % WRITE_FAIL_WARN_INTERVAL == 0) {
            spdlog::warn("[ DVR TsWriter ] flush/fdatasync failed ({} times)", write_fail_count + 1);
        }
        write_fail_count++;
        write_fail_streak.fetch_add(1, std::memory_order_relaxed);
    }
}

void TsWriter::sync_dir_of(const std::string &path) {
    std::string dir = std::filesystem::path(path).parent_path().string();
    if (dir.empty()) {
        dir = ".";
    }
    int dfd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY);
    if (dfd < 0) {
        return;
    }
    fsync(dfd);
    ::close(dfd);
}

bool TsWriter::drain() {
    std::unique_lock<std::mutex> lock(qm_);
    return qidle_.wait_for(lock, DRAIN_TIMEOUT, [this] { return q_.empty() && !processing_; });
}

bool TsWriter::open(const std::string &path) {
    if (abandoned_) {
        spdlog::error("[ DVR TsWriter ] writer is unusable, not opening {}", path);
        return false;
    }
    file = fopen(path.c_str(), "w");
    if (!file) {
        spdlog::error("[ DVR TsWriter ] unable to open DVR file {}", path);
        return false;
    }
    sync_dir_of(path);
    file_size_bytes.store(0, std::memory_order_relaxed);
    write_fail_count = 0;
    write_fail_streak.store(0, std::memory_order_relaxed);
    bytes_since_sync_ = 0;
    last_sync_ms_ = monotonic_ms();
    return true;
}

bool TsWriter::begin_video(int width, int height) {
    if (!file) {
        spdlog::error("[ DVR TsWriter ] begin_video called without an open file");
        return false;
    }
    // Reset the mux timeline so every file stands alone: PTS restarts at the preroll and the
    // continuity counters at zero.
    media_ticks_ = 0;
    last_psi_ticks_ = 0;
    psi_pending_ = true;
    cc_video_ = 0;
    cc_pat_ = 0;
    cc_pmt_ = 0;
    pkt_buf_.clear();
    video_started_ = true;
    spdlog::debug("[ DVR TsWriter ] muxing H265 {}x{} to MPEG-TS", width, height);
    return true;
}

bool TsWriter::write_nal(const uint8_t *data, int len, int duration_90k) {
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
        // the DVR fail-stops (dropping the frame here would leave a gap in the stream anyway).
        write_fail_streak.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    AuJob job;
    job.data.assign(data, data + len);
    job.duration = duration_90k;
    q_bytes_ += (size_t)len;
    q_.push(std::move(job));
    qcv_.notify_one();
    return true;
}

bool TsWriter::close() {
    if (!drain()) {
        spdlog::error("[ DVR TsWriter ] write queue did not drain in {}s (storage wedged) — "
                      "leaving the file as-is",
                      (long long)DRAIN_TIMEOUT.count());
        std::lock_guard<std::mutex> lock(qm_);
        abandoned_ = true;
        video_started_ = false;
        return false;
    }

    // No index to write: everything already flushed is a complete, playable stream.
    video_started_ = false;
    bool sync_ok = sync_now();
    int fclose_ret = 0;
    if (file) {
        fclose_ret = fclose(file);
        file = nullptr;
    }
    return sync_ok && fclose_ret == 0;
}
