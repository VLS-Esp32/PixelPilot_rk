#ifndef DVR_STORAGE_GUARD_H
#define DVR_STORAGE_GUARD_H

#include <cstdint>
#include <string>

class StorageGuard {
public:
    StorageGuard(const std::string &dir, uint64_t min_free_bytes, bool require_mount);

    bool is_ready(std::string &reason) const;

    // Cheap, I/O-free variants for the mid-recording hot path (statvfs on a busy FAT32 card can
    // stall 100ms+, which stutters the recording). Mount check is a fast stat; free-space uses a
    // caller-supplied estimate instead of statvfs.
    bool mount_ok() const;
    bool has_enough_free(uint64_t free_bytes_estimate) const { return free_bytes_estimate >= min_free_bytes; }
    uint64_t min_free() const { return min_free_bytes; }

    uint64_t file_size_cap() const;

    uint64_t free_bytes() const;

private:
    bool is_external_mount() const;

    std::string recording_dir;
    uint64_t    min_free_bytes = 0;
    bool        require_mount = false;
};

#endif // DVR_STORAGE_GUARD_H
