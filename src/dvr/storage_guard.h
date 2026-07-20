#ifndef DVR_STORAGE_GUARD_H
#define DVR_STORAGE_GUARD_H

#include <cstdint>
#include <string>

class StorageGuard {
public:
    StorageGuard(const std::string &dir, uint64_t min_free_bytes, bool require_mount);

    bool is_ready(std::string &reason) const;

    uint64_t file_size_cap() const;

    uint64_t free_bytes() const;

private:
    bool is_external_mount() const;

    std::string recording_dir;
    uint64_t    min_free_bytes = 0;
    bool        require_mount = false;
};

#endif // DVR_STORAGE_GUARD_H
