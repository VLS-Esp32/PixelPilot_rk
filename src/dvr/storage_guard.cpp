#include "storage_guard.h"

#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/statfs.h>
#include <linux/magic.h>

static const uint64_t FAT32_FILE_SIZE_CAP = (4ULL << 30) - (128ULL << 20); // ~3.9 GiB

StorageGuard::StorageGuard(const std::string &dir, uint64_t min_free_bytes, bool require_mount)
    : recording_dir(dir), min_free_bytes(min_free_bytes), require_mount(require_mount) {}

bool StorageGuard::free_bytes(uint64_t &out) const {
    struct statvfs vfs;
    if (statvfs(recording_dir.c_str(), &vfs) != 0) {
        return false;
    }
    out = (uint64_t)vfs.f_frsize * (uint64_t)vfs.f_bavail;
    return true;
}

bool StorageGuard::space_bytes(uint64_t &free_out, uint64_t &total_out) const {
    struct statvfs vfs;
    if (statvfs(recording_dir.c_str(), &vfs) != 0) {
        return false;
    }
    free_out = (uint64_t)vfs.f_frsize * (uint64_t)vfs.f_bavail;
    total_out = (uint64_t)vfs.f_frsize * (uint64_t)vfs.f_blocks;
    return true;
}

bool StorageGuard::device_id(dev_t &out) const {
    struct stat sdir;
    if (stat(recording_dir.c_str(), &sdir) != 0) {
        return false;
    }
    out = sdir.st_dev;
    return true;
}

bool StorageGuard::is_external_mount() const {
    struct stat sdir;
    struct stat sroot;
    if (stat(recording_dir.c_str(), &sdir) != 0 || stat("/", &sroot) != 0) {
        return false;
    }
    return sdir.st_dev != sroot.st_dev;
}

uint64_t StorageGuard::file_size_cap() const {
    struct statfs sf;
    if (statfs(recording_dir.c_str(), &sf) != 0) {
        return FAT32_FILE_SIZE_CAP; // unknown filesystem - be conservative
    }
    if ((uint32_t)sf.f_type == MSDOS_SUPER_MAGIC) {
        return FAT32_FILE_SIZE_CAP;
    }
    return 0; // large-file filesystem (ext4, exFAT, ...) - no cap needed
}

bool StorageGuard::mount_ok() const {
    return !require_mount || is_external_mount();
}

bool StorageGuard::is_ready(std::string &reason) const {
    struct stat sdir;
    if (stat(recording_dir.c_str(), &sdir) != 0) {
        reason = "recording directory " + recording_dir + " does not exist";
        return false;
    }
    if (require_mount && !is_external_mount()) {
        reason = "storage not mounted at " + recording_dir;
        return false;
    }
    uint64_t available = 0;
    if (!free_bytes(available)) {
        reason = "cannot read free space at " + recording_dir;
        return false;
    }
    if (available < min_free_bytes) {
        reason = "low free space at " + recording_dir + " (" +
                 std::to_string(available / (1024 * 1024)) + "MB free, need " +
                 std::to_string(min_free_bytes / (1024 * 1024)) + "MB)";
        return false;
    }
    return true;
}
