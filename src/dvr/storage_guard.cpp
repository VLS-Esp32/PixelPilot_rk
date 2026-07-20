#include "storage_guard.h"

#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/statfs.h>
#include <linux/magic.h>

static const uint64_t FAT32_FILE_SIZE_CAP = (4ULL << 30) - (128ULL << 20); // ~3.9 GiB

StorageGuard::StorageGuard(const std::string &dir, uint64_t min_free_bytes, bool require_mount)
    : recording_dir(dir), min_free_bytes(min_free_bytes), require_mount(require_mount) {}

uint64_t StorageGuard::free_bytes() const {
    struct statvfs vfs;
    if (statvfs(recording_dir.c_str(), &vfs) != 0) {
        return 0;
    }
    return (uint64_t)vfs.f_frsize * (uint64_t)vfs.f_bavail;
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

bool StorageGuard::is_ready(std::string &reason) const {
    if (require_mount && !is_external_mount()) {
        reason = "storage not mounted at " + recording_dir;
        return false;
    }
    uint64_t available = free_bytes();
    if (available < min_free_bytes) {
        reason = "low free space at " + recording_dir + " (" +
                 std::to_string(available / (1024 * 1024)) + "MB free, need " +
                 std::to_string(min_free_bytes / (1024 * 1024)) + "MB)";
        return false;
    }
    return true;
}
