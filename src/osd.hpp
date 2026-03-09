#ifndef OSDPP_H
#define OSDPP_H

extern "C" {
#include "drm.h"
}
#include <nlohmann/json.hpp>
#include <filesystem>
#include <string>
#include <atomic>

namespace nlohmann {
    template <>
    struct adl_serializer<std::filesystem::path> {
        static void to_json(json& j, const std::filesystem::path& p) {
            j = p.string(); // convert path to string
        }

        static void from_json(const json& j, std::filesystem::path& p) {
            p = j.get<std::string>(); // convert string to path
        }
    };
}

typedef struct {
	struct modeset_output *out;
	int fd;
	nlohmann::json config;
    std::string screensaver_image;
} osd_thread_params;

extern int osd_thread_signal;

#define NUMBER_BUFFERS 3

struct SharedMemoryRegion {
    uint16_t width;       // Image width
    uint16_t height;      // Image height
    uint32_t stride;      // Number of bytes per image row
    uint8_t refresh_rate; // Refresh rate

    std::atomic<int32_t> front_index; // buffer index to read from
    std::atomic<int32_t> back_index;  // buffer index to write into
    std::atomic<int32_t> ready_index; // last fully written buffer (-1 = none)

    unsigned char data[]; // Three image buffers stored consecutively: [buffer0][buffer1][buffer2]
                          // Each buffer has size = stride * height
};

void *__OSD_THREAD__(void *param);

#endif
