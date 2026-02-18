#ifndef OSDPP_H
#define OSDPP_H

extern "C" {
#include "drm.h"
}
#include <nlohmann/json.hpp>
#include <filesystem>
#include <string>

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

struct SharedMemoryRegion {
    uint16_t width;       // Image width
    uint16_t height;      // Image height
    unsigned char data[]; // Flexible array member for image data
};

void *__OSD_THREAD__(void *param);

#endif
