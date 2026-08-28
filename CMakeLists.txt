cmake_minimum_required(VERSION 3.1)

project(pixelpilot VERSION 1.3.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options("-Wno-address-of-packed-member")

set(SOURCE_FILES 
        src/common.h
        src/drm.h
        src/drm.c
        src/osd.h
        src/osd.hpp
        src/osd.cpp
        src/rtp_codec_detector.hpp
        src/rtp_codec_detector.cpp
        src/rtp_receiver.hpp
        src/rtp_receiver.cpp
        src/dvr.h
        src/dvr.cpp
        src/mavlink.h
        src/mavlink.c
        src/main.cpp
        src/wfbcli.hpp
        src/wfbcli.cpp
        src/scheduling_helper.hpp
        src/socket_handler.hpp
        src/socket_handler.cpp
        )
file(GLOB ICONS src/icons/*.png)
file(GLOB OSD_CONFIGS *_osd.json)

# Autodetect include/library paths for target system
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBRGA REQUIRED IMPORTED_TARGET librga)
pkg_check_modules(LIBDRM REQUIRED IMPORTED_TARGET libdrm)
pkg_check_modules(CAIRO REQUIRED IMPORTED_TARGET cairo)
pkg_check_modules(SPDLOG REQUIRED IMPORTED_TARGET spdlog)
pkg_check_modules(NLOHMANN REQUIRED IMPORTED_TARGET nlohmann_json)
pkg_check_modules(MSGPACK REQUIRED IMPORTED_TARGET msgpack)

include_directories(${PROJECT_NAME}
        ${CMAKE_SOURCE_DIR}/lib/librtp/include
)

add_subdirectory(${CMAKE_SOURCE_DIR}/lib/librtp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} PRIVATE m rtp
        PkgConfig::LIBDRM
        PkgConfig::CAIRO
        PkgConfig::SPDLOG
        PkgConfig::NLOHMANN
        PkgConfig::MSGPACK
        rockchip_mpp pthread m rt
        PkgConfig::LIBRGA
)

if(CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_definitions(SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror -fsanitize=undefined -fsanitize=address")
  target_link_options(${PROJECT_NAME}
          BEFORE PUBLIC -fsanitize=undefined PUBLIC -fsanitize=address
  )
endif()

target_include_directories(${PROJECT_NAME}
        PUBLIC "${PROJECT_BINARY_DIR}"
)

configure_file("${PROJECT_NAME}_config.h.in" "${PROJECT_NAME}_config.h")

include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(FILES ${ICONS}
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}
)
install(FILES ${OSD_CONFIGS}
        DESTINATION ${CMAKE_INSTALL_SYSCONFDIR}/${PROJECT_NAME}
)
