#define MODULE_TAG "pixelpilot"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <unistd.h>
#include <inttypes.h>
#include <signal.h>
#include <fstream>
#include <queue>
#include <mutex>
#include <condition_variable>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <sys/mman.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include <linux/videodev2.h>
#include <rockchip/rk_mpi.h>
#include <nlohmann/json.hpp>
#include "spdlog/spdlog.h"

extern "C" {
#include "drm.h"

#include "mavlink/common/mavlink.h"
#include "mavlink.h"
}

#include "osd.h"
#include "osd.hpp"
#include "wfbcli.hpp"
#include "dvr.h"
#include "scheduling_helper.hpp"
#include "time_util.h"
#include "pixelpilot_config.h"
#include "rtp_codec_detector.hpp"
#include "rtp_receiver.hpp"
#include "common.h"


#define READ_BUF_SIZE (1024*1024) // SZ_1M https://github.com/rockchip-linux/mpp/blob/ed377c99a733e2cdbcc457a6aa3f0fcd438a9dff/osal/inc/mpp_common.h#L179
#define MAX_FRAMES 24		// min 16 and 20+ recommended (mpp/readme.txt)

#define CODEC_ALIGN(x, a)   (((x)+(a)-1)&~((a)-1))

struct {
	MppCtx		  ctx;
	MppApi		  *mpi;
	
	struct timespec first_frame_ts;
	uint8_t 		*nal_buffer;

	MppBufferGroup	frm_grp;
	struct {
		int prime_fd;
		uint32_t fb_id;
		uint32_t handle;
	} frame_to_drm[MAX_FRAMES];
} mpi;

enum AppOption {
    OPT_SOCKET = 256,
    OPT_CODEC,
    OPT_DVR,
    OPT_DVR_START,
    OPT_DVR_TEMPLATE,
    OPT_DVR_SEQUENCED_FILES,
    OPT_DVR_FRAMERATE,
    OPT_DVR_FMP4,
    OPT_LOG_LEVEL,
    OPT_MAVLINK_PORT,
    OPT_MAVLINK_DVR_ON_ARM,
    OPT_OSD,
    OPT_OSD_CONFIG,
    OPT_OSD_REFRESH,
    OPT_OSD_ELEMENTS,
    OPT_OSD_TELEM_LVL,
    OPT_OSD_CUSTOM_MESSAGE,
    OPT_SCREEN_MODE,
	OPT_TARGET_FRAME_RATE,
    OPT_DISABLE_VSYNC,
    OPT_SCREEN_MODE_LIST,
    OPT_WFB_API_PORT,
    OPT_VERSION
};

static const struct option pixelpilot_long_options[] = {
    {"socket",              required_argument, 0, OPT_SOCKET},
    {"codec",               required_argument, 0, OPT_CODEC},
    {"dvr",                 required_argument, 0, OPT_DVR},
    {"dvr-start",           no_argument,       0, OPT_DVR_START},
    {"dvr-template",        required_argument, 0, OPT_DVR_TEMPLATE},
    {"dvr-sequenced-files", no_argument,       0, OPT_DVR_SEQUENCED_FILES},
    {"dvr-framerate",       required_argument, 0, OPT_DVR_FRAMERATE},
    {"dvr-fmp4",            no_argument,       0, OPT_DVR_FMP4},
    {"log-level",           required_argument, 0, OPT_LOG_LEVEL},
    {"mavlink-port",        required_argument, 0, OPT_MAVLINK_PORT},
    {"mavlink-dvr-on-arm",  no_argument,       0, OPT_MAVLINK_DVR_ON_ARM},
    {"osd",                 no_argument,       0, OPT_OSD},
    {"osd-config",          required_argument, 0, OPT_OSD_CONFIG},
    {"osd-refresh",         required_argument, 0, OPT_OSD_REFRESH},
    {"osd-elements",        required_argument, 0, OPT_OSD_ELEMENTS},
    {"osd-telem-lvl",       required_argument, 0, OPT_OSD_TELEM_LVL},
    {"osd-custom-message",  no_argument,       0, OPT_OSD_CUSTOM_MESSAGE},
    {"screen-mode",         required_argument, 0, OPT_SCREEN_MODE},
    {"target-frame-rate",   required_argument, 0, OPT_TARGET_FRAME_RATE},
    {"disable-vsync",       no_argument,       0, OPT_DISABLE_VSYNC},
    {"screen-mode-list",    no_argument,       0, OPT_SCREEN_MODE_LIST},
    {"wfb-api-port",        required_argument, 0, OPT_WFB_API_PORT},
    {"version",             no_argument,       0, OPT_VERSION},
    {"help",                no_argument,       0, 'h'},
    {0, 0, 0, 0}
};

struct timespec frame_stats[1000];

struct modeset_output *output_list;
int frm_eos = 0;
int drm_fd = 0;
pthread_mutex_t video_mutex;
pthread_cond_t video_cond;
extern bool osd_update_ready;
extern std::atomic<bool> drone_connected;
int video_zpos = 1;

bool mavlink_dvr_on_arm = false;
bool osd_custom_message = false;
bool disable_vsync = false;
uint32_t refresh_frequency_ms = 1000;

std::atomic<bool> codec_detected = false;
std::atomic<bool> codec_changed = false;
pthread_t tid_frame;
VideoCodec codec = VideoCodec::H265;
Dvr *dvr = NULL;
int dvr_autostart = 0;

void init_buffer(MppFrame frame) {
	output_list->video_frm_width = mpp_frame_get_width(frame);
	output_list->video_frm_height = mpp_frame_get_height(frame);
	RK_U32 hor_stride = mpp_frame_get_hor_stride(frame);
	RK_U32 ver_stride = mpp_frame_get_ver_stride(frame);
	MppFrameFormat fmt = mpp_frame_get_fmt(frame);
	assert((fmt == MPP_FMT_YUV420SP) || (fmt == MPP_FMT_YUV420SP_10BIT));

	spdlog::info("Frame info changed {}({})x{}({})",
				 output_list->video_frm_width, hor_stride, output_list->video_frm_height, ver_stride);

	output_list->video_fb_x = 0;
	output_list->video_fb_y = 0;
	output_list->video_fb_width = output_list->mode.hdisplay;
	output_list->video_fb_height =output_list->mode.vdisplay;	

	osd_publish_uint_fact("video.width", NULL, 0, output_list->video_frm_width);
	osd_publish_uint_fact("video.height", NULL, 0, output_list->video_frm_height);

	if (mpi.frm_grp) {
		spdlog::debug("Freeing current mpp_buffer_group");
		
		// First clean up all DRM resources for existing frames
		for (int i = 0; i < MAX_FRAMES; i++) {
			if (mpi.frame_to_drm[i].fb_id) {
				drmModeRmFB(drm_fd, mpi.frame_to_drm[i].fb_id);
				mpi.frame_to_drm[i].fb_id = 0;
			}
			if (mpi.frame_to_drm[i].prime_fd >= 0) {
				close(mpi.frame_to_drm[i].prime_fd);
				mpi.frame_to_drm[i].prime_fd = -1;
			}
			if (mpi.frame_to_drm[i].handle) {
				struct drm_mode_destroy_dumb dmd = {
					.handle = mpi.frame_to_drm[i].handle,
				};
				ioctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dmd);
				mpi.frame_to_drm[i].handle = 0;
			}
		}
		
		mpp_buffer_group_clear(mpi.frm_grp);
		mpp_buffer_group_put(mpi.frm_grp);  // This is important to release the group
		mpi.frm_grp = NULL;
	}

	// create new external frame group and allocate (commit flow) new DRM buffers and DRM FB
	int ret = mpp_buffer_group_get_external(&mpi.frm_grp, MPP_BUFFER_TYPE_DRM);
	assert(!ret);			

	for (int i=0; i<MAX_FRAMES; i++) {
		
		// new DRM buffer
		struct drm_mode_create_dumb dmcd;
		memset(&dmcd, 0, sizeof(dmcd));
		dmcd.bpp = fmt==MPP_FMT_YUV420SP?8:10;
		dmcd.width = hor_stride;
		dmcd.height = ver_stride*2; // documentation say not v*2/3 but v*2 (additional info included)
		do {
			ret = ioctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &dmcd);
		} while (ret == -1 && (errno == EINTR || errno == EAGAIN));
		assert(!ret);
		// assert(dmcd.pitch==(fmt==MPP_FMT_YUV420SP?hor_stride:hor_stride*10/8));
		// assert(dmcd.size==(fmt == MPP_FMT_YUV420SP?hor_stride:hor_stride*10/8)*ver_stride*2);
		mpi.frame_to_drm[i].handle = dmcd.handle;
		
		// commit DRM buffer to frame group
		struct drm_prime_handle dph;
		memset(&dph, 0, sizeof(struct drm_prime_handle));
		dph.handle = dmcd.handle;
		dph.fd = -1;
		do {
			ret = ioctl(drm_fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &dph);
		} while (ret == -1 && (errno == EINTR || errno == EAGAIN));
		assert(!ret);
		MppBufferInfo info;
		memset(&info, 0, sizeof(info));
		info.type = MPP_BUFFER_TYPE_DRM;
		info.size = dmcd.width*dmcd.height;
		info.fd = dph.fd;
		ret = mpp_buffer_commit(mpi.frm_grp, &info);
		assert(!ret);
		mpi.frame_to_drm[i].prime_fd = info.fd; // dups fd
		if (dph.fd != info.fd) {
			ret = close(dph.fd);
			assert(!ret);
		}

		// allocate DRM FB from DRM buffer
		uint32_t handles[4], pitches[4], offsets[4];
		memset(handles, 0, sizeof(handles));
		memset(pitches, 0, sizeof(pitches));
		memset(offsets, 0, sizeof(offsets));
		handles[0] = mpi.frame_to_drm[i].handle;
		offsets[0] = 0;
		pitches[0] = hor_stride;
		handles[1] = mpi.frame_to_drm[i].handle;
		offsets[1] = pitches[0] * ver_stride;
		pitches[1] = pitches[0];
		ret = drmModeAddFB2(drm_fd, output_list->video_frm_width, output_list->video_frm_height, DRM_FORMAT_NV12, handles, pitches, offsets, &mpi.frame_to_drm[i].fb_id, 0);
		assert(!ret);
	}

	// register external frame group
	ret = mpi.mpi->control(mpi.ctx, MPP_DEC_SET_EXT_BUF_GROUP, mpi.frm_grp);
	ret = mpi.mpi->control(mpi.ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);

	ret = modeset_perform_modeset(drm_fd, output_list, output_list->video_request, &output_list->video_plane, mpi.frame_to_drm[0].fb_id, output_list->video_frm_width, output_list->video_frm_height, video_zpos);
	assert(ret >= 0);

	// dvr setup
	if (dvr != NULL){
		dvr->set_video_params(output_list->video_frm_width, output_list->video_frm_height, codec);
	}
}

// __FRAME_THREAD__
//
// - allocate DRM buffers and DRM FB based on frame size
// - pick frame in blocking mode and output to screen overlay

void *__FRAME_THREAD__(void *param)
{
	SchedulingHelper::set_thread_params_max_realtime("FRAME_THREAD",SchedulingHelper::PRIORITY_REALTIME_MID);
	int i, ret;
	MppFrame  frame  = NULL;
	uint64_t last_frame_time;
	pthread_setname_np(pthread_self(), "__FRAME");

	while (!frm_eos) {
		struct timespec ts, ats;

		// Skip frame till codec will no be detected
		if (!codec_detected.load())
		{
			usleep(50000);
			continue;
		}
		if (codec_changed.load())
		{
			spdlog::info("Frame thread exiting due to changed codec.");
			return nullptr;
		}
		assert(!frame);
		ret = mpi.mpi->decode_get_frame(mpi.ctx, &frame);
		assert(!ret);
		clock_gettime(CLOCK_MONOTONIC, &ats);
		if (frame) {
			if (mpp_frame_get_info_change(frame)) {
				// new resolution
				init_buffer(frame);
			} else {
				// regular frame received
				if (!mpi.first_frame_ts.tv_sec) {
					ts = ats;
					mpi.first_frame_ts = ats;
				}

				MppBuffer buffer = mpp_frame_get_buffer(frame);
				if (buffer) {
					output_list->video_poc = mpp_frame_get_poc(frame);
					uint64_t feed_data_ts =  mpp_frame_get_pts(frame);

					MppBufferInfo info;
					ret = mpp_buffer_info_get(buffer, &info);
					assert(!ret);
					for (i=0; i<MAX_FRAMES; i++) {
						if (mpi.frame_to_drm[i].prime_fd == info.fd) break;
					}
					assert(i!=MAX_FRAMES);

					ts = ats;
					
					// send DRM FB to display thread
					ret = pthread_mutex_lock(&video_mutex);
					assert(!ret);
					output_list->video_fb_id = mpi.frame_to_drm[i].fb_id;
                    //output_list->video_fb_index=i;
                    output_list->decoding_pts=feed_data_ts;
					ret = pthread_cond_signal(&video_cond);
					assert(!ret);
					ret = pthread_mutex_unlock(&video_mutex);
					assert(!ret);
					
				}
			}
			
			frm_eos = mpp_frame_get_eos(frame);
			mpp_frame_deinit(&frame);
			frame = NULL;
		} else assert(0);
	}
	spdlog::info("Frame thread done");
	return nullptr;
}


void *__DISPLAY_THREAD__(void *param)
{
	int ret;	
	pthread_setname_np(pthread_self(), "__DISPLAY");

	while (!frm_eos) {
		int fb_id;
		bool osd_update;
		
		ret = pthread_mutex_lock(&video_mutex);
		assert(!ret);
		while (output_list->video_fb_id==0 && !osd_update_ready) {
			pthread_cond_wait(&video_cond, &video_mutex);
			assert(!ret);
			if (output_list->video_fb_id == 0 && frm_eos) {
				ret = pthread_mutex_unlock(&video_mutex);
				assert(!ret);
				goto end;
			}
		}
		fb_id = output_list->video_fb_id;
		osd_update = osd_update_ready;

        uint64_t decoding_pts=fb_id != 0 ? output_list->decoding_pts : get_time_ms();
		output_list->video_fb_id=0;
		osd_update_ready = false;
		ret = pthread_mutex_unlock(&video_mutex);
		assert(!ret);

		// create new video_request
		drmModeAtomicFree(output_list->video_request);
		output_list->video_request = drmModeAtomicAlloc();

		// show DRM FB in plane
		uint32_t flags = DRM_MODE_ATOMIC_NONBLOCK;
		if (fb_id != 0) {
			flags = disable_vsync ? DRM_MODE_ATOMIC_NONBLOCK : DRM_MODE_ATOMIC_ALLOW_MODESET;
			ret = set_drm_object_property(output_list->video_request, &output_list->video_plane, "FB_ID", fb_id);
			assert(ret>0);
		}

        ret = pthread_mutex_lock(&osd_mutex);
        assert(!ret);
        ret = set_drm_object_property(output_list->video_request, &output_list->osd_plane, "FB_ID", output_list->osd_bufs[output_list->osd_buf_switch].fb);
        assert(ret>0);

        drmModeAtomicCommit(drm_fd, output_list->video_request, flags, NULL);

        ret = pthread_mutex_unlock(&osd_mutex);
        assert(!ret);

        if (enable_osd && fb_id != 0) {
            osd_publish_uint_fact("video.displayed_frame", NULL, 0, 1);
            uint64_t decode_and_handover_display_ms = get_time_ms() - decoding_pts;
            osd_publish_uint_fact("video.decode_and_handover_ms", NULL, 0, decode_and_handover_display_ms);
        }
	}
end:	
	spdlog::info("Display thread done");
	return nullptr;
}

// signal

int signal_flag = 0;

void sig_handler(int signum)
{
	spdlog::info("Received signal {}", signum);
	signal_flag++;
	mavlink_thread_signal++;
	wfb_thread_signal++;
	osd_thread_signal++;
	if (dvr != NULL) {
		dvr->shutdown();
	}
}

void sigusr1_handler(int signum) {
	spdlog::info("Received signal {}", signum);
	if (dvr && codec == VideoCodec::H265) {
		dvr->toggle_recording();
	}
	else if (dvr && codec != VideoCodec::H265) {
		spdlog::warn("Received signal {} DVR can not start recording due to unsupported codec {}", signum, codec_type_name(codec));
	}
}

void sigusr2_handler(int signum) {
    // Toggle the disable_vsync flag
    disable_vsync = disable_vsync ^ 1;

    // Open the file for writing
    std::ofstream outFile("/run/pixelpilot.msg");
    if (!outFile.is_open()) {
        spdlog::error("Error opening file!");
        return; // Exit the function if the file cannot be opened
    }

    // Write the formatted text to the file
    outFile << "disable_vsync: " << std::boolalpha << disable_vsync << std::endl;
    outFile.close();

    // Log the new state of disable_vsync
    spdlog::info("disable_vsync: {}", disable_vsync);
}

int decoder_stalled_count=0;
bool feed_packet_to_decoder(MppPacket packet, void* data_p, int data_len){
    mpp_packet_set_data(packet, data_p);
    mpp_packet_set_size(packet, data_len);
    mpp_packet_set_pos(packet, data_p);
    mpp_packet_set_length(packet, data_len);
    mpp_packet_set_pts(packet,(RK_S64) get_time_ms());
    // Feed the data to mpp until either timeout (in which case the decoder might have stalled)
    // or success
    uint64_t data_feed_begin = get_time_ms();
    int ret=0;
    while (!signal_flag && MPP_OK != (ret = mpi.mpi->decode_put_packet(mpi.ctx, packet))) {
        uint64_t elapsed = get_time_ms() - data_feed_begin;
        osd_publish_uint_fact("video.decoder_feed_time_ms", NULL, 0, elapsed);
        if (elapsed > 100) {
            decoder_stalled_count++;
            spdlog::warn("Cannot feed decoder, stalled {} ?", decoder_stalled_count);
            return false;
        }
        usleep(2 * 1000);
    }
    return true;
}

void set_control_verbose(MppApi* mpi, MppCtx ctx, MpiCmd control, RK_U32 enable){
    RK_U32 res = mpi->control(ctx, control, &enable);
    if(res){
        spdlog::warn("Could not set control {} {}", control, enable);
        assert(false);
    }
}

void set_mpp_decoding_parameters(MppApi* mpi, MppCtx ctx) {
    // config for runtime mode
    MppDecCfg cfg       = NULL;
    mpp_dec_cfg_init(&cfg);
    // get default config from decoder context
    int ret = mpi->control(ctx, MPP_DEC_GET_CFG, cfg);
    if (ret) {
        spdlog::warn("{} failed to get decoder cfg ret {}", ctx, ret);
        assert(false);
    }
    // split_parse is to enable mpp internal frame spliter when the input
    // packet is not aplited into frames.
    RK_U32 need_split   = 1;
    ret = mpp_dec_cfg_set_u32(cfg, "base:split_parse", need_split);
    if (ret) {
        spdlog::warn("{} failed to set split_parse ret {}", ctx, ret);
        assert(false);
    }
    ret = mpi->control(ctx, MPP_DEC_SET_CFG, cfg);
    if (ret) {
        spdlog::warn("{} failed to set cfg {} ret {}", ctx, cfg, ret);
        assert(false);
    }
	int mpp_split_mode = 1;// Enable split mode
    set_control_verbose(mpi,ctx,MPP_DEC_SET_PARSER_SPLIT_MODE, mpp_split_mode);
    int disable_error = 1; // Disable error handling
	set_control_verbose(mpi,ctx,MPP_DEC_SET_DISABLE_ERROR, disable_error);
	int immediate_out = 1; // Enable immediate output
    set_control_verbose(mpi,ctx,MPP_DEC_SET_IMMEDIATE_OUT, immediate_out);
	int fast_play = 1; // Enable fast play mode
    set_control_verbose(mpi,ctx,MPP_DEC_SET_ENABLE_FAST_PLAY, fast_play);
    //set_control_verbose(mpi,ctx,MPP_DEC_SET_ENABLE_DEINTERLACE, 0xffff);
    // Docu fast mode:
    // and improve the
    // parallelism of decoder hardware and software
    // we probably don't want that, since we don't need pipelining to hit our bitrate(s)
    int fast_mode = 0;
    set_control_verbose(mpi,ctx,MPP_DEC_SET_PARSER_FAST_MODE,fast_mode);
}

void setup_mpi(MppPacket &packet)
{
	MppCodingType mpp_type = MPP_VIDEO_CodingHEVC;
	if(codec == VideoCodec::H264) {
		mpp_type = MPP_VIDEO_CodingAVC;
	}
	int ret = mpp_check_support_format(MPP_CTX_DEC, mpp_type);
	assert(!ret);

	if (!mpi.nal_buffer) {
        mpi.nal_buffer = (uint8_t*)malloc(READ_BUF_SIZE);
        assert(mpi.nal_buffer);
    }
	ret = mpp_packet_init(&packet, mpi.nal_buffer, READ_BUF_SIZE);
	assert(!ret);

	ret = mpp_create(&mpi.ctx, &mpi.mpi);
	assert(!ret);
	ret = mpp_init(mpi.ctx, MPP_CTX_DEC, mpp_type);
    assert(!ret);
    set_mpp_decoding_parameters(mpi.mpi,mpi.ctx);

    RK_S64 block = 10; // 10 ms timeout for input/output operations for catch signals
    ret = mpi.mpi->control(mpi.ctx, MPP_SET_OUTPUT_TIMEOUT, &block);
	assert(!ret);

	spdlog::info("MPI setup done");
}

void cleanup_mpi(MppPacket &packet)
{
	int i;

	int ret = mpi.mpi->reset(mpi.ctx);
	assert(!ret);

	if (mpi.frm_grp) {
		ret = mpp_buffer_group_put(mpi.frm_grp);
		assert(!ret);
		mpi.frm_grp = NULL;
		for (i=0; i<MAX_FRAMES; i++) {
			ret = drmModeRmFB(drm_fd, mpi.frame_to_drm[i].fb_id);
			assert(!ret);
			struct drm_mode_destroy_dumb dmdd;
			memset(&dmdd, 0, sizeof(dmdd));
			dmdd.handle = mpi.frame_to_drm[i].handle;
			do {
				ret = ioctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dmdd);
			} while (ret == -1 && (errno == EINTR || errno == EAGAIN));
			assert(!ret);
		}
	}
		
	mpp_packet_deinit(&packet);
	mpp_destroy(mpi.ctx);

	free(mpi.nal_buffer);
	mpi.nal_buffer = NULL;

	spdlog::info("MPI cleanup done");
}

int setup_drm(int print_modelist, uint16_t mode_width, uint16_t mode_height, uint32_t mode_vrefresh, uint32_t target_frame_rate)
{
	int ret = modeset_open(&drm_fd, "/dev/dri/card0");
	if (ret < 0) {
		spdlog::warn("modeset_open() =  {}", ret);
	}
	assert(drm_fd >= 0);
	if (print_modelist) {
		modeset_print_modes(drm_fd);
		close(drm_fd);
		return 0;
	}

	output_list = modeset_prepare(drm_fd, mode_width, mode_height, mode_vrefresh, target_frame_rate);
	if (!output_list) {
		fprintf(stderr,
				"cannot initialize display. Is display connected? Is --screen-mode correct?\n");
		return -2;
	}
	return 1;
}

void cleanup_drm()
{
	restore_planes_zpos(drm_fd, output_list);
	drmModeSetCrtc(drm_fd,
			       output_list->saved_crtc->crtc_id,
			       output_list->saved_crtc->buffer_id,
			       output_list->saved_crtc->x,
			       output_list->saved_crtc->y,
			       &output_list->connector.id,
			       1,
			       &output_list->saved_crtc->mode);
	drmModeFreeCrtc(output_list->saved_crtc);
	drmModeAtomicFree(output_list->video_request);
	drmModeAtomicFree(output_list->osd_request);
	modeset_cleanup(drm_fd, output_list);
	close(drm_fd);

	spdlog::info("DRM cleanup done");
}

void restart_mpi(MppPacket &packet, VideoCodec new_codec)
{
	codec_changed.store(true);
	spdlog::info("Current codec: {} new codec: {}", codec_type_name(codec), codec_type_name(new_codec));
	codec = new_codec;
	int ret = pthread_join(tid_frame, NULL);
	assert(!ret);

	ret = pthread_mutex_lock(&video_mutex);
	assert(!ret);
	ret = pthread_cond_signal(&video_cond);
	assert(!ret);
	ret = pthread_mutex_unlock(&video_mutex);
	assert(!ret);

	cleanup_mpi(packet);
	setup_mpi(packet);
	codec_changed.store(false);


	ret = pthread_create(&tid_frame, NULL, __FRAME_THREAD__, NULL);
	assert(!ret);

	spdlog::info("MPI restart done");
}

uint64_t first_frame_ms=0;
void read_video_stream(MppPacket &packet, int udp_port, const char* sock) {
	std::unique_ptr<RtpReceiver> rtp_receiver;
	if (sock) {
		rtp_receiver = std::make_unique<RtpReceiver>(sock);
	} else {
		rtp_receiver = std::make_unique<RtpReceiver>(udp_port);
	}

	rtp_receiver->init();
	codec = rtp_receiver->get_video_codec();

	setup_mpi(packet);
	
	codec_detected.store(true);

	long long bytes_received = 0; 
	uint64_t period_start=0;
	auto cb=[&packet, &rtp_receiver, /*&decoder_stalled_count,*/ &bytes_received, &period_start](void *data, int size, bool is_codec_changed){
		if (is_codec_changed)
		{
			codec_changed.store(true);
		}
		bytes_received += size;
		uint64_t now = get_time_ms();
		osd_publish_uint_fact("rtp.received_bytes", NULL, 0, size);
        feed_packet_to_decoder(packet, data, size);
        if (dvr_enabled && dvr != NULL && codec == VideoCodec::H265) {
			auto frame = std::make_shared<std::vector<uint8_t>>(size);
			std::memcpy(frame->data(), data, size);
			dvr->frame(frame);
        }
    };
	rtp_receiver->start_receiving(cb);
	// TODO: DVR Recording allowed only for H265 due unclear bug in librtp we receive corrupted data for H264 codec and due to that dvr can't write data correctly
	if (dvr_autostart && dvr != NULL && codec == VideoCodec::H265) {
		dvr->start_recording();
	}

    while (!signal_flag){
		if (codec_changed.load())
		{
			rtp_receiver->stop_receiving();
			mpp_packet_set_eos(packet);
			mpp_packet_set_length(packet, 0);
			int ret=0;
			while (MPP_OK != (ret = mpi.mpi->decode_put_packet(mpi.ctx, packet))) {
				usleep(10000);
			}
			restart_mpi(packet, rtp_receiver->get_video_codec());
			rtp_receiver->start_receiving(cb);
		}
        else {
            if (!drone_connected.load()) {
                void *batch = osd_batch_init(4);

                osd_add_clear_fact(batch, "video.displayed_frame", nullptr, 0);
                osd_add_clear_fact(batch, "rtp.received_bytes", nullptr, 0);
                osd_add_clear_fact(batch, "video.width", nullptr, 0);
                osd_add_clear_fact(batch, "video.height", nullptr, 0);

                osd_publish_batch(batch);
            }
            sleep(10);
		}
    }
	rtp_receiver->stop_receiving();
    spdlog::info("Feeding eos");
    mpp_packet_set_eos(packet);
    mpp_packet_set_length(packet, 0);
    int ret=0;
    while (MPP_OK != (ret = mpi.mpi->decode_put_packet(mpi.ctx, packet))) {
        usleep(10000);
    }
};


void printHelp() {
  printf(
    "PixelPilot FPV Decoder for Rockchip (%d.%d)\n"
    "\n"
    "  Usage:\n"
    "    pixelpilot [Arguments]\n"
    "\n"
    "  Arguments:\n"
    "    -p <port>                 - UDP port for RTP video stream         (Default: 5600)\n"
    "\n"
    "    --socket <socket>         - read data from socket\n"
    "\n"
    "    --mavlink-port <port>     - UDP port for mavlink telemetry        (Default: 14550)\n"
    "\n"
    "    --mavlink-dvr-on-arm      - Start recording when armed\n"
    "\n"
    "    --codec <codec>           - [ Deprecated ] Video codec, should be the same as on VTX  (Default: h265 <h264|h265>)\n"
	"                                Now codec is detected dynamically during runtime. Passed value <codec> will ignored\n"
    "\n"
    "    --log-level <level>       - Log verbosity level, debug|info|warn|error (Default: info)\n"
    "\n"
    "    --osd                     - Enable OSD\n"
    "\n"
    "    --osd-config <file>       - Path to OSD configuration file\n"
    "\n"
    "    --osd-refresh <rate>      - Defines the delay between osd refresh (Default: 1000 ms)\n"
    "\n"
    "    --osd-custom-message      - Enables the display of /run/pixelpilot.msg (beta feature, may be removed)\n"
    "\n"
    "    --dvr-template <path>     - Save the video feed (no osd) to the provided filename template.\n"
    "                                DVR is toggled by SIGUSR1 signal\n"
    "                                Supports placeholders %%Y - year, %%m - month, %%d - day,\n"
    "                                %%H - hour, %%M - minute, %%S - second. Ex: /media/DVR/%%Y-%%m-%%d_%%H-%%M-%%S.mp4\n"
    "\n"
	"    --dvr-sequenced-files     - Prepend a sequence number to the names of the dvr files\n"
	"\n"
    "    --dvr-start               - Start DVR immediately\n"
    "\n"
    "    --dvr-framerate <rate>    - Force the dvr framerate for smoother dvr, ex: 60\n"
    "\n"
    "    --dvr-fmp4                - Save the video feed as a fragmented mp4\n"
    "\n"
    "    --screen-mode <mode>      - Override default screen mode. <width>x<heigth>@<fps> ex: 1920x1080@120\n"
    "\n"
	"    --target-frame-rate <fps> - Target DRM refresh rate for mode selection (30..120), ex: 60\n"
	"                                Makes DRM choose the highest available resolution at the requested FPS\n"
	"                                For optimal smoothness, use a value equal to or divisible by the video FPS.\n"
    "\n"
	"    --disable-vsync           - Disable VSYNC commits\n"
	"\n"
    "    --screen-mode-list        - Print the list of supported screen modes and exit.\n"
    "\n"
    "    --wfb-api-port            - Port of wfb-server for cli statistics. (Default: 8003)\n"
	"                                Use \"0\" to disable this stats\n"
    "\n"
    "    --version                 - Show program version\n"
    "\n", APP_VERSION_MAJOR, APP_VERSION_MINOR
  );
}

static void printVersion()
{
    printf("PixelPilot Rockchip %d.%d\n", APP_VERSION_MAJOR, APP_VERSION_MINOR);
}

// main

int main(int argc, char **argv)
{
	int ret;	
	int i, j;
	bool mavlink_thread = false;
	int print_modelist = 0;
	char* dvr_template = NULL;
	int video_framerate = -1;
	int mp4_fragmentation_mode = 0;
	bool dvr_filenames_with_sequence = false;
	uint16_t listen_port = 5600;
	const char* unix_socket = NULL;
	uint16_t wfb_port = 8003;
	uint16_t mode_width = 0;
	uint16_t mode_height = 0;
	uint32_t mode_vrefresh = 0;
	uint32_t target_frame_rate = 0;
	std::string osd_config_path;
	auto log_level = spdlog::level::info;
	
    std::string pidFilePath = "/run/pixelpilot.pid";
    std::ofstream pidFile(pidFilePath);
    pidFile << getpid();
    pidFile.close();

	MppPacket packet;

	// Load console arguments
	int opt;
	int option_index = 0;

	while ((opt = getopt_long(argc, argv, "hp:", pixelpilot_long_options, &option_index)) != -1) {
    	switch (opt) {

    	case 'h':
        	printHelp();
        	return 0;

    	case 'p': { // -p <port>
        	char *end = nullptr;
        	long v = strtol(optarg, &end, 10);
        	if (*end != '\0' || v <= 0 || v > 65535) {
            	spdlog::error("-p: invalid port '{}'", optarg);
            	printHelp();
            	return -1;
        	}
        	listen_port = static_cast<uint16_t>(v);
        	break;
    	}

    	case OPT_SOCKET: // --socket
        	unix_socket = optarg;
        	break;

    	case OPT_CODEC: // --codec (deprecated)
        	spdlog::warn("--codec parameter is removed");
        	break;

    	case OPT_DVR: // --dvr (deprecated)
        	dvr_template = optarg;
        	dvr_autostart = 1;
        	spdlog::warn("--dvr is deprecated. Use --dvr-template and --dvr-start");
        	break;

    	case OPT_DVR_START: // --dvr-start
        	dvr_autostart = 1;
        	break;

    	case OPT_DVR_TEMPLATE: // --dvr-template
        	dvr_template = optarg;
        	break;

    	case OPT_DVR_SEQUENCED_FILES: // --dvr-sequenced-files
        	dvr_filenames_with_sequence = true;
        	break;

    	case OPT_DVR_FRAMERATE: { // --dvr-framerate
        	char *end = nullptr;
        	long v = strtol(optarg, &end, 10);
        	if (*end != '\0' || v <= 0 || v > 120) {
            	spdlog::error("--dvr-framerate: invalid value '{}'", optarg);
            	printHelp();
            	return -1;
        	}
        	video_framerate = static_cast<int>(v);
        	break;
    	}

    	case OPT_DVR_FMP4: // --dvr-fmp4
        	mp4_fragmentation_mode = 1;
        	break;

    	case OPT_LOG_LEVEL: { // --log-level
        	std::string log_l(optarg);
        	if (log_l == "info") {
            	log_level = spdlog::level::info;
        	} else if (log_l == "debug") {
            	log_level = spdlog::level::debug;
            	spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [thread %t] [%s:%#] [%^%l%$] %v");
        	} else if (log_l == "warn") {
            	log_level = spdlog::level::warn;
        	} else if (log_l == "error") {
            	log_level = spdlog::level::err;
        	} else {
            	spdlog::error("invalid log level '{}'", log_l);
            	printHelp();
            	return -1;
        	}
        	break;
   		}

    	case OPT_MAVLINK_PORT: { // --mavlink-port
        	char *end = nullptr;
        	long v = strtol(optarg, &end, 10);
        	if (*end != '\0' || v <= 0 || v > 65535) {
            	spdlog::error("--mavlink-port: invalid port '{}'", optarg);
            	printHelp();
            	return -1;
        	}
        	mavlink_port = static_cast<int>(v);
        	break;
    	}

    	case OPT_MAVLINK_DVR_ON_ARM: // --mavlink-dvr-on-arm
        	mavlink_dvr_on_arm = true;
        	break;

    	case OPT_OSD: // --osd
            enable_osd = true;
            mavlink_thread = true;
        	break;

    	case OPT_OSD_CONFIG: // --osd-config
        	osd_config_path = std::string(optarg);
        	break;

    	case OPT_OSD_REFRESH: { // --osd-refresh
        	char *end = nullptr;
        	long v = strtol(optarg, &end, 10);
        	if (*end != '\0' || v <= 0 || v > 2000) {
            	spdlog::error("--osd-refresh: invalid value '{}'", optarg);
            	printHelp();
            	return -1;
        	}
        	refresh_frequency_ms = static_cast<uint32_t>(v);
        	break;
    	}

    	case OPT_OSD_ELEMENTS: // --osd-elements (deprecated)
        	spdlog::warn("--osd-elements parameter is removed");
        	break;

    	case OPT_OSD_TELEM_LVL: // --osd-telem-lvl (deprecated)
        	spdlog::warn("--osd-telem-lvl parameter is removed");
        	break;

    	case OPT_OSD_CUSTOM_MESSAGE: // --osd-custom-message
        	osd_custom_message = true;
        	break;

    	case OPT_SCREEN_MODE: { // --screen-mode
        	int w = 0, h = 0, r = 0;
        	if (sscanf(optarg, "%dx%d@%d", &w, &h, &r) != 3 || w <= 0 || h <= 0 || r <= 0) {
            	spdlog::error("invalid --screen-mode '{}'", optarg);
            	printHelp();
            	return -1;
        	}
        	mode_width    = static_cast<uint16_t>(w);
        	mode_height   = static_cast<uint16_t>(h);
        	mode_vrefresh = static_cast<uint32_t>(r);
        	break;
    	}

    	case OPT_TARGET_FRAME_RATE: { // --target-frame-rate
        	char *end = nullptr;
        	long v = strtol(optarg, &end, 10);
        	if (*end != '\0' || v < 30 || v > 120) {
            	spdlog::error("invalid --target-frame-rate '{}'", optarg);
            	printHelp();
            	return -1;
        	}
        	target_frame_rate = static_cast<uint32_t>(v);
        	break;
    	}

		case OPT_DISABLE_VSYNC: // --disable-vsync
        	disable_vsync = true;
        	break;

    	case OPT_SCREEN_MODE_LIST: // --screen-mode-list
        	print_modelist = 1;
        	break;

    	case OPT_WFB_API_PORT: { // --wfb-api-port
        	char *end = nullptr;
        	long v = strtol(optarg, &end, 10);
        	if (*end != '\0' || v < 0 || v > 65535) {
            	spdlog::error("--wfb-api-port: invalid port '{}'", optarg);
            	printHelp();
            	return -1;
        	}
        	wfb_port = static_cast<uint16_t>(v);
        	break;
    	}

    	case OPT_VERSION: // --version
        	printVersion();
        	return 0;

    	case '?':
    	default:
    		printHelp();
        	return -1;
    	}
	}

	spdlog::set_level(log_level);

	if (dvr_template != NULL && video_framerate < 0 ) {
		printf("--dvr-framerate must be provided when dvr is enabled.\n");
		return 0;
	}

	printVersion();
	spdlog::info("disable_vsync: {}", disable_vsync);

    ////////////////////////////////////////////// DRM SETUP

	int drm_ret = setup_drm(print_modelist, mode_width, mode_height, mode_vrefresh, target_frame_rate);

	if (print_modelist) {
        remove(pidFilePath.c_str());
        return (drm_ret < 0) ? drm_ret : 0;
    }

    if (drm_ret <= 0) {
        spdlog::error("Failed to initialize DRM (code {})", drm_ret);
        remove(pidFilePath.c_str());
        return drm_ret;
    }

	////////////////////////////////////////////// SIGNAL SETUP

	signal(SIGINT, sig_handler);
	signal(SIGPIPE, sig_handler);
	if (dvr_template) {
		signal(SIGUSR1, sigusr1_handler);
	}
	signal(SIGUSR2, sigusr2_handler);

 	////////////////////////////////////////////// THREAD SETUP
	
	ret = pthread_mutex_init(&video_mutex, NULL);
	assert(!ret);
	ret = pthread_cond_init(&video_cond, NULL);
	assert(!ret);

	pthread_t tid_display, tid_osd, tid_mavlink, tid_dvr, tid_wfbcli;
	if (dvr_template != NULL) {
		dvr_thread_params args;
		args.filename_template = dvr_template;
		args.mp4_fragmentation_mode = mp4_fragmentation_mode;
		args.dvr_filenames_with_sequence = dvr_filenames_with_sequence;
		args.video_framerate = video_framerate;
		args.video_p.video_frm_width = output_list->video_frm_width;
		args.video_p.video_frm_height = output_list->video_frm_height;
		args.video_p.codec = codec;
		dvr = new Dvr(args);
		ret = pthread_create(&tid_dvr, NULL, &Dvr::__THREAD__, dvr);
	}
	ret = pthread_create(&tid_frame, NULL, __FRAME_THREAD__, NULL);
	assert(!ret);
	ret = pthread_create(&tid_display, NULL, __DISPLAY_THREAD__, NULL);
	assert(!ret);

    nlohmann::json osd_config{};
    if(osd_config_path != "") {
        std::ifstream f(osd_config_path);
        osd_config = nlohmann::json::parse(f);
    }
    if (mavlink_thread) {
        ret = pthread_create(&tid_mavlink, NULL, __MAVLINK_THREAD__, &signal_flag);
        assert(!ret);
    }
    if (wfb_port) {
        wfb_thread_params *wfb_args = (wfb_thread_params *)malloc(sizeof *wfb_args);
        wfb_args->port = wfb_port;
        ret = pthread_create(&tid_wfbcli, NULL, __WFB_CLI_THREAD__, wfb_args);
        assert(!ret);
    }

    osd_thread_params *args = (osd_thread_params *)malloc(sizeof *args);
    args->fd = drm_fd;
    args->out = output_list;
    args->config = osd_config;
    ret = pthread_create(&tid_osd, NULL, __OSD_THREAD__, args);
    assert(!ret);

	////////////////////////////////////////////// MAIN LOOP

	read_video_stream(packet, listen_port, unix_socket);

    ////////////////////////////////////////////// THREAD CLEANUP

	ret = pthread_join(tid_frame, NULL);
	assert(!ret);
	
	ret = pthread_mutex_lock(&video_mutex);
	assert(!ret);	
	ret = pthread_cond_signal(&video_cond);
	assert(!ret);	
	ret = pthread_mutex_unlock(&video_mutex);
	assert(!ret);	

	ret = pthread_join(tid_display, NULL);
	assert(!ret);	
	
	ret = pthread_cond_destroy(&video_cond);
	assert(!ret);
	ret = pthread_mutex_destroy(&video_mutex);
	assert(!ret);

	if (mavlink_thread) {
		ret = pthread_join(tid_mavlink, NULL);
		assert(!ret);
	}

    ret = pthread_join(tid_wfbcli, NULL);
    assert(!ret);
    ret = pthread_join(tid_osd, NULL);
    assert(!ret);

    if (dvr_template != NULL ){
		ret = pthread_join(tid_dvr, NULL);
		assert(!ret);
	}

	////////////////////////////////////////////// MPI CLEANUP

	cleanup_mpi(packet);

	////////////////////////////////////////////// DRM CLEANUP

	cleanup_drm();

    remove(pidFilePath.c_str());

	return 0;
}
