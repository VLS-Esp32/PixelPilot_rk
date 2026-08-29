#include "drm.h"

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include <cairo.h>
#include <pthread.h>
#include <rockchip/rk_mpi.h>
#include <assert.h>

// Header-name compatibility for older libdrm uapi headers.
#ifndef DRM_CLIENT_CAP_WRITEBACK_CONNECTORS
#define DRM_CLIENT_CAP_WRITEBACK_CONNECTORS 5
#endif
#ifndef DRM_MODE_CONNECTOR_WRITEBACK
#define DRM_MODE_CONNECTOR_WRITEBACK 18
#endif

int modeset_open(int *out, const char *node)
{
	int fd, ret;
	uint64_t cap;

	fd = open(node, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		ret = -errno;
		fprintf(stderr, "cannot open '%s': %m\n", node);
		return ret;
	}

	ret = drmSetClientCap(fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);
	if (ret) {
		fprintf(stderr, "failed to set universal planes cap, %d\n", ret);
		return ret;
	}

	ret = drmSetClientCap(fd, DRM_CLIENT_CAP_ATOMIC, 1);
	if (ret) {
		fprintf(stderr, "failed to set atomic cap, %d", ret);
		return ret;
	}

	// Opt in to writeback connectors — they are hidden from enumeration otherwise. Non-fatal:
	// if the kernel lacks writeback support, OSD-in-DVR simply records clean video (no OSD).
	if (drmSetClientCap(fd, DRM_CLIENT_CAP_WRITEBACK_CONNECTORS, 1) != 0) {
		fprintf(stdout, "writeback-connectors cap unavailable (DVR writeback capture disabled)\n");
	}

	if (drmGetCap(fd, DRM_CAP_DUMB_BUFFER, &cap) < 0 || !cap) {
		fprintf(stderr, "drm device '%s' does not support dumb buffers\n",
			node);
		close(fd);
		return -EOPNOTSUPP;
	}

	if (drmGetCap(fd, DRM_CAP_CRTC_IN_VBLANK_EVENT, &cap) < 0 || !cap) {
		fprintf(stderr, "drm device '%s' does not support atomic KMS\n",
			node);
		close(fd);
		return -EOPNOTSUPP;
	}

	*out = fd;
	return 0;
}


int64_t get_property_value(int fd, drmModeObjectPropertiesPtr props,
				  const char *name)
{
	drmModePropertyPtr prop;
	uint64_t value;
	bool found;
	int j;

	found = false;
	for (j = 0; j < props->count_props && !found; j++) {
		prop = drmModeGetProperty(fd, props->props[j]);
		if (!strcmp(prop->name, name)) {
			value = props->prop_values[j];
			found = true;
		}
		drmModeFreeProperty(prop);
	}

	if (!found)
		return -1;
	return value;
}


void modeset_get_object_properties(int fd, struct drm_object *obj,
					  uint32_t type)
{
	const char *type_str;
	unsigned int i;

	obj->props = drmModeObjectGetProperties(fd, obj->id, type);
	if (!obj->props) {
		switch(type) {
			case DRM_MODE_OBJECT_CONNECTOR:
				type_str = "connector";
				break;
			case DRM_MODE_OBJECT_PLANE:
				type_str = "plane";
				break;
			case DRM_MODE_OBJECT_CRTC:
				type_str = "CRTC";
				break;
			default:
				type_str = "unknown type";
				break;
		}
		fprintf(stderr, "cannot get %s %d properties: %s\n",
			type_str, obj->id, strerror(errno));
		return;
	}

	obj->props_info = calloc(obj->props->count_props, sizeof(obj->props_info));
	for (i = 0; i < obj->props->count_props; i++)
		obj->props_info[i] = drmModeGetProperty(fd, obj->props->props[i]);
}


int set_drm_object_property(drmModeAtomicReq *req, struct drm_object *obj,
				   const char *name, uint64_t value)
{
	int i;
	uint32_t prop_id = 0;
	for (i = 0; i < obj->props->count_props; i++) {
		if (!strcmp(obj->props_info[i]->name, name)) {
			prop_id = obj->props_info[i]->prop_id;
			break;
		}
	}

	if (prop_id == 0) {
		fprintf(stderr, "no object property: %s\n", name);
		return -EINVAL;
	}

	return drmModeAtomicAddProperty(req, obj->id, prop_id, value);
}


int modeset_find_crtc(int fd, drmModeRes *res, drmModeConnector *conn, struct modeset_output *out)
{
	drmModeEncoder *enc;
	unsigned int i, j;
	uint32_t crtc;

	if (conn->encoder_id)
		enc = drmModeGetEncoder(fd, conn->encoder_id);
	else
		enc = NULL;

	if (enc) {
		if (enc->crtc_id) {
			crtc = enc->crtc_id;
			if (crtc > 0) {
				drmModeFreeEncoder(enc);
				out->crtc.id = crtc;
				out->saved_crtc = drmModeGetCrtc(fd, crtc);
				for (i = 0; i < res->count_crtcs; ++i) {
					if (res->crtcs[i] == crtc) {
						out->crtc_index = i;
						break;
					}
				}
				return 0;
			}
		}

		drmModeFreeEncoder(enc);
	}

	for (i = 0; i < conn->count_encoders; ++i) {
		enc = drmModeGetEncoder(fd, conn->encoders[i]);
		if (!enc) {
			fprintf(stderr, "cannot retrieve encoder %u:%u (%d): %m\n",
				i, conn->encoders[i], errno);
			continue;
		}

		for (j = 0; j < res->count_crtcs; ++j) {
			if (!(enc->possible_crtcs & (1 << j)))
				continue;

			crtc = res->crtcs[j];

			if (crtc > 0) {
				out->saved_crtc = drmModeGetCrtc(fd, crtc);
				fprintf(stdout, "crtc %u found for encoder %u, will need full modeset\n",
					crtc, conn->encoders[i]);;
				drmModeFreeEncoder(enc);
				out->crtc.id = crtc;
				out->crtc_index = j;
				return 0;
			}
		}

		drmModeFreeEncoder(enc);
	}

	fprintf(stderr, "cannot find suitable crtc for connector %u\n",
		conn->connector_id);
	return -ENOENT;
}

int modeset_find_plane(int fd, struct modeset_output *out, struct drm_object *plane_out, uint32_t plane_format)
{
	drmModePlaneResPtr plane_res;
	bool found_plane = false;
	int i, ret = -EINVAL;

	plane_res = drmModeGetPlaneResources(fd);
	if (!plane_res) {
		fprintf(stderr, "drmModeGetPlaneResources failed: %s\n",
				strerror(errno));
		return -ENOENT;
	}

	for (i = 0; (i < plane_res->count_planes) && !found_plane; i++) {
		int plane_id = plane_res->planes[i];

		drmModePlanePtr plane = drmModeGetPlane(fd, plane_id);
		if (!plane) {
			fprintf(stderr, "drmModeGetPlane(%u) failed: %s\n", plane_id,
					strerror(errno));
			continue;
		}

		if (plane->possible_crtcs & (1 << out->crtc_index)) {
			for (int j=0; j<plane->count_formats; j++) {
				if (plane->formats[j] ==  plane_format) {
					found_plane = true;
				 	plane_out->id = plane_id;
				 	ret = 0;
					break;
				}
			}
		}

		drmModeFreePlane(plane);
	}

	drmModeFreePlaneResources(plane_res);

	return ret;
}


void modeset_drm_object_fini(struct drm_object *obj)
{
	for (int i = 0; i < obj->props->count_props; i++)
		drmModeFreeProperty(obj->props_info[i]);
	free(obj->props_info);
	drmModeFreeObjectProperties(obj->props);
}


int modeset_setup_objects(int fd, struct modeset_output *out)
{
	struct drm_object *connector = &out->connector;
	struct drm_object *crtc = &out->crtc;
	struct drm_object *plane_video = &out->video_plane;
	struct drm_object *plane_osd = &out->osd_plane;

	modeset_get_object_properties(fd, connector, DRM_MODE_OBJECT_CONNECTOR);
	if (!connector->props)
		goto out_conn;

	modeset_get_object_properties(fd, crtc, DRM_MODE_OBJECT_CRTC);
	if (!crtc->props)
		goto out_crtc;

	modeset_get_object_properties(fd, plane_video, DRM_MODE_OBJECT_PLANE);
	if (!plane_video->props)
		goto out_plane;
	modeset_get_object_properties(fd, plane_osd, DRM_MODE_OBJECT_PLANE);
	if (!plane_osd->props)
		goto out_plane;
	return 0;

out_plane:
	modeset_drm_object_fini(crtc);
out_crtc:
	modeset_drm_object_fini(connector);
out_conn:
	return -ENOMEM;
}


void modeset_destroy_objects(int fd, struct modeset_output *out)
{
	modeset_drm_object_fini(&out->connector);
	modeset_drm_object_fini(&out->crtc);
	modeset_drm_object_fini(&out->video_plane);
	modeset_drm_object_fini(&out->osd_plane);
	if (out->wb_connector.props)
		modeset_drm_object_fini(&out->wb_connector);
}


int modeset_create_fb(int fd, struct modeset_buf *buf)
{
	struct drm_mode_create_dumb creq;
	struct drm_mode_destroy_dumb dreq;
	struct drm_mode_map_dumb mreq;
	int ret;
	uint32_t handles[4] = {0}, pitches[4] = {0}, offsets[4] = {0};

	buf->prime_fd = -1;
	memset(&creq, 0, sizeof(creq));
	creq.width = buf->width;
	creq.height = buf->height;
	creq.bpp = 32;
	ret = drmIoctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq);
	if (ret < 0) {
		fprintf(stderr, "cannot create buffer (%d): %m\n",
			errno);
		return -errno;
	}
	buf->stride = creq.pitch;
	buf->size = creq.size;
	buf->handle = creq.handle;

	handles[0] = buf->handle;
	pitches[0] = buf->stride;

	ret = drmModeAddFB2(fd, buf->width, buf->height, DRM_FORMAT_ARGB8888,
			    handles, pitches, offsets, &buf->fb, 0);
	if (ret) {
		fprintf(stderr, "cannot create framebuffer (%d): %m\n",
			errno);
		ret = -errno;
		goto err_destroy;
	}

	memset(&mreq, 0, sizeof(mreq));
	mreq.handle = buf->handle;
	ret = drmIoctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq);
	if (ret) {
		fprintf(stderr, "cannot map buffer (%d): %m\n",
			errno);
		ret = -errno;
		goto err_fb;
	}

	buf->map = mmap(0, buf->size, PROT_READ | PROT_WRITE, MAP_SHARED,
		        fd, mreq.offset);
	if (buf->map == MAP_FAILED) {
		fprintf(stderr, "cannot mmap buffer (%d): %m\n",
			errno);
		ret = -errno;
		goto err_fb;
	}

	ret = drmPrimeHandleToFD(fd, buf->handle, DRM_CLOEXEC | DRM_RDWR, &buf->prime_fd);
	if (ret) {
		fprintf(stderr, "cannot export prime fd (%d): %m\n", errno);
		ret = -errno;
		goto err_unmap;
	}

	memset(buf->map, 0, buf->size);

	return 0;

err_unmap:
	munmap(buf->map, buf->size);
	buf->map = NULL;
err_fb:
	drmModeRmFB(fd, buf->fb);
err_destroy:
	memset(&dreq, 0, sizeof(dreq));
	dreq.handle = buf->handle;
	drmIoctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
	return ret;
}


void modeset_destroy_fb(int fd, struct modeset_buf *buf)
{
	struct drm_mode_destroy_dumb dreq;

	if (buf->prime_fd >= 0) {
		close(buf->prime_fd);
		buf->prime_fd = -1;
	}

	munmap(buf->map, buf->size);

	drmModeRmFB(fd, buf->fb);

	memset(&dreq, 0, sizeof(dreq));
	dreq.handle = buf->handle;
	drmIoctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
}


int modeset_find_writeback(int fd, struct modeset_output *out)
{
	out->wb_available = false;
    out->wb_enabled = false;
	out->wb_connector.props = NULL;

	drmModeRes *res = drmModeGetResources(fd);
	if (!res) {
		fprintf(stderr, "modeset_find_writeback: cannot get resources: %m\n");
		return -ENOENT;
	}

	for (int i = 0; i < res->count_connectors && !out->wb_available; i++) {
		drmModeConnector *conn = drmModeGetConnector(fd, res->connectors[i]);
		if (!conn)
			continue;
		if (conn->connector_type != DRM_MODE_CONNECTOR_WRITEBACK) {
			drmModeFreeConnector(conn);
			continue;
		}

		// The writeback connector's encoder must be able to drive our CRTC.
		bool crtc_ok = false;
		for (int e = 0; e < conn->count_encoders && !crtc_ok; e++) {
			drmModeEncoder *enc = drmModeGetEncoder(fd, conn->encoders[e]);
			if (!enc)
				continue;
			if (enc->possible_crtcs & (1 << out->crtc_index))
				crtc_ok = true;
			drmModeFreeEncoder(enc);
		}
		if (!crtc_ok) {
			drmModeFreeConnector(conn);
			continue;
		}

		out->wb_connector.id = conn->connector_id;
		modeset_get_object_properties(fd, &out->wb_connector, DRM_MODE_OBJECT_CONNECTOR);
		if (!out->wb_connector.props) {
			drmModeFreeConnector(conn);
			continue;
		}

		int64_t fb_prop    = get_property_value(fd, out->wb_connector.props, "WRITEBACK_FB_ID");
		int64_t fence_prop = get_property_value(fd, out->wb_connector.props, "WRITEBACK_OUT_FENCE_PTR");
		int64_t fmts_blob  = get_property_value(fd, out->wb_connector.props, "WRITEBACK_PIXEL_FORMATS");
		if (fb_prop < 0 || fence_prop < 0) {
			modeset_drm_object_fini(&out->wb_connector);
			out->wb_connector.props = NULL;
			drmModeFreeConnector(conn);
			continue;
		}

        bool has_nv12 = false;
		if (fmts_blob > 0) {
			drmModePropertyBlobPtr blob = drmModeGetPropertyBlob(fd, (uint32_t)fmts_blob);
			if (blob && blob->data) {
				const uint32_t *fmts = (const uint32_t *)blob->data;
				int n = blob->length / sizeof(uint32_t);
                for (int k = 0; k < n; k++)
                    if (fmts[k] == DRM_FORMAT_NV12) { has_nv12 = true; break; }
			}
			if (blob)
				drmModeFreePropertyBlob(blob);
		}
        if (!has_nv12) {
			// Do not guess a format the connector did not advertise: the commit would fail later
			// with an opaque EINVAL instead of here, where the cause is obvious.
            fprintf(stdout, "Writeback: connector %u does not advertise NV12\n",
				conn->connector_id);
			modeset_drm_object_fini(&out->wb_connector);
			out->wb_connector.props = NULL;
			drmModeFreeConnector(conn);
			continue;
		}
		out->wb_available = true;

        fprintf(stdout, "Writeback connector %u available for CRTC %u, output format NV12\n",
            conn->connector_id, out->crtc.id);
		drmModeFreeConnector(conn);
	}

	drmModeFreeResources(res);
	if (!out->wb_available)
		fprintf(stdout, "No usable DRM writeback connector for CRTC %u\n", out->crtc.id);
	return out->wb_available ? 0 : -ENOENT;
}

int modeset_create_wb_fb(int fd, struct modeset_buf *buf)
{
	struct drm_mode_create_dumb creq;
	struct drm_mode_destroy_dumb dreq;
	int ret;
	uint32_t handles[4] = {0}, pitches[4] = {0}, offsets[4] = {0};

	buf->prime_fd = -1;
	buf->map = NULL;

	// Pad the vertical stride to 16 so the MPP encoder (which aligns ver_stride) reads the CbCr
	// plane / trailing rows from the same offset the buffer is laid out at. The FB is added at the
	// true display height; the VOP writes only those rows.
	uint32_t ver = (buf->height + 15) & ~15u;

    memset(&creq, 0, sizeof(creq));
    creq.width  = buf->width;
    creq.height = ver * 3 / 2;
    creq.bpp    = 8;
    ret = drmIoctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq);
    if (ret < 0) {
        fprintf(stderr, "cannot create NV12 wb buffer (%d): %m\n", errno);
        return -errno;
    }
    buf->stride = creq.pitch;
    buf->size   = creq.size;
    buf->handle = creq.handle;

    handles[0] = buf->handle; pitches[0] = buf->stride; offsets[0] = 0;
    handles[1] = buf->handle; pitches[1] = buf->stride; offsets[1] = buf->stride * ver;
    ret = drmModeAddFB2(fd, buf->width, buf->height, DRM_FORMAT_NV12,
                handles, pitches, offsets, &buf->fb, 0);

	if (ret) {
		fprintf(stderr, "cannot create wb framebuffer (%d): %m\n", errno);
		ret = -errno;
		goto err_destroy;
	}

	ret = drmPrimeHandleToFD(fd, buf->handle, DRM_CLOEXEC | DRM_RDWR, &buf->prime_fd);
	if (ret) {
		fprintf(stderr, "cannot export wb prime fd (%d): %m\n", errno);
		ret = -errno;
		goto err_fb;
	}
	return 0;

err_fb:
	drmModeRmFB(fd, buf->fb);
err_destroy:
	memset(&dreq, 0, sizeof(dreq));
	dreq.handle = buf->handle;
	drmIoctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
	return ret;
}

void modeset_destroy_wb_fb(int fd, struct modeset_buf *buf)
{
	struct drm_mode_destroy_dumb dreq;

	if (buf->prime_fd >= 0) {
		close(buf->prime_fd);
		buf->prime_fd = -1;
	}
	if (buf->fb)
		drmModeRmFB(fd, buf->fb);
	if (buf->handle) {
		memset(&dreq, 0, sizeof(dreq));
		dreq.handle = buf->handle;
		drmIoctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
	}
}

int modeset_arm_writeback(struct modeset_output *out, drmModeAtomicReq *req, uint32_t wb_fb, int32_t *out_fence_ptr)
{
    if (!out->wb_enabled)
		return -1;
	if (set_drm_object_property(req, &out->wb_connector, "WRITEBACK_FB_ID", wb_fb) < 0)
		return -1;
	if (set_drm_object_property(req, &out->wb_connector, "WRITEBACK_OUT_FENCE_PTR",
				    (uint64_t)(uintptr_t)out_fence_ptr) < 0)
		return -1;
	return 0;
}

int modeset_attach_writeback(int fd, struct modeset_output *out)
{
    drmModeAtomicReq *req;
    struct modeset_buf *buf;
    int64_t zpos;
    int ret;

    if (!out->wb_available)
        return -ENODEV;
    if (out->wb_enabled)
        return 0;

    out->wb_enabled = true;

    req = drmModeAtomicAlloc();
    if (!req) {
        out->wb_enabled = false;
        return -ENOMEM;
    }

    buf = &out->osd_bufs[0];
    zpos = get_property_value(fd, out->osd_plane.props, "zpos");
    ret = modeset_perform_modeset(fd, out, req, &out->osd_plane, buf->fb,
                        buf->width, buf->height, (int)zpos);
    drmModeAtomicFree(req);

    if (ret < 0) {
        fprintf(stderr, "writeback attach modeset failed (%d): %m\n", ret);
        out->wb_enabled = false;
        return ret;
    }
    fprintf(stdout, "Writeback connector %u attached to CRTC %u (persistent)\n",
        out->wb_connector.id, out->crtc.id);
    return 0;
}

void modeset_detach_writeback(int fd, struct modeset_output *out)
{
    drmModeAtomicReq *req;

    if (!out->wb_available)
        return;

    out->wb_enabled = false;

    req = drmModeAtomicAlloc();
    if (!req)
        return;
    if (set_drm_object_property(req, &out->wb_connector, "CRTC_ID", 0) > 0) {
        if (drmModeAtomicCommit(fd, req, DRM_MODE_ATOMIC_ALLOW_MODESET, NULL) < 0)
            fprintf(stderr, "writeback detach commit failed: %m\n");
    }
    drmModeAtomicFree(req);
}

int modeset_check_writeback(int fd, struct modeset_output *out, uint32_t wb_fb)
{
    drmModeAtomicReq *req;
    int32_t dummy_fence = -1;
    int ret;

    req = drmModeAtomicAlloc();
    if (!req)
        return -ENOMEM;
    if (set_drm_object_property(req, &out->osd_plane, "FB_ID", out->osd_bufs[0].fb) < 0) {
        drmModeAtomicFree(req);
        return -EINVAL;
    }
    ret = drmModeAtomicCommit(fd, req, DRM_MODE_ATOMIC_TEST_ONLY, NULL);
    drmModeAtomicFree(req);
    if (ret < 0) {
        fprintf(stderr, "writeback idle-commit check failed (%d): %m - this kernel appears to "
            "require a writeback job on every commit\n", ret);
        return ret;
	}

    req = drmModeAtomicAlloc();
    if (!req)
        return -ENOMEM;
    if (set_drm_object_property(req, &out->osd_plane, "FB_ID", out->osd_bufs[0].fb) < 0 ||
        modeset_arm_writeback(out, req, wb_fb, &dummy_fence) < 0) {
        drmModeAtomicFree(req);
        return -EINVAL;
    }
    ret = drmModeAtomicCommit(fd, req, DRM_MODE_ATOMIC_TEST_ONLY, NULL);
    drmModeAtomicFree(req);
    if (ret < 0) {
        fprintf(stderr, "writeback capture-commit check failed (%d): %m\n", ret);
        return ret;
    }
    return 0;
}

int modeset_setup_framebuffers(int fd, drmModeConnector *conn, struct modeset_output *out)
{
	for (int i=0; i<OSD_BUF_COUNT; i++) {
		out->osd_bufs[i].width = out->mode.hdisplay;
		out->osd_bufs[i].height = out->mode.vdisplay;
		int ret = modeset_create_fb(fd, &out->osd_bufs[i]);
		if (ret) {
			return ret;
		}
	}
	out->video_crtc_width = out->mode.hdisplay;
	out->video_crtc_height = out->mode.vdisplay;
	return 0;
}


void modeset_output_destroy(int fd, struct modeset_output *out)
{
	modeset_destroy_objects(fd, out);

	for (int i=0; i<OSD_BUF_COUNT; i++) { 
		modeset_destroy_fb(fd, &out->osd_bufs[i]);
	}
	drmModeDestroyPropertyBlob(fd, out->mode_blob_id);
	free(out);
}

int select_best_mode(drmModeConnector *conn, uint32_t desired_refresh) {
    const uint32_t max_width = 1920;
    const uint32_t max_height = 1080;

    int best_index = -1;
    uint32_t best_width = 0;
    uint32_t best_height = 0;
    uint32_t best_refresh = 0;

    for (int i = 0; i < conn->count_modes; i++) {
        drmModeModeInfo *mode = &conn->modes[i];
        
        uint32_t refresh = mode->vrefresh;
        
        if (mode->hdisplay > max_width || mode->vdisplay > max_height) {
            continue;
        }

        if (desired_refresh == 0 && refresh % 60 != 0) {
            continue;
        }
        else if (desired_refresh != 0 && refresh != desired_refresh) {
            continue;
        }

        uint32_t pixels = mode->hdisplay * mode->vdisplay;
        uint32_t best_pixels = best_width * best_height;

        if (pixels > best_pixels || (pixels == best_pixels && refresh > best_refresh)) {
            best_index = i;
            best_width = mode->hdisplay;
            best_height = mode->vdisplay;
            best_refresh = refresh;
        }
    }

    return best_index;
}

struct modeset_output *modeset_output_create(int fd, drmModeRes *res, drmModeConnector *conn, uint16_t mode_width, uint16_t mode_height, uint32_t mode_vrefresh, uint32_t target_frame_rate)
{
	int ret;
	struct modeset_output *out;

	out = malloc(sizeof(*out));
	memset(out, 0, sizeof(*out));
	out->connector.id = conn->connector_id;

	if (conn->connection != DRM_MODE_CONNECTED) {
		fprintf(stderr, "ignoring unused connector %u\n",
			conn->connector_id);
		goto out_error;
	}

	if (conn->count_modes == 0) {
		fprintf(stderr, "no valid mode for connector %u\n",
			conn->connector_id);
		goto out_error;
	}

	int fc = 0;
	int preferred_fc = -1;
	if (mode_width>0 && mode_height>0 && mode_vrefresh>0) {
		fc = -1;
		printf( "Available modes:\n");
		for (int i = 0; i < conn->count_modes; i++ ) {
			printf( "%d : %dx%d@%d\n",i, conn->modes[i].hdisplay, conn->modes[i].vdisplay , conn->modes[i].vrefresh );
			if (conn->modes[i].hdisplay == mode_width &&
			conn->modes[i].vdisplay == mode_height &&
			conn->modes[i].vrefresh == mode_vrefresh
			) {
				if (fc < 0)
					fc = i;
				if (fc >= 0 && (conn->modes[i].flags & DRM_MODE_FLAG_INTERLACE) == 0) // prefer progressive modes
					fc = i;
			} else if (conn->modes[i].type & DRM_MODE_TYPE_PREFERRED) {
				preferred_fc = i;
			}
		}
		if (fc < 0  && preferred_fc < 0) {
			fprintf(stderr, "couldn't find a matching mode for %dx%d@%d\n", mode_width , mode_height , mode_vrefresh);
			goto out_error;
		} else if (fc < 0  && preferred_fc >= 0) {
			fprintf(stderr, "couldn't find a matching mode, useing preferred mode %dx%d@%d\n", conn->modes[preferred_fc].hdisplay, conn->modes[preferred_fc].vdisplay , conn->modes[preferred_fc].vrefresh);
			fc = preferred_fc;
		}
		printf( "Using screen mode %dx%d@%d\n",conn->modes[fc].hdisplay, conn->modes[fc].vdisplay , conn->modes[fc].vrefresh );
    }
    else if (target_frame_rate > 0) {
        int idx = 0;
        idx = select_best_mode(conn, target_frame_rate);
        if (idx >= 0) {
            printf("Desired target refresh rate %d Hz. Selected mode index %d : %dx%d@%d\n", 
               target_frame_rate, idx, conn->modes[idx].hdisplay, conn->modes[idx].vdisplay , conn->modes[idx].vrefresh);
            fc = idx;
        }
    }
    else if (mode_width == 0 && mode_height == 0 && mode_vrefresh == 0) {
        int idx = 0;
        idx = select_best_mode(conn, 0);
        if (idx >= 0) {
            printf("Selected mode index %d : %dx%d@%d\n", 
                  idx, conn->modes[idx].hdisplay, conn->modes[idx].vdisplay , conn->modes[idx].vrefresh);
            fc = idx;
        }
    }
	memcpy(&out->mode, &conn->modes[fc], sizeof(out->mode));
	if (drmModeCreatePropertyBlob(fd, &out->mode, sizeof(out->mode), &out->mode_blob_id) != 0) {
		fprintf(stderr, "couldn't create a blob property\n");
		goto out_error;
	}

	ret = modeset_find_crtc(fd, res, conn, out);
	if (ret) {
		fprintf(stderr, "no valid crtc for connector %u\n", conn->connector_id);
		goto out_blob;
	}

	ret = modeset_find_plane(fd, out, &out->video_plane, DRM_FORMAT_NV12);
	if (ret) {
		fprintf(stderr, "no valid video plane with format NV12 for crtc %u\n", out->crtc.id);
		goto out_blob;
	}
	fprintf(stdout, "Using plane %d (NV12) for Video\n",  out->video_plane.id);

	ret = modeset_find_plane(fd, out, &out->osd_plane, DRM_FORMAT_ARGB8888);
	if (ret) {
		fprintf(stderr, "no valid osd plane with format ARGB8888 for crtc %u\n", out->crtc.id);
		goto out_blob;
	}
	fprintf(stdout, "Using plane %d (ARGB8888) for OSD\n",  out->osd_plane.id);

	ret = modeset_setup_objects(fd, out);
	if (ret) {
		fprintf(stderr, "cannot get plane properties\n");
		goto out_blob;
	}

	ret = modeset_setup_framebuffers(fd, conn, out);
	if (ret) {
		fprintf(stderr, "cannot create framebuffers for connector %u\n",
			conn->connector_id);
		goto out_obj;
	}

	out->video_request = drmModeAtomicAlloc();
	assert(out->video_request);
	out->osd_request = drmModeAtomicAlloc();
	assert(out->video_request);

	return out;

out_obj:
	modeset_destroy_objects(fd, out);
out_blob:
	drmModeDestroyPropertyBlob(fd, out->mode_blob_id);
out_error:
	free(out);
	return NULL;
}

void *modeset_print_modes(int fd)
{
	drmModeRes *res;
	drmModeConnector *conn;
	drmModeModeInfo info;
	uint prev_h, prev_v, prev_refresh = 0;
	int at_least_one = 0;
	res = drmModeGetResources(fd);
	if (!res) {
		fprintf(stderr, "cannot retrieve DRM resources (%d): %m\n",
			errno);
		return NULL;
	}

	for (int i = 0; i < res->count_connectors; ++i) {
		conn = drmModeGetConnector(fd, res->connectors[i]);
		if (!conn) {
			fprintf(stderr, "cannot retrieve DRM connector %u:%u (%d): %m\n",
				i, res->connectors[i], errno);
			continue;
		}
		for (int i = 0; i < conn->count_modes; i++ ) {
			info = conn->modes[i];
			// Assuming modes list is sorted
			if (info.hdisplay == prev_h && info.vdisplay == prev_v && info.vrefresh == prev_refresh)
				continue;
			printf("%dx%d@%d\n", info.hdisplay, info.vdisplay, info.vrefresh);
			prev_h = info.hdisplay;
			prev_v = info.vdisplay;
			prev_refresh = info.vrefresh;
			at_least_one = 1;
		}
		drmModeFreeConnector(conn);
	}
	if (!at_least_one) {
		fprintf(stderr, "No displays found\n");
	}
	drmModeFreeResources(res);
	return NULL;

}

struct modeset_output *modeset_prepare(int fd, uint16_t mode_width, uint16_t mode_height, uint32_t mode_vrefresh, uint32_t target_frame_rate)
{
	drmModeRes *res;
	drmModeConnector *conn;
	unsigned int i;
	struct modeset_output *out;

	res = drmModeGetResources(fd);
	if (!res) {
		fprintf(stderr, "cannot retrieve DRM resources (%d): %m\n",
			errno);
		return NULL;
	}

	for (i = 0; i < res->count_connectors; ++i) {
		conn = drmModeGetConnector(fd, res->connectors[i]);
		if (!conn) {
			fprintf(stderr, "cannot retrieve DRM connector %u:%u (%d): %m\n",
				i, res->connectors[i], errno);
			continue;
		}

		out = modeset_output_create(fd, res, conn, mode_width, mode_height, mode_vrefresh, target_frame_rate);
		drmModeFreeConnector(conn);
		if (out) {
			drmModeFreeResources(res);
			return out;
		}
	}
	fprintf(stderr, "couldn't create any outputs\n");
	drmModeFreeResources(res);
	return NULL;
}

int modeset_perform_modeset(int fd, struct modeset_output *out, drmModeAtomicReq * req, struct drm_object *plane, int fb_id, uint32_t width, uint32_t height, int zpos)
{
	int ret, flags;

	ret = modeset_atomic_prepare_commit(fd, out, req, plane, fb_id, width, height, zpos);
	if (ret < 0) {
		fprintf(stderr, "prepare atomic commit failed for plane %d: %m\n", plane->id);
		return ret;
	}

	/* perform test-only atomic commit */
	flags = DRM_MODE_ATOMIC_TEST_ONLY | DRM_MODE_ATOMIC_ALLOW_MODESET;
	ret = drmModeAtomicCommit(fd, req, flags, NULL);
	if (ret < 0) {
		fprintf(stderr, "test-only atomic commit failed for plane %d: %m\n", plane->id);
		return ret;
	}

	/* initial modeset on all outputs */
	flags = DRM_MODE_ATOMIC_ALLOW_MODESET;
	ret = drmModeAtomicCommit(fd, req, flags, NULL);
	if (ret < 0)
		fprintf(stderr, "modeset atomic commit failed for plane %d: %m\n", plane->id);

	return ret;
}


int modeset_atomic_prepare_commit(int fd, struct modeset_output *out, drmModeAtomicReq *req, struct drm_object *plane, 
	int fb_id, uint32_t width, uint32_t height, int zpos)
{
	if (set_drm_object_property(req, &out->connector, "CRTC_ID", out->crtc.id) < 0)
		return -1;
	if (out->wb_enabled) {
		if (set_drm_object_property(req, &out->wb_connector, "CRTC_ID", out->crtc.id) < 0)
			return -1;
	}
	if (set_drm_object_property(req, &out->crtc, "MODE_ID", out->mode_blob_id) < 0)
		return -1;
	if (set_drm_object_property(req, &out->crtc, "ACTIVE", 1) < 0)
		return -1;
	if (set_drm_object_property(req, plane, "FB_ID", fb_id) < 0)
		return -1;
	if (set_drm_object_property(req, plane, "CRTC_ID", out->crtc.id) < 0)
		return -1;
	if (set_drm_object_property(req, plane, "SRC_X", 0) < 0)
		return -1;
	if (set_drm_object_property(req, plane, "SRC_Y", 0) < 0)
		return -1;
	if (set_drm_object_property(req, plane, "SRC_W", width << 16) < 0)
		return -1;
	if (set_drm_object_property(req, plane, "SRC_H", height << 16) < 0)
		return -1;

	uint32_t crtcw =  out->video_crtc_width;
	uint32_t crtch = out->video_crtc_height;
	float video_ratio = (float)width/height;
	if (crtcw / video_ratio > crtch) {
		crtcw = crtch * video_ratio;
		crtch = crtch;
	} else {
		crtcw = crtcw;
		crtch = crtcw / video_ratio;
	}
	int crtcx = (out->video_crtc_width - crtcw) / 2;
	int crtcy = (out->video_crtc_height - crtch) / 2;
	if (set_drm_object_property(req, plane, "CRTC_X", crtcx) < 0)
		return -1;
	if (set_drm_object_property(req, plane, "CRTC_Y", crtcy) < 0)
		return -1;
	if (set_drm_object_property(req, plane, "CRTC_W", crtcw) < 0)
		return -1;
	if (set_drm_object_property(req, plane, "CRTC_H", crtch) < 0)
		return -1;
	if (set_drm_object_property(req, plane, "zpos", zpos) < 0)
		return -1;

	return 0;
}

void restore_planes_zpos(int fd, struct modeset_output *output_list) {
	// restore osd zpos
	int ret, flags;
	struct modeset_buf *buf = &output_list->osd_bufs[0];

	// Start from empty requests. These objects are the ones the display/OSD threads were filling
	// per frame, and they still carry every property from the last frame those threads built - for
	// video_request that can include WRITEBACK_FB_ID naming a writeback buffer that has since been
	// removed by cleanup_writeback(), which makes the commit below fail with EINVAL.
	drmModeAtomicFree(output_list->osd_request);
	output_list->osd_request = drmModeAtomicAlloc();
	drmModeAtomicFree(output_list->video_request);
	output_list->video_request = drmModeAtomicAlloc();
	if (!output_list->osd_request || !output_list->video_request) {
		fprintf(stderr, "restore_planes_zpos: cannot allocate atomic requests\n");
		return;
	}

	// TODO(geehe) Find a more elegant way to do this.
	int64_t zpos = get_property_value(fd, output_list->osd_plane.props, "zpos");
	ret = modeset_atomic_prepare_commit(fd, output_list, output_list->osd_request, &output_list->osd_plane, buf->fb, buf->width, buf->height, zpos);
	if (ret < 0) {
		fprintf(stderr, "prepare atomic commit failed for plane %d, %m\n", output_list->osd_plane.id);
		return;
	}
	ret = drmModeAtomicCommit(fd, output_list->osd_request, DRM_MODE_ATOMIC_ALLOW_MODESET, NULL);
	if (ret < 0) 
		fprintf(stderr, "modeset atomic commit failed for plane %d, %m\n", output_list->osd_plane.id);

	zpos = get_property_value(fd, output_list->video_plane.props, "zpos");
	ret = modeset_atomic_prepare_commit(fd, output_list, output_list->video_request, &output_list->video_plane, buf->fb, buf->width, buf->height, zpos);
	if (ret < 0) {
		fprintf(stderr, "prepare atomic commit failed for plane %d, %m\n", output_list->video_plane.id);
		return;
	}
	ret = drmModeAtomicCommit(fd, output_list->video_request, DRM_MODE_ATOMIC_ALLOW_MODESET, NULL);
	if (ret < 0) 
		fprintf(stderr, "modeset atomic commit failed for plane %d, %m\n", output_list->video_plane.id);
}

void modeset_cleanup(int fd, struct modeset_output *output_list)
{
	modeset_output_destroy(fd, output_list);
}
