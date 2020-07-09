/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "mdp4_kms.h"

#define DOWN_SCALE_MAX	8
#define UP_SCALE_MAX	8

struct mdp4_plane {
	struct drm_plane base;
	const char *name;

	enum mdp4_pipe pipe;

	uint32_t nformats;
	uint32_t formats[32];

	bool enabled;
};
#define to_mdp4_plane(x) container_of(x, struct mdp4_plane, base)

static struct mdp4_kms *get_kms(struct drm_plane *plane)
{
	struct msm_drm_private *priv = plane->dev->dev_private;
	return to_mdp4_kms(to_mdp_kms(priv->kms));
}

static int mdp4_plane_update(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb,
		int crtc_x, int crtc_y,
		unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y,
		uint32_t src_w, uint32_t src_h)
{
	struct mdp4_plane *mdp4_plane = to_mdp4_plane(plane);

	mdp4_plane->enabled = true;

	if (plane->fb)
		drm_framebuffer_unreference(plane->fb);

	drm_framebuffer_reference(fb);

	return mdp4_plane_mode_set(plane, crtc, fb,
			crtc_x, crtc_y, crtc_w, crtc_h,
			src_x, src_y, src_w, src_h);
}

static int mdp4_plane_disable(struct drm_plane *plane)
{
	struct mdp4_plane *mdp4_plane = to_mdp4_plane(plane);
	DBG("%s: disable", mdp4_plane->name);
	if (plane->crtc)
		mdp4_crtc_detach(plane->crtc, plane);
	return 0;
}

static void mdp4_plane_destroy(struct drm_plane *plane)
{
	struct mdp4_plane *mdp4_plane = to_mdp4_plane(plane);

	mdp4_plane_disable(plane);
	drm_plane_cleanup(plane);

	kfree(mdp4_plane);
}

/* helper to install properties which are common to planes and crtcs */
void mdp4_plane_install_properties(struct drm_plane *plane,
		struct drm_mode_object *obj)
{
	// XXX
}

int mdp4_plane_set_property(struct drm_plane *plane,
		struct drm_property *property, uint64_t val)
{
	// XXX
	return -EINVAL;
}

static const struct drm_plane_funcs mdp4_plane_funcs = {
		.update_plane = mdp4_plane_update,
		.disable_plane = mdp4_plane_disable,
		.destroy = mdp4_plane_destroy,
		.set_property = mdp4_plane_set_property,
};

void mdp4_plane_set_scanout(struct drm_plane *plane,
		struct drm_framebuffer *fb)
{
	struct mdp4_plane *mdp4_plane = to_mdp4_plane(plane);
	struct mdp4_kms *mdp4_kms = get_kms(plane);
	enum mdp4_pipe pipe = mdp4_plane->pipe;
	uint32_t iova;

	mdp4_write(mdp4_kms, REG_MDP4_PIPE_SRC_STRIDE_A(pipe),
			MDP4_PIPE_SRC_STRIDE_A_P0(fb->pitches[0]) |
			MDP4_PIPE_SRC_STRIDE_A_P1(fb->pitches[1]));

	mdp4_write(mdp4_kms, REG_MDP4_PIPE_SRC_STRIDE_B(pipe),
			MDP4_PIPE_SRC_STRIDE_B_P2(fb->pitches[2]) |
			MDP4_PIPE_SRC_STRIDE_B_P3(fb->pitches[3]));

	msm_gem_get_iova(msm_framebuffer_bo(fb, 0), mdp4_kms->id, &iova);
	mdp4_write(mdp4_kms, REG_MDP4_PIPE_SRCP0_BASE(pipe), iova);

	plane->fb = fb;
}

#define MDP4_VG_PHASE_STEP_DEFAULT	0x20000000

int mdp4_plane_mode_set(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb,
		int crtc_x, int crtc_y,
		unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y,
		uint32_t src_w, uint32_t src_h)
{
	struct drm_device *dev = plane->dev;
	struct mdp4_plane *mdp4_plane = to_mdp4_plane(plane);
	struct mdp4_kms *mdp4_kms = get_kms(plane);
	enum mdp4_pipe pipe = mdp4_plane->pipe;
	const struct mdp_format *format;
	uint32_t op_mode = 0;
	uint32_t phasex_step = MDP4_VG_PHASE_STEP_DEFAULT;
	uint32_t phasey_step = MDP4_VG_PHASE_STEP_DEFAULT;

	/* src values are in Q16 fixed point, convert to integer: */
	src_x = src_x >> 16;
	src_y = src_y >> 16;
	src_w = src_w >> 16;
	src_h = src_h >> 16;

	DBG("%s: FB[%u] %u,%u,%u,%u -> CRTC[%u] %d,%d,%u,%u", mdp4_plane->name,
			fb->base.id, src_x, src_y, src_w, src_h,
			crtc->base.id, crtc_x, crtc_y, crtc_w, crtc_h);

	format = to_mdp_format(msm_framebuffer_format(fb));

	if (src_w > (crtc_w * DOWN_SCALE_MAX)) {
		dev_err(dev->dev, "Width down scaling exceeds limits!\n");
		return -ERANGE;
	}

	if (src_h > (crtc_h * DOWN_SCALE_MAX)) {
		dev_err(dev->dev, "Height down scaling exceeds limits!\n");
		return -ERANGE;
	}

	if (crtc_w > (src_w * UP_SCALE_MAX)) {
		dev_err(dev->dev, "Width up scaling exceeds limits!\n");
		return -ERANGE;
	}

	if (crtc_h > (src_h * UP_SCALE_MAX)) {
		dev_err(dev->dev, "Height up scaling exceeds limits!\n");
		return -ERANGE;
	}

	if (src_w != crtc_w) {
		uint32_t sel_unit = SCALE_FIR;
		op_mode |= MDP4_PIPE_OP_MODE_SCALEX_EN;

		if (MDP_FORMAT_IS_YUV(format)) {
			if (crtc_w > src_w)
				sel_unit = SCALE_PIXEL_RPT;
			else if (crtc_w <= (src_w / 4))
				sel_unit = SCALE_MN_PHASE;

			op_mode |= MDP4_PIPE_OP_MODE_SCALEX_UNIT_SEL(sel_unit);
			phasex_step = mult_frac(MDP4_VG_PHASE_STEP_DEFAULT,
					src_w, crtc_w);
		}
	}

	if (src_h != crtc_h) {
		uint32_t sel_unit = SCALE_FIR;
		op_mode |= MDP4_PIPE_OP_MODE_SCALEY_EN;

		if (MDP_FORMAT_IS_YUV(format)) {

			if (crtc_h > src_h)
				sel_unit = SCALE_PIXEL_RPT;
			else if (crtc_h <= (src_h / 4))
				sel_unit = SCALE_MN_PHASE;

			op_mode |= MDP4_PIPE_OP_MODE_SCALEY_UNIT_SEL(sel_unit);
			phasey_step = mult_frac(MDP4_VG_PHASE_STEP_DEFAULT,
					src_h, crtc_h);
		}
	}

	mdp4_write(mdp4_kms, REG_MDP4_PIPE_SRC_SIZE(pipe),
			MDP4_PIPE_SRC_SIZE_WIDTH(src_w) |
			MDP4_PIPE_SRC_SIZE_HEIGHT(src_h));

	mdp4_write(mdp4_kms, REG_MDP4_PIPE_SRC_XY(pipe),
			MDP4_PIPE_SRC_XY_X(src_x) |
			MDP4_PIPE_SRC_XY_Y(src_y));

	mdp4_write(mdp4_kms, REG_MDP4_PIPE_DST_SIZE(pipe),
			MDP4_PIPE_DST_SIZE_WIDTH(crtc_w) |
			MDP4_PIPE_DST_SIZE_HEIGHT(crtc_h));

	mdp4_write(mdp4_kms, REG_MDP4_PIPE_DST_XY(pipe),
			MDP4_PIPE_DST_XY_X(crtc_x) |
			MDP4_PIPE_DST_XY_Y(crtc_y));

	mdp4_plane_set_scanout(plane, fb);

	mdp4_write(mdp4_kms, REG_MDP4_PIPE_SRC_FORMAT(pipe),
			MDP4_PIPE_SRC_FORMAT_A_BPC(format->bpc_a) |
			MDP4_PIPE_SRC_FORMAT_R_BPC(format->bpc_r) |
			MDP4_PIPE_SRC_FORMAT_G_BPC(format->bpc_g) |
			MDP4_PIPE_SRC_FORMAT_B_BPC(format->bpc_b) |
			COND(format->alpha_enable, MDP4_PIPE_SRC_FORMAT_ALPHA_ENABLE) |
			MDP4_PIPE_SRC_FORMAT_CPP(format->cpp - 1) |
			MDP4_PIPE_SRC_FORMAT_UNPACK_COUNT(format->unpack_count - 1) |
			MDP4_PIPE_SRC_FORMAT_FETCH_PLANES(format->fetch_type) |
			MDP4_PIPE_SRC_FORMAT_CHROMA_SAMP(format->chroma_sample) |
			COND(format->unpack_tight, MDP4_PIPE_SRC_FORMAT_UNPACK_TIGHT));

	mdp4_write(mdp4_kms, REG_MDP4_PIPE_SRC_UNPACK(pipe),
			MDP4_PIPE_SRC_UNPACK_ELEM0(format->unpack[0]) |
			MDP4_PIPE_SRC_UNPACK_ELEM1(format->unpack[1]) |
			MDP4_PIPE_SRC_UNPACK_ELEM2(format->unpack[2]) |
			MDP4_PIPE_SRC_UNPACK_ELEM3(format->unpack[3]));

	if (MDP_FORMAT_IS_YUV(format)) {
		struct csc_cfg *csc = mdp_get_default_csc_cfg(CSC_YUV2RGB);

		op_mode |= MDP4_PIPE_OP_MODE_SRC_YCBCR;
		op_mode |= MDP4_PIPE_OP_MODE_CSC_EN;
		mdp4_write_csc_config(mdp4_kms, pipe, csc);
	}

	mdp4_write(mdp4_kms, REG_MDP4_PIPE_OP_MODE(pipe), op_mode);
	mdp4_write(mdp4_kms, REG_MDP4_PIPE_PHASEX_STEP(pipe), phasex_step);
	mdp4_write(mdp4_kms, REG_MDP4_PIPE_PHASEY_STEP(pipe), phasey_step);

	/* TODO detach from old crtc (if we had more than one) */
	mdp4_crtc_attach(crtc, plane);

	return 0;
}

static const char *pipe_names[] = {
		"VG1", "VG2",
		"RGB1", "RGB2", "RGB3",
		"VG3", "VG4",
};

enum mdp4_pipe mdp4_plane_pipe(struct drm_plane *plane)
{
	struct mdp4_plane *mdp4_plane = to_mdp4_plane(plane);
	return mdp4_plane->pipe;
}

/* initialize plane */
struct drm_plane *mdp4_plane_init(struct drm_device *dev,
		enum mdp4_pipe pipe_id, bool private_plane)
{
	struct drm_plane *plane = NULL;
	struct mdp4_plane *mdp4_plane;
	int ret;
	enum drm_plane_type type;

	mdp4_plane = kzalloc(sizeof(*mdp4_plane), GFP_KERNEL);
	if (!mdp4_plane) {
		ret = -ENOMEM;
		goto fail;
	}

	plane = &mdp4_plane->base;

	mdp4_plane->pipe = pipe_id;
	mdp4_plane->name = pipe_names[pipe_id];

	mdp4_plane->nformats = mdp4_get_formats(pipe_id, mdp4_plane->formats,
			ARRAY_SIZE(mdp4_plane->formats));

	type = private_plane ? DRM_PLANE_TYPE_PRIMARY : DRM_PLANE_TYPE_OVERLAY;
	drm_universal_plane_init(dev, plane, 0xff, &mdp4_plane_funcs,
				 mdp4_plane->formats, mdp4_plane->nformats,
				 type);

	mdp4_plane_install_properties(plane, &plane->base);

	return plane;

fail:
	if (plane)
		mdp4_plane_destroy(plane);

	return ERR_PTR(ret);
}
