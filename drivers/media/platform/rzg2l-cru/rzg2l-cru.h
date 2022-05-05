/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Driver for Renesas RZ/G2L CRU
 *
 * Copyright (C) 2021 Renesas Electronics Corp.
 *
 * Based on the rcar_vin driver
 */

#ifndef __RZG2L_CRU__
#define __RZG2L_CRU__

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-v4l2.h>
#include <linux/reset.h>
#include <linux/clk.h>

/* Number of HW buffers */
#define HW_BUFFER_MAX		8
#define HW_BUFFER_DEFAULT	4

/* Address alignment mask for HW buffers */
#define HW_BUFFER_MASK	0x1ff

/* Maximum bumber of CSI2 virtual channels */
#define CSI2_VCHANNEL	4
/**
 * STOPPED  - No operation in progress
 * STARTING - Capture starting up
 * RUNNING  - Operation in progress have buffers
 * STOPPING - Stopping operation
 */
enum rzg2l_cru_dma_state {
	STOPPED = 0,
	STARTING,
	RUNNING,
	STOPPING,
};

/**
 * struct rzg2l_cru_video_format - Data format stored in memory
 * @fourcc:	Pixelformat
 * @bpp:	Bytes per pixel
 */
struct rzg2l_cru_video_format {
	u32 fourcc;
	u8 bpp;
};

/**
 * struct rzg2l_cru_parallel_entity - Parallel video input endpoint descriptor
 * @asd:	sub-device descriptor for async framework
 * @subdev:	subdevice matched using async framework
 * @mbus_type:	media bus type
 * @mbus_flags:	media bus configuration flags
 * @source_pad:	source pad of remote subdevice
 * @sink_pad:	sink pad of remote subdevice
 *
 */
struct rzg2l_cru_parallel_entity {
	struct v4l2_async_subdev asd;
	struct v4l2_subdev *subdev;

	enum v4l2_mbus_type mbus_type;
	unsigned int mbus_flags;

	unsigned int source_pad;
	unsigned int sink_pad;
};

/**
 * struct rzg2l_cru_info - Information about the particular CRU implementation
 * @max_width:		max input width the CRU supports
 * @max_height:		max input height the CRU supports
 */
struct rzg2l_cru_info {
	unsigned int max_width;
	unsigned int max_height;
};

/**
 * struct rzg2l_cru_dev - Renesas CRU device structure
 * @dev:		(OF) device
 * @base:		device I/O register space remapped to virtual memory
 * @info:		info about CRU instance
 *
 * @vdev:		V4L2 video device associated with CRU
 * @v4l2_dev:		V4L2 device
 * @ctrl_handler:	V4L2 control handler
 * @notifier:		V4L2 asynchronous subdevs notifier
 *
 * @parallel:		parallel input subdevice descriptor
 *
 * @group:		Gen3 CSI group
 * @pad:		media pad for the video device entity
 *
 * @lock:		protects @queue
 * @queue:		vb2 buffers queue
 * @scratch:		cpu address for scratch buffer
 * @scratch_phys:	physical address of the scratch buffer
 *
 * @qlock:		protects @queue_buf, @buf_list, @sequence
 *			@state
 * @queue_buf:		Keeps track of buffers given to HW slot
 * @buf_list:		list of queued buffers
 * @sequence:		V4L2 buffers sequence number
 * @state:		keeps track of operation state
 *
 * @is_csi:		flag to mark the CRU as using a CSI-2 subdevice
 *
 * @input_is_yuv:	flag to mark the input format of CRU
 * @output_is_yuv:	flag to mark the output format of CRU
 *
 * @mbus_code:		media bus format code
 * @format:		active V4L2 pixel format
 *
 * @crop:		active cropping
 * @source:		active size of the video source
 * @std:		active video standard of the video source
 */
struct rzg2l_cru_dev {
	struct device *dev;
	void __iomem *base;
	const struct rzg2l_cru_info *info;

	struct video_device vdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_ctrl_handler ctrl_handler;
	u8 num_buf;
	struct v4l2_async_notifier notifier;

	struct rzg2l_cru_parallel_entity *parallel;

	struct rzg2l_cru_group *group;
	struct media_pad pad;

	struct mutex lock;
	struct vb2_queue queue;
	void *scratch;
	dma_addr_t scratch_phys;

	spinlock_t qlock;
	struct vb2_v4l2_buffer *queue_buf[HW_BUFFER_MAX];
	struct list_head buf_list;
	unsigned int sequence;
	enum rzg2l_cru_dma_state state;

	bool is_csi;

	bool input_is_yuv;
	bool output_is_yuv;

	u32 mbus_code;
	struct v4l2_pix_format format;

	struct v4l2_rect crop;
	struct v4l2_rect compose;
	struct v4l2_rect source;
	v4l2_std_id std;

	struct reset_control *rstc;
};

#define cru_to_source(cru)		((cru)->parallel->subdev)

/* Debug */
#define cru_dbg(d, fmt, arg...)		dev_dbg(d->dev, fmt, ##arg)
#define cru_info(d, fmt, arg...)	dev_info(d->dev, fmt, ##arg)
#define cru_warn(d, fmt, arg...)	dev_warn(d->dev, fmt, ##arg)
#define cru_err(d, fmt, arg...)		dev_err(d->dev, fmt, ##arg)

//#define DEBUG
#ifdef DEBUG
#define dprintk(fmt, arg...)		printk(fmt, ##arg)
#else
#define dprintk(fmt, arg...)
#endif

/**
 * struct rzg2l_cru_group - CRU CSI2 group information
 * @mdev:	media device which represents the group
 *
 * @notifier:	group notifier for CSI-2 async subdevices
 * @cru:	CRU instances which are part of the group
 * @csi:	array of pairs of fwnode and subdev pointers
 *		to all CSI-2 subdevices.
 */
struct rzg2l_cru_group {
	struct media_device mdev;

	struct v4l2_async_notifier notifier;
	struct rzg2l_cru_dev *cru;

	struct {
		struct fwnode_handle *fwnode;
		struct v4l2_subdev *subdev;

		u32 channel;
	} csi;
};

int rzg2l_cru_dma_register(struct rzg2l_cru_dev *cru, int irq);
void rzg2l_cru_dma_unregister(struct rzg2l_cru_dev *cru);

int rzg2l_cru_v4l2_register(struct rzg2l_cru_dev *cru);
void rzg2l_cru_v4l2_unregister(struct rzg2l_cru_dev *cru);

const struct rzg2l_cru_video_format
*rzg2l_cru_format_from_pixel(u32 pixelformat);

#endif
