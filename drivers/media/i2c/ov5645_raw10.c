/*
 * Driver for the OV5645 camera sensor.
 *
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015 By Tech Design S.L. All Rights Reserved.
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Based on:
 * - the OV5645 driver from QC msm-3.10 kernel on codeaurora.org:
 *   https://us.codeaurora.org/cgit/quic/la/kernel/msm-3.10/tree/drivers/
 *       media/platform/msm/camera_v2/sensor/ov5645.c?h=LA.BR.1.2.4_rb1.41
 * - the OV5640 driver posted on linux-media:
 *   https://www.mail-archive.com/linux-media%40vger.kernel.org/msg92671.html
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define OV5645_VOLTAGE_ANALOG               2800000
#define OV5645_VOLTAGE_DIGITAL_CORE         1500000
#define OV5645_VOLTAGE_DIGITAL_IO           1800000

#define OV5645_SYSTEM_CTRL0		0x3008
#define		OV5645_SYSTEM_CTRL0_START	0x02
#define		OV5645_SYSTEM_CTRL0_STOP	0x42
#define OV5645_CHIP_ID_HIGH		0x300a
#define		OV5645_CHIP_ID_HIGH_BYTE	0x56
#define OV5645_CHIP_ID_LOW		0x300b
#define		OV5645_CHIP_ID_LOW_BYTE		0x45
#define OV5645_IO_MIPI_CTRL00		0x300e
#define OV5645_PAD_OUTPUT00		0x3019
#define OV5645_AWB_MANUAL_CONTROL	0x3406
#define		OV5645_AWB_MANUAL_ENABLE	BIT(0)
#define OV5645_AEC_PK_MANUAL		0x3503
#define		OV5645_AEC_MANUAL_ENABLE	BIT(0)
#define		OV5645_AGC_MANUAL_ENABLE	BIT(1)
#define OV5645_TIMING_TC_REG20		0x3820
#define		OV5645_SENSOR_VFLIP		BIT(1)
#define		OV5645_ISP_VFLIP		BIT(2)
#define OV5645_TIMING_TC_REG21		0x3821
#define		OV5645_SENSOR_MIRROR		BIT(1)
#define OV5645_MIPI_CTRL00		0x4800
#define OV5645_PRE_ISP_TEST_SETTING_1	0x503d
#define		OV5645_TEST_PATTERN_MASK	0x3
#define		OV5645_SET_TEST_PATTERN(x)	((x) & OV5645_TEST_PATTERN_MASK)
#define		OV5645_TEST_PATTERN_ENABLE	BIT(7)
#define OV5645_SDE_SAT_U		0x5583
#define OV5645_SDE_SAT_V		0x5584
#define OV5645_GAIN_HIGH_BYTE   0x350A	// [ISP Support PKG] Gain/Expose
#define OV5645_GAIN_LOW_BYTE    0x350B
#define OV5645_EXPOSE_HIGH_BYTE 0x3500
#define OV5645_EXPOSE_MID_BYTE  0x3501
#define OV5645_EXPOSE_LOW_BYTE  0x3502

enum ov5645_frame_rate {
	OV5645_15_FPS = 0,
	OV5645_30_FPS,
	OV5645_45_FPS,
	OV5645_60_FPS,
	OV5645_NUM_FRAMERATES,
};

static const int ov5645_framerates[] = {
	[OV5645_15_FPS] = 15,
	[OV5645_30_FPS] = 30,
	[OV5645_45_FPS] = 45,
	[OV5645_60_FPS] = 60,
};

struct ov5645_pixfmt {
	u32 code;
	u32 colorspace;
};

static const struct ov5645_pixfmt ov5645_formats[] = {
	{ MEDIA_BUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_SRGB, },
};

struct reg_value {
	u16 reg;
	u8 val;
};

struct ov5645_mode_info {
	u32 width;
	u32 height;
	const struct reg_value *data;
	u32 data_size;
	u32 pixel_clock;
	u32 link_freq;
	u32 max_fps;
};

struct ov5645 {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;
	struct clk *xclk;

	struct regulator *io_regulator;
	struct regulator *core_regulator;
	struct regulator *analog_regulator;

	const struct ov5645_mode_info *current_mode;
	enum ov5645_frame_rate current_fr;
	struct v4l2_fract frame_interval;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *pixel_clock;
	struct v4l2_ctrl *link_freq;

	/* Cached register values */
	u8 aec_pk_manual;
	u8 timing_tc_reg20;
	u8 timing_tc_reg21;

	struct mutex power_lock; /* lock to protect power state */
	int power_count;

	struct gpio_desc *enable_gpio;
	struct gpio_desc *rst_gpio;

	bool streaming;
};

static inline struct ov5645 *to_ov5645(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov5645, sd);
}

static const struct reg_value ov5645_global_init_setting[] = {
	{ 0x3103, 0x11 },
	{ 0x3008, 0x82 },
	{ 0x3008, 0x42 },
	{ 0x3103, 0x03 },
    { 0x3503, 0x03 },   // [ISP Support PKG] ISP OFF
	{ 0x3002, 0x1c },
	{ 0x3006, 0xc3 },
	{ 0x3017, 0x00 },
	{ 0x3018, 0x00 },
	{ 0x302e, 0x0b },
	{ 0x3037, 0x13 },
	{ 0x3108, 0x01 },
	{ 0x3611, 0x06 },
	{ 0x3500, 0x00 },
    { 0x3501, 0x20 },   // [ISP Support PKG] Expose time 0x01 -> 0x20
	{ 0x3502, 0x00 },
	{ 0x350a, 0x00 },
    { 0x350b, 0x50 },   // [ISP Support PKG] default gain 0x3f ->0x50
	{ 0x3620, 0x33 },
	{ 0x3621, 0xe0 },
	{ 0x3622, 0x01 },
	{ 0x3630, 0x2e },
	{ 0x3631, 0x00 },
	{ 0x3632, 0x32 },
	{ 0x3633, 0x52 },
	{ 0x3634, 0x70 },
	{ 0x3635, 0x13 },
	{ 0x3636, 0x03 },
	{ 0x3703, 0x5a },
	{ 0x3704, 0xa0 },
	{ 0x3705, 0x1a },
	{ 0x3709, 0x12 },
	{ 0x370b, 0x61 },
	{ 0x370f, 0x10 },
	{ 0x3715, 0x78 },
	{ 0x3717, 0x01 },
	{ 0x371b, 0x20 },
	{ 0x3731, 0x12 },
	{ 0x3901, 0x0a },
	{ 0x3905, 0x02 },
	{ 0x3906, 0x10 },
	{ 0x3719, 0x86 },
	{ 0x3810, 0x00 },
    { 0x3811, 0x0F },   // [ISP Support PKG] BGGR -> RGGB
	{ 0x3812, 0x00 },
	{ 0x3821, 0x01 },
	{ 0x3824, 0x01 },
	{ 0x3826, 0x03 },
	{ 0x3828, 0x08 },
	{ 0x3a19, 0xf8 },
	{ 0x3c01, 0x34 },
	{ 0x3c04, 0x28 },
	{ 0x3c05, 0x98 },
	{ 0x3c07, 0x07 },
	{ 0x3c09, 0xc2 },
	{ 0x3c0a, 0x9c },
	{ 0x3c0b, 0x40 },
	{ 0x3c01, 0x34 },
	{ 0x4001, 0x02 },
	{ 0x4514, 0x00 },
	{ 0x4520, 0xb0 },
	{ 0x460b, 0x37 },
	{ 0x460c, 0x20 },
	{ 0x4818, 0x01 },
	{ 0x481d, 0xf0 },
	{ 0x481f, 0x50 },
	{ 0x4823, 0x70 },
	{ 0x4831, 0x14 },
    { 0x5000, 0x00 },   // [ISP Support PKG] ISP OFF
    { 0x5001, 0x00 },   // [ISP Support PKG] ISP OFF
	{ 0x501d, 0x00 },
    { 0x501f, 0x03 },   // [ISP Support PKG] ISP OFF
	{ 0x503d, 0x00 },
	{ 0x505c, 0x30 },
	{ 0x5181, 0x59 },
	{ 0x5183, 0x00 },
	{ 0x5191, 0xf0 },
	{ 0x5192, 0x03 },
	{ 0x5684, 0x10 },
	{ 0x5685, 0xa0 },
	{ 0x5686, 0x0c },
	{ 0x5687, 0x78 },
	{ 0x5a00, 0x08 },
	{ 0x5a21, 0x00 },
	{ 0x5a24, 0x00 },
	{ 0x3008, 0x02 },
    { 0x3503, 0x03 },   // [ISP Support PKG] ISP OFF
	{ 0x5180, 0xff },
	{ 0x5181, 0xf2 },
	{ 0x5182, 0x00 },
	{ 0x5183, 0x14 },
	{ 0x5184, 0x25 },
	{ 0x5185, 0x24 },
	{ 0x5186, 0x09 },
	{ 0x5187, 0x09 },
	{ 0x5188, 0x0a },
	{ 0x5189, 0x75 },
	{ 0x518a, 0x52 },
	{ 0x518b, 0xea },
	{ 0x518c, 0xa8 },
	{ 0x518d, 0x42 },
	{ 0x518e, 0x38 },
	{ 0x518f, 0x56 },
	{ 0x5190, 0x42 },
	{ 0x5191, 0xf8 },
	{ 0x5192, 0x04 },
	{ 0x5193, 0x70 },
	{ 0x5194, 0xf0 },
	{ 0x5195, 0xf0 },
	{ 0x5196, 0x03 },
	{ 0x5197, 0x01 },
	{ 0x5198, 0x04 },
	{ 0x5199, 0x12 },
	{ 0x519a, 0x04 },
	{ 0x519b, 0x00 },
	{ 0x519c, 0x06 },
	{ 0x519d, 0x82 },
	{ 0x519e, 0x38 },
	{ 0x5381, 0x1e },
	{ 0x5382, 0x5b },
	{ 0x5383, 0x08 },
	{ 0x5384, 0x0a },
	{ 0x5385, 0x7e },
	{ 0x5386, 0x88 },
	{ 0x5387, 0x7c },
	{ 0x5388, 0x6c },
	{ 0x5389, 0x10 },
	{ 0x538a, 0x01 },
	{ 0x538b, 0x98 },
	{ 0x5300, 0x08 },
	{ 0x5301, 0x30 },
	{ 0x5302, 0x10 },
	{ 0x5303, 0x00 },
	{ 0x5304, 0x08 },
	{ 0x5305, 0x30 },
	{ 0x5306, 0x08 },
	{ 0x5307, 0x16 },
	{ 0x5309, 0x08 },
	{ 0x530a, 0x30 },
	{ 0x530b, 0x04 },
	{ 0x530c, 0x06 },
	{ 0x5480, 0x01 },
	{ 0x5481, 0x08 },
	{ 0x5482, 0x14 },
	{ 0x5483, 0x28 },
	{ 0x5484, 0x51 },
	{ 0x5485, 0x65 },
	{ 0x5486, 0x71 },
	{ 0x5487, 0x7d },
	{ 0x5488, 0x87 },
	{ 0x5489, 0x91 },
	{ 0x548a, 0x9a },
	{ 0x548b, 0xaa },
	{ 0x548c, 0xb8 },
	{ 0x548d, 0xcd },
	{ 0x548e, 0xdd },
	{ 0x548f, 0xea },
	{ 0x5490, 0x1d },
	{ 0x5580, 0x02 },
	{ 0x5583, 0x40 },
	{ 0x5584, 0x10 },
	{ 0x5589, 0x10 },
	{ 0x558a, 0x00 },
	{ 0x558b, 0xf8 },
	{ 0x5800, 0x3f },
	{ 0x5801, 0x16 },
	{ 0x5802, 0x0e },
	{ 0x5803, 0x0d },
	{ 0x5804, 0x17 },
	{ 0x5805, 0x3f },
	{ 0x5806, 0x0b },
	{ 0x5807, 0x06 },
	{ 0x5808, 0x04 },
	{ 0x5809, 0x04 },
	{ 0x580a, 0x06 },
	{ 0x580b, 0x0b },
	{ 0x580c, 0x09 },
	{ 0x580d, 0x03 },
	{ 0x580e, 0x00 },
	{ 0x580f, 0x00 },
	{ 0x5810, 0x03 },
	{ 0x5811, 0x08 },
	{ 0x5812, 0x0a },
	{ 0x5813, 0x03 },
	{ 0x5814, 0x00 },
	{ 0x5815, 0x00 },
	{ 0x5816, 0x04 },
	{ 0x5817, 0x09 },
	{ 0x5818, 0x0f },
	{ 0x5819, 0x08 },
	{ 0x581a, 0x06 },
	{ 0x581b, 0x06 },
	{ 0x581c, 0x08 },
	{ 0x581d, 0x0c },
	{ 0x581e, 0x3f },
	{ 0x581f, 0x1e },
	{ 0x5820, 0x12 },
	{ 0x5821, 0x13 },
	{ 0x5822, 0x21 },
	{ 0x5823, 0x3f },
	{ 0x5824, 0x68 },
	{ 0x5825, 0x28 },
	{ 0x5826, 0x2c },
	{ 0x5827, 0x28 },
	{ 0x5828, 0x08 },
	{ 0x5829, 0x48 },
	{ 0x582a, 0x64 },
	{ 0x582b, 0x62 },
	{ 0x582c, 0x64 },
	{ 0x582d, 0x28 },
	{ 0x582e, 0x46 },
	{ 0x582f, 0x62 },
	{ 0x5830, 0x60 },
	{ 0x5831, 0x62 },
	{ 0x5832, 0x26 },
	{ 0x5833, 0x48 },
	{ 0x5834, 0x66 },
	{ 0x5835, 0x44 },
	{ 0x5836, 0x64 },
	{ 0x5837, 0x28 },
	{ 0x5838, 0x66 },
	{ 0x5839, 0x48 },
	{ 0x583a, 0x2c },
	{ 0x583b, 0x28 },
	{ 0x583c, 0x26 },
	{ 0x583d, 0xae },
	{ 0x5025, 0x00 },
	{ 0x3a0f, 0x30 },
	{ 0x3a10, 0x28 },
	{ 0x3a1b, 0x30 },
	{ 0x3a1e, 0x26 },
	{ 0x3a11, 0x60 },
	{ 0x3a1f, 0x14 },
	{ 0x0601, 0x02 },
	{ 0x3008, 0x42 },
	{ 0x3008, 0x02 },
	{ OV5645_IO_MIPI_CTRL00, 0x40 },
	{ OV5645_MIPI_CTRL00, 0x24 },
	{ OV5645_PAD_OUTPUT00, 0x70 }
};

static const struct reg_value ov5645_setting_VGA_640_480[] = {
	{ 0x3612, 0xa9 },
	{ 0x3614, 0x50 },
	{ 0x3618, 0x00 },
	{ 0x3034, 0x1A },   // RAW10
	{ 0x3035, 0x21 },
	{ 0x3036, 0x90 },   // [ISP Support PKG] 30fps
	{ 0x3600, 0x09 },
	{ 0x3601, 0x43 },
	{ 0x3708, 0x66 },
	{ 0x370c, 0xc3 },
	{ 0x3800, 0x02 },   // [ISP Support PKG] Camera position tuning
	{ 0x3801, 0x80 },   // [ISP Support PKG] Camera position tuning
	{ 0x3802, 0x01 },   // [ISP Support PKG] Camera position tuning
	{ 0x3803, 0xD8 },   // [ISP Support PKG] Camera position tuning
	{ 0x3804, 0x0a },
	{ 0x3805, 0x3D },
	{ 0x3806, 0x07 },   // [ISP Support PKG] Camera position tuning
	{ 0x3807, 0xE2 },   // [ISP Support PKG] Camera position tuning
	{ 0x3808, 0x02 },
	{ 0x3809, 0x88 },   // [ISP Support PKG] Multiple of 12 are allowd on RAW10 Stride
	{ 0x380a, 0x01 },
	{ 0x380b, 0xe0 },
	{ 0x380c, 0x07 },
	{ 0x380d, 0x68 },
	{ 0x380e, 0x03 },
	{ 0x380f, 0xd8 },
    { 0x3810, 0x00 },   // [ISP Support PKG] Camera position tuning
    { 0x3811, 0x0F },   // [ISP Support PKG] Camera position tuning 
    { 0x3812, 0x00 },   // [ISP Support PKG] Camera position tuning 
    { 0x3813, 0x0F },   // [ISP Support PKG] Camera position tuning 
	{ 0x3814, 0x31 },
	{ 0x3815, 0x31 },
	{ 0x3820, 0x41 },
	{ 0x3a02, 0x03 },
	{ 0x3a03, 0xd8 },
	{ 0x3a08, 0x01 },
	{ 0x3a09, 0xf8 },
	{ 0x3a0a, 0x01 },
	{ 0x3a0b, 0xa4 },
	{ 0x3a0e, 0x02 },
	{ 0x3a0d, 0x02 },
	{ 0x3a14, 0x03 },
	{ 0x3a15, 0xd8 },
	{ 0x3a18, 0x00 },
	{ 0x4004, 0x02 },
	{ 0x4005, 0x18 },
    { 0x4300, 0x03 },   // [ISP Support PKG] YUV -> RAW
	{ 0x4202, 0x00 }
};

static const struct reg_value ov5645_setting_30fps_NTSC_720_480[] = {
	{ 0x3612, 0xa9 },
	{ 0x3614, 0x50 },
	{ 0x3618, 0x00 },
	{ 0x3034, 0x1A },   //RAW10
	{ 0x3035, 0x11 },
	{ 0x3036, 0x54 },
	{ 0x3600, 0x09 },
	{ 0x3601, 0x43 },
	{ 0x3708, 0x66 },
	{ 0x370c, 0xc3 },
	{ 0x3800, 0x00 },
	{ 0x3801, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3803, 0x6c },
	{ 0x3804, 0x0a },
	{ 0x3805, 0x3f },
	{ 0x3806, 0x07 },
	{ 0x3807, 0x33 },
	{ 0x3808, 0x02 },
	{ 0x3809, 0xd0 },
	{ 0x380a, 0x01 },
	{ 0x380b, 0xe0 },
	{ 0x380c, 0x07 },
	{ 0x380d, 0x68 },
	{ 0x380e, 0x03 },
	{ 0x380f, 0xd8 },
    { 0x3813, 0x05 },   // [ISP Support PKG] BGGR -> RGGB
	{ 0x3814, 0x31 },
	{ 0x3815, 0x31 },
	{ 0x3820, 0x41 },
	{ 0x3a02, 0x03 },
	{ 0x3a03, 0xd8 },
	{ 0x3a08, 0x01 },
	{ 0x3a09, 0xf8 },
	{ 0x3a0a, 0x01 },
	{ 0x3a0b, 0xa4 },
	{ 0x3a0e, 0x02 },
	{ 0x3a0d, 0x02 },
	{ 0x3a14, 0x03 },
	{ 0x3a15, 0xd8 },
	{ 0x3a18, 0x00 },
	{ 0x4004, 0x02 },
	{ 0x4005, 0x18 },
    { 0x4300, 0x03 },   // [ISP Support PKG] YUV -> RAW
	{ 0x4202, 0x00 }
};

static const struct reg_value ov5645_setting_720P_1280_720[] = {
	{ 0x3612, 0xa9 },
	{ 0x3614, 0x50 },
	{ 0x3618, 0x00 },
	{ 0x3034, 0x1A },   //RAW10
	{ 0x3035, 0x11 },
	{ 0x3036, 0x34 },   //[ISP Support PKG] 30fps
	{ 0x3500, 0x00},
	{ 0x3501, 0x20},   // [ISP Support PKG] Expose time 0x01 -> 0x20
	{ 0x3502, 0x00},
	{ 0x350a, 0x00},
	{ 0x350b, 0x50},   // [ISP Support PKG] default gain 0x3f ->0x50
	{ 0x3600, 0x0a},
	{ 0x3601, 0x75},
	{ 0x3620, 0x33},
	{ 0x3621, 0xe0},
	{ 0x3622, 0x01},
	{ 0x3630, 0x2d},
	{ 0x3631, 0x00},
	{ 0x3632, 0x32},
	{ 0x3633, 0x52},
	{ 0x3634, 0x70},
	{ 0x3635, 0x13},
	{ 0x3636, 0x03},
	{ 0x3702, 0x6e},
	{ 0x3703, 0x52},
	{ 0x3704, 0xa0},
	{ 0x3705, 0x33},
	{ 0x3708, 0x66},
	{ 0x3709, 0x12},
	{ 0x370b, 0x61},
	{ 0x370c, 0xc3},
	{ 0x370f, 0x10},
	{ 0x3715, 0x08},
	{ 0x3717, 0x01},
	{ 0x371b, 0x20},
	{ 0x3731, 0x22},
	{ 0x3739, 0x70},
	{ 0x3901, 0x0a},
	{ 0x3905, 0x02},
	{ 0x3906, 0x10},
	{ 0x3719, 0x86},
	{ 0x3800, 0x00},
	{ 0x3801, 0x00},
	{ 0x3802, 0x00},
	{ 0x3803, 0xfa},
	{ 0x3804, 0x0a},
	{ 0x3805, 0x3f},
	{ 0x3806, 0x06},
	{ 0x3807, 0xa9},
	{ 0x3808, 0x05},
	{ 0x3809, 0x04},    // [ISP Support PKG] Multiple of 12 are allowd on RAW10 Stride
	{ 0x380a, 0x02},
	{ 0x380b, 0xd0},
	{ 0x380c, 0x07},
	{ 0x380d, 0x64},
	{ 0x380e, 0x02},
	{ 0x380f, 0xe4},
	{ 0x3810, 0x00},
    { 0x3811, 0x0F },   // [ISP Support PKG] BGGR -> RGGB
	{ 0x3812, 0x00},
    { 0x3813, 0x03},   // [ISP Support PKG] BGGR -> RGGB
	{ 0x3814, 0x31},
	{ 0x3815, 0x31},
	{ 0x3820, 0x41},
	{ 0x3821, 0x07},
	{ 0x3824, 0x01},
	{ 0x3826, 0x03},
	{ 0x3828, 0x08},
	{ 0x3a02, 0x02},
	{ 0x3a03, 0xe4},
	{ 0x3a08, 0x01},
	{ 0x3a09, 0xbc},
	{ 0x3a0a, 0x01},
	{ 0x3a0b, 0x72},
	{ 0x3a0e, 0x01},
	{ 0x3a0d, 0x02},
	{ 0x3a14, 0x02},
	{ 0x3a15, 0xe4},
	{ 0x3a18, 0x00},
	{ 0x3a19, 0xf8},
	{ 0x3c01, 0x34},
	{ 0x3c04, 0x28},
	{ 0x3c05, 0x98},
	{ 0x3c07, 0x07},
	{ 0x3c09, 0xc2},
	{ 0x3c0a, 0x9c},
	{ 0x3c0b, 0x40},
	{ 0x3c01, 0x34},
	{ 0x4004, 0x02 },
	{ 0x4005, 0x18 },
    { 0x4300, 0x03 },   // [ISP Support PKG] YUV -> RAW
	{ 0x4202, 0x00 }
};

static const struct reg_value ov5645_setting_sxga[] = {
	{ 0x3612, 0xa9 },
	{ 0x3614, 0x50 },
	{ 0x3618, 0x00 },
	{ 0x3034, 0x1A },   //RAW10
	{ 0x3035, 0x21 },
	{ 0x3036, 0x70 },
	{ 0x3600, 0x09 },
	{ 0x3601, 0x43 },
	{ 0x3708, 0x66 },
	{ 0x370c, 0xc3 },
	{ 0x3800, 0x00 },
	{ 0x3801, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3803, 0x06 },
	{ 0x3804, 0x0a },
	{ 0x3805, 0x3f },
	{ 0x3806, 0x07 },
	{ 0x3807, 0x9d },
	{ 0x3808, 0x05 },
	{ 0x3809, 0x04 },   // [ISP Support PKG] Multiple of 12 are allowd on RAW10 Stride
	{ 0x380a, 0x03 },
	{ 0x380b, 0xc0 },
	{ 0x380c, 0x07 },
	{ 0x380d, 0x68 },
	{ 0x380e, 0x03 },
	{ 0x380f, 0xd8 },
    { 0x3813, 0x05 },   // [ISP Support PKG] BGGR -> RGGB
    { 0x3814, 0x31 },
    { 0x3815, 0x31 },
    { 0x3820, 0x47 },
    { 0x3a02, 0x03 },
    { 0x3a03, 0xd8 },
    { 0x3a08, 0x01 },
    { 0x3a09, 0xf8 },
    { 0x3a0a, 0x01 },
    { 0x3a0b, 0xa4 },
    { 0x3a0e, 0x02 },
    { 0x3a0d, 0x02 },
    { 0x3a14, 0x03 },
    { 0x3a15, 0xd8 },
    { 0x3a18, 0x00 },
    { 0x4004, 0x02 },
    { 0x4005, 0x18 },
    { 0x4300, 0x03 },   // [ISP Support PKG] YUV -> RAW
    { 0x4202, 0x00 }
};

static const struct reg_value ov5645_setting_1080p[] = {
	{ 0x3612, 0xab },
	{ 0x3614, 0x50 },
	{ 0x3618, 0x04 },
	{ 0x3034, 0x1A },   //RAW10
	{ 0x3035, 0x11 },
	{ 0x3036, 0x70 },   // [ISP Support PKG] 30fps
	{ 0x3600, 0x08 },
	{ 0x3601, 0x33 },
	{ 0x3708, 0x63 },
	{ 0x370c, 0xc0 },
	{ 0x3800, 0x01 },
	{ 0x3801, 0x50 },
	{ 0x3802, 0x01 },
	{ 0x3803, 0xb2 },
	{ 0x3804, 0x08 },
	{ 0x3805, 0xef },
	{ 0x3806, 0x05 },
	{ 0x3807, 0xf1 },
	{ 0x3808, 0x07 },
	{ 0x3809, 0x80 },
	{ 0x380a, 0x04 },
	{ 0x380b, 0x38 },
	{ 0x380c, 0x09 },
	{ 0x380d, 0xc4 },
	{ 0x380e, 0x04 },
	{ 0x380f, 0x60 },
    { 0x3810, 0x00 },   // [ISP Support PKG] Camera position tuning
    { 0x3811, 0x0F },   // [ISP Support PKG] Camera position tuning
    { 0x3812, 0x00 },   // [ISP Support PKG] Camera position tuning
    { 0x3813, 0x03 },   // [ISP Support PKG] Camera position tuning
	{ 0x3814, 0x11 },
	{ 0x3815, 0x11 },
	{ 0x3820, 0x47 },
	{ 0x4514, 0x88 },
	{ 0x3a02, 0x04 },
	{ 0x3a03, 0x60 },
	{ 0x3a08, 0x01 },
	{ 0x3a09, 0x50 },
	{ 0x3a0a, 0x01 },
	{ 0x3a0b, 0x18 },
	{ 0x3a0e, 0x03 },
	{ 0x3a0d, 0x04 },
	{ 0x3a14, 0x04 },
	{ 0x3a15, 0x60 },
	{ 0x3a18, 0x00 },
	{ 0x4004, 0x06 },
	{ 0x4005, 0x18 },
    { 0x4300, 0x03 },   // [ISP Support PKG] YUV -> RAW
	{ 0x4202, 0x00 },
	{ 0x4837, 0x0b }
};

static const struct reg_value ov5645_setting_full[] = {
	{ 0x3612, 0xab },
	{ 0x3614, 0x50 },
	{ 0x3618, 0x04 },
	{ 0x3034, 0x1A },   //RAW10
	{ 0x3035, 0x11 },
	{ 0x3036, 0x54 },
	{ 0x3600, 0x08 },
	{ 0x3601, 0x33 },
	{ 0x3708, 0x63 },
	{ 0x370c, 0xc0 },
	{ 0x3800, 0x00 },
	{ 0x3801, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3803, 0x00 },
	{ 0x3804, 0x0a },
	{ 0x3805, 0x3f },
	{ 0x3806, 0x07 },
	{ 0x3807, 0x9f },
	{ 0x3808, 0x0a },
	{ 0x3809, 0x20 },
	{ 0x380a, 0x07 },
	{ 0x380b, 0x98 },
	{ 0x380c, 0x0b },
	{ 0x380d, 0x1c },
	{ 0x380e, 0x07 },
	{ 0x380f, 0xb0 },
    { 0x3813, 0x05 },   // [ISP Support PKG] BGGR -> RGGB
	{ 0x3814, 0x11 },
	{ 0x3815, 0x11 },
	{ 0x3820, 0x47 },
	{ 0x4514, 0x88 },
	{ 0x3a02, 0x07 },
	{ 0x3a03, 0xb0 },
	{ 0x3a08, 0x01 },
	{ 0x3a09, 0x27 },
	{ 0x3a0a, 0x00 },
	{ 0x3a0b, 0xf6 },
	{ 0x3a0e, 0x06 },
	{ 0x3a0d, 0x08 },
	{ 0x3a14, 0x07 },
	{ 0x3a15, 0xb0 },
	{ 0x3a18, 0x01 },
	{ 0x4004, 0x06 },
	{ 0x4005, 0x18 },
    { 0x4300, 0x03 },   // [ISP Support PKG] YUV -> RAW
	{ 0x4837, 0x0b },
	{ 0x4202, 0x00 }
};

static const s64 link_freq[] = {
	224000000,
	336000000
};

static const struct ov5645_mode_info ov5645_mode_info_data[] = {
	{
		.width = 640,
		.height = 480,
		.data = ov5645_setting_VGA_640_480,
		.data_size = ARRAY_SIZE(ov5645_setting_VGA_640_480),
		.pixel_clock = 112000000,
		.link_freq = 0, /* an index in link_freq[] */
		.max_fps = OV5645_30_FPS
	},
	{
		.width = 720,
		.height = 480,
		.data = ov5645_setting_30fps_NTSC_720_480,
		.data_size = ARRAY_SIZE(ov5645_setting_30fps_NTSC_720_480),
		.pixel_clock = 112000000,
		.link_freq = 0, /* an index in link_freq[] */
		.max_fps = OV5645_45_FPS
	},
	{
		.width = 1280,
		.height = 720,
		.data = ov5645_setting_720P_1280_720,
		.data_size = ARRAY_SIZE(ov5645_setting_720P_1280_720),
		.pixel_clock = 112000000,
		.link_freq = 0, /* an index in link_freq[] */
		.max_fps = OV5645_60_FPS
	},
	{
		.width = 1280,
		.height = 960,
		.data = ov5645_setting_sxga,
		.data_size = ARRAY_SIZE(ov5645_setting_sxga),
		.pixel_clock = 112000000,
		.link_freq = 0, /* an index in link_freq[] */
		.max_fps = OV5645_30_FPS
	},
	{
		.width = 1920,
		.height = 1080,
		.data = ov5645_setting_1080p,
		.data_size = ARRAY_SIZE(ov5645_setting_1080p),
		.pixel_clock = 168000000,
		.link_freq = 1, /* an index in link_freq[] */
		.max_fps = OV5645_30_FPS
	},
	{
		.width = 2592,
		.height = 1944,
		.data = ov5645_setting_full,
		.data_size = ARRAY_SIZE(ov5645_setting_full),
		.pixel_clock = 168000000,
		.link_freq = 1, /* an index in link_freq[] */
		.max_fps = OV5645_15_FPS
	},
};

static int ov5645_regulators_enable(struct ov5645 *ov5645)
{
	int ret;

	ret = regulator_enable(ov5645->io_regulator);
	if (ret < 0) {
		dev_err(ov5645->dev, "set io voltage failed\n");
		return ret;
	}

	ret = regulator_enable(ov5645->analog_regulator);
	if (ret) {
		dev_err(ov5645->dev, "set analog voltage failed\n");
		goto err_disable_io;
	}

	ret = regulator_enable(ov5645->core_regulator);
	if (ret) {
		dev_err(ov5645->dev, "set core voltage failed\n");
		goto err_disable_analog;
	}

	return 0;

err_disable_analog:
	regulator_disable(ov5645->analog_regulator);
err_disable_io:
	regulator_disable(ov5645->io_regulator);

	return ret;
}

static void ov5645_regulators_disable(struct ov5645 *ov5645)
{
	int ret;

	ret = regulator_disable(ov5645->core_regulator);
	if (ret < 0)
		dev_err(ov5645->dev, "core regulator disable failed\n");

	ret = regulator_disable(ov5645->analog_regulator);
	if (ret < 0)
		dev_err(ov5645->dev, "analog regulator disable failed\n");

	ret = regulator_disable(ov5645->io_regulator);
	if (ret < 0)
		dev_err(ov5645->dev, "io regulator disable failed\n");
}

static int ov5645_write_reg(struct ov5645 *ov5645, u16 reg, u8 val)
{
	u8 regbuf[3];
	int ret;

	regbuf[0] = reg >> 8;
	regbuf[1] = reg & 0xff;
	regbuf[2] = val;

	ret = i2c_master_send(ov5645->i2c_client, regbuf, 3);
	if (ret < 0) {
		dev_err(ov5645->dev, "%s: write reg error %d: reg=%x, val=%x\n",
			__func__, ret, reg, val);
		return ret;
	}

	return 0;
}

static int ov5645_read_reg(struct ov5645 *ov5645, u16 reg, u8 *val)
{
	u8 regbuf[2];
	int ret;

	regbuf[0] = reg >> 8;
	regbuf[1] = reg & 0xff;

	ret = i2c_master_send(ov5645->i2c_client, regbuf, 2);
	if (ret < 0) {
		dev_err(ov5645->dev, "%s: write reg error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}

	ret = i2c_master_recv(ov5645->i2c_client, val, 1);
	if (ret < 0) {
		dev_err(ov5645->dev, "%s: read reg error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}

	return 0;
}

static const struct ov5645_mode_info *
ov5645_find_mode(struct ov5645 *sensor, enum ov5645_frame_rate fr,
		 int width, int height, bool nearest)
{
	const struct ov5645_mode_info *mode;

	mode = v4l2_find_nearest_size(ov5645_mode_info_data,
			       ARRAY_SIZE(ov5645_mode_info_data),
			       width, height,
			       width, height);
	if (!mode ||
	    (!nearest && (mode->width != width || mode->height != height)))
		return NULL;

	/* Check to see if the current mode exceeds the max frame rate */
	if (ov5645_framerates[fr] > ov5645_framerates[mode->max_fps])
		return NULL;

	return mode;
}

static int ov5645_set_aec_mode(struct ov5645 *ov5645, u32 mode)
{
// [ISP Support PKG] ISP OFF
#if 0
	u8 val = ov5645->aec_pk_manual;
	int ret;

	if (mode == V4L2_EXPOSURE_AUTO)
		val &= ~OV5645_AEC_MANUAL_ENABLE;
	else /* V4L2_EXPOSURE_MANUAL */
		val |= OV5645_AEC_MANUAL_ENABLE;

	ret = ov5645_write_reg(ov5645, OV5645_AEC_PK_MANUAL, val);
	if (!ret)
		ov5645->aec_pk_manual = val;
#endif
    return 0;
}

static int ov5645_set_agc_mode(struct ov5645 *ov5645, u32 enable)
{
// [ISP Support PKG] ISP OFF
#if 0
	u8 val = ov5645->aec_pk_manual;
	int ret;

	if (enable)
		val &= ~OV5645_AGC_MANUAL_ENABLE;
	else
		val |= OV5645_AGC_MANUAL_ENABLE;

	ret = ov5645_write_reg(ov5645, OV5645_AEC_PK_MANUAL, val);
	if (!ret)
		ov5645->aec_pk_manual = val;
#endif
    return 0;
}

static int ov5645_set_register_array(struct ov5645 *ov5645,
				     const struct reg_value *settings,
				     unsigned int num_settings)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_settings; ++i, ++settings) {
		ret = ov5645_write_reg(ov5645, settings->reg, settings->val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ov5645_set_power_on(struct ov5645 *ov5645)
{
	int ret;

	ret = ov5645_regulators_enable(ov5645);
	if (ret < 0) {
		return ret;
	}

	ret = clk_prepare_enable(ov5645->xclk);
	if (ret < 0) {
		dev_err(ov5645->dev, "clk prepare enable failed\n");
		ov5645_regulators_disable(ov5645);
		return ret;
	}

	usleep_range(5000, 15000);
	gpiod_set_value_cansleep(ov5645->enable_gpio, 1);

	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ov5645->rst_gpio, 0);

	msleep(20);

	return 0;
}

static void ov5645_set_power_off(struct ov5645 *ov5645)
{
	gpiod_set_value_cansleep(ov5645->rst_gpio, 1);
	gpiod_set_value_cansleep(ov5645->enable_gpio, 0);
	clk_disable_unprepare(ov5645->xclk);
	ov5645_regulators_disable(ov5645);
}

static int ov5645_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov5645 *ov5645 = to_ov5645(sd);
	int ret = 0;

	mutex_lock(&ov5645->power_lock);

	/* If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (ov5645->power_count == !on) {
		if (on) {
			ret = ov5645_set_power_on(ov5645);
			if (ret < 0)
				goto exit;

			ret = ov5645_set_register_array(ov5645,
					ov5645_global_init_setting,
					ARRAY_SIZE(ov5645_global_init_setting));
			if (ret < 0) {
				dev_err(ov5645->dev,
					"could not set init registers\n");
				ov5645_set_power_off(ov5645);
				goto exit;
			}

			usleep_range(500, 1000);
		} else {
			ov5645_write_reg(ov5645, OV5645_IO_MIPI_CTRL00, 0x58);
			ov5645_set_power_off(ov5645);
		}
	}

	/* Update the power count. */
	ov5645->power_count += on ? 1 : -1;
	WARN_ON(ov5645->power_count < 0);

exit:
	mutex_unlock(&ov5645->power_lock);

	return ret;
}

static int ov5645_try_frame_interval(struct ov5645 *sensor,
				     struct v4l2_fract *fi,
				     u32 width, u32 height)
{
	const struct ov5645_mode_info *mode;
	enum ov5645_frame_rate rate = OV5645_15_FPS;
	int minfps, maxfps, best_fps, fps;
	int i;

	minfps = ov5645_framerates[OV5645_15_FPS];
	maxfps = ov5645_framerates[OV5645_60_FPS];

	if (fi->numerator == 0) {
		fi->denominator = maxfps;
		fi->numerator = 1;
		rate = OV5645_60_FPS;
		goto find_mode;
	}

	fps = clamp_val(DIV_ROUND_CLOSEST(fi->denominator, fi->numerator),
			minfps, maxfps);

	best_fps = minfps;
	fps = clamp_val(DIV_ROUND_CLOSEST(fi->denominator, fi->numerator),
			minfps, maxfps);

	for (i = 0; i < ARRAY_SIZE(ov5645_framerates); i++) {
		int curr_fps = ov5645_framerates[i];

		if (abs(curr_fps - fps) < abs(best_fps - fps)) {
			best_fps = curr_fps;
			rate = i;
		}
	}

	fi->numerator = 1;
	fi->denominator = best_fps;

find_mode:
	mode = ov5645_find_mode(sensor, rate, width, height, false);
	return mode ? rate : -EINVAL;
}

static int ov5645_set_saturation(struct ov5645 *ov5645, s32 value)
{
	u32 reg_value = (value * 0x10) + 0x40;
	int ret;

	ret = ov5645_write_reg(ov5645, OV5645_SDE_SAT_U, reg_value);
	if (ret < 0)
		return ret;

	return ov5645_write_reg(ov5645, OV5645_SDE_SAT_V, reg_value);
}

static int ov5645_set_hflip(struct ov5645 *ov5645, s32 value)
{
	u8 val = ov5645->timing_tc_reg21;
	int ret;

	if (value == 0)
		val &= ~(OV5645_SENSOR_MIRROR);
	else
		val |= (OV5645_SENSOR_MIRROR);

	ret = ov5645_write_reg(ov5645, OV5645_TIMING_TC_REG21, val);
	if (!ret)
		ov5645->timing_tc_reg21 = val;

	return ret;
}

static int ov5645_set_vflip(struct ov5645 *ov5645, s32 value)
{
	u8 val = ov5645->timing_tc_reg20;
	int ret;

	if (value == 0)
		val |= (OV5645_SENSOR_VFLIP | OV5645_ISP_VFLIP);
	else
		val &= ~(OV5645_SENSOR_VFLIP | OV5645_ISP_VFLIP);

	ret = ov5645_write_reg(ov5645, OV5645_TIMING_TC_REG20, val);
	if (!ret)
		ov5645->timing_tc_reg20 = val;

	return ret;
}

static int ov5645_set_test_pattern(struct ov5645 *ov5645, s32 value)
{
	u8 val = 0;

	if (value) {
		val = OV5645_SET_TEST_PATTERN(value - 1);
		val |= OV5645_TEST_PATTERN_ENABLE;
	}

	return ov5645_write_reg(ov5645, OV5645_PRE_ISP_TEST_SETTING_1, val);
}

static const char * const ov5645_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bars",
	"Pseudo-Random Data",
	"Color Square",
	"Black Image",
};

static int ov5645_set_awb(struct ov5645 *ov5645, s32 enable_auto)
{
// [ISP Support PKG] ISP OFF
#if 0
	u8 val = 0;

	if (!enable_auto)
		val = OV5645_AWB_MANUAL_ENABLE;

	return ov5645_write_reg(ov5645, OV5645_AWB_MANUAL_CONTROL, val);
#else
    return 0;
#endif
}

static int ov5645_set_gain(struct ov5645 *ov5645, s32 value)
{
    unsigned short const gain_to_reg[301] = {   // 0dB to 30 dB
        0x0010,                                                                         // 0dB
        0x0010, 0x0010, 0x0011, 0x0011, 0x0011, 0x0011, 0x0011, 0x0012, 0x0012, 0x0012, // 0.1dB to  1.0dB
        0x0012, 0x0012, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0014, 0x0014, 0x0014, // 1.1dB to  2.0dB
        0x0014, 0x0015, 0x0015, 0x0015, 0x0015, 0x0016, 0x0016, 0x0016, 0x0016, 0x0017, // 2.1dB to  3.0dB
        0x0017, 0x0017, 0x0017, 0x0018, 0x0018, 0x0018, 0x0018, 0x0019, 0x0019, 0x0019, // 3.1dB to  4.0dB
        0x001A, 0x001A, 0x001A, 0x001B, 0x001B, 0x001B, 0x001B, 0x001C, 0x001C, 0x001C, // 4.1dB to  5.0dB
        0x001D, 0x001D, 0x001D, 0x001E, 0x001E, 0x001E, 0x001F, 0x001F, 0x0020, 0x0020, // 5.1dB to  6.0dB
        0x0020, 0x0021, 0x0021, 0x0021, 0x0022, 0x0022, 0x0023, 0x0023, 0x0023, 0x0024, // 6.1dB to  7.0dB
        0x0024, 0x0025, 0x0025, 0x0026, 0x0026, 0x0026, 0x0027, 0x0027, 0x0028, 0x0028, // 7.1dB to  8.0dB
        0x0029, 0x0029, 0x002A, 0x002A, 0x002B, 0x002B, 0x002C, 0x002C, 0x002D, 0x002D, // 8.1dB to  9.0dB
        0x002E, 0x002E, 0x002F, 0x002F, 0x0030, 0x0030, 0x0031, 0x0031, 0x0032, 0x0033, // 9.1dB to 10.0dB
        0x0033, 0x0034, 0x0034, 0x0035, 0x0036, 0x0036, 0x0037, 0x0037, 0x0038, 0x0039, //10.1dB to 11.0dB
        0x0039, 0x003A, 0x003B, 0x003B, 0x003C, 0x003D, 0x003E, 0x003E, 0x003F, 0x0040, // 11.1dB to 12.0dB
        0x0040, 0x0041, 0x0042, 0x0043, 0x0043, 0x0044, 0x0045, 0x0046, 0x0047, 0x0047, // 12.1dB to 13.0dB
        0x0048, 0x0049, 0x004A, 0x004B, 0x004C, 0x004D, 0x004D, 0x004E, 0x004F, 0x0050, // 13.1dB to 14.0dB
        0x0051, 0x0052, 0x0053, 0x0054, 0x0055, 0x0056, 0x0057, 0x0058, 0x0059, 0x005A, // 14.1dB to 15.0dB
        0x005B, 0x005C, 0x005D, 0x005E, 0x005F, 0x0060, 0x0062, 0x0063, 0x0064, 0x0065, // 15.1dB to 16.0dB
        0x0066, 0x0067, 0x0069, 0x006A, 0x006B, 0x006C, 0x006D, 0x006F, 0x0070, 0x0071, // 16.1dB to 17.0dB
        0x0073, 0x0074, 0x0075, 0x0077, 0x0078, 0x0079, 0x007B, 0x007C, 0x007E, 0x007F, // 17.1dB to 18.0dB
        0x0081, 0x0082, 0x0084, 0x0085, 0x0087, 0x0088, 0x008A, 0x008B, 0x008D, 0x008F, // 18.1dB to 19.0dB
        0x0090, 0x0092, 0x0094, 0x0095, 0x0097, 0x0099, 0x009B, 0x009C, 0x009E, 0x00A0, // 19.1dB to 20.0dB
        0x00A2, 0x00A4, 0x00A6, 0x00A8, 0x00A9, 0x00AB, 0x00AD, 0x00AF, 0x00B1, 0x00B4, // 20.1dB to 21.0dB
        0x00B6, 0x00B8, 0x00BA, 0x00BC, 0x00BE, 0x00C0, 0x00C3, 0x00C5, 0x00C7, 0x00C9, // 21.1dB to 22.0dB
        0x00CC, 0x00CE, 0x00D1, 0x00D3, 0x00D5, 0x00D8, 0x00DA, 0x00DD, 0x00DF, 0x00E2, // 22.1dB to 23.0dB
        0x00E5, 0x00E7, 0x00EA, 0x00ED, 0x00EF, 0x00F2, 0x00F5, 0x00F8, 0x00FB, 0x00FE, // 23.1dB to 24.0dB
        0x0101, 0x0103, 0x0106, 0x010A, 0x010D, 0x0110, 0x0113, 0x0116, 0x0119, 0x011D, // 24.1dB to 25.0dB
        0x0120, 0x0123, 0x0127, 0x012A, 0x012D, 0x0131, 0x0134, 0x0138, 0x013C, 0x013F, // 25.1dB to 26.0dB
        0x0143, 0x0147, 0x014A, 0x014E, 0x0152, 0x0156, 0x015A, 0x015E, 0x0162, 0x0166, // 26.1dB to 27.0dB
        0x016A, 0x016F, 0x0173, 0x0177, 0x017B, 0x0180, 0x0184, 0x0189, 0x018D, 0x0192, // 27.1dB to 28.0dB
        0x0197, 0x019B, 0x01A0, 0x01A5, 0x01AA, 0x01AF, 0x01B4, 0x01B9, 0x01BE, 0x01C3, // 28.1dB to 29.0dB
        0x01C8, 0x01CD, 0x01D3, 0x01D8, 0x01DE, 0x01E3, 0x01E9, 0x01EE, 0x01F4, 0x01FA  // 29.1dB to 30.0dB

    };
	int ret;
	int db = value;

	if( db < 0)
	    db = 0;
	if( db > 300)
	    db = 300;
	    
	unsigned short reg_value_h = gain_to_reg[db] >> 8;
	unsigned short reg_value_l = gain_to_reg[db] & 0xFF;

	ret = ov5645_write_reg(ov5645, OV5645_GAIN_HIGH_BYTE, reg_value_h);
	if (ret < 0)
		return ret;

	return ov5645_write_reg(ov5645, OV5645_GAIN_LOW_BYTE, reg_value_l);
}

static int ov5645_set_expose(struct ov5645 *ov5645, s32 value)
{
	int ret;
    int shutter_time = value;
    if( shutter_time < 1 )
        shutter_time = 1;

    shutter_time = (shutter_time * 3 * 0x10); /* (shutter_time * 30 * 0x10) / 10 */
    if( shutter_time > 0xFFFFF0 )
        shutter_time = 0xFFFFF0;

	ret = ov5645_write_reg(ov5645, OV5645_EXPOSE_HIGH_BYTE, (shutter_time >> 16) & 0xFF);
	if (ret < 0)
		return ret;

	ret = ov5645_write_reg(ov5645, OV5645_EXPOSE_MID_BYTE, (shutter_time >> 8) & 0xFF);
	if (ret < 0)
		return ret;

	return ov5645_write_reg(ov5645, OV5645_EXPOSE_LOW_BYTE, shutter_time & 0xFF);

}

static int ov5645_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov5645 *ov5645 = container_of(ctrl->handler,
					     struct ov5645, ctrls);
	int ret;

	mutex_lock(&ov5645->power_lock);
	if (!ov5645->power_count) {
		mutex_unlock(&ov5645->power_lock);
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_SATURATION:
		ret = ov5645_set_saturation(ov5645, ctrl->val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = ov5645_set_awb(ov5645, ctrl->val);
		break;
	case V4L2_CID_AUTOGAIN:
		ret = ov5645_set_agc_mode(ov5645, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = ov5645_set_aec_mode(ov5645, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ov5645_set_test_pattern(ov5645, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = ov5645_set_hflip(ov5645, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = ov5645_set_vflip(ov5645, ctrl->val);
		break;
// [ISP Support PKG] Enabled to set GAIN/EXPOSE 
	case V4L2_CID_GAIN:
		ret = ov5645_set_gain(ov5645, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = ov5645_set_expose(ov5645, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&ov5645->power_lock);

	return ret;
}

static struct v4l2_ctrl_ops ov5645_ctrl_ops = {
	.s_ctrl = ov5645_s_ctrl,
};

static int ov5645_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

    code->code = MEDIA_BUS_FMT_SRGGB10_1X10;   // [ISP Support PKG] YUV -> RAW10

	return 0;
}

static int ov5645_enum_frame_size(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
    if (fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)   // [ISP Support PKG] YUV -> RAW10
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(ov5645_mode_info_data))
		return -EINVAL;

	fse->min_width = ov5645_mode_info_data[fse->index].width;
	fse->max_width = ov5645_mode_info_data[fse->index].width;
	fse->min_height = ov5645_mode_info_data[fse->index].height;
	fse->max_height = ov5645_mode_info_data[fse->index].height;

	return 0;
}

static int ov5645_enum_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	struct ov5645 *sensor = to_ov5645(sd);
	struct v4l2_fract tpf;
	int ret;

	if (fie->pad != 0)
		return -EINVAL;
	if (fie->index >= OV5645_NUM_FRAMERATES)
		return -EINVAL;

	tpf.numerator = 1;
	tpf.denominator = ov5645_framerates[fie->index];

	ret = ov5645_try_frame_interval(sensor, &tpf,
					fie->width, fie->height);
	if (ret < 0)
		return -EINVAL;

	fie->interval = tpf;
	return 0;
}

static int ov5645_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ov5645 *sensor = to_ov5645(sd);

	mutex_lock(&sensor->power_lock);
	fi->interval = sensor->frame_interval;
	mutex_unlock(&sensor->power_lock);

	return 0;
}

static int ov5645_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ov5645 *sensor = to_ov5645(sd);
	const struct ov5645_mode_info *mode;
	int frame_rate, ret = 0;

	if (fi->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->power_lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	mode = sensor->current_mode;

	frame_rate = ov5645_try_frame_interval(sensor, &fi->interval,
					       mode->width, mode->height);
	if (frame_rate < 0)
		frame_rate = OV5645_15_FPS;

	mode = ov5645_find_mode(sensor, frame_rate, mode->width,
				mode->height, true);
	if (!mode) {
		ret = -EINVAL;
		goto out;
	}

	if (mode != sensor->current_mode ||
	    frame_rate != sensor->current_fr) {
		sensor->current_fr = frame_rate;
		sensor->frame_interval = fi->interval;
		sensor->current_mode = mode;
	}
out:
	mutex_unlock(&sensor->power_lock);
	return ret;
}

static struct v4l2_mbus_framefmt *
__ov5645_get_pad_format(struct ov5645 *ov5645,
			struct v4l2_subdev_pad_config *cfg,
			unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&ov5645->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ov5645->fmt;
	default:
		return NULL;
	}
}

static int ov5645_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct ov5645 *ov5645 = to_ov5645(sd);

	format->format = *__ov5645_get_pad_format(ov5645, cfg, format->pad,
						  format->which);
	return 0;
}

static struct v4l2_rect *
__ov5645_get_pad_crop(struct ov5645 *ov5645, struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&ov5645->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ov5645->crop;
	default:
		return NULL;
	}
}

static int ov5645_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct ov5645 *ov5645 = to_ov5645(sd);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	const struct ov5645_mode_info *new_mode;
	int ret;

	__crop = __ov5645_get_pad_crop(ov5645, cfg, format->pad,
			format->which);

	new_mode = v4l2_find_nearest_size(ov5645_mode_info_data,
			       ARRAY_SIZE(ov5645_mode_info_data),
			       width, height,
			       format->format.width, format->format.height);

	__crop->width = new_mode->width;
	__crop->height = new_mode->height;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ret = v4l2_ctrl_s_ctrl_int64(ov5645->pixel_clock,
					     new_mode->pixel_clock);
		if (ret < 0)
			return ret;

		ret = v4l2_ctrl_s_ctrl(ov5645->link_freq,
				       new_mode->link_freq);
		if (ret < 0)
			return ret;

		ov5645->current_mode = new_mode;
	}

	__format = __ov5645_get_pad_format(ov5645, cfg, format->pad,
			format->which);
	__format->width = __crop->width;
	__format->height = __crop->height;
    __format->code = MEDIA_BUS_FMT_SRGGB10_1X10;   // [ISP Support PKG] YUV -> RAW10
	__format->field = V4L2_FIELD_NONE;
	__format->colorspace = V4L2_COLORSPACE_SRGB;

	format->format = *__format;

	return 0;
}

static int ov5645_entity_init_cfg(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = 1920;
	fmt.format.height = 1080;

	ov5645_set_format(subdev, cfg, &fmt);

	return 0;
}

static int ov5645_get_selection(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_selection *sel)
{
	struct ov5645 *ov5645 = to_ov5645(sd);

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sel->r = *__ov5645_get_pad_crop(ov5645, cfg, sel->pad,
					sel->which);
	return 0;
}

static int ov5645_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct ov5645 *ov5645 = to_ov5645(subdev);
	int ret;

	if (enable) {
		ret = ov5645_set_register_array(ov5645,
					ov5645->current_mode->data,
					ov5645->current_mode->data_size);
		if (ret < 0) {
			dev_err(ov5645->dev, "could not set mode %dx%d\n",
				ov5645->current_mode->width,
				ov5645->current_mode->height);
			return ret;
		}
		ret = v4l2_ctrl_handler_setup(&ov5645->ctrls);
		if (ret < 0) {
			dev_err(ov5645->dev, "could not sync v4l2 controls\n");
			return ret;
		}

		ret = ov5645_write_reg(ov5645, OV5645_IO_MIPI_CTRL00, 0x45);
		if (ret < 0)
			return ret;

		ret = ov5645_write_reg(ov5645, OV5645_SYSTEM_CTRL0,
				       OV5645_SYSTEM_CTRL0_START);
		if (ret < 0)
			return ret;

		ov5645->streaming = enable;
	} else {
		ov5645->streaming = false;
		ret = ov5645_write_reg(ov5645, OV5645_IO_MIPI_CTRL00, 0x40);
		if (ret < 0)
			return ret;

		ret = ov5645_write_reg(ov5645, OV5645_SYSTEM_CTRL0,
				       OV5645_SYSTEM_CTRL0_STOP);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static const struct v4l2_subdev_core_ops ov5645_core_ops = {
	.s_power = ov5645_s_power,
};

static const struct v4l2_subdev_video_ops ov5645_video_ops = {
	.g_frame_interval = ov5645_g_frame_interval,
	.s_frame_interval = ov5645_s_frame_interval,
	.s_stream = ov5645_s_stream,
};

static const struct v4l2_subdev_pad_ops ov5645_subdev_pad_ops = {
	.init_cfg = ov5645_entity_init_cfg,
	.enum_mbus_code = ov5645_enum_mbus_code,
	.enum_frame_size = ov5645_enum_frame_size,
	.get_fmt = ov5645_get_format,
	.set_fmt = ov5645_set_format,
	.get_selection = ov5645_get_selection,
	.enum_frame_interval = ov5645_enum_frame_interval,
};

static const struct v4l2_subdev_ops ov5645_subdev_ops = {
	.core = &ov5645_core_ops,
	.video = &ov5645_video_ops,
	.pad = &ov5645_subdev_pad_ops,
};

static int ov5645_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *endpoint;
	struct ov5645 *ov5645;
	u8 chip_id_high, chip_id_low;
	u32 xclk_freq;
	int ret;

	ov5645 = devm_kzalloc(dev, sizeof(struct ov5645), GFP_KERNEL);
	if (!ov5645)
		return -ENOMEM;

	ov5645->i2c_client = client;
	ov5645->dev = dev;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint),
					 &ov5645->ep);

	of_node_put(endpoint);

	if (ret < 0) {
		dev_err(dev, "parsing endpoint node failed\n");
		return ret;
	}

	if (ov5645->ep.bus_type != V4L2_MBUS_CSI2) {
		dev_err(dev, "invalid bus type, must be CSI2\n");
		return -EINVAL;
	}

	/* get system clock (xclk) */
	ov5645->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(ov5645->xclk)) {
		dev_err(dev, "could not get xclk");
		return PTR_ERR(ov5645->xclk);
	}

	ret = of_property_read_u32(dev->of_node, "clock-frequency", &xclk_freq);
	if (ret) {
		dev_err(dev, "could not get xclk frequency\n");
		return ret;
	}

	/* external clock must be 24MHz, allow 1% tolerance */
	if (xclk_freq < 23760000 || xclk_freq > 24240000) {
		dev_err(dev, "external clock frequency %u is not supported\n",
			xclk_freq);
		return -EINVAL;
	}

	ret = clk_set_rate(ov5645->xclk, xclk_freq);
	if (ret) {
		dev_err(dev, "could not set xclk frequency\n");
		return ret;
	}

	ov5645->io_regulator = devm_regulator_get(dev, "vdddo");
	if (IS_ERR(ov5645->io_regulator)) {
		dev_err(dev, "cannot get io regulator\n");
		return PTR_ERR(ov5645->io_regulator);
	}

	ret = regulator_set_voltage(ov5645->io_regulator,
				    OV5645_VOLTAGE_DIGITAL_IO,
				    OV5645_VOLTAGE_DIGITAL_IO);
	if (ret < 0) {
		dev_err(dev, "cannot set io voltage\n");
		return ret;
	}

	ov5645->core_regulator = devm_regulator_get(dev, "vddd");
	if (IS_ERR(ov5645->core_regulator)) {
		dev_err(dev, "cannot get core regulator\n");
		return PTR_ERR(ov5645->core_regulator);
	}

	ret = regulator_set_voltage(ov5645->core_regulator,
				    OV5645_VOLTAGE_DIGITAL_CORE,
				    OV5645_VOLTAGE_DIGITAL_CORE);
	if (ret < 0) {
		dev_err(dev, "cannot set core voltage\n");
		return ret;
	}

	ov5645->analog_regulator = devm_regulator_get(dev, "vdda");
	if (IS_ERR(ov5645->analog_regulator)) {
		dev_err(dev, "cannot get analog regulator\n");
		return PTR_ERR(ov5645->analog_regulator);
	}

	ret = regulator_set_voltage(ov5645->analog_regulator,
				    OV5645_VOLTAGE_ANALOG,
				    OV5645_VOLTAGE_ANALOG);
	if (ret < 0) {
		dev_err(dev, "cannot set analog voltage\n");
		return ret;
	}

	ov5645->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ov5645->enable_gpio)) {
		dev_err(dev, "cannot get enable gpio\n");
		return PTR_ERR(ov5645->enable_gpio);
	}

	ov5645->rst_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ov5645->rst_gpio)) {
		dev_err(dev, "cannot get reset gpio\n");
		return PTR_ERR(ov5645->rst_gpio);
	}

	mutex_init(&ov5645->power_lock);

	v4l2_ctrl_handler_init(&ov5645->ctrls, 9);
	v4l2_ctrl_new_std(&ov5645->ctrls, &ov5645_ctrl_ops,
			  V4L2_CID_SATURATION, -4, 4, 1, 0);
	v4l2_ctrl_new_std(&ov5645->ctrls, &ov5645_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&ov5645->ctrls, &ov5645_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&ov5645->ctrls, &ov5645_ctrl_ops,
			  V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
	v4l2_ctrl_new_std(&ov5645->ctrls, &ov5645_ctrl_ops,
			  V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 1);
// [ISP Support PKG] Enabled to set GAIN/EXPOSE 
	v4l2_ctrl_new_std(&ov5645->ctrls, &ov5645_ctrl_ops,
			  V4L2_CID_GAIN, 0, 300, 1, 1);
	v4l2_ctrl_new_std(&ov5645->ctrls, &ov5645_ctrl_ops,
			  V4L2_CID_EXPOSURE, 0, 0xFFFF0, 1, 1);
	v4l2_ctrl_new_std_menu(&ov5645->ctrls, &ov5645_ctrl_ops,
			       V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL,
			       0, V4L2_EXPOSURE_AUTO);
	v4l2_ctrl_new_std_menu_items(&ov5645->ctrls, &ov5645_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ov5645_test_pattern_menu) - 1,
				     0, 0, ov5645_test_pattern_menu);
	ov5645->pixel_clock = v4l2_ctrl_new_std(&ov5645->ctrls,
						&ov5645_ctrl_ops,
						V4L2_CID_PIXEL_RATE,
						1, INT_MAX, 1, 1);
	ov5645->link_freq = v4l2_ctrl_new_int_menu(&ov5645->ctrls,
						   &ov5645_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(link_freq) - 1,
						   0, link_freq);
	if (ov5645->link_freq)
		ov5645->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ov5645->sd.ctrl_handler = &ov5645->ctrls;

	if (ov5645->ctrls.error) {
		dev_err(dev, "%s: control initialization error %d\n",
		       __func__, ov5645->ctrls.error);
		ret = ov5645->ctrls.error;
		goto free_ctrl;
	}

	v4l2_i2c_subdev_init(&ov5645->sd, client, &ov5645_subdev_ops);
	ov5645->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov5645->pad.flags = MEDIA_PAD_FL_SOURCE;
	ov5645->sd.dev = &client->dev;
	ov5645->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&ov5645->sd.entity, 1, &ov5645->pad);
	if (ret < 0) {
		dev_err(dev, "could not register media entity\n");
		goto free_ctrl;
	}

	ret = ov5645_s_power(&ov5645->sd, true);
	if (ret < 0) {
		dev_err(dev, "could not power up OV5645\n");
		goto free_entity;
	}

	ret = ov5645_read_reg(ov5645, OV5645_CHIP_ID_HIGH, &chip_id_high);
	if (ret < 0 || chip_id_high != OV5645_CHIP_ID_HIGH_BYTE) {
		dev_err(dev, "could not read ID high\n");
		ret = -ENODEV;
		goto power_down;
	}
	ret = ov5645_read_reg(ov5645, OV5645_CHIP_ID_LOW, &chip_id_low);
	if (ret < 0 || chip_id_low != OV5645_CHIP_ID_LOW_BYTE) {
		dev_err(dev, "could not read ID low\n");
		ret = -ENODEV;
		goto power_down;
	}

	dev_info(dev, "OV5645 detected at address 0x%02x\n", client->addr);

	ret = ov5645_read_reg(ov5645, OV5645_AEC_PK_MANUAL,
			      &ov5645->aec_pk_manual);
	if (ret < 0) {
		dev_err(dev, "could not read AEC/AGC mode\n");
		ret = -ENODEV;
		goto power_down;
	}

	ret = ov5645_read_reg(ov5645, OV5645_TIMING_TC_REG20,
			      &ov5645->timing_tc_reg20);
	if (ret < 0) {
		dev_err(dev, "could not read vflip value\n");
		ret = -ENODEV;
		goto power_down;
	}

	ret = ov5645_read_reg(ov5645, OV5645_TIMING_TC_REG21,
			      &ov5645->timing_tc_reg21);
	if (ret < 0) {
		dev_err(dev, "could not read hflip value\n");
		ret = -ENODEV;
		goto power_down;
	}

	ov5645_s_power(&ov5645->sd, false);

	ret = v4l2_async_register_subdev(&ov5645->sd);
	if (ret < 0) {
		dev_err(dev, "could not register v4l2 device\n");
		goto free_entity;
	}

	ov5645_entity_init_cfg(&ov5645->sd, NULL);

	return 0;

power_down:
	ov5645_s_power(&ov5645->sd, false);
free_entity:
	media_entity_cleanup(&ov5645->sd.entity);
free_ctrl:
	v4l2_ctrl_handler_free(&ov5645->ctrls);
	mutex_destroy(&ov5645->power_lock);

	return ret;
}

static int ov5645_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5645 *ov5645 = to_ov5645(sd);

	v4l2_async_unregister_subdev(&ov5645->sd);
	media_entity_cleanup(&ov5645->sd.entity);
	v4l2_ctrl_handler_free(&ov5645->ctrls);
	mutex_destroy(&ov5645->power_lock);

	return 0;
}

static const struct i2c_device_id ov5645_id[] = {
	{ "ov5645", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ov5645_id);

static const struct of_device_id ov5645_of_match[] = {
	{ .compatible = "ovti,ov5645" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov5645_of_match);

static struct i2c_driver ov5645_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ov5645_of_match),
		.name  = "ov5645",
	},
	.probe  = ov5645_probe,
	.remove = ov5645_remove,
	.id_table = ov5645_id,
};

module_i2c_driver(ov5645_i2c_driver);

MODULE_DESCRIPTION("Omnivision OV5645 Camera Driver");
MODULE_AUTHOR("Todor Tomov <todor.tomov@linaro.org>");
MODULE_LICENSE("GPL v2");
