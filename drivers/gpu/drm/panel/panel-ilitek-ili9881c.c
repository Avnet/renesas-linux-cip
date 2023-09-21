/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2017-2018, Bootlin
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_crtc.h>
#include <video/mipi_display.h>
#include <video/videomode.h>

/* #define CONFIG_SELFTEST_MODE */

enum ili9881c_op {
	ILI9881C_SWITCH_PAGE,
	ILI9881C_COMMAND,
};

struct ili9881c_instr {
	enum ili9881c_op	op;

	union arg {
		struct cmd {
			u8	cmd;
			u8	data;
		} cmd;
		u8	page;
	} arg;
};

struct panel_desc {
	const struct drm_display_mode *mode;
	unsigned int width_mm;
	unsigned int height_mm;

	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;
	unsigned int lanes;

	const struct ili9881c_instr *init_code;
	unsigned int init_code_length;
};

struct ili9881c_panel {
	struct drm_panel	panel;
	struct mipi_dsi_device	*dsi;
	const struct panel_desc *desc;

	struct gpio_desc	*power_gpio;
	struct gpio_desc	*enable_gpio;
	struct gpio_desc	*reset_gpio;
};


#define ILI9881C_SWITCH_PAGE_INSTR(_page)	\
	{					\
		.op = ILI9881C_SWITCH_PAGE, \
		.arg = {			\
			.page = (_page),	\
		},				\
	}

#define ILI9881C_COMMAND_INSTR(_cmd, _data)		\
	{						\
		.op = ILI9881C_COMMAND,		\
		.arg = {				\
			.cmd = {			\
				.cmd = (_cmd),		\
				.data = (_data),	\
			},				\
		},					\
	}

/* support old panel PH720128T003-ZBC02 */
static const struct ili9881c_instr ili9881c_init_ph720128t003[] = {
	ILI9881C_SWITCH_PAGE_INSTR(3),
	ILI9881C_COMMAND_INSTR(0x01, 0x00),
	ILI9881C_COMMAND_INSTR(0x02, 0x00),
	ILI9881C_COMMAND_INSTR(0x03, 0x55),
	ILI9881C_COMMAND_INSTR(0x04, 0x13),
	ILI9881C_COMMAND_INSTR(0x05, 0x00),
	ILI9881C_COMMAND_INSTR(0x06, 0x06),
	ILI9881C_COMMAND_INSTR(0x07, 0x01),
	ILI9881C_COMMAND_INSTR(0x08, 0x00),
	ILI9881C_COMMAND_INSTR(0x09, 0x01),
	ILI9881C_COMMAND_INSTR(0x0a, 0x01),
	ILI9881C_COMMAND_INSTR(0x0b, 0x00),
	ILI9881C_COMMAND_INSTR(0x0c, 0x00),
	ILI9881C_COMMAND_INSTR(0x0d, 0x00),
	ILI9881C_COMMAND_INSTR(0x0e, 0x00),
	ILI9881C_COMMAND_INSTR(0x0f, 0x18),
	ILI9881C_COMMAND_INSTR(0x10, 0x18),
	ILI9881C_COMMAND_INSTR(0x11, 0x00),
	ILI9881C_COMMAND_INSTR(0x12, 0x00),
	ILI9881C_COMMAND_INSTR(0x13, 0x00),
	ILI9881C_COMMAND_INSTR(0x14, 0x00),
	ILI9881C_COMMAND_INSTR(0x15, 0x00),
	ILI9881C_COMMAND_INSTR(0x16, 0x00),
	ILI9881C_COMMAND_INSTR(0x17, 0x00),
	ILI9881C_COMMAND_INSTR(0x18, 0x00),
	ILI9881C_COMMAND_INSTR(0x19, 0x00),
	ILI9881C_COMMAND_INSTR(0x1a, 0x00),
	ILI9881C_COMMAND_INSTR(0x1b, 0x00),
	ILI9881C_COMMAND_INSTR(0x1c, 0x00),
	ILI9881C_COMMAND_INSTR(0x1d, 0x00),
	ILI9881C_COMMAND_INSTR(0x1e, 0x44),
	ILI9881C_COMMAND_INSTR(0x1f, 0x80),
	ILI9881C_COMMAND_INSTR(0x20, 0x02),
	ILI9881C_COMMAND_INSTR(0x21, 0x03),
	ILI9881C_COMMAND_INSTR(0x22, 0x00),
	ILI9881C_COMMAND_INSTR(0x23, 0x00),
	ILI9881C_COMMAND_INSTR(0x24, 0x00),
	ILI9881C_COMMAND_INSTR(0x25, 0x00),
	ILI9881C_COMMAND_INSTR(0x26, 0x00),
	ILI9881C_COMMAND_INSTR(0x27, 0x00),
	ILI9881C_COMMAND_INSTR(0x28, 0x33),
	ILI9881C_COMMAND_INSTR(0x29, 0x03),
	ILI9881C_COMMAND_INSTR(0x2a, 0x00),
	ILI9881C_COMMAND_INSTR(0x2b, 0x00),
	ILI9881C_COMMAND_INSTR(0x2c, 0x00),
	ILI9881C_COMMAND_INSTR(0x2d, 0x00),
	ILI9881C_COMMAND_INSTR(0x2e, 0x00),
	ILI9881C_COMMAND_INSTR(0x2f, 0x00),
	ILI9881C_COMMAND_INSTR(0x30, 0x00),
	ILI9881C_COMMAND_INSTR(0x31, 0x00),
	ILI9881C_COMMAND_INSTR(0x32, 0x00),
	ILI9881C_COMMAND_INSTR(0x33, 0x00),
	ILI9881C_COMMAND_INSTR(0x34, 0x04),
	ILI9881C_COMMAND_INSTR(0x35, 0x00),
	ILI9881C_COMMAND_INSTR(0x36, 0x00),
	ILI9881C_COMMAND_INSTR(0x37, 0x00),
	ILI9881C_COMMAND_INSTR(0x38, 0x01),
	ILI9881C_COMMAND_INSTR(0x39, 0x00),
	ILI9881C_COMMAND_INSTR(0x3a, 0x00),
	ILI9881C_COMMAND_INSTR(0x3b, 0x00),
	ILI9881C_COMMAND_INSTR(0x3c, 0x00),
	ILI9881C_COMMAND_INSTR(0x3d, 0x00),
	ILI9881C_COMMAND_INSTR(0x3e, 0x00),
	ILI9881C_COMMAND_INSTR(0x3f, 0x00),
	ILI9881C_COMMAND_INSTR(0x40, 0x00),
	ILI9881C_COMMAND_INSTR(0x41, 0x00),
	ILI9881C_COMMAND_INSTR(0x42, 0x00),
	ILI9881C_COMMAND_INSTR(0x43, 0x00),
	ILI9881C_COMMAND_INSTR(0x44, 0x00),
	ILI9881C_COMMAND_INSTR(0x50, 0x01),
	ILI9881C_COMMAND_INSTR(0x51, 0x23),
	ILI9881C_COMMAND_INSTR(0x52, 0x45),
	ILI9881C_COMMAND_INSTR(0x53, 0x67),
	ILI9881C_COMMAND_INSTR(0x54, 0x89),
	ILI9881C_COMMAND_INSTR(0x55, 0xab),
	ILI9881C_COMMAND_INSTR(0x56, 0x01),
	ILI9881C_COMMAND_INSTR(0x57, 0x23),
	ILI9881C_COMMAND_INSTR(0x58, 0x45),
	ILI9881C_COMMAND_INSTR(0x59, 0x67),
	ILI9881C_COMMAND_INSTR(0x5a, 0x89),
	ILI9881C_COMMAND_INSTR(0x5b, 0xab),
	ILI9881C_COMMAND_INSTR(0x5c, 0xcd),
	ILI9881C_COMMAND_INSTR(0x5d, 0xef),
	ILI9881C_COMMAND_INSTR(0x5e, 0x11),
	ILI9881C_COMMAND_INSTR(0x5f, 0x14),
	ILI9881C_COMMAND_INSTR(0x60, 0x15),
	ILI9881C_COMMAND_INSTR(0x61, 0x0f),
	ILI9881C_COMMAND_INSTR(0x62, 0x0d),
	ILI9881C_COMMAND_INSTR(0x63, 0x0e),
	ILI9881C_COMMAND_INSTR(0x64, 0x0c),
	ILI9881C_COMMAND_INSTR(0x65, 0x06),
	ILI9881C_COMMAND_INSTR(0x66, 0x02),
	ILI9881C_COMMAND_INSTR(0x67, 0x02),
	ILI9881C_COMMAND_INSTR(0x68, 0x02),
	ILI9881C_COMMAND_INSTR(0x69, 0x02),
	ILI9881C_COMMAND_INSTR(0x6a, 0x02),
	ILI9881C_COMMAND_INSTR(0x6b, 0x02),
	ILI9881C_COMMAND_INSTR(0x6c, 0x02),
	ILI9881C_COMMAND_INSTR(0x6d, 0x02),
	ILI9881C_COMMAND_INSTR(0x6e, 0x02),
	ILI9881C_COMMAND_INSTR(0x6f, 0x02),
	ILI9881C_COMMAND_INSTR(0x70, 0x02),
	ILI9881C_COMMAND_INSTR(0x71, 0x00),
	ILI9881C_COMMAND_INSTR(0x72, 0x01),
	ILI9881C_COMMAND_INSTR(0x73, 0x08),
	ILI9881C_COMMAND_INSTR(0x74, 0x02),
	ILI9881C_COMMAND_INSTR(0x75, 0x14),
	ILI9881C_COMMAND_INSTR(0x76, 0x15),
	ILI9881C_COMMAND_INSTR(0x77, 0x0f),
	ILI9881C_COMMAND_INSTR(0x78, 0x0d),
	ILI9881C_COMMAND_INSTR(0x79, 0x0e),
	ILI9881C_COMMAND_INSTR(0x7a, 0x0c),
	ILI9881C_COMMAND_INSTR(0x7b, 0x08),
	ILI9881C_COMMAND_INSTR(0x7c, 0x02),
	ILI9881C_COMMAND_INSTR(0x7d, 0x02),
	ILI9881C_COMMAND_INSTR(0x7e, 0x02),
	ILI9881C_COMMAND_INSTR(0x7f, 0x02),
	ILI9881C_COMMAND_INSTR(0x80, 0x02),
	ILI9881C_COMMAND_INSTR(0x81, 0x02),
	ILI9881C_COMMAND_INSTR(0x82, 0x02),
	ILI9881C_COMMAND_INSTR(0x83, 0x02),
	ILI9881C_COMMAND_INSTR(0x84, 0x02),
	ILI9881C_COMMAND_INSTR(0x85, 0x02),
	ILI9881C_COMMAND_INSTR(0x86, 0x02),
	ILI9881C_COMMAND_INSTR(0x87, 0x00),
	ILI9881C_COMMAND_INSTR(0x88, 0x01),
	ILI9881C_COMMAND_INSTR(0x89, 0x06),
	ILI9881C_COMMAND_INSTR(0x8A, 0x02),
	ILI9881C_SWITCH_PAGE_INSTR(4),
	ILI9881C_COMMAND_INSTR(0x6C, 0x15),
	ILI9881C_COMMAND_INSTR(0x6E, 0x2a),
	ILI9881C_COMMAND_INSTR(0x6F, 0x33),
	ILI9881C_COMMAND_INSTR(0x3A, 0x24),
	ILI9881C_COMMAND_INSTR(0x8D, 0x14),
	ILI9881C_COMMAND_INSTR(0x87, 0xBA),
	ILI9881C_COMMAND_INSTR(0x26, 0x76),
	ILI9881C_COMMAND_INSTR(0xB2, 0xD1),
	ILI9881C_COMMAND_INSTR(0xB5, 0xD7),
	ILI9881C_COMMAND_INSTR(0x35, 0x1f),
	ILI9881C_SWITCH_PAGE_INSTR(1),
	ILI9881C_COMMAND_INSTR(0x22, 0x0A),
	ILI9881C_COMMAND_INSTR(0x53, 0x72),
	ILI9881C_COMMAND_INSTR(0x55, 0x77),
	ILI9881C_COMMAND_INSTR(0x50, 0xa6),
	ILI9881C_COMMAND_INSTR(0x51, 0xa6),
	ILI9881C_COMMAND_INSTR(0x31, 0x00),
	ILI9881C_COMMAND_INSTR(0x60, 0x20),
	ILI9881C_COMMAND_INSTR(0xA0, 0x08),
	ILI9881C_COMMAND_INSTR(0xA1, 0x1a),
	ILI9881C_COMMAND_INSTR(0xA2, 0x2a),
	ILI9881C_COMMAND_INSTR(0xA3, 0x14),
	ILI9881C_COMMAND_INSTR(0xA4, 0x17),
	ILI9881C_COMMAND_INSTR(0xA5, 0x2b),
	ILI9881C_COMMAND_INSTR(0xA6, 0x1d),
	ILI9881C_COMMAND_INSTR(0xA7, 0x20),
	ILI9881C_COMMAND_INSTR(0xA8, 0x9d),
	ILI9881C_COMMAND_INSTR(0xA9, 0x1C),
	ILI9881C_COMMAND_INSTR(0xAA, 0x29),
	ILI9881C_COMMAND_INSTR(0xAB, 0x8f),
	ILI9881C_COMMAND_INSTR(0xAC, 0x20),
	ILI9881C_COMMAND_INSTR(0xAD, 0x1f),
	ILI9881C_COMMAND_INSTR(0xAE, 0x4f),
	ILI9881C_COMMAND_INSTR(0xAF, 0x23),
	ILI9881C_COMMAND_INSTR(0xB0, 0x29),
	ILI9881C_COMMAND_INSTR(0xB1, 0x56),
	ILI9881C_COMMAND_INSTR(0xB2, 0x66),
	ILI9881C_COMMAND_INSTR(0xB3, 0x39),
	ILI9881C_COMMAND_INSTR(0xC0, 0x08),
	ILI9881C_COMMAND_INSTR(0xC1, 0x1a),
	ILI9881C_COMMAND_INSTR(0xC2, 0x2a),
	ILI9881C_COMMAND_INSTR(0xC3, 0x15),
	ILI9881C_COMMAND_INSTR(0xC4, 0x17),
	ILI9881C_COMMAND_INSTR(0xC5, 0x2b),
	ILI9881C_COMMAND_INSTR(0xC6, 0x1d),
	ILI9881C_COMMAND_INSTR(0xC7, 0x20),
	ILI9881C_COMMAND_INSTR(0xC8, 0x9d),
	ILI9881C_COMMAND_INSTR(0xC9, 0x1d),
	ILI9881C_COMMAND_INSTR(0xCA, 0x29),
	ILI9881C_COMMAND_INSTR(0xCB, 0x8f),
	ILI9881C_COMMAND_INSTR(0xCC, 0x20),
	ILI9881C_COMMAND_INSTR(0xCD, 0x1f),
	ILI9881C_COMMAND_INSTR(0xCE, 0x4f),
	ILI9881C_COMMAND_INSTR(0xCF, 0x24),
	ILI9881C_COMMAND_INSTR(0xD0, 0x29),
	ILI9881C_COMMAND_INSTR(0xD1, 0x56),
	ILI9881C_COMMAND_INSTR(0xD2, 0x66),
	ILI9881C_COMMAND_INSTR(0xD3, 0x39),
#ifdef CONFIG_SELFTEST_MODE
/* BIST mode (Built-in Self-test Pattern)*/
	ILI9881C_SWITCH_PAGE_INSTR(4),
	ILI9881C_COMMAND_INSTR(0x2d, 0x08),
	ILI9881C_COMMAND_INSTR(0x2f, 0x11),
#endif
};

/* support new panel PH720128T005ZBC */
static const struct ili9881c_instr ili9881c_init_ph720128t005[] = {
	ILI9881C_SWITCH_PAGE_INSTR(3),
	ILI9881C_COMMAND_INSTR(0x01, 0x00),
	ILI9881C_COMMAND_INSTR(0x02, 0x00),
	ILI9881C_COMMAND_INSTR(0x03, 0x73),
	ILI9881C_COMMAND_INSTR(0x04, 0x00),
	ILI9881C_COMMAND_INSTR(0x05, 0x00),
	ILI9881C_COMMAND_INSTR(0x06, 0x0A),
	ILI9881C_COMMAND_INSTR(0x07, 0x00),
	ILI9881C_COMMAND_INSTR(0x08, 0x00),
	ILI9881C_COMMAND_INSTR(0x09, 0x61),
	ILI9881C_COMMAND_INSTR(0x0A, 0x00),
	ILI9881C_COMMAND_INSTR(0x0B, 0x00),
	ILI9881C_COMMAND_INSTR(0x0C, 0x01),
	ILI9881C_COMMAND_INSTR(0x0D, 0x00),
	ILI9881C_COMMAND_INSTR(0x0E, 0x00),
	ILI9881C_COMMAND_INSTR(0x0F, 0x61),
	ILI9881C_COMMAND_INSTR(0x10, 0x61),
	ILI9881C_COMMAND_INSTR(0x11, 0x00),
	ILI9881C_COMMAND_INSTR(0x12, 0x00),
	ILI9881C_COMMAND_INSTR(0x13, 0x00),
	ILI9881C_COMMAND_INSTR(0x14, 0x00),
	ILI9881C_COMMAND_INSTR(0x15, 0x00),
	ILI9881C_COMMAND_INSTR(0x16, 0x00),
	ILI9881C_COMMAND_INSTR(0x17, 0x00),
	ILI9881C_COMMAND_INSTR(0x18, 0x00),
	ILI9881C_COMMAND_INSTR(0x19, 0x00),
	ILI9881C_COMMAND_INSTR(0x1A, 0x00),
	ILI9881C_COMMAND_INSTR(0x1B, 0x00),
	ILI9881C_COMMAND_INSTR(0x1C, 0x00),
	ILI9881C_COMMAND_INSTR(0x1D, 0x00),
	ILI9881C_COMMAND_INSTR(0x1E, 0x40),
	ILI9881C_COMMAND_INSTR(0x1F, 0x80),
	ILI9881C_COMMAND_INSTR(0x20, 0x06),
	ILI9881C_COMMAND_INSTR(0x21, 0x01),
	ILI9881C_COMMAND_INSTR(0x22, 0x00),
	ILI9881C_COMMAND_INSTR(0x23, 0x00),
	ILI9881C_COMMAND_INSTR(0x24, 0x00),
	ILI9881C_COMMAND_INSTR(0x25, 0x00),
	ILI9881C_COMMAND_INSTR(0x26, 0x00),
	ILI9881C_COMMAND_INSTR(0x27, 0x00),
	ILI9881C_COMMAND_INSTR(0x28, 0x33),
	ILI9881C_COMMAND_INSTR(0x29, 0x03),
	ILI9881C_COMMAND_INSTR(0x2A, 0x00),
	ILI9881C_COMMAND_INSTR(0x2B, 0x00),
	ILI9881C_COMMAND_INSTR(0x2C, 0x00),
	ILI9881C_COMMAND_INSTR(0x2D, 0x00),
	ILI9881C_COMMAND_INSTR(0x2E, 0x00),
	ILI9881C_COMMAND_INSTR(0x2F, 0x00),
	ILI9881C_COMMAND_INSTR(0x30, 0x00),
	ILI9881C_COMMAND_INSTR(0x31, 0x00),
	ILI9881C_COMMAND_INSTR(0x32, 0x00),
	ILI9881C_COMMAND_INSTR(0x33, 0x00),
	ILI9881C_COMMAND_INSTR(0x34, 0x04),
	ILI9881C_COMMAND_INSTR(0x35, 0x00),
	ILI9881C_COMMAND_INSTR(0x36, 0x00),
	ILI9881C_COMMAND_INSTR(0x37, 0x00),
	ILI9881C_COMMAND_INSTR(0x38, 0x3C),
	ILI9881C_COMMAND_INSTR(0x39, 0x00),
	ILI9881C_COMMAND_INSTR(0x3A, 0x00),
	ILI9881C_COMMAND_INSTR(0x3B, 0x00),
	ILI9881C_COMMAND_INSTR(0x3C, 0x00),
	ILI9881C_COMMAND_INSTR(0x3D, 0x00),
	ILI9881C_COMMAND_INSTR(0x3E, 0x00),
	ILI9881C_COMMAND_INSTR(0x3F, 0x00),
	ILI9881C_COMMAND_INSTR(0x40, 0x00),
	ILI9881C_COMMAND_INSTR(0x41, 0x00),
	ILI9881C_COMMAND_INSTR(0x42, 0x00),
	ILI9881C_COMMAND_INSTR(0x43, 0x00),
	ILI9881C_COMMAND_INSTR(0x44, 0x00),
	ILI9881C_COMMAND_INSTR(0x50, 0x10),
	ILI9881C_COMMAND_INSTR(0x51, 0x32),
	ILI9881C_COMMAND_INSTR(0x52, 0x54),
	ILI9881C_COMMAND_INSTR(0x53, 0x76),
	ILI9881C_COMMAND_INSTR(0x54, 0x98),
	ILI9881C_COMMAND_INSTR(0x55, 0xBA),
	ILI9881C_COMMAND_INSTR(0x56, 0x10),
	ILI9881C_COMMAND_INSTR(0x57, 0x32),
	ILI9881C_COMMAND_INSTR(0x58, 0x54),
	ILI9881C_COMMAND_INSTR(0x59, 0x76),
	ILI9881C_COMMAND_INSTR(0x5A, 0x98),
	ILI9881C_COMMAND_INSTR(0x5B, 0xBA),
	ILI9881C_COMMAND_INSTR(0x5C, 0xDC),
	ILI9881C_COMMAND_INSTR(0x5D, 0xFE),
	ILI9881C_COMMAND_INSTR(0x5E, 0x00),
	ILI9881C_COMMAND_INSTR(0x5F, 0x0E),
	ILI9881C_COMMAND_INSTR(0x60, 0x0F),
	ILI9881C_COMMAND_INSTR(0x61, 0x0C),
	ILI9881C_COMMAND_INSTR(0x62, 0x0D),
	ILI9881C_COMMAND_INSTR(0x63, 0x06),
	ILI9881C_COMMAND_INSTR(0x64, 0x07),
	ILI9881C_COMMAND_INSTR(0x65, 0x02),
	ILI9881C_COMMAND_INSTR(0x66, 0x02),
	ILI9881C_COMMAND_INSTR(0x67, 0x02),
	ILI9881C_COMMAND_INSTR(0x68, 0x02),
	ILI9881C_COMMAND_INSTR(0x69, 0x01),
	ILI9881C_COMMAND_INSTR(0x6A, 0x00),
	ILI9881C_COMMAND_INSTR(0x6B, 0x02),
	ILI9881C_COMMAND_INSTR(0x6C, 0x15),
	ILI9881C_COMMAND_INSTR(0x6D, 0x14),
	ILI9881C_COMMAND_INSTR(0x6E, 0x02),
	ILI9881C_COMMAND_INSTR(0x6F, 0x02),
	ILI9881C_COMMAND_INSTR(0x70, 0x02),
	ILI9881C_COMMAND_INSTR(0x71, 0x02),
	ILI9881C_COMMAND_INSTR(0x72, 0x02),
	ILI9881C_COMMAND_INSTR(0x73, 0x02),
	ILI9881C_COMMAND_INSTR(0x74, 0x02),
	ILI9881C_COMMAND_INSTR(0x75, 0x0E),
	ILI9881C_COMMAND_INSTR(0x76, 0x0F),
	ILI9881C_COMMAND_INSTR(0x77, 0x0C),
	ILI9881C_COMMAND_INSTR(0x78, 0x0D),
	ILI9881C_COMMAND_INSTR(0x79, 0x06),
	ILI9881C_COMMAND_INSTR(0x7A, 0x07),
	ILI9881C_COMMAND_INSTR(0x7B, 0x02),
	ILI9881C_COMMAND_INSTR(0x7C, 0x02),
	ILI9881C_COMMAND_INSTR(0x7D, 0x02),
	ILI9881C_COMMAND_INSTR(0x7E, 0x02),
	ILI9881C_COMMAND_INSTR(0x7F, 0x01),
	ILI9881C_COMMAND_INSTR(0x80, 0x00),
	ILI9881C_COMMAND_INSTR(0x81, 0x02),
	ILI9881C_COMMAND_INSTR(0x82, 0x14),
	ILI9881C_COMMAND_INSTR(0x83, 0x15),
	ILI9881C_COMMAND_INSTR(0x84, 0x02),
	ILI9881C_COMMAND_INSTR(0x85, 0x02),
	ILI9881C_COMMAND_INSTR(0x86, 0x02),
	ILI9881C_COMMAND_INSTR(0x87, 0x02),
	ILI9881C_COMMAND_INSTR(0x88, 0x02),
	ILI9881C_COMMAND_INSTR(0x89, 0x02),
	ILI9881C_COMMAND_INSTR(0x8A, 0x02),
	ILI9881C_COMMAND_INSTR(0x043, 0x00),
	ILI9881C_SWITCH_PAGE_INSTR(4),
	ILI9881C_COMMAND_INSTR(0x6C, 0x15),
	ILI9881C_COMMAND_INSTR(0x6E, 0x2A),
	ILI9881C_COMMAND_INSTR(0x6F, 0x33),
	ILI9881C_COMMAND_INSTR(0x3B, 0x98),
	ILI9881C_COMMAND_INSTR(0x3A, 0x94),
	ILI9881C_COMMAND_INSTR(0x8D, 0x14),
	ILI9881C_COMMAND_INSTR(0x87, 0xBA),
	ILI9881C_COMMAND_INSTR(0x26, 0x76),
	ILI9881C_COMMAND_INSTR(0xB2, 0xD1),
	ILI9881C_COMMAND_INSTR(0xB5, 0x06),
	ILI9881C_COMMAND_INSTR(0x38, 0x01),
	ILI9881C_COMMAND_INSTR(0x39, 0x00),
	ILI9881C_COMMAND_INSTR(0x088, 0x0B),
	ILI9881C_SWITCH_PAGE_INSTR(1),
	#ifdef AVT_DISPALY_ROTATE_180
	ILI9881C_COMMAND_INSTR(0x22, 0x09),
	#else
	ILI9881C_COMMAND_INSTR(0x22, 0x0A),
	#endif
	ILI9881C_COMMAND_INSTR(0x31, 0x00),
	ILI9881C_COMMAND_INSTR(0x53, 0x7D),
	ILI9881C_COMMAND_INSTR(0x55, 0x8F),
	ILI9881C_COMMAND_INSTR(0x40, 0x33),
	ILI9881C_COMMAND_INSTR(0x50, 0x96),
	ILI9881C_COMMAND_INSTR(0x51, 0x96),
	ILI9881C_COMMAND_INSTR(0x60, 0x23),
	ILI9881C_COMMAND_INSTR(0xA0, 0x08),
	ILI9881C_COMMAND_INSTR(0xA1, 0x1D),
	ILI9881C_COMMAND_INSTR(0xA2, 0x2A),
	ILI9881C_COMMAND_INSTR(0xA3, 0x10),
	ILI9881C_COMMAND_INSTR(0xA4, 0x15),
	ILI9881C_COMMAND_INSTR(0xA5, 0x28),
	ILI9881C_COMMAND_INSTR(0xA6, 0x1C),
	ILI9881C_COMMAND_INSTR(0xA7, 0x1D),
	ILI9881C_COMMAND_INSTR(0xA8, 0x7E),
	ILI9881C_COMMAND_INSTR(0xA9, 0x1D),
	ILI9881C_COMMAND_INSTR(0xAA, 0x29),
	ILI9881C_COMMAND_INSTR(0xAB, 0x6B),
	ILI9881C_COMMAND_INSTR(0xAC, 0x1A),
	ILI9881C_COMMAND_INSTR(0xAD, 0x18),
	ILI9881C_COMMAND_INSTR(0xAE, 0x4B),
	ILI9881C_COMMAND_INSTR(0xAF, 0x20),
	ILI9881C_COMMAND_INSTR(0xB0, 0x27),
	ILI9881C_COMMAND_INSTR(0xB1, 0x50),
	ILI9881C_COMMAND_INSTR(0xB2, 0x64),
	ILI9881C_COMMAND_INSTR(0xB3, 0x39),
	ILI9881C_COMMAND_INSTR(0xC0, 0x08),
	ILI9881C_COMMAND_INSTR(0xC1, 0x1D),
	ILI9881C_COMMAND_INSTR(0xC2, 0x2A),
	ILI9881C_COMMAND_INSTR(0xC3, 0x10),
	ILI9881C_COMMAND_INSTR(0xC4, 0x15),
	ILI9881C_COMMAND_INSTR(0xC5, 0x28),
	ILI9881C_COMMAND_INSTR(0xC6, 0x1C),
	ILI9881C_COMMAND_INSTR(0xC7, 0x1D),
	ILI9881C_COMMAND_INSTR(0xC8, 0x7E),
	ILI9881C_COMMAND_INSTR(0xC9, 0x1D),
	ILI9881C_COMMAND_INSTR(0xCA, 0x29),
	ILI9881C_COMMAND_INSTR(0xCB, 0x6B),
	ILI9881C_COMMAND_INSTR(0xCC, 0x1A),
	ILI9881C_COMMAND_INSTR(0xCD, 0x18),
	ILI9881C_COMMAND_INSTR(0xCE, 0x4B),
	ILI9881C_COMMAND_INSTR(0xCF, 0x20),
	ILI9881C_COMMAND_INSTR(0xD0, 0x27),
	ILI9881C_COMMAND_INSTR(0xD1, 0x50),
	ILI9881C_COMMAND_INSTR(0xD2, 0x64),
	ILI9881C_COMMAND_INSTR(0xD3, 0x39),
	ILI9881C_SWITCH_PAGE_INSTR(0),
	ILI9881C_COMMAND_INSTR(0x35, 0x00),
	ILI9881C_COMMAND_INSTR(0x11, 0x00),
#ifdef CONFIG_SELFTEST_MODE
/* BIST mode (Built-in Self-test Pattern)*/
	ILI9881C_SWITCH_PAGE_INSTR(4),
	ILI9881C_COMMAND_INSTR(0x2d, 0x08),
	ILI9881C_COMMAND_INSTR(0x2f, 0x11),
#endif
};


static inline struct ili9881c_panel *panel_to_ili9881c(struct drm_panel *panel)
{
	return container_of(panel, struct ili9881c_panel, panel);
}

static int ili9881c_switch_page(struct ili9881c_panel *ctx, u8 page)
{
	u8 buf[4] = { 0xff, 0x98, 0x81, page };
	int ret;

	ret = mipi_dsi_dcs_write_buffer(ctx->dsi, buf, sizeof(buf));
	if (ret < 0) {
		dev_err(&ctx->dsi->dev, "Failed to switch_page[%d] (%d)\n", page, ret);
		return ret;
	}

	return 0;
}

static int ili9881c_send_cmd_data(struct ili9881c_panel *ctx, u8 cmd, u8 data)
{
	u8 buf[2] = { cmd, data };
	int ret;

	ret = mipi_dsi_dcs_write_buffer(ctx->dsi, buf, sizeof(buf));
	if (ret < 0) {
		dev_err(&ctx->dsi->dev, "Failed to send_cmd_data[%02x,%02X] (%d)\n", cmd, data, ret);
		return ret;
	}

	return 0;
}

static int ili9881c_read_cmd_data(struct ili9881c_panel *ctx, u8 cmd)
{
	u8 buf = 0;
	int ret;

	ret = mipi_dsi_dcs_read(ctx->dsi, cmd, &buf, sizeof(buf));
	if (ret < 0) {
		dev_err(&ctx->dsi->dev, "Failed to get ID (%d)\n", ret);
		return ret;
	}

	return buf;
}

static void ili9881c_getID(struct ili9881c_panel *ctx)
{
	u8 id[3];

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ili9881c_switch_page(ctx, 1);
	id[0] = ili9881c_read_cmd_data(ctx, 0x00);
	id[1] = ili9881c_read_cmd_data(ctx, 0x01);
	id[2] = ili9881c_read_cmd_data(ctx, 0x02);

	dev_info(&ctx->dsi->dev, "ID: 0x%02X 0x%02X 0x%02X \n", id[0], id[1], id[2]);
}

static void ili9881c_reset(struct ili9881c_panel *ctx)
{
	/* Reset 5ms */
	if (ctx->reset_gpio) {
		dev_dbg(&ctx->dsi->dev,"reset the chip\n");
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		usleep_range(5000, 10000);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		usleep_range(20000, 25000);
	}
}

static int ili9881c_prepare(struct drm_panel *panel)
{
	struct ili9881c_panel *ctx = panel_to_ili9881c(panel);

	dev_dbg(&ctx->dsi->dev,"%s\n",__func__);


	ili9881c_reset(ctx);

	return 0;
}

static int ili9881c_enable(struct drm_panel *panel)
{
	struct ili9881c_panel *ctx = panel_to_ili9881c(panel);
	const struct ili9881c_instr *cmds = ctx->desc->init_code;
	unsigned int len = ctx->desc->init_code_length;
	int ret;
	unsigned int i;

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	for (i = 0; i < len; i++) {
		const struct ili9881c_instr *instr = &cmds[i];

		if (instr->op == ILI9881C_SWITCH_PAGE)
			ret = ili9881c_switch_page(ctx, instr->arg.page);
		else if (instr->op == ILI9881C_COMMAND)
			ret = ili9881c_send_cmd_data(ctx, instr->arg.cmd.cmd,
							instr->arg.cmd.data);
		if (ret)
			return ret;
	}

	ret = ili9881c_switch_page(ctx, 0);
	if (ret)
		return ret;

	/* Set tear ON */
	ret = mipi_dsi_dcs_set_tear_on(ctx->dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(&ctx->dsi->dev, "Failed to set tear ON (%d)\n", ret);
		return ret;
	}

	/* Exit sleep mode */
	ret = mipi_dsi_dcs_exit_sleep_mode(ctx->dsi);
	if (ret < 0) {
		dev_err(&ctx->dsi->dev, "Failed to exit sleep mode (%d)\n", ret);
		return ret;
	}

	usleep_range(120000, 130000);

	ret = mipi_dsi_dcs_set_display_on(ctx->dsi);
	if (ret < 0) {
		dev_err(&ctx->dsi->dev, "Failed to set display ON (%d)\n", ret);
		return ret;
	}

	dev_dbg(&ctx->dsi->dev,"%s\n",__func__);
	ili9881c_getID(ctx);

	return 0;
}

static int ili9881c_disable(struct drm_panel *panel)
{
	struct ili9881c_panel *ctx = panel_to_ili9881c(panel);
	int ret;

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(ctx->dsi);
	if (ret < 0) {
		dev_err(&ctx->dsi->dev, "Failed to set display OFF (%d)\n", ret);
		return ret;
	}

	usleep_range(100000, 110000);

	ret = mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	if (ret < 0) {
		dev_err(&ctx->dsi->dev, "Failed to enter sleep mode (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int ili9881c_unprepare(struct drm_panel *panel)
{
	struct ili9881c_panel *ctx = panel_to_ili9881c(panel);

	dev_dbg(&ctx->dsi->dev,"%s\n",__func__);

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		usleep_range(5000, 10000);
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	}
	if (ctx->enable_gpio != NULL)
	{
		gpiod_set_value_cansleep(ctx->enable_gpio, 0);
	}

	return 0;
}

static const struct drm_display_mode ph720128t003_default_mode = {
	.clock		= 70000,
	.hdisplay	= 720,
	.hsync_start = 720 + 120,
	.hsync_end	= 720 + 120 + 40,
	.htotal		= 720 + 120 + 40 + 20,

	.vdisplay	= 1280,
	.vsync_start = 1280 + 10,
	.vsync_end	= 1280 + 10 + 2,
	.vtotal		= 1280 + 10 + 2 + 15,
};

static const struct drm_display_mode ph720128t005_default_mode = {
	.clock		= 68000,
	.hdisplay	= 720,
	.hsync_start = 720 + 120,
	.hsync_end	= 720 + 120 + 40,
	.htotal		= 720 + 120 + 40 + 20,

	.vdisplay	= 1280,
	.vsync_start = 1280 + 10,
	.vsync_end	= 1280 + 10 + 2,
	.vtotal		= 1280 + 10 + 2 + 15,
};

static int ili9881c_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct ili9881c_panel *ctx = panel_to_ili9881c(panel);
	const struct drm_display_mode *m = ctx->desc->mode;
	struct drm_display_mode *mode;

	dev_dbg(panel->dev,"%s get drm_display_mode\n",__func__);
	mode = drm_mode_duplicate(connector->dev, m);
	if (!mode) {
		dev_err(&ctx->dsi->dev, "failed to add mode %ux%ux@%u\n",
			m->hdisplay, m->vdisplay, drm_mode_vrefresh(m));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = ctx->desc->width_mm;
	connector->display_info.height_mm = ctx->desc->height_mm;

	return 1;
}

static const struct drm_panel_funcs ili9881c_funcs = {
	.prepare	= ili9881c_prepare,
	.unprepare	= ili9881c_unprepare,
	.enable		= ili9881c_enable,
	.disable	= ili9881c_disable,
	.get_modes	= ili9881c_get_modes,
};

static int ili9881c_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct ili9881c_panel *ctx;
	const struct panel_desc *desc;
	int ret;

	ctx = devm_kzalloc(&dsi->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	desc = of_device_get_match_data(&dsi->dev);
	dsi->mode_flags = desc->mode_flags;
	dsi->format = desc->format;
	dsi->lanes = desc->lanes;

	ctx->desc = desc;
	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

    /* The power GPIO is MIPI VDD5V pin on the LCD adapter baord. */
    ctx->power_gpio = devm_gpiod_get(&dsi->dev, "power", GPIOD_OUT_LOW);
    if (IS_ERR(ctx->power_gpio)) {
		dev_err(&dsi->dev, "failed to get mipi power gpio");
		return PTR_ERR(ctx->power_gpio);
    } else {
		gpiod_set_value_cansleep(ctx->power_gpio, 0);
		msleep(50);
		gpiod_set_value_cansleep(ctx->power_gpio, 1);
		msleep(400);
    }

    /* The enable GPIO is optional, this pin is MIPI DSI/HDMI switch select input. */
    ctx->enable_gpio = devm_gpiod_get_optional(&dsi->dev, "switch", GPIOD_OUT_HIGH);
    if (IS_ERR_OR_NULL(ctx->enable_gpio)) {
		dev_err(&dsi->dev, "No switch enable GPIO");
    } else {
		gpiod_set_value_cansleep(ctx->enable_gpio, 1);
    }

	ctx->reset_gpio = devm_gpiod_get_optional(&dsi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		dev_dbg(&dsi->dev, "Couldn't get our reset GPIO\n");

	ret = of_property_read_u32(dsi->dev.of_node, "dsi-lanes", &dsi->lanes);
	if (ret < 0) {
		dev_dbg(&dsi->dev, "Failed to get dsi-lanes property, use default setting\n");
		dsi->lanes = 2;
	}

	drm_panel_init(&ctx->panel, &dsi->dev, &ili9881c_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret) {
		dev_err(&dsi->dev, "Failed to get backlight handle\n");
		return ret;
	}

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

	return ret;
}

static int ili9881c_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct ili9881c_panel *ctx = mipi_dsi_get_drvdata(dsi);

	ili9881c_disable(&ctx->panel);
	mipi_dsi_detach(dsi);

	if (ctx->panel.dev)
		drm_panel_remove(&ctx->panel);

	return 0;
}

static void ili9881c_dsi_shutdown(struct mipi_dsi_device *dsi)
{
	struct ili9881c_panel *ctx = mipi_dsi_get_drvdata(dsi);

	ili9881c_disable(&ctx->panel);
	ili9881c_unprepare(&ctx->panel);
	if (ctx->power_gpio) {
		gpiod_set_value_cansleep(ctx->power_gpio, 0);
	}
}


static const struct panel_desc ili9881c_t003_panel_desc = {
	.mode = &ph720128t003_default_mode,
	.width_mm = 153,
	.height_mm = 90,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_LPM,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 2,
	.init_code = ili9881c_init_ph720128t003,
	.init_code_length = ARRAY_SIZE(ili9881c_init_ph720128t003),
};

static const struct panel_desc ili9881c_t005_panel_desc = {
	.mode = &ph720128t005_default_mode,
	.width_mm = 155,
	.height_mm = 87,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_LPM,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 2,
	.init_code = ili9881c_init_ph720128t005,
	.init_code_length = ARRAY_SIZE(ili9881c_init_ph720128t005),
};

static const struct of_device_id ili9881c_of_match[] = {
	{ .compatible = "ilitek,ph720128t003",
	  .data = &ili9881c_t003_panel_desc
	},
	{ .compatible = "ilitek,ph720128t005",
	  .data = &ili9881c_t005_panel_desc
	},
	{ }
};
MODULE_DEVICE_TABLE(of, ili9881c_of_match);

static struct mipi_dsi_driver ili9881c_dsi_driver = {
	.probe		= ili9881c_dsi_probe,
	.remove		= ili9881c_dsi_remove,
	.shutdown	= ili9881c_dsi_shutdown,
	.driver = {
		.name		= "panel-ili9881c-dsi",
		.of_match_table = ili9881c_of_match,
	},
};
module_mipi_dsi_driver(ili9881c_dsi_driver);

MODULE_AUTHOR("Avnet, inc.");
MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_DESCRIPTION("Ilitek ILI9881C Controller Driver");
MODULE_LICENSE("GPL v2");
