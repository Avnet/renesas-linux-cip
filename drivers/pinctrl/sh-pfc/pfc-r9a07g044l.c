// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas R9A07G044L SoC Pin Controller and GPIO support
 * Copyright (C) 2020 Renesas Electronics Corp.
 *
 */

#include "pinctrl-rzg2l.h"

static struct pin_data TMS_SWDIO_data[] = {
	{2, 0, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_INPUT_ENABLE)},
};

static struct pin_data TDO_SWO_data[] = {
	{3, 0, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_INPUT_ENABLE)},
};

static struct pin_data AUDIO_CLK1_data[] = {
	{4, 0, PIN_CFG_INPUT_ENABLE},
};

static struct pin_data AUDIO_CLK2_data[] = {
	{4, 1, PIN_CFG_INPUT_ENABLE},
};

static struct pin_data SD0_CLK_data[] = {
	{6, 0, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_SD0)},
};

static struct pin_data SD0_CMD_data[] = {
	{6, 1, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_INPUT_ENABLE
	      | PIN_CFG_IO_VOLTAGE_SD0)},
};

static struct pin_data SD0_RST_N_data[] = {
	{6, 2, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_SD0)},
};

static struct pin_data SD0_DATA0_data[] = {
	{7, 0, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_INPUT_ENABLE
	      | PIN_CFG_IO_VOLTAGE_SD0)},
};

static struct pin_data SD0_DATA1_data[] = {
	{7, 1, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_INPUT_ENABLE
	      | PIN_CFG_IO_VOLTAGE_SD0)},
};

static struct pin_data SD0_DATA2_data[] = {
	{7, 2, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_INPUT_ENABLE
	      | PIN_CFG_IO_VOLTAGE_SD0)},
};

static struct pin_data SD0_DATA3_data[] = {
	{7, 3, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_INPUT_ENABLE
	      | PIN_CFG_IO_VOLTAGE_SD0)},
};

static struct pin_data SD0_DATA4_data[] = {
	{7, 4, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_INPUT_ENABLE
	      | PIN_CFG_IO_VOLTAGE_SD0)},
};

static struct pin_data SD0_DATA5_data[] = {
	{7, 5, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_INPUT_ENABLE
	      | PIN_CFG_IO_VOLTAGE_SD0)},
};

static struct pin_data SD0_DATA6_data[] = {
	{7, 6, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_INPUT_ENABLE
	      | PIN_CFG_IO_VOLTAGE_SD0)},
};

static struct pin_data SD0_DATA7_data[] = {
	{7, 7, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_INPUT_ENABLE
	      | PIN_CFG_IO_VOLTAGE_SD0)},
};

static struct pin_data SD1_CLK_data[] = {
	{8, 0, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_SD1)},
};

static struct pin_data SD1_CMD_data[] = {
	{8, 1, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_INPUT_ENABLE
	      | PIN_CFG_IO_VOLTAGE_SD1)},
};

static struct pin_data SD1_DATA0_data[] = {
	{9, 0, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_INPUT_ENABLE
	      | PIN_CFG_IO_VOLTAGE_SD1)},
};

static struct pin_data SD1_DATA1_data[] = {
	{9, 1, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_INPUT_ENABLE
	      | PIN_CFG_IO_VOLTAGE_SD1)},
};

static struct pin_data SD1_DATA2_data[] = {
	{9, 2, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_INPUT_ENABLE
	      | PIN_CFG_IO_VOLTAGE_SD1)},
};

static struct pin_data SD1_DATA3_data[] = {
	{9, 3, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_INPUT_ENABLE
	      | PIN_CFG_IO_VOLTAGE_SD1)},
};

static struct pin_data QSPI0_SPCLK_data[] = {
	{10, 0, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_QSPI)},
};

static struct pin_data QSPI0_IO0_data[] = {
	{10, 1, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_QSPI)},
};

static struct pin_data QSPI0_IO1_data[] = {
	{10, 2, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_QSPI)},
};

static struct pin_data QSPI0_IO2_data[] = {
	{10, 3, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_QSPI)},
};

static struct pin_data QSPI0_IO3_data[] = {
	{10, 4, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_QSPI)},
};

static struct pin_data QSPI0_SSL_data[] = {
	{10, 5, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_QSPI)},
};

static struct pin_data QSPI1_SPCLK_data[] = {
	{11, 0, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_QSPI)},
};

static struct pin_data QSPI1_IO0_data[] = {
	{11, 1, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_QSPI)},
};

static struct pin_data QSPI1_IO1_data[] = {
	{11, 2, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_QSPI)},
};

static struct pin_data QSPI1_IO2_data[] = {
	{11, 3, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_QSPI)},
};

static struct pin_data QSPI1_IO3_data[] = {
	{11, 4, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_QSPI)},
};

static struct pin_data QSPI1_SSL_data[] = {
	{11, 5, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_QSPI)},
};

static struct pin_data QSPI_RESET_N_data[] = {
	{12, 0, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_QSPI)},
};

static struct pin_data QSPI_WP_N_data[] = {
	{12, 1, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_QSPI)},
};

static struct pin_data QSPI_INT_N_data[] = {
	{12, 2, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE
	      | PIN_CFG_IO_VOLTAGE_QSPI)},
};

static struct pin_data WDTOVF_PERROUT_data[] = {
	{13, 0, (PIN_CFG_DRIVE_STRENGTH
	      | PIN_CFG_SLEW_RATE)},
};

static struct pin_data RIIC0_SDA_data[] = {
	{14, 0, PIN_CFG_INPUT_ENABLE},
};

static struct pin_data RIIC0_SCL_data[] = {
	{14, 1, PIN_CFG_INPUT_ENABLE},
};

static struct pin_data RIIC1_SDA_data[] = {
	{14, 2, PIN_CFG_INPUT_ENABLE},
};

static struct pin_data RIIC1_SCL_data[] = {
	{14, 3, PIN_CFG_INPUT_ENABLE},
};

static const struct {
	struct pinctrl_pin_desc pin_gpio[392];
	struct pinctrl_pin_desc pin_no_gpio[41];
} r9a07g044l_pins = {
	.pin_gpio = {
		RZ_G2L_PINCTRL_PIN_GPIO(0,  (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(1,  (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(2,  (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(3,  (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(4,  (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(5,  (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(6,  (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(7,  (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(8,  (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(9,  (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(10, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(11, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(12, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(13, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(14, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(15, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(16, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(17, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(18, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(19, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(20, PIN_CFG_IO_VOLTAGE_ETH0),
		RZ_G2L_PINCTRL_PIN_GPIO(21, PIN_CFG_IO_VOLTAGE_ETH0),
		RZ_G2L_PINCTRL_PIN_GPIO(22, PIN_CFG_IO_VOLTAGE_ETH0),
		RZ_G2L_PINCTRL_PIN_GPIO(23, PIN_CFG_IO_VOLTAGE_ETH0),
		RZ_G2L_PINCTRL_PIN_GPIO(24, PIN_CFG_IO_VOLTAGE_ETH0),
		RZ_G2L_PINCTRL_PIN_GPIO(25, PIN_CFG_IO_VOLTAGE_ETH0),
		RZ_G2L_PINCTRL_PIN_GPIO(26, PIN_CFG_IO_VOLTAGE_ETH0),
		RZ_G2L_PINCTRL_PIN_GPIO(27, PIN_CFG_IO_VOLTAGE_ETH0),
		RZ_G2L_PINCTRL_PIN_GPIO(28, PIN_CFG_IO_VOLTAGE_ETH0),
		RZ_G2L_PINCTRL_PIN_GPIO(29, PIN_CFG_IO_VOLTAGE_ETH1),
		RZ_G2L_PINCTRL_PIN_GPIO(30, PIN_CFG_IO_VOLTAGE_ETH1),
		RZ_G2L_PINCTRL_PIN_GPIO(31, PIN_CFG_IO_VOLTAGE_ETH1),
		RZ_G2L_PINCTRL_PIN_GPIO(32, PIN_CFG_IO_VOLTAGE_ETH1),
		RZ_G2L_PINCTRL_PIN_GPIO(33, PIN_CFG_IO_VOLTAGE_ETH1),
		RZ_G2L_PINCTRL_PIN_GPIO(34, PIN_CFG_IO_VOLTAGE_ETH1),
		RZ_G2L_PINCTRL_PIN_GPIO(35, PIN_CFG_IO_VOLTAGE_ETH1),
		RZ_G2L_PINCTRL_PIN_GPIO(36, PIN_CFG_IO_VOLTAGE_ETH1),
		RZ_G2L_PINCTRL_PIN_GPIO(37, PIN_CFG_IO_VOLTAGE_ETH1),
		RZ_G2L_PINCTRL_PIN_GPIO(38, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(39, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(40, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(41, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(42, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(43, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(44, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(45, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(46, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(47, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
		RZ_G2L_PINCTRL_PIN_GPIO(48, (PIN_CFG_DRIVE_STRENGTH
					   | PIN_CFG_SLEW_RATE)),
	},
	.pin_no_gpio = {
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 0,  TMS_SWDIO),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 1,  TDO_SWO),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 2,  AUDIO_CLK1),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 3,  AUDIO_CLK2),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 4,  SD0_CLK),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 5,  SD0_CMD),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 6,  SD0_RST_N),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 7,  SD0_DATA0),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 8,  SD0_DATA1),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 9,  SD0_DATA2),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 10, SD0_DATA3),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 11, SD0_DATA4),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 12, SD0_DATA5),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 13, SD0_DATA6),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 14, SD0_DATA7),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 15, SD1_CLK),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 16, SD1_CMD),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 17, SD1_DATA0),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 18, SD1_DATA1),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 19, SD1_DATA2),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 20, SD1_DATA3),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 21, QSPI0_SPCLK),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 22, QSPI0_IO0),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 23, QSPI0_IO1),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 24, QSPI0_IO2),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 25, QSPI0_IO3),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 26, QSPI0_SSL),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 27, QSPI1_SPCLK),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 28, QSPI1_IO0),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 29, QSPI1_IO1),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 30, QSPI1_IO2),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 31, QSPI1_IO3),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 32, QSPI1_SSL),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 33, QSPI_RESET_N),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 34, QSPI_WP_N),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 35, QSPI_INT_N),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 36, WDTOVF_PERROUT),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 37, RIIC0_SDA),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 38, RIIC0_SCL),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 39, RIIC1_SDA),
		RZ_G2L_PINCTRL_PIN_NO_GPIO(
			ARRAY_SIZE(r9a07g044l_pins.pin_gpio), 40, RIIC1_SCL),
	}
};

/* External IRQ*/
static int irq0_a_pins[] = {
	RZ_G2L_PIN(0, 0),
};

static int irq0_b_pins[] = {
	RZ_G2L_PIN(13, 0),
};

static int irq0_c_pins[] = {
	RZ_G2L_PIN(47, 0),
};

static int irq1_a_pins[] = {
	RZ_G2L_PIN(0, 1),
};

static int irq1_b_pins[] = {
	RZ_G2L_PIN(13, 1),
};

static int irq1_c_pins[] = {
	RZ_G2L_PIN(47, 1),
};

static int irq2_a_pins[] = {
	RZ_G2L_PIN(1, 0),
};

static int irq2_b_pins[] = {
	RZ_G2L_PIN(13, 2),
};

static int irq2_c_pins[] = {
	RZ_G2L_PIN(47, 2),
};

static int irq3_a_pins[] = {
	RZ_G2L_PIN(1, 1),
};

static int irq3_b_pins[] = {
	RZ_G2L_PIN(16, 0),
};

static int irq3_c_pins[] = {
	RZ_G2L_PIN(47, 3),
};

static int irq4_a_pins[] = {
	RZ_G2L_PIN(2, 0),
};

static int irq4_b_pins[] = {
	RZ_G2L_PIN(16, 1),
};

static int irq4_c_pins[] = {
	RZ_G2L_PIN(43, 0),
};

static int irq5_a_pins[] = {
	RZ_G2L_PIN(2, 1),
};

static int irq5_b_pins[] = {
	RZ_G2L_PIN(17, 0),
};

static int irq5_c_pins[] = {
	RZ_G2L_PIN(43, 1),
};

static int irq6_a_pins[] = {
	RZ_G2L_PIN(3, 0),
};

static int irq6_b_pins[] = {
	RZ_G2L_PIN(17, 1),
};

static int irq6_c_pins[] = {
	RZ_G2L_PIN(43, 2),
};

static int irq7_a_pins[] = {
	RZ_G2L_PIN(3, 1),
};

static int irq7_b_pins[] = {
	RZ_G2L_PIN(13, 2),
};

static int irq7_c_pins[] = {
	RZ_G2L_PIN(17, 2),
};

static int irq7_d_pins[] = {
	RZ_G2L_PIN(43, 3),
};

/* SCI0 */
static int sci0_data_a_pins[] = {
	/* RX, TX */
	RZ_G2L_PIN(0, 0), RZ_G2L_PIN(0, 1),
};

static int sci0_clk_a_pins[] = {
	/* SCK */
	RZ_G2L_PIN(1, 0),
};

static int sci0_ctrl_a_pins[] = {
	/* CTS_RTS */
	RZ_G2L_PIN(1, 1),
};

static int sci0_data_b_pins[] = {
	/* RX, TX */
	RZ_G2L_PIN(40, 0), RZ_G2L_PIN(40, 1),
};

static int sci0_clk_b_pins[] = {
	/* SCK */
	RZ_G2L_PIN(40, 2),
};

static int sci0_ctrl_b_pins[] = {
	/* CTS_RTS */
	RZ_G2L_PIN(41, 0),
};

static int sci0_data_c_pins[] = {
	/* RX, TX */
	RZ_G2L_PIN(47, 0), RZ_G2L_PIN(47, 1),
};

static int sci0_clk_c_pins[] = {
	/* SCK */
	RZ_G2L_PIN(47, 2),
};

static int sci0_ctrl_c_pins[] = {
	/* CTS_RTS */
	RZ_G2L_PIN(47, 3),
};

/* SCI1 */
static int sci1_data_a_pins[] = {
	/* RX, TX */
	RZ_G2L_PIN(31, 0), RZ_G2L_PIN(31, 1),
};

static int sci1_clk_a_pins[] = {
	/* SCK */
	RZ_G2L_PIN(32, 0),
};

static int sci1_ctrl_a_pins[] = {
	/* CTS_RTS */
	RZ_G2L_PIN(32, 1),
};

static int sci1_data_b_pins[] = {
	/* RX, TX */
	RZ_G2L_PIN(45, 0), RZ_G2L_PIN(45, 1),
};

static int sci1_clk_b_pins[] = {
	/* SCK */
	RZ_G2L_PIN(45, 2),
};

static int sci1_ctrl_b_pins[] = {
	/* CTS_RTS */
	RZ_G2L_PIN(45, 3),
};

/* SCIF0 */
static int scif0_data_pins[] = {
	/* TX, RX */
	RZ_G2L_PIN(38, 0), RZ_G2L_PIN(38, 1),
};

static int scif0_clk_pins[] = {
	/* SCK */
	RZ_G2L_PIN(39, 0),
};

static int scif0_ctrl_pins[] = {
	/* CTS, RTS */
	RZ_G2L_PIN(39, 1), RZ_G2L_PIN(39, 2),
};

/* SCIF1 */
static int scif1_data_pins[] = {
	/* TX, RX */
	RZ_G2L_PIN(40, 0), RZ_G2L_PIN(40, 1),
};

static int scif1_clk_pins[] = {
	/* SCK */
	RZ_G2L_PIN(40, 2),
};

static int scif1_ctrl_pins[] = {
	/* CTS, RTS */
	RZ_G2L_PIN(41, 0), RZ_G2L_PIN(41, 1),
};

/* SCIF2 */
static int scif2_data_a_pins[] = {
	/* TX, RX */
	RZ_G2L_PIN(4, 0), RZ_G2L_PIN(4, 1),
};

static int scif2_clk_a_pins[] = {
	/* SCK */
	RZ_G2L_PIN(5, 0),
};

static int scif2_ctrl_a_pins[] = {
	/* CTS, RTS */
	RZ_G2L_PIN(5, 1), RZ_G2L_PIN(5, 2),
};

static int scif2_data_b_pins[] = {
	/* TX, RX */
	RZ_G2L_PIN(16, 0), RZ_G2L_PIN(16, 1),
};

static int scif2_clk_b_pins[] = {
	/* SCK */
	RZ_G2L_PIN(17, 0),
};

static int scif2_ctrl_b_pins[] = {
	/* CTS, RTS */
	RZ_G2L_PIN(17, 1), RZ_G2L_PIN(17, 2),
};

static int scif2_data_c_pins[] = {
	/* TX, RX */
	RZ_G2L_PIN(33, 0), RZ_G2L_PIN(33, 1),
};

static int scif2_clk_c_pins[] = {
	/* SCK */
	RZ_G2L_PIN(37, 0),
};

static int scif2_ctrl_c_pins[] = {
	/* CTS, RTS */
	RZ_G2L_PIN(37, 1), RZ_G2L_PIN(37, 2),
};

static int scif2_data_d_pins[] = {
	/* TX, RX */
	RZ_G2L_PIN(42, 0), RZ_G2L_PIN(42, 1),
};

static int scif2_clk_d_pins[] = {
	/* SCK */
	RZ_G2L_PIN(42, 2),
};

static int scif2_ctrl_d_pins[] = {
	/* CTS, RTS */
	RZ_G2L_PIN(42, 3), RZ_G2L_PIN(42, 4),
};

static int scif2_data_e_pins[] = {
	/* TX, RX */
	RZ_G2L_PIN(48, 0), RZ_G2L_PIN(48, 1),
};

static int scif2_clk_e_pins[] = {
	/* SCK */
	RZ_G2L_PIN(48, 2),
};

static int scif2_ctrl_e_pins[] = {
	/* CTS, RTS */
	RZ_G2L_PIN(48, 3), RZ_G2L_PIN(48, 4),
};

/* SCIF3 */
static int scif3_data_pins[] = {
	/* TX, RX */
	RZ_G2L_PIN(0, 0), RZ_G2L_PIN(0, 1),
};

static int scif3_clk_pins[] = {
	/* SCK */
	RZ_G2L_PIN(1, 0),
};

/* SCIF4 */
static int scif4_data_pins[] = {
	/* TX, RX */
	RZ_G2L_PIN(2, 0), RZ_G2L_PIN(2, 1),
};

static int scif4_clk_pins[] = {
	/* SCK */
	RZ_G2L_PIN(3, 0),
};

/* Camera In */
static int cam_field_a_pins[] = {
	/* CAM_FIELD */
	RZ_G2L_PIN(3, 1),
};

static int cam_field_b_pins[] = {
	/* CAM_FIELD */
	RZ_G2L_PIN(42, 4),
};

static int cam_clk_pins[] = {
	/* CAM_PCLK */
	RZ_G2L_PIN(29, 0),
};

static int cam_sync_pins[] = {
	/* CAM_HREF, CAM_VSYNC */
	RZ_G2L_PIN(29, 1), RZ_G2L_PIN(30, 0),
};

static int cam_data8_pins[] = {
	/* CAM_DATA[0-7] */
	RZ_G2L_PIN(31, 0), RZ_G2L_PIN(31, 1),
	RZ_G2L_PIN(32, 0), RZ_G2L_PIN(32, 1),
	RZ_G2L_PIN(33, 0), RZ_G2L_PIN(33, 1),
	RZ_G2L_PIN(34, 0), RZ_G2L_PIN(34, 1),
};

static int cam_data10_pins[] = {
	/* CAM_DATA[0-9] */
	RZ_G2L_PIN(31, 0), RZ_G2L_PIN(31, 1),
	RZ_G2L_PIN(32, 0), RZ_G2L_PIN(32, 1),
	RZ_G2L_PIN(33, 0), RZ_G2L_PIN(33, 1),
	RZ_G2L_PIN(34, 0), RZ_G2L_PIN(34, 1),
	RZ_G2L_PIN(35, 0), RZ_G2L_PIN(35, 1),
};

static int cam_data12_pins[] = {
	/* CAM_DATA[0-11] */
	RZ_G2L_PIN(31, 0), RZ_G2L_PIN(31, 1),
	RZ_G2L_PIN(32, 0), RZ_G2L_PIN(32, 1),
	RZ_G2L_PIN(33, 0), RZ_G2L_PIN(33, 1),
	RZ_G2L_PIN(34, 0), RZ_G2L_PIN(34, 1),
	RZ_G2L_PIN(35, 0), RZ_G2L_PIN(35, 1),
	RZ_G2L_PIN(36, 0), RZ_G2L_PIN(36, 1),
};

static int cam_data16_pins[] = {
	/* CAM_DATA[0-11] */
	RZ_G2L_PIN(31, 0), RZ_G2L_PIN(31, 1),
	RZ_G2L_PIN(32, 0), RZ_G2L_PIN(32, 1),
	RZ_G2L_PIN(33, 0), RZ_G2L_PIN(33, 1),
	RZ_G2L_PIN(34, 0), RZ_G2L_PIN(34, 1),
	RZ_G2L_PIN(35, 0), RZ_G2L_PIN(35, 1),
	RZ_G2L_PIN(36, 0), RZ_G2L_PIN(36, 1),
	RZ_G2L_PIN(37, 0), RZ_G2L_PIN(37, 1),
	RZ_G2L_PIN(37, 2), RZ_G2L_PIN(30, 1),
};

/* GPT External Trigger */
static int gpt_ext_trg_a_a_pins[] = {
	RZ_G2L_PIN(10, 0),
};

static int gpt_ext_trg_b_a_pins[] = {
	RZ_G2L_PIN(10, 1),
};

static int gpt_ext_trg_c_a_pins[] = {
	RZ_G2L_PIN(11, 0),
};

static int gpt_ext_trg_d_a_pins[] = {
	RZ_G2L_PIN(11, 1),
};

static int gpt_ext_trg_a_b_pins[] = {
	RZ_G2L_PIN(36, 0),
};

static int gpt_ext_trg_b_b_pins[] = {
	RZ_G2L_PIN(36, 1),
};

static int gpt_ext_trg_c_b_pins[] = {
	RZ_G2L_PIN(37, 0),
};

static int gpt_ext_trg_d_b_pins[] = {
	RZ_G2L_PIN(37, 1),
};

static int gpt_ext_trg_a_c_pins[] = {
	RZ_G2L_PIN(38, 0),
};

static int gpt_ext_trg_b_c_pins[] = {
	RZ_G2L_PIN(38, 1),
};

static int gpt_ext_trg_c_c_pins[] = {
	RZ_G2L_PIN(39, 0),
};

static int gpt_ext_trg_d_c_pins[] = {
	RZ_G2L_PIN(39, 1),
};

static int gpt_ext_trg_a_d_pins[] = {
	RZ_G2L_PIN(46, 0),
};

static int gpt_ext_trg_b_d_pins[] = {
	RZ_G2L_PIN(46, 1),
};

static int gpt_ext_trg_c_d_pins[] = {
	RZ_G2L_PIN(46, 2),
};

static int gpt_ext_trg_d_d_pins[] = {
	RZ_G2L_PIN(46, 3),
};

/* GPT channel 0 */
static int gpt0_a_a_pins[] = {
	RZ_G2L_PIN(0, 0),
};

static int gpt0_b_a_pins[] = {
	RZ_G2L_PIN(0, 1),
};

static int gpt0_a_b_pins[] = {
	RZ_G2L_PIN(18, 0),
};

static int gpt0_b_b_pins[] = {
	RZ_G2L_PIN(18, 1),
};

static int gpt0_a_c_pins[] = {
	RZ_G2L_PIN(33, 0),
};

static int gpt0_b_c_pins[] = {
	RZ_G2L_PIN(33, 1),
};

/* GPT channel 1 */
static int gpt1_a_a_pins[] = {
	RZ_G2L_PIN(1, 0),
};

static int gpt1_b_a_pins[] = {
	RZ_G2L_PIN(1, 1),
};

static int gpt1_a_b_pins[] = {
	RZ_G2L_PIN(34, 0),
};

static int gpt1_b_b_pins[] = {
	RZ_G2L_PIN(34, 1),
};

/* GPT channel 2 */
static int gpt2_a_a_pins[] = {
	RZ_G2L_PIN(2, 0),
};

static int gpt2_b_a_pins[] = {
	RZ_G2L_PIN(2, 1),
};

static int gpt2_a_b_pins[] = {
	RZ_G2L_PIN(33, 0),
};

static int gpt2_b_b_pins[] = {
	RZ_G2L_PIN(33, 1),
};

static int gpt2_a_c_pins[] = {
	RZ_G2L_PIN(35, 0),
};

static int gpt2_b_c_pins[] = {
	RZ_G2L_PIN(35, 1),
};

/* GPT channel 3 */
static int gpt3_a_a_pins[] = {
	RZ_G2L_PIN(3, 0),
};

static int gpt3_b_a_pins[] = {
	RZ_G2L_PIN(3, 1),
};

static int gpt3_a_b_pins[] = {
	RZ_G2L_PIN(19, 0),
};

static int gpt3_b_b_pins[] = {
	RZ_G2L_PIN(19, 1),
};

static int gpt3_a_c_pins[] = {
	RZ_G2L_PIN(34, 0),
};

static int gpt3_b_c_pins[] = {
	RZ_G2L_PIN(34, 1),
};

static int gpt3_a_d_pins[] = {
	RZ_G2L_PIN(41, 0),
};

static int gpt3_b_d_pins[] = {
	RZ_G2L_PIN(41, 1),
};

/* GPT channel 4 */
static int gpt4_a_a_pins[] = {
	RZ_G2L_PIN(15, 0),
};

static int gpt4_b_a_pins[] = {
	RZ_G2L_PIN(15, 1),
};

static int gpt4_a_b_pins[] = {
	RZ_G2L_PIN(43, 0),
};

static int gpt4_b_b_pins[] = {
	RZ_G2L_PIN(43, 1),
};

/* GPT channel 5 */
static int gpt5_a_a_pins[] = {
	RZ_G2L_PIN(16, 0),
};

static int gpt5_b_a_pins[] = {
	RZ_G2L_PIN(16, 1),
};

static int gpt5_a_b_pins[] = {
	RZ_G2L_PIN(43, 2),
};

static int gpt5_b_b_pins[] = {
	RZ_G2L_PIN(43, 3),
};

/* GPT channel 6 */
static int gpt6_a_a_pins[] = {
	RZ_G2L_PIN(17, 0),
};

static int gpt6_b_a_pins[] = {
	RZ_G2L_PIN(17, 1),
};

static int gpt6_a_b_pins[] = {
	RZ_G2L_PIN(40, 0),
};

static int gpt6_b_b_pins[] = {
	RZ_G2L_PIN(40, 1),
};

static int gpt6_a_c_pins[] = {
	RZ_G2L_PIN(43, 0),
};

static int gpt6_b_c_pins[] = {
	RZ_G2L_PIN(43, 1),
};

static int gpt6_a_d_pins[] = {
	RZ_G2L_PIN(44, 0),
};

static int gpt6_b_d_pins[] = {
	RZ_G2L_PIN(44, 1),
};

/* GPT channel 7 */
static int gpt7_a_a_pins[] = {
	RZ_G2L_PIN(12, 0),
};

static int gpt7_b_a_pins[] = {
	RZ_G2L_PIN(12, 1),
};

static int gpt7_a_b_pins[] = {
	RZ_G2L_PIN(41, 0),
};

static int gpt7_b_b_pins[] = {
	RZ_G2L_PIN(41, 1),
};

static int gpt7_a_c_pins[] = {
	RZ_G2L_PIN(44, 2),
};

static int gpt7_b_c_pins[] = {
	RZ_G2L_PIN(44, 3),
};

/* USB channel 0 */
static int usb0_a_pins[] = {
	/* VBUS, OVC, OTG_ID, OTG_EXICEN */
	RZ_G2L_PIN(4, 0), RZ_G2L_PIN(5, 0),
	RZ_G2L_PIN(5, 1), RZ_G2L_PIN(5, 2),
};

static int usb0_b_pins[] = {
	/* VBUS, OVC, OTG_ID, OTG_EXICEN */
	RZ_G2L_PIN(6, 0), RZ_G2L_PIN(7, 0),
	RZ_G2L_PIN(7, 1), RZ_G2L_PIN(7, 2),
};

/* USB channel 1 */
static int usb1_a_pins[] = {
	/* VBUS, OVC */
	RZ_G2L_PIN(8, 0), RZ_G2L_PIN(8, 1),
};

static int usb1_b_pins[] = {
	/* VBUS, OVC */
	RZ_G2L_PIN(29, 0), RZ_G2L_PIN(29, 1),
};

static int usb1_c_pins[] = {
	/* VBUS, OVC */
	RZ_G2L_PIN(38, 0), RZ_G2L_PIN(38, 1),
};

static int usb1_d_pins[] = {
	/* VBUS, OVC */
	RZ_G2L_PIN(42, 0), RZ_G2L_PIN(42, 1),
};

/* MTU channel 0 */
static int mtu0_a_pins[] = {
	/* MTIOC0A, MTIOC0B, MTIOC0C, MTIOC0D */
	RZ_G2L_PIN(0, 0), RZ_G2L_PIN(0, 1),
	RZ_G2L_PIN(1, 0), RZ_G2L_PIN(1, 1),
};

static int mtu0_b_pins[] = {
	/* MTIOC0A, MTIOC0B, MTIOC0C, MTIOC0D */
	RZ_G2L_PIN(34, 0), RZ_G2L_PIN(34, 1),
	RZ_G2L_PIN(35, 0), RZ_G2L_PIN(35, 1),
};

/* MTU channel 1 */
static int mtu1_a_pins[] = {
	/* MTIOC1A, MTIOC1B */
	RZ_G2L_PIN(2, 0), RZ_G2L_PIN(2, 1),
};

static int mtu1_b_pins[] = {
	/* MTIOC1A, MTIOC1B */
	RZ_G2L_PIN(6, 0), RZ_G2L_PIN(6, 1),
};

static int mtu1_c_pins[] = {
	/* MTIOC1A, MTIOC1B */
	RZ_G2L_PIN(19, 0), RZ_G2L_PIN(19, 1),
};

/* MTU channel 2 */
static int mtu2_a_pins[] = {
	/* MTIOC2A, MTIOC2B */
	RZ_G2L_PIN(3, 0), RZ_G2L_PIN(3, 1),
};

static int mtu2_b_pins[] = {
	/* MTIOC2A, MTIOC2B */
	RZ_G2L_PIN(9, 0), RZ_G2L_PIN(9, 1),
};

static int mtu2_c_pins[] = {
	/* MTIOC2A, MTIOC2B */
	RZ_G2L_PIN(18, 0), RZ_G2L_PIN(18, 1),
};

/* MTU channel 3 */
static int mtu3_a_pins[] = {
	/* MTIOC3A, MTIOC3B, MTIOC3C, MTIOC3D */
	RZ_G2L_PIN(32, 0), RZ_G2L_PIN(32, 1),
	RZ_G2L_PIN(36, 0), RZ_G2L_PIN(36, 1),
};

static int mtu3_b_pins[] = {
	/* MTIOC3A, MTIOC3B, MTIOC3C, MTIOC3D */
	RZ_G2L_PIN(44, 0), RZ_G2L_PIN(44, 1),
	RZ_G2L_PIN(44, 2), RZ_G2L_PIN(44, 3),
};

/* MTU channel 4 */
static int mtu4_pins[] = {
	/* MTIOC4A, MTIOC4B, MTIOC4C, MTIOC4D */
	RZ_G2L_PIN(38, 0), RZ_G2L_PIN(38, 1),
	RZ_G2L_PIN(39, 0), RZ_G2L_PIN(39, 1),
};

/* MTU channel 5 */
static int mtu5_a_pins[] = {
	/* MTIC5U, MTIC5V, MTIC5W */
	RZ_G2L_PIN(7, 0), RZ_G2L_PIN(7, 1), RZ_G2L_PIN(7, 2),
};

static int mtu5_b_pins[] = {
	/* MTIC5U, MTIC5V, MTIC5W */
	RZ_G2L_PIN(40, 0), RZ_G2L_PIN(40, 1), RZ_G2L_PIN(40, 2),
};

/* MTU channel 6 */
static int mtu6_pins[] = {
	/* MTIOC6A, MTIOC6B, MTIOC6C, MTIOC6D */
	RZ_G2L_PIN(10, 0), RZ_G2L_PIN(10, 1),
	RZ_G2L_PIN(11, 0), RZ_G2L_PIN(11, 1),
};

/* MTU channel 7 */
static int mtu7_a_pins[] = {
	/* MTIOC7A, MTIOC7B, MTIOC7C, MTIOC7D */
	RZ_G2L_PIN(4, 0), RZ_G2L_PIN(4, 1),
	RZ_G2L_PIN(5, 0), RZ_G2L_PIN(5, 1),
};

static int mtu7_b_pins[] = {
	/* MTIOC7A, MTIOC7B, MTIOC7C, MTIOC7D */
	RZ_G2L_PIN(42, 0), RZ_G2L_PIN(42, 1),
	RZ_G2L_PIN(42, 2), RZ_G2L_PIN(42, 3),
};

/* MTU channel 8 */
static int mtu8_a_pins[] = {
	/* MTIOC8A, MTIOC8B, MTIOC8C, MTIOC8D */
	RZ_G2L_PIN(26, 0), RZ_G2L_PIN(26, 1),
	RZ_G2L_PIN(27, 0), RZ_G2L_PIN(27, 1),
};

static int mtu8_b_pins[] = {
	/* MTIOC8A, MTIOC8B, MTIOC8C, MTIOC8D */
	RZ_G2L_PIN(43, 0), RZ_G2L_PIN(43, 1),
	RZ_G2L_PIN(43, 2), RZ_G2L_PIN(43, 3),
};

/* Display Out */
static int disp_clk_pins[] = {
	/* CLK */
	RZ_G2L_PIN(6, 0),
};

static int disp_sync_pins[] = {
	/* HSYNC, VSYNC */
	RZ_G2L_PIN(6, 1), RZ_G2L_PIN(7, 0),
};

static int disp_de_pins[] = {
	/* DE */
	RZ_G2L_PIN(7, 1),
};

static int disp_bgr666_pins[] = {
	/* B[7:2], G[7:2], R[7:2] */
	RZ_G2L_PIN(8,  1), RZ_G2L_PIN(8,  2), RZ_G2L_PIN(9,  0),
	RZ_G2L_PIN(9,  1), RZ_G2L_PIN(10, 0), RZ_G2L_PIN(10, 1),
	RZ_G2L_PIN(12, 0), RZ_G2L_PIN(12, 1), RZ_G2L_PIN(13, 0),
	RZ_G2L_PIN(13, 1), RZ_G2L_PIN(13, 2), RZ_G2L_PIN(14, 0),
	RZ_G2L_PIN(15, 1), RZ_G2L_PIN(16, 0), RZ_G2L_PIN(16, 1),
	RZ_G2L_PIN(17, 0), RZ_G2L_PIN(17, 1), RZ_G2L_PIN(17, 2),
};

static int disp_bgr888_pins[] = {
	/* B[7:0], G[7:0], R[7:0] */
	RZ_G2L_PIN(7,  2), RZ_G2L_PIN(8,  0), RZ_G2L_PIN(8,  1),
	RZ_G2L_PIN(8,  2), RZ_G2L_PIN(9,  0), RZ_G2L_PIN(9,  1),
	RZ_G2L_PIN(10, 0), RZ_G2L_PIN(10, 1), RZ_G2L_PIN(11, 0),
	RZ_G2L_PIN(11, 1), RZ_G2L_PIN(12, 0), RZ_G2L_PIN(12, 1),
	RZ_G2L_PIN(13, 0), RZ_G2L_PIN(13, 1), RZ_G2L_PIN(13, 2),
	RZ_G2L_PIN(14, 0), RZ_G2L_PIN(14, 1), RZ_G2L_PIN(15, 0),
	RZ_G2L_PIN(15, 1), RZ_G2L_PIN(16, 0), RZ_G2L_PIN(16, 1),
	RZ_G2L_PIN(17, 0), RZ_G2L_PIN(17, 1), RZ_G2L_PIN(17, 2),
};

/* ADC */
static int adc_a_pins[] = {
	/* ADC_TRG */
	RZ_G2L_PIN(2, 0),
};

static int adc_b_pins[] = {
	/* ADC_TRG */
	RZ_G2L_PIN(4, 0),
};

static int adc_c_pins[] = {
	/* ADC_TRG */
	RZ_G2L_PIN(9, 0),
};

static int adc_d_pins[] = {
	/* ADC_TRG */
	RZ_G2L_PIN(42, 2),
};

static int adc_e_pins[] = {
	/* ADC_TRG */
	RZ_G2L_PIN(48, 4),
};

/* I2C channel 2 */
static int i2c2_a_pins[] = {
	/* SDA, SCL */
	RZ_G2L_PIN(3, 0), RZ_G2L_PIN(3, 1),
};

static int i2c2_b_pins[] = {
	/* SDA, SCL */
	RZ_G2L_PIN(19, 0), RZ_G2L_PIN(19, 1),
};

static int i2c2_c_pins[] = {
	/* SDA, SCL */
	RZ_G2L_PIN(42, 3), RZ_G2L_PIN(42, 4),
};

static int i2c2_d_pins[] = {
	/* SDA, SCL */
	RZ_G2L_PIN(46, 0), RZ_G2L_PIN(46, 1),
};

static int i2c2_e_pins[] = {
	/* SDA, SCL */
	RZ_G2L_PIN(48, 0), RZ_G2L_PIN(48, 1),
};

/* I2C channel 3 */
static int i2c3_a_pins[] = {
	/* SDA, SCL */
	RZ_G2L_PIN(8, 1), RZ_G2L_PIN(8, 0),
};

static int i2c3_b_pins[] = {
	/* SDA, SCL */
	RZ_G2L_PIN(18, 0), RZ_G2L_PIN(18, 1),
};

static int i2c3_c_pins[] = {
	/* SDA, SCL */
	RZ_G2L_PIN(46, 2), RZ_G2L_PIN(46, 3),
};

static int i2c3_d_pins[] = {
	/* SDA, SCL */
	RZ_G2L_PIN(48, 2), RZ_G2L_PIN(48, 3),
};

/* SSI channel 0 */
static int ssi0_ctrl_a_pins[] = {
	/* BCK, RCK */
	RZ_G2L_PIN(6, 0), RZ_G2L_PIN(6, 1),
};

static int ssi0_data_a_pins[] = {
	/* TXD, RXD */
	RZ_G2L_PIN(7, 0), RZ_G2L_PIN(7, 1),
};

static int ssi0_ctrl_b_pins[] = {
	/* BCK, RCK */
	RZ_G2L_PIN(22, 0), RZ_G2L_PIN(22, 1),
};

static int ssi0_data_b_pins[] = {
	/* TXD, RXD */
	RZ_G2L_PIN(23, 0), RZ_G2L_PIN(23, 1),
};

static int ssi0_ctrl_c_pins[] = {
	/* BCK, RCK */
	RZ_G2L_PIN(35, 0), RZ_G2L_PIN(35, 1),
};

static int ssi0_data_c_pins[] = {
	/* TXD, RXD */
	RZ_G2L_PIN(36, 0), RZ_G2L_PIN(36, 1),
};

static int ssi0_ctrl_d_pins[] = {
	/* BCK, RCK */
	RZ_G2L_PIN(45, 0), RZ_G2L_PIN(45, 1),
};

static int ssi0_data_d_pins[] = {
	/* TXD, RXD */
	RZ_G2L_PIN(45, 2), RZ_G2L_PIN(45, 3),
};

/* SSI channel 1 */
static int ssi1_ctrl_a_pins[] = {
	/* BCK, RCK */
	RZ_G2L_PIN(14, 0), RZ_G2L_PIN(14, 1),
};

static int ssi1_data_a_pins[] = {
	/* TXD, RXD */
	RZ_G2L_PIN(15, 0), RZ_G2L_PIN(15, 1),
};

static int ssi1_ctrl_b_pins[] = {
	/* BCK, RCK */
	RZ_G2L_PIN(24, 0), RZ_G2L_PIN(24, 1),
};

static int ssi1_data_b_pins[] = {
	/* TXD, RXD */
	RZ_G2L_PIN(25, 0), RZ_G2L_PIN(25, 1),
};

static int ssi1_ctrl_c_pins[] = {
	/* BCK, RCK */
	RZ_G2L_PIN(44, 0), RZ_G2L_PIN(44, 1),
};

static int ssi1_data_c_pins[] = {
	/* TXD, RXD */
	RZ_G2L_PIN(44, 2), RZ_G2L_PIN(44, 3),
};

static int ssi1_ctrl_d_pins[] = {
	/* BCK, RCK */
	RZ_G2L_PIN(46, 0), RZ_G2L_PIN(46, 1),
};

static int ssi1_data_d_pins[] = {
	/* TXD, RXD */
	RZ_G2L_PIN(46, 2), /* RZ_G2L_PIN(46, 3), */
};

static int ssi1_ctrl_e_pins[] = {
	/* BCK, RCK */
	RZ_G2L_PIN(47, 0), RZ_G2L_PIN(47, 1),
};

static int ssi1_data_e_pins[] = {
	/* TXD, RXD */
	RZ_G2L_PIN(47, 2), RZ_G2L_PIN(47, 3),
};

/* SSI channel 2 */
static int ssi2_ctrl_a_pins[] = {
	/* BCK, RCK */
	RZ_G2L_PIN(5, 0), RZ_G2L_PIN(5, 1),
};

static int ssi2_data_a_pins[] = {
	/* DATA */
	RZ_G2L_PIN(5, 2),
};

static int ssi2_ctrl_b_pins[] = {
	/* BCK, RCK */
	RZ_G2L_PIN(29, 0), RZ_G2L_PIN(29, 1),
};

static int ssi2_data_b_pins[] = {
	/* DATA */
	RZ_G2L_PIN(30, 0),
};

/* SSI channel 3 */
static int ssi3_ctrl_pins[] = {
	/* BCK, RCK */
	RZ_G2L_PIN(31, 0), RZ_G2L_PIN(31, 1),
};

static int ssi3_data_pins[] = {
	/* TXD, RXD */
	RZ_G2L_PIN(32, 0), RZ_G2L_PIN(32, 1),
};

/* RSPI channel 0 */
static int rspi0_clk_a_pins[] = {
	/* CK */
	RZ_G2L_PIN(20, 0),
};

static int rspi0_data_a_pins[] = {
	/* MOSI, MISO */
	RZ_G2L_PIN(20, 1), RZ_G2L_PIN(20, 2),
};

static int rspi0_ssl_a_pins[] = {
	/* SSL */
	RZ_G2L_PIN(21, 0),
};

static int rspi0_clk_b_pins[] = {
	/* CK */
	RZ_G2L_PIN(43, 0),
};

static int rspi0_data_b_pins[] = {
	/* MOSI, MISO */
	RZ_G2L_PIN(43, 1), RZ_G2L_PIN(43, 2),
};

static int rspi0_ssl_b_pins[] = {
	/* SSL */
	RZ_G2L_PIN(43, 3),
};

static int rspi0_clk_c_pins[] = {
	/* CK */
	RZ_G2L_PIN(47, 0),
};

static int rspi0_data_c_pins[] = {
	/* MOSI, MISO */
	RZ_G2L_PIN(47, 1), RZ_G2L_PIN(47, 2),
};

static int rspi0_ssl_c_pins[] = {
	/* SSL */
	RZ_G2L_PIN(47, 3),
};

/* RSPI channel 1 */
static int rspi1_clk_a_pins[] = {
	/* CK */
	RZ_G2L_PIN(26, 0),
};

static int rspi1_data_a_pins[] = {
	/* MOSI, MISO */
	RZ_G2L_PIN(26, 1), RZ_G2L_PIN(27, 0),
};

static int rspi1_ssl_a_pins[] = {
	/* SSL */
	RZ_G2L_PIN(27, 1),
};

static int rspi1_clk_b_pins[] = {
	/* CK */
	RZ_G2L_PIN(44, 0),
};

static int rspi1_data_b_pins[] = {
	/* MOSI, MISO */
	RZ_G2L_PIN(44, 1), RZ_G2L_PIN(44, 2),
};

static int rspi1_ssl_b_pins[] = {
	/* SSL */
	RZ_G2L_PIN(44, 3),
};

static int rspi1_clk_c_pins[] = {
	/* CK */
	RZ_G2L_PIN(48, 0),
};

static int rspi1_data_c_pins[] = {
	/* MOSI, MISO */
	RZ_G2L_PIN(48, 1), RZ_G2L_PIN(48, 2),
};

static int rspi1_ssl_c_pins[] = {
	/* SSL */
	RZ_G2L_PIN(48, 3),
};

/* RSPI channel 2 */
static int rspi2_clk_a_pins[] = {
	/* CK */
	RZ_G2L_PIN(8, 0),
};

static int rspi2_data_a_pins[] = {
	/* MOSI, MISO */
	RZ_G2L_PIN(8, 1), RZ_G2L_PIN(8, 2),
};

static int rspi2_ssl_a_pins[] = {
	/* SSL */
	RZ_G2L_PIN(9, 0),
};

static int rspi2_clk_b_pins[] = {
	/* CK */
	RZ_G2L_PIN(42, 0),
};

static int rspi2_data_b_pins[] = {
	/* MOSI, MISO */
	RZ_G2L_PIN(42, 1), RZ_G2L_PIN(42, 2),
};

static int rspi2_ssl_b_pins[] = {
	/* SSL */
	RZ_G2L_PIN(42, 3),
};

/* CAN */
static int can_clk_a_pins[] = {
	/* CLK */
	RZ_G2L_PIN(10, 0),
};

static int can0_tx_a_pins[] = {
	/* TX */
	RZ_G2L_PIN(10, 1),
};

static int can0_rx_a_pins[] = {
	/* RX */
	RZ_G2L_PIN(11, 0),
};

static int can0_tx_datarate_en_a_pins[] = {
	/* TX_DATARATE_EN */
	RZ_G2L_PIN(11, 1),
};

static int can0_rx_datarate_en_a_pins[] = {
	/* RX_DATARATE_EN */
	RZ_G2L_PIN(12, 0),
};

static int can1_tx_a_pins[] = {
	/* TX */
	RZ_G2L_PIN(12, 1),
};

static int can1_rx_a_pins[] = {
	/* RX */
	RZ_G2L_PIN(13, 0),
};

static int can1_tx_datarate_en_a_pins[] = {
	/* TX_DATARATE_EN */
	RZ_G2L_PIN(13, 1),
};

static int can1_rx_datarate_en_a_pins[] = {
	/* RX_DATARATE_EN */
	RZ_G2L_PIN(13, 2),
};

static int can_clk_b_pins[] = {
	/* CLK */
	RZ_G2L_PIN(20, 0),
};

static int can0_tx_b_pins[] = {
	/* TX */
	RZ_G2L_PIN(20, 1),
};

static int can0_rx_b_pins[] = {
	/* RX */
	RZ_G2L_PIN(20, 2),
};

static int can0_tx_datarate_en_b_pins[] = {
	/* TX_DATARATE_EN */
	RZ_G2L_PIN(21, 0),
};

static int can0_rx_datarate_en_b_pins[] = {
	/* RX_DATARATE_EN */
	RZ_G2L_PIN(21, 1),
};

static int can1_tx_b_pins[] = {
	/* TX */
	RZ_G2L_PIN(22, 0),
};

static int can1_rx_b_pins[] = {
	/* RX */
	RZ_G2L_PIN(22, 1),
};

static int can1_tx_datarate_en_b_pins[] = {
	/* TX_DATARATE_EN */
	RZ_G2L_PIN(23, 0),
};

static int can1_rx_datarate_en_b_pins[] = {
	/* RX_DATARATE_EN */
	RZ_G2L_PIN(23, 1),
};

static int can_clk_c_pins[] = {
	/* CLK */
	RZ_G2L_PIN(38, 0),
};

static int can0_tx_c_pins[] = {
	/* TX */
	RZ_G2L_PIN(38, 1),
};

static int can0_rx_c_pins[] = {
	/* RX */
	RZ_G2L_PIN(39, 0),
};

static int can0_tx_datarate_en_c_pins[] = {
	/* TX_DATARATE_EN */
	RZ_G2L_PIN(39, 1),
};

static int can0_rx_datarate_en_c_pins[] = {
	/*  RX_DATARATE_EN */
	RZ_G2L_PIN(39, 2),
};

static int can1_tx_c_pins[] = {
	/* TX */
	RZ_G2L_PIN(40, 0),
};

static int can1_rx_c_pins[] = {
	/* RX */
	RZ_G2L_PIN(40, 1),
};

static int can1_tx_datarate_en_c_pins[] = {
	/* TX_DATARATE_EN */
	RZ_G2L_PIN(40, 2),
};

static int can1_rx_datarate_en_c_pins[] = {
	/* RX_DATARATE_EN */
	RZ_G2L_PIN(41, 0),
};

static int can_clk_d_pins[] = {
	/* CLK */
	RZ_G2L_PIN(42, 0),
};

static int can0_tx_d_pins[] = {
	/* TX */
	RZ_G2L_PIN(42, 1),
};

static int can0_rx_d_pins[] = {
	/* RX */
	RZ_G2L_PIN(42, 2),
};

static int can0_tx_datarate_en_d_pins[] = {
	/* TX_DATARATE_EN */
	RZ_G2L_PIN(42, 3),
};

static int can0_rx_datarate_en_d_pins[] = {
	/* RX_DATARATE_EN */
	RZ_G2L_PIN(42, 4),
};

static int can1_tx_d_pins[] = {
	/* TX */
	RZ_G2L_PIN(44, 0),
};

static int can1_rx_d_pins[] = {
	/* RX */
	RZ_G2L_PIN(44, 1),
};

static int can1_tx_datarate_en_d_pins[] = {
	/* TX_DATARATE_EN */
	RZ_G2L_PIN(44, 2),
};

static int can1_rx_datarate_en_d_pins[] = {
	/* RX_DATARATE_EN */
	RZ_G2L_PIN(44, 3),
};

static int can1_tx_e_pins[] = {
	/* TX */
	RZ_G2L_PIN(46, 0),
};

static int can1_rx_e_pins[] = {
	/* RX */
	RZ_G2L_PIN(46, 1),
};

static int can1_tx_datarate_en_e_pins[] = {
	/* TX_DATARATE_EN */
	RZ_G2L_PIN(46, 2),
};

static int can1_rx_datarate_en_e_pins[] = {
	/* RX_DATARATE_EN */
	RZ_G2L_PIN(46, 3),
};

/* POE */
static int poe_a_pins[] = {
	/* POE0_N, POE4_N, POE8_N, POE10_N */
	RZ_G2L_PIN(12, 0), RZ_G2L_PIN(12, 1),
	RZ_G2L_PIN(13, 0), RZ_G2L_PIN(13, 1),
};

static int poe_b_pins[] = {
	/* POE0_N, POE4_N, POE8_N, POE10_N */
	RZ_G2L_PIN(24, 0), RZ_G2L_PIN(24, 1),
	RZ_G2L_PIN(25, 0), RZ_G2L_PIN(25, 1),
};

static int poe_c_pins[] = {
	/* POE0_N, POE4_N, POE8_N, POE10_N */
	RZ_G2L_PIN(45, 0), RZ_G2L_PIN(45, 1),
	RZ_G2L_PIN(45, 2), RZ_G2L_PIN(45, 3),
};

/* SDHI channel 0 */
static int sdhi0_cd_a_pins[] = {
	/* CD */
	RZ_G2L_PIN(18, 0),
};

static int sdhi0_wp_a_pins[] = {
	/* WP */
	RZ_G2L_PIN(18, 1),
};

static int sdhi0_cd_b_pins[] = {
	/* CD */
	RZ_G2L_PIN(47, 0),
};

static int sdhi0_wp_b_pins[] = {
	/* WP */
	RZ_G2L_PIN(47, 1),
};

/* SDHI channel 1 */
static int sdhi1_cd_a_pins[] = {
	/* CD */
	RZ_G2L_PIN(14, 0),
};

static int sdhi1_wp_a_pins[] = {
	/* WP */
	RZ_G2L_PIN(14, 1),
};

static int sdhi1_cd_b_pins[] = {
	/* CD */
	RZ_G2L_PIN(19, 0),
};

static int sdhi1_wp_b_pins[] = {
	/* WP */
	RZ_G2L_PIN(19, 1),
};

static int sdhi1_cd_c_pins[] = {
	/* CD */
	RZ_G2L_PIN(47, 2),
};

static int sdhi1_wp_c_pins[] = {
	/* WP */
	RZ_G2L_PIN(47, 3),
};

/* MTCLK*/
static int mtclka_a_pins[] = {
	/* MTCLKA */
	RZ_G2L_PIN(14, 0),
};

static int mtclkb_a_pins[] = {
	/* MTCLKB */
	RZ_G2L_PIN(14, 1),
};

static int mtclkc_a_pins[] = {
	/* MTCLKC */
	RZ_G2L_PIN(15, 0),
};

static int mtclkd_a_pins[] = {
	/* MTCLKD */
	RZ_G2L_PIN(15, 1),
};

static int mtclka_b_pins[] = {
	/* MTCLKA */
	RZ_G2L_PIN(22, 0),
};

static int mtclkb_b_pins[] = {
	/* MTCLKB */
	RZ_G2L_PIN(22, 1),
};

static int mtclkc_b_pins[] = {
	/* MTCLKC */
	RZ_G2L_PIN(23, 0),
};

static int mtclkd_b_pins[] = {
	/* MTCLKD */
	RZ_G2L_PIN(23, 1),
};

static int mtclka_c_pins[] = {
	/* MTCLKA */
	RZ_G2L_PIN(48, 0),
};

static int mtclkb_c_pins[] = {
	/* MTCLKB */
	RZ_G2L_PIN(48, 1),
};

static int mtclkc_c_pins[] = {
	/* MTCLKC */
	RZ_G2L_PIN(48, 2),
};

static int mtclkd_c_pins[] = {
	/* MTCLKD */
	RZ_G2L_PIN(48, 3),
};

/* Ethernet channel 0 */
static int eth0_link_pins[] = {
	/* ETH0_LINKSTA */
	RZ_G2L_PIN(28, 1),
};

static int eth0_mdio_pins[] = {
	/* ETH0_MDC, ETH0_MDIO */
	RZ_G2L_PIN(27, 1), RZ_G2L_PIN(28, 0),
};

static int eth0_mii_pins[] = {
	RZ_G2L_PIN(20, 0),	/* ETH0_TXC_TX_CLK */
	RZ_G2L_PIN(20, 1),	/* ETH0_TX_CTL_TX_EN */
	RZ_G2L_PIN(20, 2),	/* ETH0_TXD0 */
	RZ_G2L_PIN(21, 0),	/* ETH0_TXD1 */
	RZ_G2L_PIN(21, 1),	/* ETH0_TXD2 */
	RZ_G2L_PIN(22, 0),	/* ETH0_TXD3 */
	RZ_G2L_PIN(22, 1),	/* ETH0_TX_ERR */
	RZ_G2L_PIN(23, 0),	/* ETH0_TX_COL */
	RZ_G2L_PIN(23, 1),	/* ETH0_TX_CRS */
	RZ_G2L_PIN(24, 0),	/* ETH0_RXC_RX_CLK */
	RZ_G2L_PIN(24, 1),	/* ETH0_RX_CTL_RX_DV */
	RZ_G2L_PIN(25, 0),	/* ETH0_RXD0 */
	RZ_G2L_PIN(25, 1),	/* ETH0_RXD1 */
	RZ_G2L_PIN(26, 0),	/* ETH0_RXD2 */
	RZ_G2L_PIN(26, 1),	/* ETH0_RXD3 */
	RZ_G2L_PIN(27, 0),	/* ETH0_RX_ERR */
};

static int eth0_rgmii_pins[] = {
	RZ_G2L_PIN(20, 0),	/* ETH0_TXC_TX_CLK */
	RZ_G2L_PIN(20, 1),	/* ETH0_TX_CTL_TX_EN */
	RZ_G2L_PIN(20, 2),	/* ETH0_TXD0 */
	RZ_G2L_PIN(21, 0),	/* ETH0_TXD1 */
	RZ_G2L_PIN(21, 1),	/* ETH0_TXD2 */
	RZ_G2L_PIN(22, 0),	/* ETH0_TXD3 */
	RZ_G2L_PIN(24, 0),	/* ETH0_RXC_RX_CLK */
	RZ_G2L_PIN(24, 1),	/* ETH0_RX_CTL_RX_DV */
	RZ_G2L_PIN(25, 0),	/* ETH0_RXD0 */
	RZ_G2L_PIN(25, 1),	/* ETH0_RXD1 */
	RZ_G2L_PIN(26, 0),	/* ETH0_RXD2 */
	RZ_G2L_PIN(26, 1),	/* ETH0_RXD3 */
};

/* Ethernet channel 1 */
static int eth1_link_pins[] = {
	/* ETH1_LINKSTA */
	RZ_G2L_PIN(37, 2),
};

static int eth1_mdio_pins[] = {
	/* ETH1_MDC, ETH1_MDIO */
	RZ_G2L_PIN(37, 0), RZ_G2L_PIN(37, 1),
};

static int eth1_mii_pins[] = {
	RZ_G2L_PIN(29, 0),	/* ETH1_TXC_TX_CLK */
	RZ_G2L_PIN(29, 1),	/* ETH1_TX_CTL_TX_EN */
	RZ_G2L_PIN(30, 0),	/* ETH1_TXD0 */
	RZ_G2L_PIN(30, 1),	/* ETH1_TXD1 */
	RZ_G2L_PIN(31, 0),	/* ETH1_TXD2 */
	RZ_G2L_PIN(31, 1),	/* ETH1_TXD3 */
	RZ_G2L_PIN(32, 0),	/* ETH1_TX_ERR */
	RZ_G2L_PIN(32, 1),	/* ETH1_TX_COL */
	RZ_G2L_PIN(33, 0),	/* ETH1_TX_CRS */
	RZ_G2L_PIN(33, 1),	/* ETH1_RXC_RX_CLK */
	RZ_G2L_PIN(34, 0),	/* ETH1_RX_CTL_RX_DV */
	RZ_G2L_PIN(34, 1),	/* ETH1_RXD0 */
	RZ_G2L_PIN(35, 0),	/* ETH1_RXD1 */
	RZ_G2L_PIN(35, 1),	/* ETH1_RXD2 */
	RZ_G2L_PIN(36, 0),	/* ETH1_RXD3 */
	RZ_G2L_PIN(36, 1),	/* ETH1_RX_ERR */
};

static int eth1_rgmii_pins[] = {
	RZ_G2L_PIN(29, 0),	/* ETH1_TXC_TX_CLK */
	RZ_G2L_PIN(29, 1),	/* ETH1_TX_CTL_TX_EN */
	RZ_G2L_PIN(30, 0),	/* ETH1_TXD0 */
	RZ_G2L_PIN(30, 1),	/* ETH1_TXD1 */
	RZ_G2L_PIN(31, 0),	/* ETH1_TXD2 */
	RZ_G2L_PIN(31, 1),	/* ETH1_TXD3 */
	RZ_G2L_PIN(33, 1),	/* ETH1_RXC_RX_CLK */
	RZ_G2L_PIN(34, 0),	/* ETH1_RX_CTL_RX_DV */
	RZ_G2L_PIN(34, 1),	/* ETH1_RXD0 */
	RZ_G2L_PIN(35, 0),	/* ETH1_RXD1 */
	RZ_G2L_PIN(35, 1),	/* ETH1_RXD2 */
	RZ_G2L_PIN(36, 0),	/* ETH1_RXD3 */
};

static struct group_desc r9a07g044l_groups[] = {
	RZ_G2L_PINCTRL_PIN_GROUP(irq0_a, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(irq0_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(irq0_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(irq1_a, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(irq1_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(irq1_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(irq2_a, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(irq2_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(irq2_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(irq3_a, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(irq3_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(irq3_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(irq4_a, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(irq4_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(irq4_c, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(irq5_a, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(irq5_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(irq5_c, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(irq6_a, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(irq6_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(irq6_c, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(irq7_a, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(irq7_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(irq7_c, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(irq7_d, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(usb0_a, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(usb0_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(usb1_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(usb1_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(usb1_c, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(usb1_d, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(sci0_data_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(sci0_clk_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(sci0_ctrl_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(sci0_data_b, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(sci0_clk_b, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(sci0_ctrl_b, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(sci0_data_c, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(sci0_clk_c, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(sci0_ctrl_c, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(sci1_data_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(sci1_clk_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(sci1_ctrl_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(sci1_data_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(sci1_clk_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(sci1_ctrl_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(scif0_data, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(scif0_clk, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(scif0_ctrl, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(scif1_data, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(scif1_clk, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(scif1_ctrl, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(scif2_data_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(scif2_clk_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(scif2_ctrl_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(scif2_data_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(scif2_clk_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(scif2_ctrl_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(scif2_data_c, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(scif2_clk_c, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(scif2_ctrl_c, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(scif2_data_d, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(scif2_clk_d, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(scif2_ctrl_d, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(scif2_data_e, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(scif2_clk_e, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(scif2_ctrl_e, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(scif3_data, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(scif3_clk, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(scif4_data, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(scif4_clk, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt_ext_trg_a_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt_ext_trg_b_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt_ext_trg_c_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt_ext_trg_d_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt_ext_trg_a_b, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt_ext_trg_b_b, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt_ext_trg_c_b, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt_ext_trg_d_b, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt_ext_trg_a_c, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt_ext_trg_b_c, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt_ext_trg_c_c, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt_ext_trg_d_c, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt_ext_trg_a_d, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt_ext_trg_b_d, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt_ext_trg_c_d, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt_ext_trg_d_d, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt0_a_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt0_b_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt0_a_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt0_b_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt0_a_c, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt0_b_c, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt1_a_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt1_b_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt1_a_b, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt1_b_b, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt2_a_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt2_b_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt2_a_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt2_b_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt2_a_c, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt2_b_c, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt3_a_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt3_b_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt3_a_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt3_b_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt3_a_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt3_b_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt3_a_d, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt3_b_d, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt4_a_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt4_b_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt4_a_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt4_b_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt5_a_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt5_b_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt5_a_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt5_b_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt6_a_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt6_b_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt6_a_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt6_b_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt6_a_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt6_b_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt6_a_d, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt6_b_d, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt7_a_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt7_b_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt7_a_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt7_b_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt7_a_c, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(gpt7_b_c, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu0_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu0_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu1_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu1_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu1_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu2_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu2_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu2_c, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu4, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu3_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu3_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu5_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu5_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu6, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu7_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu7_b, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu8_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(mtu8_b, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(cam_field_a, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(cam_field_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(cam_clk, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(cam_sync, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(cam_data8, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(cam_data10, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(cam_data12, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(cam_data16, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(disp_clk, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(disp_sync, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(disp_de, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(disp_bgr666, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(disp_bgr888, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(adc_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(adc_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(adc_c, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(adc_d, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(adc_e, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(i2c2_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(i2c2_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(i2c2_c, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(i2c2_d, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(i2c2_e, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(i2c3_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(i2c3_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(i2c3_c, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(i2c3_d, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi0_data_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi0_ctrl_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi0_data_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi0_ctrl_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi0_data_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi0_ctrl_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi0_data_d, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi0_ctrl_d, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi1_data_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi1_ctrl_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi1_data_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi1_ctrl_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi1_data_c, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi1_ctrl_c, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi1_data_d, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi1_ctrl_d, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi1_data_e, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi1_ctrl_e, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi2_data_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi2_ctrl_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi2_data_b, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi2_ctrl_b, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi3_data, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(ssi3_ctrl, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi0_clk_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi0_data_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi0_ssl_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi0_clk_b, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi0_data_b, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi0_ssl_b, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi0_clk_c, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi0_data_c, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi0_ssl_c, 5),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi1_clk_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi1_data_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi1_ssl_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi1_clk_b, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi1_data_b, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi1_ssl_b, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi1_clk_c, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi1_data_c, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi1_ssl_c, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi2_clk_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi2_data_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi2_ssl_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi2_clk_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi2_data_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(rspi2_ssl_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(can_clk_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(can0_tx_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(can0_rx_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(can0_tx_datarate_en_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(can0_rx_datarate_en_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_tx_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_rx_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_tx_datarate_en_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_rx_datarate_en_a, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(can_clk_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can0_tx_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can0_rx_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can0_tx_datarate_en_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can0_rx_datarate_en_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_tx_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_rx_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_tx_datarate_en_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_rx_datarate_en_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can_clk_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can0_tx_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can0_rx_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can0_tx_datarate_en_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can0_rx_datarate_en_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_tx_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_rx_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_tx_datarate_en_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_rx_datarate_en_c, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can_clk_d, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can0_tx_d, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can0_rx_d, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can0_tx_datarate_en_d, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can0_rx_datarate_en_d, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_tx_d, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_rx_d, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_tx_datarate_en_d, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_rx_datarate_en_d, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_tx_e, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_rx_e, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_tx_datarate_en_e, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(can1_rx_datarate_en_e, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(poe_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(poe_b, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(poe_c, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(sdhi0_cd_a, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(sdhi0_wp_a, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(sdhi0_cd_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(sdhi0_wp_b, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(sdhi1_cd_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(sdhi1_wp_a, 3),
	RZ_G2L_PINCTRL_PIN_GROUP(sdhi1_cd_b, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(sdhi1_wp_b, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(sdhi1_cd_c, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(sdhi1_wp_c, 2),
	RZ_G2L_PINCTRL_PIN_GROUP(mtclka_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtclkb_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtclkc_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtclkd_a, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtclka_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtclkb_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtclkc_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtclkd_b, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtclka_c, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtclkb_c, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtclkc_c, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(mtclkd_c, 4),
	RZ_G2L_PINCTRL_PIN_GROUP(eth0_link, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(eth0_mdio, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(eth0_mii, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(eth0_rgmii, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(eth1_link, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(eth1_mdio, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(eth1_mii, 1),
	RZ_G2L_PINCTRL_PIN_GROUP(eth1_rgmii, 1),
};

static const char *r9a07g044l_irq_groups[] = {
	"irq0_a", "irq1_a", "irq2_a", "irq3_a",
	"irq4_a", "irq5_a", "irq6_a", "irq7_a",
	"irq0_b", "irq1_b", "irq2_b", "irq3_b",
	"irq4_b", "irq5_b", "irq6_b", "irq7_b",
	"irq0_c", "irq1_c", "irq2_c", "irq3_c",
	"irq4_c", "irq5_c", "irq6_c", "irq7_c",
	"irq7_d",
};

static const char *r9a07g044l_sci0_groups[] = {
	"sci0_data_a", "sci0_clk_a", "sci0_ctrl_a",
	"sci0_data_b", "sci0_clk_b", "sci0_ctrl_b",
	"sci0_data_c", "sci0_clk_c", "sci0_ctrl_c",
};

static const char *r9a07g044l_sci1_groups[] = {
	"sci1_data_a", "sci1_clk_a", "sci1_ctrl_a",
	"sci1_data_b", "sci1_clk_b", "sci1_ctrl_b",
};

static const char *r9a07g044l_scif0_groups[] = {
	"scif0_data", "scif0_clk", "scif0_ctrl",
};

static const char *r9a07g044l_scif1_groups[] = {
	"scif1_data", "scif1_clk", "scif1_ctrl",
};

static const char *r9a07g044l_scif2_groups[] = {
	"scif2_data_a", "scif2_clk_a", "scif2_ctrl_a",
	"scif2_data_b", "scif2_clk_b", "scif2_ctrl_b",
	"scif2_data_c", "scif2_clk_c", "scif2_ctrl_c",
	"scif2_data_d", "scif2_clk_d", "scif2_ctrl_d",
	"scif2_data_e", "scif2_clk_e", "scif2_ctrl_e",
};

static const char *r9a07g044l_scif3_groups[] = {
	"scif3_data", "scif3_clk",
};

static const char *r9a07g044l_scif4_groups[] = {
	"scif4_data", "scif4_clk",
};

static const char *r9a07g044l_cam_groups[] = {
	"cam_field_a", "cam_field_b",
	"cam_clk", "cam_sync",
	"cam_data8", "cam_data10", "cam_data12", "cam_data16",
};

static const char *r9a07g044l_gpt_ext_trg_groups[] = {
	"gpt_ext_trg_a_a", "gpt_ext_trg_b_a",
	"gpt_ext_trg_c_a", "gpt_ext_trg_d_a",
	"gpt_ext_trg_a_b", "gpt_ext_trg_b_b",
	"gpt_ext_trg_c_b", "gpt_ext_trg_d_b",
	"gpt_ext_trg_a_c", "gpt_ext_trg_b_c",
	"gpt_ext_trg_c_c", "gpt_ext_trg_d_c",
	"gpt_ext_trg_a_b", "gpt_ext_trg_b_d",
	"gpt_ext_trg_c_d", "gpt_ext_trg_d_d",
};

static const char *r9a07g044l_gpt0_groups[] = {
	"gpt0_a_a", "gpt0_b_a",
	"gpt0_a_b", "gpt0_b_b",
	"gpt0_a_c", "gpt0_b_c",
};

static const char *r9a07g044l_gpt1_groups[] = {
	"gpt1_a_a", "gpt1_b_a",
	"gpt1_a_b", "gpt1_b_b",
};

static const char *r9a07g044l_gpt2_groups[] = {
	"gpt2_a_a", "gpt2_b_a",
	"gpt2_a_b", "gpt2_b_b",
	"gpt2_a_c", "gpt2_b_c",
};

static const char *r9a07g044l_gpt3_groups[] = {
	"gpt3_a_a", "gpt3_b_a",
	"gpt3_a_b", "gpt3_b_b",
	"gpt3_a_c", "gpt3_b_c",
	"gpt3_a_d", "gpt3_b_d",
};

static const char *r9a07g044l_gpt4_groups[] = {
	"gpt4_a_a", "gpt4_b_a",
	"gpt4_a_b", "gpt4_b_b",
};

static const char *r9a07g044l_gpt5_groups[] = {
	"gpt5_a_a", "gpt5_b_a",
	"gpt5_a_b", "gpt5_b_b",
};

static const char *r9a07g044l_gpt6_groups[] = {
	"gpt6_a_a", "gpt6_b_a",
	"gpt6_a_b", "gpt6_b_b",
	"gpt6_a_c", "gpt6_b_c",
	"gpt6_a_d", "gpt6_b_d",
};

static const char *r9a07g044l_gpt7_groups[] = {
	"gpt7_a_a", "gpt7_b_a",
	"gpt7_a_b", "gpt7_b_b",
	"gpt7_a_c", "gpt7_b_c",
};

static const char *r9a07g044l_mtu0_groups[] = {
	"mtu0_a", "mtu0_b",
};

static const char *r9a07g044l_mtu1_groups[] = {
	"mtu1_a", "mtu1_b", "mtu1_c",
};

static const char *r9a07g044l_mtu2_groups[] = {
	"mtu2_a", "mtu2_b", "mtu2_c",
};

static const char *r9a07g044l_mtu3_groups[] = {
	"mtu3_a", "mtu3_b",
};

static const char *r9a07g044l_mtu4_groups[] = {
	"mtu4",
};

static const char *r9a07g044l_mtu5_groups[] = {
	"mtu5_a", "mtu5_b",
};

static const char *r9a07g044l_mtu6_groups[] = {
	"mtu6",
};

static const char *r9a07g044l_mtu7_groups[] = {
	"mtu7_a", "mtu7_b",
};

static const char *r9a07g044l_mtu8_groups[] = {
	"mtu8_a", "mtu8_b",
};

static const char *r9a07g044l_usb0_groups[] = {
	"usb0_a", "usb0_b",
};

static const char *r9a07g044l_usb1_groups[] = {
	"usb1_a", "usb1_b", "usb1_c", "usb1_d",
};

static const char *r9a07g044l_disp_groups[] = {
	"disp_clk", "disp_sync", "disp_de",
	"disp_bgr666", "disp_bgr888",
};

static const char *r9a07g044l_adc_groups[] = {
	"adc_a", "adc_b", "adc_c", "adc_d", "adc_e",
};

static const char *r9a07g044l_i2c2_groups[] = {
	"i2c2_a", "i2c2_b", "i2c2_c", "i2c2_d", "i2c2_e",
};

static const char *r9a07g044l_i2c3_groups[] = {
	"i2c3_a", "i2c3_b", "i2c3_c", "i2c3_d",
};

static const char *r9a07g044l_ssi0_groups[] = {
	"ssi0_data_a", "ssi0_ctrl_a",
	"ssi0_data_b", "ssi0_ctrl_b",
	"ssi0_data_c", "ssi0_ctrl_c",
	"ssi0_data_d", "ssi0_ctrl_d",
};

static const char *r9a07g044l_ssi1_groups[] = {
	"ssi1_data_a", "ssi1_ctrl_a",
	"ssi1_data_b", "ssi1_ctrl_b",
	"ssi1_data_c", "ssi1_ctrl_c",
	"ssi1_data_d", "ssi1_ctrl_d",
	"ssi1_data_e", "ssi1_ctrl_e",
};

static const char *r9a07g044l_ssi2_groups[] = {
	"ssi2_data_a", "ssi2_ctrl_a",
	"ssi2_data_b", "ssi2_ctrl_b",
};

static const char *r9a07g044l_ssi3_groups[] = {
	"ssi3_data", "ssi3_ctrl",
};

static const char *r9a07g044l_rspi0_groups[] = {
	"rspi0_clk_a", "rspi0_data_a", "rspi0_ssl_a",
	"rspi0_clk_b", "rspi0_data_b", "rspi0_ssl_b",
	"rspi0_clk_c", "rspi0_data_c", "rspi0_ssl_c",
};

static const char *r9a07g044l_rspi1_groups[] = {
	"rspi1_clk_a", "rspi1_data_a", "rspi1_ssl_a",
	"rspi1_clk_b", "rspi1_data_b", "rspi1_ssl_b",
	"rspi1_clk_c", "rspi1_data_c", "rspi1_ssl_c",
};

static const char *r9a07g044l_rspi2_groups[] = {
	"rspi2_clk_a", "rspi2_data_a", "rspi2_ssl_a",
	"rspi2_clk_b", "rspi2_data_b", "rspi2_ssl_b",
};

static const char *r9a07g044l_can_clk_groups[] = {
	"can_clk_a", "can_clk_b", "can_clk_c", "can_clk_d",
};

static const char *r9a07g044l_can0_groups[] = {
	"can0_tx_a", "can0_rx_a",
	"can0_tx_datarate_en_a", "can0_rx_datarate_en_a",
	"can0_tx_b", "can0_rx_b",
	"can0_tx_datarate_en_b", "can0_rx_datarate_en_b",
	"can0_tx_c", "can0_rx_c",
	"can0_tx_datarate_en_c", "can0_rx_datarate_en_c",
	"can0_tx_d", "can0_rx_d",
	"can0_tx_datarate_en_d", "can0_rx_datarate_en_d",
};

static const char *r9a07g044l_can1_groups[] = {
	"can1_tx_a", "can1_rx_a",
	"can1_tx_datarate_en_a", "can1_rx_datarate_en_a",
	"can1_tx_b", "can1_rx_b",
	"can1_tx_datarate_en_b", "can1_rx_datarate_en_b",
	"can1_tx_c", "can1_rx_c",
	"can1_tx_datarate_en_c", "can1_rx_datarate_en_c",
	"can1_tx_d", "can1_rx_d",
	"can1_tx_datarate_en_d", "can1_rx_datarate_en_d",
	"can1_tx_e", "can1_rx_e",
	"can1_tx_datarate_en_e", "can1_rx_datarate_en_e",
};

static const char *r9a07g044l_poe_groups[] = {
	"poe_a", "poe_b", "poe_c",
};

static const char *r9a07g044l_sdhi0_groups[] = {
	"sdhi0_cd_a", "sdhi0_wp_a",
	"sdhi0_cd_b", "sdhi0_wp_b",
};

static const char *r9a07g044l_sdhi1_groups[] = {
	"sdhi1_cd_a", "sdhi1_wp_a",
	"sdhi1_cd_b", "sdhi1_wp_b",
	"sdhi1_cd_c", "sdhi1_wp_c",
};

static const char *r9a07g044l_mtclk_groups[] = {
	"mtclka_a", "mtclkb_a", "mtclkc_a", "mtclkd_a",
	"mtclka_b", "mtclkb_b", "mtclkc_b", "mtclkd_b",
	"mtclka_c", "mtclkb_c", "mtclkc_c", "mtclkd_c",
};

static const char *r9a07g044l_eth0_groups[] = {
	"eth0_link", "eth0_mdio", "eth0_mii", "eth0_rgmii",
};

static const char *r9a07g044l_eth1_groups[] = {
	"eth1_link", "eth1_mdio", "eth1_mii", "eth1_rgmii",
};

static const struct function_desc r9a07g044l_funcs[] = {
	{"irq", r9a07g044l_irq_groups,
		ARRAY_SIZE(r9a07g044l_irq_groups)},
	{"sci0", r9a07g044l_sci0_groups,
		ARRAY_SIZE(r9a07g044l_sci0_groups)},
	{"sci1", r9a07g044l_sci1_groups,
		ARRAY_SIZE(r9a07g044l_sci1_groups)},
	{"scif0", r9a07g044l_scif0_groups,
		ARRAY_SIZE(r9a07g044l_scif0_groups)},
	{"scif1", r9a07g044l_scif1_groups,
		ARRAY_SIZE(r9a07g044l_scif1_groups)},
	{"scif2", r9a07g044l_scif2_groups,
		ARRAY_SIZE(r9a07g044l_scif2_groups)},
	{"scif3", r9a07g044l_scif3_groups,
		ARRAY_SIZE(r9a07g044l_scif3_groups)},
	{"scif4", r9a07g044l_scif4_groups,
		ARRAY_SIZE(r9a07g044l_scif4_groups)},
	{"gpt_ext_trg", r9a07g044l_gpt_ext_trg_groups,
		ARRAY_SIZE(r9a07g044l_gpt_ext_trg_groups)},
	{"gpt0", r9a07g044l_gpt0_groups,
		ARRAY_SIZE(r9a07g044l_gpt0_groups)},
	{"gpt1", r9a07g044l_gpt1_groups,
		ARRAY_SIZE(r9a07g044l_gpt1_groups)},
	{"gpt2", r9a07g044l_gpt2_groups,
		ARRAY_SIZE(r9a07g044l_gpt2_groups)},
	{"gpt3", r9a07g044l_gpt3_groups,
		ARRAY_SIZE(r9a07g044l_gpt3_groups)},
	{"gpt4", r9a07g044l_gpt4_groups,
		ARRAY_SIZE(r9a07g044l_gpt4_groups)},
	{"gpt5", r9a07g044l_gpt5_groups,
		ARRAY_SIZE(r9a07g044l_gpt5_groups)},
	{"gpt6", r9a07g044l_gpt6_groups,
		ARRAY_SIZE(r9a07g044l_gpt6_groups)},
	{"gpt7", r9a07g044l_gpt7_groups,
		ARRAY_SIZE(r9a07g044l_gpt7_groups)},
	{"mtu0", r9a07g044l_mtu0_groups,
		ARRAY_SIZE(r9a07g044l_mtu0_groups)},
	{"mtu1", r9a07g044l_mtu1_groups,
		ARRAY_SIZE(r9a07g044l_mtu1_groups)},
	{"mtu2", r9a07g044l_mtu2_groups,
		ARRAY_SIZE(r9a07g044l_mtu2_groups)},
	{"mtu3", r9a07g044l_mtu3_groups,
		ARRAY_SIZE(r9a07g044l_mtu3_groups)},
	{"mtu4", r9a07g044l_mtu4_groups,
		ARRAY_SIZE(r9a07g044l_mtu4_groups)},
	{"mtu5", r9a07g044l_mtu5_groups,
		ARRAY_SIZE(r9a07g044l_mtu5_groups)},
	{"mtu6", r9a07g044l_mtu6_groups,
		ARRAY_SIZE(r9a07g044l_mtu6_groups)},
	{"mtu7", r9a07g044l_mtu7_groups,
		ARRAY_SIZE(r9a07g044l_mtu7_groups)},
	{"mtu8", r9a07g044l_mtu8_groups,
		ARRAY_SIZE(r9a07g044l_mtu8_groups)},
	{"cam", r9a07g044l_cam_groups,
		ARRAY_SIZE(r9a07g044l_cam_groups)},
	{"usb0", r9a07g044l_usb0_groups,
		ARRAY_SIZE(r9a07g044l_usb0_groups)},
	{"usb1", r9a07g044l_usb1_groups,
		ARRAY_SIZE(r9a07g044l_usb1_groups)},
	{"disp", r9a07g044l_disp_groups,
		ARRAY_SIZE(r9a07g044l_disp_groups)},
	{"adc", r9a07g044l_adc_groups,
		ARRAY_SIZE(r9a07g044l_adc_groups)},
	{"i2c2", r9a07g044l_i2c2_groups,
		ARRAY_SIZE(r9a07g044l_i2c2_groups)},
	{"i2c3", r9a07g044l_i2c3_groups,
		ARRAY_SIZE(r9a07g044l_i2c2_groups)},
	{"ssi0", r9a07g044l_ssi0_groups,
		ARRAY_SIZE(r9a07g044l_ssi0_groups)},
	{"ssi1", r9a07g044l_ssi1_groups,
		ARRAY_SIZE(r9a07g044l_ssi1_groups)},
	{"ssi2", r9a07g044l_ssi2_groups,
		ARRAY_SIZE(r9a07g044l_ssi2_groups)},
	{"ssi3", r9a07g044l_ssi3_groups,
		ARRAY_SIZE(r9a07g044l_ssi3_groups)},
	{"rspi0", r9a07g044l_rspi0_groups,
		ARRAY_SIZE(r9a07g044l_rspi0_groups)},
	{"rspi1", r9a07g044l_rspi1_groups,
		ARRAY_SIZE(r9a07g044l_rspi1_groups)},
	{"rspi2", r9a07g044l_rspi2_groups,
		ARRAY_SIZE(r9a07g044l_rspi2_groups)},
	{"can_clk", r9a07g044l_can_clk_groups,
		ARRAY_SIZE(r9a07g044l_can_clk_groups)},
	{"can0", r9a07g044l_can0_groups,
		ARRAY_SIZE(r9a07g044l_can0_groups)},
	{"can1", r9a07g044l_can1_groups,
		ARRAY_SIZE(r9a07g044l_can1_groups)},
	{"poe", r9a07g044l_poe_groups,
		ARRAY_SIZE(r9a07g044l_poe_groups)},
	{"sdhi0", r9a07g044l_sdhi0_groups,
		ARRAY_SIZE(r9a07g044l_sdhi0_groups)},
	{"sdhi1", r9a07g044l_sdhi1_groups,
		ARRAY_SIZE(r9a07g044l_sdhi1_groups)},
	{"mtclk", r9a07g044l_mtclk_groups,
		ARRAY_SIZE(r9a07g044l_mtclk_groups)},
	{"eth0", r9a07g044l_eth0_groups,
		ARRAY_SIZE(r9a07g044l_eth0_groups)},
	{"eth1", r9a07g044l_eth1_groups,
		ARRAY_SIZE(r9a07g044l_eth1_groups)},
};

/* R9A07G044L support 123 interrupt input pins */
static struct rzg2l_pin_info r9a07g044l_pin_info[] = {
	{0, 0, 0}, {0, 1, 1},
	{1, 0, 2}, {1, 1, 3},
	{2, 0, 4}, {2, 1, 5},
	{3, 0, 6}, {3, 1, 7},
	{4, 0, 8}, {4, 1, 9},
	{5, 0, 10}, {5, 1, 11}, {5, 2, 12},
	{6, 0, 13}, {6, 1, 14},
	{7, 0, 15}, {7, 1, 16}, {7, 2, 17},
	{8, 0, 18}, {8, 1, 19}, {8, 2, 20},
	{9, 0, 21}, {9, 1, 22},
	{10, 0, 23}, {10, 1, 24},
	{11, 0, 25}, {11, 1, 26},
	{12, 0, 27}, {12, 1, 28},
	{13, 0, 29}, {13, 1, 30}, {13, 2, 31},
	{14, 0, 32}, {14, 1, 33},
	{15, 0, 34}, {15, 1, 35},
	{16, 0, 36}, {16, 1, 37},
	{17, 0, 38}, {17, 1, 39}, {17, 2, 40},
	{18, 0, 41}, {18, 1, 42},
	{19, 0, 43}, {19, 1, 44},
	{20, 0, 45}, {20, 1, 46}, {20, 2, 47},
	{21, 0, 48}, {21, 1, 49},
	{22, 0, 50}, {22, 1, 51},
	{23, 0, 52}, {23, 1, 53},
	{24, 0, 54}, {24, 1, 55},
	{25, 0, 56}, {25, 1, 57},
	{26, 0, 58}, {26, 1, 59},
	{27, 0, 60}, {27, 1, 61},
	{28, 0, 62}, {28, 1, 63},
	{29, 0, 64}, {29, 1, 65},
	{30, 0, 66}, {30, 1, 67},
	{31, 0, 68}, {31, 1, 69},
	{32, 0, 70}, {32, 1, 71},
	{33, 0, 72}, {33, 1, 73},
	{34, 0, 74}, {34, 1, 75},
	{35, 0, 76}, {35, 1, 77},
	{36, 0, 78}, {36, 1, 79},
	{37, 0, 80}, {37, 1, 81}, {37, 2, 82},
	{38, 0, 83}, {38, 1, 84},
	{39, 0, 85}, {39, 1, 86}, {39, 2, 87},
	{40, 0, 88}, {40, 1, 89}, {40, 2, 90},
	{41, 0, 91}, {41, 1, 92},
	{42, 0, 93}, {42, 1, 94}, {42, 2, 95}, {42, 3, 96}, {42, 4, 97},
	{43, 0, 98}, {43, 1, 99}, {43, 2, 100}, {43, 3, 101},
	{44, 0, 102}, {44, 1, 103}, {44, 2, 104}, {44, 3, 105},
	{45, 0, 106}, {45, 1, 107}, {45, 2, 108}, {45, 3, 109},
	{46, 0, 110}, {46, 1, 111}, {46, 2, 112}, {46, 3, 113},
	{47, 0, 114}, {47, 1, 115}, {47, 2, 116}, {47, 3, 117},
	{48, 0, 118}, {48, 1, 119}, {48, 2, 120}, {48, 3, 121}, {48, 3, 122},
};

const struct rzg2l_pin_soc r9a07g044l_pinctrl_data = {
	.pins =	r9a07g044l_pins.pin_gpio,
	.npins = ARRAY_SIZE(r9a07g044l_pins.pin_gpio) +
		 ARRAY_SIZE(r9a07g044l_pins.pin_no_gpio),
	.groups	= r9a07g044l_groups,
	.ngroups = ARRAY_SIZE(r9a07g044l_groups),
	.funcs = r9a07g044l_funcs,
	.nfuncs	= ARRAY_SIZE(r9a07g044l_funcs),
	.nports = 49,
	.nirqs = 32,
	.pin_info = r9a07g044l_pin_info,
	.ngpioints = ARRAY_SIZE(r9a07g044l_pin_info),
};
