/*
 * DHCOM DH-iMX6 PDK board support
 *
 * Copyright (C) 2017 Marek Vasut <marex@denx.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/sata.h>
#include <asm/mach-imx/video.h>
#include <errno.h>
#include <fsl_esdhc.h>
#include <fuse.h>
#include <i2c.h>
#include <miiphy.h>
#include <mmc.h>
#include <net.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <usb.h>
#include <usb/ehci-ci.h>

/* Special MXCFB sync flags are here. */
#include "../drivers/video/mxcfb.h"

DECLARE_GLOBAL_DATA_PTR;

#define I2C_PAD_CTRL							\
	(PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |	\
	PAD_CTL_HYS | PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define EEPROM_I2C_ADDRESS	0x50

#define PC			MUX_PAD_CTRL(I2C_PAD_CTRL)

static iomux_v3_cfg_t const rgb_pads[] = {
	IOMUX_PADS(PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN15__IPU1_DI0_PIN15 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN2__IPU1_DI0_PIN02 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN3__IPU1_DI0_PIN03 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN4__IPU1_DI0_PIN04 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT0__IPU1_DISP0_DATA00 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT1__IPU1_DISP0_DATA01 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT2__IPU1_DISP0_DATA02 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT3__IPU1_DISP0_DATA03 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT4__IPU1_DISP0_DATA04 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT5__IPU1_DISP0_DATA05 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT6__IPU1_DISP0_DATA06 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT7__IPU1_DISP0_DATA07 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT8__IPU1_DISP0_DATA08 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT9__IPU1_DISP0_DATA09 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT10__IPU1_DISP0_DATA10 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT11__IPU1_DISP0_DATA11 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT12__IPU1_DISP0_DATA12 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT13__IPU1_DISP0_DATA13 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT14__IPU1_DISP0_DATA14 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT15__IPU1_DISP0_DATA15 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT16__IPU1_DISP0_DATA16 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT17__IPU1_DISP0_DATA17 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT18__IPU1_DISP0_DATA18 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT19__IPU1_DISP0_DATA19 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT20__IPU1_DISP0_DATA20 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT21__IPU1_DISP0_DATA21 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT22__IPU1_DISP0_DATA22 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT23__IPU1_DISP0_DATA23 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void enable_backlight(void)
{
	/* let dhcom_settings handle backlight */
	run_command ("backlight", 0);
}

static void enable_rgb(struct display_info_t const *dev)
{
	SETUP_IOMUX_PADS(rgb_pads);
	enable_backlight();
}

static void enable_lvds(struct display_info_t const *dev)
{
	enable_backlight();
}

static struct i2c_pads_info dh6sdl_i2c_pad_info0 = {
	.scl = {
		.i2c_mode  = MX6DL_PAD_EIM_D21__I2C1_SCL | PC,
		.gpio_mode = MX6DL_PAD_EIM_D21__GPIO3_IO21 | PC,
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		 .i2c_mode = MX6DL_PAD_EIM_D28__I2C1_SDA | PC,
		 .gpio_mode = MX6DL_PAD_EIM_D28__GPIO3_IO28 | PC,
		 .gp = IMX_GPIO_NR(3, 28)
	 }
};

static struct i2c_pads_info dh6sdl_i2c_pad_info1 = {
	.scl = {
		.i2c_mode  = MX6DL_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6DL_PAD_KEY_COL3__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		 .i2c_mode = MX6DL_PAD_KEY_ROW3__I2C2_SDA | PC,
		 .gpio_mode = MX6DL_PAD_KEY_ROW3__GPIO4_IO13 | PC,
		 .gp = IMX_GPIO_NR(4, 13)
	 }
};

static struct i2c_pads_info dh6sdl_i2c_pad_info2 = {
	.scl = {
		.i2c_mode  = MX6DL_PAD_GPIO_3__I2C3_SCL | PC,
		.gpio_mode = MX6DL_PAD_GPIO_3__GPIO1_IO03 | PC,
		.gp = IMX_GPIO_NR(1, 3)
	},
	.sda = {
		 .i2c_mode = MX6DL_PAD_GPIO_6__I2C3_SDA | PC,
		 .gpio_mode = MX6DL_PAD_GPIO_6__GPIO1_IO06 | PC,
		 .gp = IMX_GPIO_NR(1, 6)
	 }
};

static struct i2c_pads_info dh6dq_i2c_pad_info0 = {
	.scl = {
		.i2c_mode  = MX6Q_PAD_EIM_D21__I2C1_SCL | PC,
		.gpio_mode = MX6Q_PAD_EIM_D21__GPIO3_IO21 | PC,
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		 .i2c_mode = MX6Q_PAD_EIM_D28__I2C1_SDA | PC,
		 .gpio_mode = MX6Q_PAD_EIM_D28__GPIO3_IO28 | PC,
		 .gp = IMX_GPIO_NR(3, 28)
	 }
};

static struct i2c_pads_info dh6dq_i2c_pad_info1 = {
	.scl = {
		.i2c_mode  = MX6Q_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6Q_PAD_KEY_COL3__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		 .i2c_mode = MX6Q_PAD_KEY_ROW3__I2C2_SDA | PC,
		 .gpio_mode = MX6Q_PAD_KEY_ROW3__GPIO4_IO13 | PC,
		 .gp = IMX_GPIO_NR(4, 13)
	 }
};

static struct i2c_pads_info dh6dq_i2c_pad_info2 = {
	.scl = {
		.i2c_mode  = MX6Q_PAD_GPIO_3__I2C3_SCL | PC,
		.gpio_mode = MX6Q_PAD_GPIO_3__GPIO1_IO03 | PC,
		.gp = IMX_GPIO_NR(1, 3)
	},
	.sda = {
		 .i2c_mode = MX6Q_PAD_GPIO_6__I2C3_SDA | PC,
		 .gpio_mode = MX6Q_PAD_GPIO_6__GPIO1_IO06 | PC,
		 .gp = IMX_GPIO_NR(1, 6)
	 }
};

#if defined(CONFIG_VIDEO_IPUV3)
static iomux_v3_cfg_t const di0_pads[] = {
	IOMUX_PADS(PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK),	/* DISP0_CLK */
	IOMUX_PADS(PAD_DI0_PIN15__IPU1_DI0_PIN15),
	IOMUX_PADS(PAD_DI0_PIN2__IPU1_DI0_PIN02),		/* DISP0_HSYNC */
	IOMUX_PADS(PAD_DI0_PIN3__IPU1_DI0_PIN03),		/* DISP0_VSYNC */
};

static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}

struct display_info_t displays[] = {{
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "RGB",
		.refresh        = 57,
		.xres           = 800,
		.yres           = 480,
                .pixclock       = 33260,
                .left_margin    = 42,
                .right_margin   = 86,
		.upper_margin   = 10,
                .lower_margin   = 33,
                .hsync_len      = 128,
                .vsync_len      = 2,
		.sync           = FB_SYNC_CLK_LAT_FALL,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "lvds_24bit",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 40000,
		.left_margin    = 42,
		.right_margin   = 86,
		.upper_margin   = 10,
		.lower_margin   = 33,
		.hsync_len      = 128,
		.vsync_len      = 2,
		.sync           = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT | FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_hdmi,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15384,
		.left_margin    = 160,
		.right_margin   = 24,
		.upper_margin   = 29,
		.lower_margin   = 3,
		.hsync_len      = 136,
		.vsync_len      = 6,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	/* Setup HSYNC, VSYNC, DISP_CLK for debugging purposes */
	SETUP_IOMUX_PADS(di0_pads);

	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
			| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

/*
 * skip the cfb initialization.
 */
int board_cfb_skip(void)
{
	return 1;
}

#ifdef CONFIG_FEC_MXC
static void eth_phy_reset(void)
{
	/* Reset PHY */
	gpio_direction_output(IMX_GPIO_NR(5, 0) , 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(5, 0), 1);

	/* Enable VIO */
	gpio_direction_output(IMX_GPIO_NR(1, 7) , 0);

	/*
	 * KSZ9021 PHY needs at least 10 mSec after PHY reset
	 * is released to stabilize
	 */
	mdelay(10);
}

static int setup_fec_clock(void)
{
	struct iomuxc *iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

	/* set gpr1[21] to select anatop clock */
	clrsetbits_le32(&iomuxc_regs->gpr[1], 0x1 << 21, 0x1 << 21);

	return enable_fec_anatop_clock(0, ENET_50MHZ);
}

int board_eth_init(bd_t *bis)
{
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;

	setup_fec_clock();

	eth_phy_reset();

	bus = fec_get_miibus(base, -1);
	if (!bus)
		return -EINVAL;

	/* Scan PHY 0 */
	phydev = phy_find_by_mask(bus, 0xf, PHY_INTERFACE_MODE_RGMII);
	if (!phydev) {
		printf("Ethernet PHY not found!\n");
		return -EINVAL;
	}

	return fec_probe(bis, -1, base, bus, phydev);
}
#endif

#ifdef CONFIG_FSL_ESDHC

#define USDHC2_CD_GPIO	IMX_GPIO_NR(6, 16)
#define USDHC3_CD_GPIO	IMX_GPIO_NR(7, 8)

static struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{ USDHC2_BASE_ADDR },
	{ USDHC3_BASE_ADDR },
	{ USDHC4_BASE_ADDR },
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		return gpio_get_value(USDHC2_CD_GPIO);
	case USDHC3_BASE_ADDR:
		return !gpio_get_value(USDHC3_CD_GPIO);
	case USDHC4_BASE_ADDR:
		return 1; /* eMMC/uSDHC4 is always present */
	}

	return 0;
}

int board_mmc_init(bd_t *bis)
{
	int i, ret;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    SD interface
	 * mmc1                    micro SD
	 * mmc2                    eMMC
	 */
	gpio_direction_input(USDHC2_CD_GPIO);
	gpio_direction_input(USDHC3_CD_GPIO);

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	usdhc_cfg[2].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
}
#endif

#ifdef CONFIG_USB_EHCI_MX6
static void setup_usb(void)
{
	/*
	 * Set daisy chain for otg_pin_id on MX6Q.
	 * For MX6DL, this bit is reserved.
	 */
	imx_iomux_set_gpr_register(1, 13, 1, 0);
}

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return USB_INIT_DEVICE;
}

int board_ehci_power(int port, int on)
{
	switch (port) {
	case 0:
		break;
	case 1:
		gpio_direction_output(IMX_GPIO_NR(3, 31), !!on);
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}
#endif

static int setup_dhcom_mac_from_fuse(void)
{
	unsigned char enetaddr[6];
	int ret;

	ret = eth_env_get_enetaddr("ethaddr", enetaddr);
	if (ret)	/* ethaddr is already set */
		return 0;

	imx_get_mac_from_fuse(0, enetaddr);

	if (is_valid_ethaddr(enetaddr)) {
		eth_env_set_enetaddr("ethaddr", enetaddr);
		env_save();
		return 0;
	}

	ret = i2c_set_bus_num(2);
	if (ret) {
		printf("Error switching I2C bus!\n");
		return ret;
	}

	ret = i2c_read(EEPROM_I2C_ADDRESS, 0xfa, 0x1, enetaddr, 0x6);
	if (ret) {
		printf("Error reading configuration EEPROM!\n");
		return ret;
	}

	if (is_valid_ethaddr(enetaddr)) {
		eth_env_set_enetaddr("ethaddr", enetaddr);
		env_save();
	}

	return 0;
}

int board_early_init_f(void)
{
#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif

	return 0;
}

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	if (bus == 0 && cs == 0)
		return IMX_GPIO_NR(2, 30);
	else
		return -1;
}
#endif

int board_init(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	/* Enable eim_slow clocks */
	setbits_le32(&mxc_ccm->CCGR6, 0x1 << MXC_CCM_CCGR6_EMI_SLOW_OFFSET);

#ifdef CONFIG_SYS_I2C_MXC
	if (is_mx6dq()) {
		setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &dh6dq_i2c_pad_info0);
		setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &dh6dq_i2c_pad_info1);
		setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &dh6dq_i2c_pad_info2);
	} else {
		setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &dh6sdl_i2c_pad_info0);
		setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &dh6sdl_i2c_pad_info1);
		setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &dh6sdl_i2c_pad_info2);
	}
#endif

#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif
#ifdef CONFIG_SATA
	setup_sata();
#endif
	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd2",	 MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

#define HW_CODE_BIT_0	IMX_GPIO_NR(2, 19)
#define HW_CODE_BIT_1	IMX_GPIO_NR(6, 6)
#define HW_CODE_BIT_2	IMX_GPIO_NR(2, 16)

int board_get_hwcode(void)
{
	int hw_code;

	gpio_direction_input(HW_CODE_BIT_0);
	gpio_direction_input(HW_CODE_BIT_1);
	gpio_direction_input(HW_CODE_BIT_2);

	/* HW 100 + HW 200 = 00b; HW 300 = 01b */
	hw_code = ((gpio_get_value(HW_CODE_BIT_2) << 2) |
		   (gpio_get_value(HW_CODE_BIT_1) << 1) |
		    gpio_get_value(HW_CODE_BIT_0)) + 2;

	return hw_code;
}

int board_late_init(void)
{
	u32 hw_code;
	char buf[16];

	setup_dhcom_mac_from_fuse();

	hw_code = board_get_hwcode();

	switch (get_cpu_type()) {
	case MXC_CPU_MX6SOLO:
		snprintf(buf, sizeof(buf), "imx6s-dhcom%1d", hw_code);
		break;
	case MXC_CPU_MX6DL:
		snprintf(buf, sizeof(buf), "imx6dl-dhcom%1d", hw_code);
		break;
	case MXC_CPU_MX6D:
		snprintf(buf, sizeof(buf), "imx6d-dhcom%1d", hw_code);
		break;
	case MXC_CPU_MX6Q:
		snprintf(buf, sizeof(buf), "imx6q-dhcom%1d", hw_code);
		break;
	default:
		snprintf(buf, sizeof(buf), "UNKNOWN%1d", hw_code);
		break;
	}

	env_set("dhcom", buf);

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	return 0;
}

int checkboard(void)
{
	puts("Board: DHCOM i.MX6\n");
	return 0;
}
