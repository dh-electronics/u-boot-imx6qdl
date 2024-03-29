// SPDX-License-Identifier: GPL-2.0+
/*
 * DHCOM DH-iMX6 PDK SPL support
 *
 * Copyright (C) 2017 Marek Vasut <marex@denx.de>
 */

#include <common.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-ddr.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/io.h>
#include <errno.h>
#include <fuse.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <mmc.h>
#include <spl.h>

#define ENET_PAD_CTRL							\
	(PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |	\
	 PAD_CTL_HYS)

#define GPIO_PAD_CTRL							\
	(PAD_CTL_HYS | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define SPI_PAD_CTRL							\
	(PAD_CTL_HYS | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |		\
	PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL							\
	(PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |	\
	 PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL							\
	(PAD_CTL_PUS_47K_UP | PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |	\
	 PAD_CTL_SRE_FAST | PAD_CTL_HYS)

static const struct mx6dq_iomux_ddr_regs dhcom6dq_ddr_ioregs = {
	.dram_sdclk_0	= 0x00020030,
	.dram_sdclk_1	= 0x00020030,
	.dram_cas	= 0x00020030,
	.dram_ras	= 0x00020030,
	.dram_reset	= 0x00020030,
	.dram_sdcke0	= 0x00003000,
	.dram_sdcke1	= 0x00003000,
	.dram_sdba2	= 0x00000000,
	.dram_sdodt0	= 0x00003030,
	.dram_sdodt1	= 0x00003030,
	.dram_sdqs0	= 0x00000030,
	.dram_sdqs1	= 0x00000030,
	.dram_sdqs2	= 0x00000030,
	.dram_sdqs3	= 0x00000030,
	.dram_sdqs4	= 0x00000030,
	.dram_sdqs5	= 0x00000030,
	.dram_sdqs6	= 0x00000030,
	.dram_sdqs7	= 0x00000030,
	.dram_dqm0	= 0x00020030,
	.dram_dqm1	= 0x00020030,
	.dram_dqm2	= 0x00020030,
	.dram_dqm3	= 0x00020030,
	.dram_dqm4	= 0x00020030,
	.dram_dqm5	= 0x00020030,
	.dram_dqm6	= 0x00020030,
	.dram_dqm7	= 0x00020030,
};

static const struct mx6dq_iomux_grp_regs dhcom6dq_grp_ioregs = {
	.grp_ddr_type	= 0x000C0000,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_ddrpke	= 0x00000000,
	.grp_addds	= 0x00000030,
	.grp_ctlds	= 0x00000030,
	.grp_ddrmode	= 0x00020000,
	.grp_b0ds	= 0x00000030,
	.grp_b1ds	= 0x00000030,
	.grp_b2ds	= 0x00000030,
	.grp_b3ds	= 0x00000030,
	.grp_b4ds	= 0x00000030,
	.grp_b5ds	= 0x00000030,
	.grp_b6ds	= 0x00000030,
	.grp_b7ds	= 0x00000030,
};

static const struct mx6sdl_iomux_ddr_regs dhcom6sdl_ddr_ioregs = {
	.dram_sdclk_0	= 0x00020030,
	.dram_sdclk_1	= 0x00020030,
	.dram_cas	= 0x00020030,
	.dram_ras	= 0x00020030,
	.dram_reset	= 0x00020030,
	.dram_sdcke0	= 0x00003000,
	.dram_sdcke1	= 0x00003000,
	.dram_sdba2	= 0x00000000,
	.dram_sdodt0	= 0x00003030,
	.dram_sdodt1	= 0x00003030,
	.dram_sdqs0	= 0x00000030,
	.dram_sdqs1	= 0x00000030,
	.dram_sdqs2	= 0x00000030,
	.dram_sdqs3	= 0x00000030,
	.dram_sdqs4	= 0x00000030,
	.dram_sdqs5	= 0x00000030,
	.dram_sdqs6	= 0x00000030,
	.dram_sdqs7	= 0x00000030,
	.dram_dqm0	= 0x00020030,
	.dram_dqm1	= 0x00020030,
	.dram_dqm2	= 0x00020030,
	.dram_dqm3	= 0x00020030,
	.dram_dqm4	= 0x00020030,
	.dram_dqm5	= 0x00020030,
	.dram_dqm6	= 0x00020030,
	.dram_dqm7	= 0x00020030,
};

static const struct mx6sdl_iomux_grp_regs dhcom6sdl_grp_ioregs = {
	.grp_ddr_type	= 0x000C0000,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_ddrpke	= 0x00000000,
	.grp_addds	= 0x00000030,
	.grp_ctlds	= 0x00000030,
	.grp_ddrmode	= 0x00020000,
	.grp_b0ds	= 0x00000030,
	.grp_b1ds	= 0x00000030,
	.grp_b2ds	= 0x00000030,
	.grp_b3ds	= 0x00000030,
	.grp_b4ds	= 0x00000030,
	.grp_b5ds	= 0x00000030,
	.grp_b6ds	= 0x00000030,
	.grp_b7ds	= 0x00000030,
};

static const struct mx6_mmdc_calibration mmdc_calib_4x4G_1066 = {
	.p0_mpwldectrl0	= 0x00150019,
	.p0_mpwldectrl1	= 0x001C000B,
	.p1_mpwldectrl0	= 0x00020018,
	.p1_mpwldectrl1	= 0x0002000C,
	.p0_mpdgctrl0	= 0x43140320,
	.p0_mpdgctrl1	= 0x03080304,
	.p1_mpdgctrl0	= 0x43180320,
	.p1_mpdgctrl1	= 0x03100254,
	.p0_mprddlctl	= 0x4830383C,
	.p1_mprddlctl	= 0x3836323E,
	.p0_mpwrdlctl	= 0x3E444642,
	.p1_mpwrdlctl	= 0x42344442,
};

static const struct mx6_mmdc_calibration mmdc_calib_2x4G_800 = {
	.p0_mpwldectrl0	= 0x0040003C,
	.p0_mpwldectrl1	= 0x0032003E,
	.p0_mpdgctrl0	= 0x42350231,
	.p0_mpdgctrl1	= 0x021A0218,
	.p0_mprddlctl	= 0x4B4B4E49,
	.p0_mpwrdlctl	= 0x3F3F3035,
};

static const struct mx6_mmdc_calibration mmdc_calib_4x2G_1066 = {
	.p0_mpwldectrl0	= 0x001a001a,
	.p0_mpwldectrl1	= 0x00260015,
	.p0_mpdgctrl0	= 0x030c0320,
	.p0_mpdgctrl1	= 0x03100304,
	.p0_mprddlctl	= 0x432e3538,
	.p0_mpwrdlctl	= 0x363f423d,
	.p1_mpwldectrl0	= 0x0006001e,
	.p1_mpwldectrl1	= 0x00050015,
	.p1_mpdgctrl0	= 0x031c0324,
	.p1_mpdgctrl1	= 0x030c0258,
	.p1_mprddlctl	= 0x3834313f,
	.p1_mpwrdlctl	= 0x47374a42,
};

static const struct mx6_mmdc_calibration mmdc_calib_4x2G_800 = {
	.p0_mpwldectrl0	= 0x003A003A,
	.p0_mpwldectrl1	= 0x0030002F,
	.p1_mpwldectrl0	= 0x002F0038,
	.p1_mpwldectrl1	= 0x00270039,
	.p0_mpdgctrl0	= 0x420F020F,
	.p0_mpdgctrl1	= 0x01760175,
	.p1_mpdgctrl0	= 0x41640171,
	.p1_mpdgctrl1	= 0x015E0160,
	.p0_mprddlctl	= 0x45464B4A,
	.p1_mprddlctl	= 0x49484A46,
	.p0_mpwrdlctl	= 0x40402E32,
	.p1_mpwrdlctl	= 0x3A3A3231,
};

static const struct mx6_mmdc_calibration mmdc_calib_2x2G_800 = {
	.p0_mpwldectrl0	= 0x0040003C,
	.p0_mpwldectrl1	= 0x0032003E,
	.p0_mpdgctrl0	= 0x42350231,
	.p0_mpdgctrl1	= 0x021A0218,
	.p0_mprddlctl	= 0x4B4B4E49,
	.p0_mpwrdlctl	= 0x3F3F3035,
};

/*
 * 2 Gbit DDR3 memory
 *   - NANYA #NT5CB128M16JR-EKI
 *   - NANYA #NT5CC128M16IP-DII
 *   - NANYA #NT5CB128M16FP-DII
 */
static const struct mx6_ddr3_cfg mem_ddr_2G = {
	.mem_speed	= 1600,
	.density	= 2,
	.width		= 16,
	.banks		= 8,
	.rowaddr	= 14,
	.coladdr	= 10,
	.pagesz		= 2,
	.trcd		= 1375,
	.trcmin		= 4875,
	.trasmin	= 3500,
};

/*
 * 4 Gbit DDR3 memory
 *   - Intelligent Memory #IM4G16D3EABG-125I
 */
static const struct mx6_ddr3_cfg mem_ddr_4G = {
	.mem_speed	= 1600,
	.density	= 4,
	.width		= 16,
	.banks		= 8,
	.rowaddr	= 15,
	.coladdr	= 10,
	.pagesz		= 2,
	.trcd		= 1375,
	.trcmin		= 4875,
	.trasmin	= 3500,
};

/* DDR3 64bit */
static const struct mx6_ddr_sysinfo dhcom_ddr_64bit = {
	/* width of data bus:0=16,1=32,2=64 */
	.dsize		= 2,
	.cs_density	= 32,
	.ncs		= 1,	/* single chip select */
	.cs1_mirror	= 1,
	.rtt_wr		= 1,	/* DDR3_RTT_60_OHM, RTT_Wr = RZQ/4 */
	.rtt_nom	= 1,	/* DDR3_RTT_60_OHM, RTT_Nom = RZQ/4 */
	.walat		= 1,	/* Write additional latency */
	.ralat		= 5,	/* Read additional latency */
	.mif3_mode	= 3,	/* Command prediction working mode */
	.bi_on		= 1,	/* Bank interleaving enabled */
	.sde_to_rst	= 0x10,	/* 14 cycles, 200us (JEDEC default) */
	.rst_to_cke	= 0x23,	/* 33 cycles, 500us (JEDEC default) */
	.refsel		= 1,	/* Refresh cycles at 32KHz */
	.refr		= 7,	/* 8 refresh commands per refresh cycle */
};

/* DDR3 32bit */
static const struct mx6_ddr_sysinfo dhcom_ddr_32bit = {
	/* width of data bus:0=16,1=32,2=64 */
	.dsize		= 1,
	.cs_density	= 32,
	.ncs		= 1,	/* single chip select */
	.cs1_mirror	= 1,
	.rtt_wr		= 1,	/* DDR3_RTT_60_OHM, RTT_Wr = RZQ/4 */
	.rtt_nom	= 1,	/* DDR3_RTT_60_OHM, RTT_Nom = RZQ/4 */
	.walat		= 1,	/* Write additional latency */
	.ralat		= 5,	/* Read additional latency */
	.mif3_mode	= 3,	/* Command prediction working mode */
	.bi_on		= 1,	/* Bank interleaving enabled */
	.sde_to_rst	= 0x10,	/* 14 cycles, 200us (JEDEC default) */
	.rst_to_cke	= 0x23,	/* 33 cycles, 500us (JEDEC default) */
	.refsel		= 1,	/* Refresh cycles at 32KHz */
	.refr		= 7,	/* 8 refresh commands per refresh cycle */
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

/* Board ID */
static iomux_v3_cfg_t const hwcode_pads[] = {
	IOMUX_PADS(PAD_EIM_A19__GPIO2_IO19	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A23__GPIO6_IO06	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A22__GPIO2_IO16	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
};

static void setup_iomux_boardid(void)
{
	/* HW code pins: Setup alternate function and configure pads */
	SETUP_IOMUX_PADS(hwcode_pads);
}

/* DDR Code */
static iomux_v3_cfg_t const ddrcode_pads[] = {
	IOMUX_PADS(PAD_EIM_A16__GPIO2_IO22	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A17__GPIO2_IO21	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
};

static void setup_iomux_ddrcode(void)
{
	/* ddr code pins */
	SETUP_IOMUX_PADS(ddrcode_pads);
}

#define DDR3_CODE_BIT_0   IMX_GPIO_NR(2, 22)
#define DDR3_CODE_BIT_1   IMX_GPIO_NR(2, 21)

int get_ddr3_size(void)
{
	int size;

	gpio_direction_input(DDR3_CODE_BIT_0);
	gpio_direction_input(DDR3_CODE_BIT_1);

	/* 256MB = 00b; 512MB = 01b; 1GB = 10b; 2GB = 11b */
	size = (gpio_get_value(DDR3_CODE_BIT_1) << 1)
	     | (gpio_get_value(DDR3_CODE_BIT_0));
	return 256 << size;
}

/* Unused uart pins */
static iomux_v3_cfg_t const uart1_rtscts_pads[] = {
	IOMUX_PADS(PAD_EIM_D20__GPIO3_IO20	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D19__GPIO3_IO19	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
};

static iomux_v3_cfg_t const uart2_pads[] = {
	IOMUX_PADS(PAD_CSI0_DAT14__GPIO6_IO00	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT15__GPIO6_IO01	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT18__GPIO6_IO04	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT19__GPIO6_IO05	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
};

static void setup_iomux_unused_uart_pins_as_gpio_input(void)
{
	/*
	 * For now uart1 rts/cts is muxed as gpios input, because if used as
	 * rs485 it would not enable Tx to the bus and therefore not block it.
	 */
	SETUP_IOMUX_PADS(uart1_rtscts_pads);
	gpio_direction_input(IMX_GPIO_NR(3, 20));
	gpio_direction_input(IMX_GPIO_NR(3, 19));

	/* All uart2 pins are muxed as gpio input to avoid blocking if used as rs485 */
	SETUP_IOMUX_PADS(uart2_pads);
	gpio_direction_input(IMX_GPIO_NR(6, 0));
	gpio_direction_input(IMX_GPIO_NR(6, 1));
	gpio_direction_input(IMX_GPIO_NR(6, 4));
	gpio_direction_input(IMX_GPIO_NR(6, 5));
}

/* GPIO */
static iomux_v3_cfg_t const gpio_pads[] = {
	IOMUX_PADS(PAD_GPIO_2__GPIO1_IO02	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_4__GPIO1_IO04	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_5__GPIO1_IO05	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT17__GPIO6_IO03	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_19__GPIO4_IO05	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN4__GPIO4_IO20	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D27__GPIO3_IO27	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW0__GPIO4_IO07	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_COL1__GPIO4_IO08	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_CS1__GPIO6_IO14	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_CS2__GPIO6_IO15	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW1__GPIO4_IO09	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT5__GPIO7_IO00	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT4__GPIO7_IO01	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_VSYNC__GPIO5_IO21	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_18__GPIO7_IO13	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_SD1_CMD__GPIO1_IO18	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_SD1_DAT0__GPIO1_IO16	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_SD1_DAT1__GPIO1_IO17	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_SD1_DAT2__GPIO1_IO19	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_SD1_CLK__GPIO1_IO20	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_PIXCLK__GPIO5_IO18	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_MCLK__GPIO5_IO19	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
};

static void setup_iomux_gpio(void)
{
	SETUP_IOMUX_PADS(gpio_pads);
}

/* PWM (init in gpio mode) */
static iomux_v3_cfg_t const pwm_pads[] = {
	IOMUX_PADS(PAD_SD1_DAT3__GPIO1_IO21	| MUX_PAD_CTRL(GPIO_PAD_CTRL)),
};

static void setup_iomux_pwm(void)
{
	SETUP_IOMUX_PADS(pwm_pads);
}

/* Ethernet */
static iomux_v3_cfg_t const enet_pads[] = {
	IOMUX_PADS(PAD_ENET_MDIO__ENET_MDIO	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_MDC__ENET_MDC	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_TX_EN__ENET_TX_EN	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_TXD0__ENET_TX_DATA0	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_TXD1__ENET_TX_DATA1	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_16__ENET_REF_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_RX_ER__ENET_RX_ER	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_RXD0__ENET_RX_DATA0	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_RXD1__ENET_RX_DATA1	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_CRS_DV__ENET_RX_EN	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	/* SMSC PHY Reset */
	IOMUX_PADS(PAD_EIM_WAIT__GPIO5_IO00	| MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* ENET_VIO_GPIO */
	IOMUX_PADS(PAD_GPIO_7__GPIO1_IO07	| MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* ENET_Interrupt - (not used) */
	IOMUX_PADS(PAD_RGMII_RD0__GPIO6_IO25	| MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void setup_iomux_enet(void)
{
	SETUP_IOMUX_PADS(enet_pads);
}

/* SD interface */
static iomux_v3_cfg_t const usdhc2_pads[] = {
	IOMUX_PADS(PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_CLK__SD2_CLK		| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_CMD__SD2_CMD		| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_CS3__GPIO6_IO16	| MUX_PAD_CTRL(NO_PAD_CTRL)), /* CD */
};

/* onboard microSD */
static iomux_v3_cfg_t const usdhc3_pads[] = {
	IOMUX_PADS(PAD_SD3_DAT0__SD3_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT1__SD3_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT2__SD3_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT3__SD3_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_CLK__SD3_CLK		| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_CMD__SD3_CMD		| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_RST__GPIO7_IO08	| MUX_PAD_CTRL(NO_PAD_CTRL)), /* CD */
};

/* eMMC */
static iomux_v3_cfg_t const usdhc4_pads[] = {
	IOMUX_PADS(PAD_SD4_DAT0__SD4_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT1__SD4_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT2__SD4_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT3__SD4_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT4__SD4_DATA4	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT5__SD4_DATA5	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT6__SD4_DATA6	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT7__SD4_DATA7	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_CLK__SD4_CLK		| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_CMD__SD4_CMD		| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

/* SD */
static void setup_iomux_sd(void)
{
	SETUP_IOMUX_PADS(usdhc2_pads);
	SETUP_IOMUX_PADS(usdhc3_pads);
	SETUP_IOMUX_PADS(usdhc4_pads);
}

/* SPI */
static iomux_v3_cfg_t const ecspi1_pads[] = {
	/* SS0 - SS of boot flash */
	IOMUX_PADS(PAD_EIM_EB2__GPIO2_IO30	|
		MUX_PAD_CTRL(SPI_PAD_CTRL | PAD_CTL_PUS_100K_UP)),
	/* SS2 - SS of DHCOM SPI1 */
	IOMUX_PADS(PAD_KEY_ROW2__GPIO4_IO11	|
		MUX_PAD_CTRL(SPI_PAD_CTRL | PAD_CTL_PUS_100K_UP)),

	IOMUX_PADS(PAD_EIM_D17__ECSPI1_MISO	| MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D18__ECSPI1_MOSI	| MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D16__ECSPI1_SCLK	| MUX_PAD_CTRL(SPI_PAD_CTRL)),
};

#define DHCOM_SPI1_CS	IMX_GPIO_NR(4, 11)

static void setup_iomux_spi(void)
{
	SETUP_IOMUX_PADS(ecspi1_pads);

	/*
	 * disable devices on DHCOM SPI1 (CS is active low)
	 * - prevent problems with bootflash access.
	 */
	gpio_direction_output(DHCOM_SPI1_CS, 1);
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	if (bus == 0 && cs == 0)
		return IMX_GPIO_NR(2, 30);
	else
		return -1;
}

/* UART */
static iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_SD3_DAT7__UART1_TX_DATA	| MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT6__UART1_RX_DATA	| MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart1_pads);
}

/* USB */
static iomux_v3_cfg_t const usb_pads[] = {
	IOMUX_PADS(PAD_GPIO_1__USB_OTG_ID	| MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D31__GPIO3_IO31	| MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void setup_iomux_usb(void)
{
	SETUP_IOMUX_PADS(usb_pads);
}

/* Perform DDR DRAM calibration */
static int spl_dram_perform_cal(struct mx6_ddr_sysinfo const *sysinfo)
{
	int ret = 0;

#ifdef CONFIG_MX6_DDRCAL
	udelay(100);
	ret = mmdc_do_write_level_calibration(sysinfo);
	if (ret) {
		printf("DDR3: Write level calibration error [%d]\n", ret);
		return ret;
	}

	ret = mmdc_do_dqs_calibration(sysinfo);
	if (ret) {
		printf("DDR3: DQS calibration error [%d]\n", ret);
		return ret;
	}
#endif /* CONFIG_MX6_DDRCAL */

	return ret;
}


/* DRAM */
static void spl_dram_init(void)
{
	int ddr3_mb = get_ddr3_size();

	if (is_mx6dq()) {
		mx6dq_dram_iocfg(64, &dhcom6dq_ddr_ioregs,
					&dhcom6dq_grp_ioregs);
		switch (ddr3_mb) {
		default:
			printf("imx6qd: unsupported ddr3 size %dMB\n", ddr3_mb);
			printf("        choosing 1024 MB\n");
			/* fall through */
		case 1024:
			mx6_dram_cfg(&dhcom_ddr_64bit, &mmdc_calib_4x2G_1066,
				     &mem_ddr_2G);
			break;
		case 2048:
			mx6_dram_cfg(&dhcom_ddr_64bit, &mmdc_calib_4x4G_1066,
				     &mem_ddr_4G);
			break;
		}

		/* Perform DDR DRAM calibration */
		spl_dram_perform_cal(&dhcom_ddr_64bit);

	} else if (is_cpu_type(MXC_CPU_MX6DL)) {
		mx6sdl_dram_iocfg(64, &dhcom6sdl_ddr_ioregs,
					  &dhcom6sdl_grp_ioregs);
		switch (ddr3_mb) {
		default:
			printf("imx6dl: unsupported ddr3 size %dMB\n", ddr3_mb);
			printf("        choosing 1024 MB\n");
			/* fall through */
		case 1024:
			mx6_dram_cfg(&dhcom_ddr_64bit, &mmdc_calib_4x2G_800,
				     &mem_ddr_2G);
			break;
		}

		/* Perform DDR DRAM calibration */
		spl_dram_perform_cal(&dhcom_ddr_64bit);

	} else if (is_cpu_type(MXC_CPU_MX6SOLO)) {
		mx6sdl_dram_iocfg(32, &dhcom6sdl_ddr_ioregs,
					  &dhcom6sdl_grp_ioregs);
		switch (ddr3_mb) {
		default:
			printf("imx6s: unsupported ddr3 size %dMB\n", ddr3_mb);
			printf("       choosing 512 MB\n");
			/* fall through */
		case 512:
			mx6_dram_cfg(&dhcom_ddr_32bit, &mmdc_calib_2x2G_800,
				     &mem_ddr_2G);
			break;
		case 1024:
			mx6_dram_cfg(&dhcom_ddr_32bit, &mmdc_calib_2x4G_800,
				     &mem_ddr_4G);
			break;
		}

		/* Perform DDR DRAM calibration */
		spl_dram_perform_cal(&dhcom_ddr_32bit);
	}
}

#define DH_BOOT_DEVICE_SPI	BOOT_DEVICE_SPI
#define DH_BOOT_DEVICE_SD	BOOT_DEVICE_MMC1   /* mmc0 */
#define DH_BOOT_DEVICE_USD	BOOT_DEVICE_MMC2   /* mmc1 */
#define DH_BOOT_DEVICE_EMMC	BOOT_DEVICE_MMC2_2 /* mmc2 */
int spl_mmc_get_device_index(u32 boot_device)
{
	switch (boot_device) {
	case DH_BOOT_DEVICE_SD:
		return 0;
	case DH_BOOT_DEVICE_USD:
		return 1;
	case DH_BOOT_DEVICE_EMMC:
		return 2;
	}

#ifdef CONFIG_SPL_LIBCOMMON_SUPPORT
	printf("spl: unsupported mmc boot device.\n");
#endif

	return -ENODEV;
}

void board_boot_order(u32 *spl_boot_list)
{
	u32 dir_bt_dis = (readl(&src_base->sbmr2) >> 3) & 0x01;
	u32 gpr9 = readl(&src_base->gpr9);
	u32 sbmr1 = readl(&src_base->sbmr1);
	u32 reg, bmode, bt_fuse_sel;
	u8 boot_dev;
	u32 index, cs;
	char bootdev_str[][8] = { "MMC1", "MMC2", "MMC2_2", "NAND", "ONENAND",
				 "NOR", "UART", "SPI", "USB", "SATA", "I2C",
				 "BOARD", "DFU", "XIP", "BOOTROM", "NONE" };

	/*
	 * Dealing with bmode command:
	 * The bit location IMX6_SRC_GPR10_BMODE in SRC_GPR10 determines where
	 * BOOT_CFGX come from (only if it isn't disabled by DIR_BT_DIS fuse):
	 *   0 = SBMR1:    normal boot
	 *   1 = SRC_GPR9: boot values by bmode command
	 * This is requested by function imx6_is_bmode_from_gpr9().
	 * DIR_BT_DIS fuse prevents rom code from fetching SPL from bmode source.
	 * So if active reset IMX6_SRC_GPR10_BMODE in SRC_GPR10 to prevent the
	 * next bootloader stage to boot from bmode source.
	 */
	if (dir_bt_dis) {
		reg = readl(&src_base->gpr10);
		reg &= ~IMX6_SRC_GPR10_BMODE;
		writel(reg, &src_base->gpr10);
	}

	printf("BOOT:");
	if (imx6_is_bmode_from_gpr9() && !dir_bt_dis) {
		printf(" Using SRC_GPR9 (0x%08X)", gpr9);
	} else {
		fuse_read(0, 6, &reg);
		/*
		 * Shows reg 0x460 (bank 0 word 6) if other bits besides
		 * DIR_BT_DIS (bit 3), BT_FUSE_SEL (bit 4) and
		 * FORCE_INTERNAL_BOOT (bit 16) are active
		 */
		if (reg & ~0x10018)
			printf(" [0x460]=0x%08X,", reg);
		/* Showing DIR_BT_DIS to indicate that bmode command is ignored */
		if (dir_bt_dis)
			printf(" DIR_BT_DIS=1,");
		/* FORCE_INTERNAL_BOOT (bit 16) */
		if (reg & 0x10000) {
			printf(" FORCE_INTERNAL_BOOT=1 => force boot from fuses");
		/* Mode + BT_FUSE_SEL */
		} else {
			bmode = (readl(&src_base->sbmr2) >> 24) & 0x03;
			bt_fuse_sel = (readl(&src_base->sbmr2) >> 4) & 0x01;
			printf(" mode=%d + BT_FUSE_SEL=%d", bmode, bt_fuse_sel);
			if (bt_fuse_sel) {
				if (bmode == 0x1)
					printf(" => serial downloader");
				else
					printf(" => force boot from fuses");
			} else {
				if (bmode == 0x2)
					printf(" => boot from gpios");
				else
					printf(" => serial downloader");
			}
		}
		/*
		 * Infer that the boot ROM used the USB serial downloader by
		 * checking whether the USB PHY is currently active... This
		 * assumes that SPL did not (yet) initialize the USB PHY...
		 */
		if (is_usbotg_phy_active())
			printf(" + USB OTG phy active");
	}

	/*
	 * Boot order rules:
	 * - Default order = SPI, eMMC, uSD, SD
	 * - Selected device has priority
	 * - External devices doesn't need boot order
	 */
	boot_dev = spl_boot_device();
#ifdef CONFIG_NAND_MXS /* eMMC isn't available */
	switch (boot_dev) {
	case BOOT_DEVICE_SPI: /* SERIAL_ROM SPI */
		reg = imx6_src_get_boot_mode();
		index = ((reg & IMX6_BMODE_SERIAL_ROM_MASK) >> IMX6_BMODE_SERIAL_ROM_SHIFT) + 1;
		cs = ((reg & IMX6_BMODE_SERIAL_ROM_CS_MASK) >> IMX6_BMODE_SERIAL_ROM_CS_SHIFT);
		printf(", device=ECSPI%d:SS%d (set order: SPI, uSD, SD)\n", index, cs);
		spl_boot_list[0] = DH_BOOT_DEVICE_SPI;
		spl_boot_list[1] = DH_BOOT_DEVICE_USD;
		spl_boot_list[2] = DH_BOOT_DEVICE_SD;
		break;
	case BOOT_DEVICE_MMC1: /* SD, eSD, MMC, eMMC */
		reg = imx6_src_get_boot_mode();
		index = ((reg & IMX6_BMODE_USDHC_MASK) >> IMX6_BMODE_USDHC_SHIFT) + 1;
		printf(", device=uSDHC%d", index);
		switch (index) {
		case 2: /* uSDHC2 */
			printf("\n");
			spl_boot_list[0] = DH_BOOT_DEVICE_SD;
			break;
		case 3: /* uSDHC3 */
			printf("\n");
			spl_boot_list[0] = DH_BOOT_DEVICE_USD;
			break;
		default:
			printf(" (unsupported => set order: SPI, uSD, SD)\n");
			spl_boot_list[0] = DH_BOOT_DEVICE_SPI;
			spl_boot_list[1] = DH_BOOT_DEVICE_USD;
			spl_boot_list[2] = DH_BOOT_DEVICE_SD;
			break;
		}
		break;
	case BOOT_DEVICE_BOARD: /* USB SDP */
		printf(", device=USB SDP\n");
		spl_boot_list[0] = boot_dev;
		break;
	default:
		printf(", device=unsupported\n");
		fuse_read(0, 5, &reg);
		printf("      Fuse 0x450 = 0x%08X\n", reg);
		printf("      SRC_GPR9   = 0x%08X\n", gpr9);
		printf("      SBMR1      = 0x%08X\n", sbmr1);
		printf("      BOOT_DEVICE_%s\n", bootdev_str[boot_dev]);
		spl_boot_list[0] = boot_dev;
		spl_boot_list[1] = DH_BOOT_DEVICE_SPI;
		spl_boot_list[2] = DH_BOOT_DEVICE_USD;
		spl_boot_list[3] = DH_BOOT_DEVICE_SD;
		break;
	}
#else /* CONFIG_NAND_MXS */
#ifdef CONFIG_FSL_ESDHC
	switch (boot_dev) {
	case BOOT_DEVICE_SPI: /* SERIAL_ROM SPI */
		reg = imx6_src_get_boot_mode();
		index = ((reg & IMX6_BMODE_SERIAL_ROM_MASK) >> IMX6_BMODE_SERIAL_ROM_SHIFT) + 1;
		cs = ((reg & IMX6_BMODE_SERIAL_ROM_CS_MASK) >> IMX6_BMODE_SERIAL_ROM_CS_SHIFT);
		printf(", device=ECSPI%d:SS%d (set order: SPI, eMMC, uSD, SD)\n", index, cs);
		spl_boot_list[0] = DH_BOOT_DEVICE_SPI;
		spl_boot_list[1] = DH_BOOT_DEVICE_EMMC;
		spl_boot_list[2] = DH_BOOT_DEVICE_USD;
		spl_boot_list[3] = DH_BOOT_DEVICE_SD;
		break;
	case BOOT_DEVICE_MMC1: /* SD, eSD, MMC, eMMC */
		reg = imx6_src_get_boot_mode();
		index = ((reg & IMX6_BMODE_USDHC_MASK) >> IMX6_BMODE_USDHC_SHIFT) + 1;
		printf(", device=uSDHC%d", index);
		switch (index) {
		case 2: /* uSDHC2 */
			printf("\n");
			spl_boot_list[0] = DH_BOOT_DEVICE_SD;
			break;
		case 3: /* uSDHC3 */
			printf("\n");
			spl_boot_list[0] = DH_BOOT_DEVICE_USD;
			break;
		case 4: /* uSDHC4 */
			printf(" (set order: eMMC, SPI, uSD, SD)\n");
			spl_boot_list[0] = DH_BOOT_DEVICE_EMMC;
			spl_boot_list[1] = DH_BOOT_DEVICE_SPI;
			spl_boot_list[2] = DH_BOOT_DEVICE_USD;
			spl_boot_list[3] = DH_BOOT_DEVICE_SD;
			break;
		default:
			printf(" (unsupported => set order: SPI, eMMC, uSD, SD)\n");
			spl_boot_list[0] = DH_BOOT_DEVICE_SPI;
			spl_boot_list[1] = DH_BOOT_DEVICE_EMMC;
			spl_boot_list[2] = DH_BOOT_DEVICE_USD;
			spl_boot_list[3] = DH_BOOT_DEVICE_SD;
			break;
		}
		break;
	case BOOT_DEVICE_BOARD: /* USB SDP */
		printf(", device=USB SDP\n");
		spl_boot_list[0] = boot_dev;
		break;
	default:
		printf(", device=unsupported\n");
		fuse_read(0, 5, &reg);
		printf("      Fuse 0x450 = 0x%08X\n", reg);
		printf("      SRC_GPR9   = 0x%08X\n", gpr9);
		printf("      SBMR1      = 0x%08X\n", sbmr1);
		printf("      BOOT_DEVICE_%s\n", bootdev_str[boot_dev]);
		spl_boot_list[0] = boot_dev;
		spl_boot_list[1] = DH_BOOT_DEVICE_SPI;
		spl_boot_list[2] = DH_BOOT_DEVICE_EMMC;
		spl_boot_list[3] = DH_BOOT_DEVICE_USD;
		spl_boot_list[4] = DH_BOOT_DEVICE_SD;
		break;
	}
#endif /* CONFIG_FSL_ESDHC */
#endif /* CONFIG_NAND_MXS */

#if defined(DEBUG)
	fuse_read(0, 5, &reg);
	printf("      Fuse 0x450:  CFG1=0x%02X CFG2=0x%02X CFG3=0x%02X CFG4=0x%02X\n",
	       reg & 0xFF, (reg >> 8) & 0xFF, (reg >> 16) & 0xFF, (reg >> 24) & 0xFF );
	reg = gpr9;
	printf("      GPR9:        CFG1=0x%02X CFG2=0x%02X CFG4=0x%02X CFG4=0x%02X\n",
	       reg & 0xFF, (reg >> 8) & 0xFF, (reg >> 16) & 0xFF, (reg >> 24) & 0xFF );
	reg = sbmr1;
	printf("      SBMR1:       CFG1=0x%02X CFG2=0x%02X CFG4=0x%02X CFG4=0x%02X\n",
	       reg & 0xFF, (reg >> 8) & 0xFF, (reg >> 16) & 0xFF, (reg >> 24) & 0xFF );
	fuse_read(0, 6, &reg);
	printf("      Fuse 0x460:  0x%08X\n", reg);
	fuse_read(0, 7, &reg);
	printf("      Fuse 0x470:  0x%08X\n", reg);
	fuse_read(5, 5, &reg);
	printf("      Fuse 0x6D0:  0x%08X\n", reg);
#endif /* DEBUG */
}

void board_init_f(ulong dummy)
{
	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();
	gpr_init();

	setup_iomux_unused_uart_pins_as_gpio_input();

	/* setup GP timer */
	timer_init();

	setup_iomux_boardid();
	setup_iomux_ddrcode();
	setup_iomux_gpio();
	setup_iomux_pwm();
	setup_iomux_enet();
	setup_iomux_sd();
	setup_iomux_spi();
	setup_iomux_uart();
	setup_iomux_usb();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* DDR3 initialization */
	spl_dram_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
