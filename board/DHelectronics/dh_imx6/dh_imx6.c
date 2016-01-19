/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <asm/imx-common/mxc_i2c.h>

DECLARE_GLOBAL_DATA_PTR;

extern int do_mmcops(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
extern int do_mem_cp(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
extern int do_spi_flash(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
extern int fuse_read(u32 bank, u32 word, u32 *val);
extern int fuse_prog(u32 bank, u32 word, u32 val);
extern int do_i2c_bus_num(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)
	
#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm  | PAD_CTL_SRE_FAST)

/* LZ: Set Drive Strength Field to Hi-Z */
/* LZ: Disable Pull/Keeper for DHCOM GPIOs*/
#define GPIO_PAD_CTRL  (PAD_CTL_HYS |			\
	/* PAD_CTL_PUS_100K_UP  | PAD_CTL_PUE | */		\
	/* PAD_CTL_PKE  |*/ PAD_CTL_SPEED_MED |			\
        PAD_CTL_DSE_40ohm)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |	\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |		\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)	
	
#define FB_SYNC_OE_LOW_ACT	0x80000000
#define FB_SYNC_CLK_LAT_FALL	0x40000000
#define FB_SYNC_DATA_INVERT	0x20000000

#define BT_FUSE_SEL		0x00000010
#define SPI_E_FUSE_CONFIG	0x08000030

#define EEPROM_I2C_ADDRESS	0x50

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode  = MX6_PAD_EIM_D21__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_EIM_D21__GPIO_3_21 | PC,
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		 .i2c_mode = MX6_PAD_EIM_D28__I2C1_SDA | PC,
		 .gpio_mode = MX6_PAD_EIM_D28__GPIO_3_28 | PC,
		 .gp = IMX_GPIO_NR(3, 28)
	 }
};

struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode  = MX6_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO_4_12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		 .i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | PC,
		 .gpio_mode = MX6_PAD_KEY_ROW3__GPIO_4_13 | PC,
		 .gp = IMX_GPIO_NR(4, 13)
	 }
};

struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode  = MX6_PAD_GPIO_3__I2C3_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO_3__GPIO_1_3 | PC,
		.gp = IMX_GPIO_NR(1, 3)
	},
	.sda = {
		 .i2c_mode = MX6_PAD_GPIO_6__I2C3_SDA | PC,
		 .gpio_mode = MX6_PAD_GPIO_6__GPIO_1_6 | PC,
		 .gp = IMX_GPIO_NR(1, 6)
	 }
};

struct scu_regs {
	u32	ctrl;
	u32	config;
	u32	status;
	u32	invalidate;
	u32	fpga_rev;
};

unsigned DHCOM_gpios[] = {
	DHCOM_GPIO_A,
	DHCOM_GPIO_B,
	DHCOM_GPIO_C,
	DHCOM_GPIO_D,
	DHCOM_GPIO_E,
	DHCOM_GPIO_F,
	DHCOM_GPIO_G,
	DHCOM_GPIO_H,
	DHCOM_GPIO_I,
	DHCOM_GPIO_J,
	DHCOM_GPIO_K,
	DHCOM_GPIO_L,
	DHCOM_GPIO_M,
	DHCOM_GPIO_N,
	DHCOM_GPIO_O,
	DHCOM_GPIO_P,
	DHCOM_GPIO_Q,
	DHCOM_GPIO_R,
	DHCOM_GPIO_S,
	DHCOM_GPIO_T,
	DHCOM_GPIO_U,
	DHCOM_GPIO_V,
	DHCOM_GPIO_W,
};

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT7__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT6__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TX_EN__ENET_TX_EN	        | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TXD0__ENET_TDATA_0	        | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TXD1__ENET_TDATA_1	        | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_GPIO_16__ENET_ETHERNET_REF_OUT	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RX_ER__ENET_RX_ER	        | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RXD0__ENET_RDATA_0 	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RXD1__ENET_RDATA_1 	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_CRS_DV__ENET_RX_EN	        | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* SMSC PHY Reset */
	MX6_PAD_EIM_WAIT__GPIO_5_0		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* ENET_VIO_GPIO */
	MX6_PAD_GPIO_7__GPIO_1_7                | MUX_PAD_CTRL(NO_PAD_CTRL),
        /* ENET_Interrupt - (not used) */
	MX6_PAD_RGMII_RD0__GPIO_6_25            | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const usb_pads[] = {
	MX6_PAD_EIM_D31__GPIO_3_31		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__USDHC2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__USDHC2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__USDHC2_DAT0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__USDHC2_DAT1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__USDHC2_DAT2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__USDHC2_DAT3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/*MX6_PAD_NANDF_D4__USDHC2_DAT4	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D5__USDHC2_DAT5	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D6__USDHC2_DAT6	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D7__USDHC2_DAT7	| MUX_PAD_CTRL(USDHC_PAD_CTRL),*/
	MX6_PAD_NANDF_CS3__GPIO_6_16	| MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__USDHC3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__USDHC3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__USDHC3_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__USDHC3_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__USDHC3_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__USDHC3_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/* MX6_PAD_SD3_DAT4__USDHC3_DAT4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT5__USDHC3_DAT5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT6__USDHC3_DAT6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT7__USDHC3_DAT7 | MUX_PAD_CTRL(USDHC_PAD_CTRL), */
	MX6_PAD_SD3_RST__GPIO_7_8     | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__USDHC4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__USDHC4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__USDHC4_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__USDHC4_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__USDHC4_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__USDHC4_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__USDHC4_DAT4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__USDHC4_DAT5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__USDHC4_DAT6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__USDHC4_DAT7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

iomux_v3_cfg_t nfc_pads[] = {
	MX6_PAD_NANDF_CLE__RAWNAND_CLE		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_ALE__RAWNAND_ALE		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_WP_B__RAWNAND_RESETN	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_RB0__RAWNAND_READY0	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_CS0__RAWNAND_CE0N		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/*MX6_PAD_NANDF_CS1__RAWNAND_CE1N		| MUX_PAD_CTRL(NO_PAD_CTRL),*/
	/*MX6_PAD_NANDF_CS2__RAWNAND_CE2N		| MUX_PAD_CTRL(NO_PAD_CTRL),*/
	/*MX6_PAD_NANDF_CS3__RAWNAND_CE3N		| MUX_PAD_CTRL(NO_PAD_CTRL),*/
	MX6_PAD_SD4_CMD__RAWNAND_RDN		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SD4_CLK__RAWNAND_WRN		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D0__RAWNAND_D0		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D1__RAWNAND_D1		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D2__RAWNAND_D2		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D3__RAWNAND_D3		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D4__RAWNAND_D4		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D5__RAWNAND_D5		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D6__RAWNAND_D6		| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D7__RAWNAND_D7		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/*MX6_PAD_SD4_DAT0__RAWNAND_DQS		| MUX_PAD_CTRL(NO_PAD_CTRL),*/
};

iomux_v3_cfg_t const gpio_pads[] = {
	MX6_PAD_GPIO_2__GPIO_1_2   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_GPIO_4__GPIO_1_4   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_GPIO_5__GPIO_1_5   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_CSI0_DAT17__GPIO_6_3   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_GPIO_19__GPIO_4_5   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_DI0_PIN4__GPIO_4_20   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_EIM_D27__GPIO_3_27   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_KEY_ROW0__GPIO_4_7   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_KEY_COL1__GPIO_4_8   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_CS1__GPIO_6_14   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_NANDF_CS2__GPIO_6_15   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_KEY_ROW1__GPIO_4_9   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_SD3_DAT5__GPIO_7_0   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_SD3_DAT4__GPIO_7_1   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_CSI0_VSYNC__GPIO_5_21   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_GPIO_18__GPIO_7_13   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_SD1_CMD__GPIO_1_18   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_SD1_DAT0__GPIO_1_16   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_SD1_DAT1__GPIO_1_17   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_SD1_DAT2__GPIO_1_19   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_SD1_CLK__GPIO_1_20   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_CSI0_PIXCLK__GPIO_5_18   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_CSI0_MCLK__GPIO_5_19   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

iomux_v3_cfg_t const hw_code_pads[] = {
	MX6_PAD_EIM_A19__GPIO_2_19   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_EIM_A23__GPIO_6_6   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_EIM_A22__GPIO_2_16   | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

int dram_init(void)
{
	//gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);

	return 0;
}

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
	
	/* Reset PHY */
	gpio_direction_output(IMX_GPIO_NR(5, 0) , 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(5, 0), 1);

    /* Enable VIO */
	gpio_direction_output(IMX_GPIO_NR(1, 7) , 0);
}

#ifdef DH_IMX6_NAND_VERSION
static void setup_gpmi_nand(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	/* config gpmi nand iomux */
	imx_iomux_v3_setup_multiple_pads(nfc_pads,
					 ARRAY_SIZE(nfc_pads));

	/* config gpmi and bch clock to 100 MHz */
	clrsetbits_le32(&mxc_ccm->cs2cdr,
			MXC_CCM_CS2CDR_ENFC_CLK_PODF_MASK |
			MXC_CCM_CS2CDR_ENFC_CLK_PRED_MASK |
			MXC_CCM_CS2CDR_ENFC_CLK_SEL_MASK,
			MXC_CCM_CS2CDR_ENFC_CLK_PODF(0) |
			MXC_CCM_CS2CDR_ENFC_CLK_PRED(3) |
			MXC_CCM_CS2CDR_ENFC_CLK_SEL(3));

	/* enable gpmi and bch clock gating */
	setbits_le32(&mxc_ccm->CCGR4,
		     MXC_CCM_CCGR4_RAWNAND_U_BCH_INPUT_APB_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_BCH_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_GPMI_IO_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_INPUT_APB_MASK |
		     MXC_CCM_CCGR4_PL301_MX6QPER1_BCH_OFFSET);

	/* enable apbh clock gating */
	setbits_le32(&mxc_ccm->CCGR0, MXC_CCM_CCGR0_APBHDMA_MASK);
}
#endif

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC2_BASE_ADDR},
	{USDHC3_BASE_ADDR},
#ifndef DH_IMX6_NAND_VERSION
	{USDHC4_BASE_ADDR},
#endif
};

#define USDHC2_CD_GPIO	IMX_GPIO_NR(6, 16)
#define USDHC3_CD_GPIO	IMX_GPIO_NR(7, 8)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		ret = gpio_get_value(USDHC2_CD_GPIO);
		break;
	case USDHC3_BASE_ADDR:
		ret = !gpio_get_value(USDHC3_CD_GPIO);
		break;
#ifndef DH_IMX6_NAND_VERSION
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
#endif
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SD2
	 * mmc1                    SD3
	 * mmc2                    eMMC
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			gpio_direction_input(USDHC2_CD_GPIO);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			gpio_direction_input(USDHC3_CD_GPIO);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		case 2:
			imx_iomux_v3_setup_multiple_pads(
				usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			usdhc_cfg[2].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return status;
		}

		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
	}

	return status;
}
#endif

int mx6_rgmii_rework(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

	return 0;
}

#ifdef  CONFIG_FEC_MXC
static int setup_fec(void)
{
        struct iomuxc_base_regs *iomuxc_regs =
                                (struct iomuxc_base_regs *)IOMUXC_BASE_ADDR;
        int ret;

        /* set gpr1[21] to select anatop clock */
        clrsetbits_le32(&iomuxc_regs->gpr[1], 0x1 << 21, 0x1 << 21);

        ret = enable_fec_anatop_clock(ENET_50MHz);
        if (ret)
                return ret;

        return 0;
}
#endif

int board_phy_config(struct phy_device *phydev)
{
	//mx6_rgmii_rework(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)

static iomux_v3_cfg_t const backlight_pwm_pad[] = {
	MX6_PAD_SD1_DAT3__GPIO_1_21 | MUX_PAD_CTRL(NO_PAD_CTRL)
#define PWM_BACKLIGHT_GP IMX_GPIO_NR(1, 21)
};

//static iomux_v3_cfg_t const backlight_pads[] = {
	/* Backlight on RGB connector: J15 */
//	MX6_PAD_SD1_DAT3__GPIO_1_21 | MUX_PAD_CTRL(NO_PAD_CTRL),
//	MX6_PAD_EIM_D27__GPIO_3_27 | MUX_PAD_CTRL(NO_PAD_CTRL),
//#define RGB_BACKLIGHT_GP IMX_GPIO_NR(3, 27)
//#define PWM_BACKLIGHT_GP IMX_GPIO_NR(1, 21)

	/* Backlight on LVDS connector: J6 */
//	MX6_PAD_SD1_CMD__GPIO_1_18 | MUX_PAD_CTRL(NO_PAD_CTRL),
//#define LVDS_BACKLIGHT_GP IMX_GPIO_NR(1, 18)
//};

static iomux_v3_cfg_t const rgb_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15,
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN2,
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN3,
	MX6_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
	MX6_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
	MX6_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
	MX6_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
	MX6_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
	MX6_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
	MX6_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
	MX6_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
	MX6_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
	MX6_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
	MX6_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
	MX6_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
	MX6_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
	MX6_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
	MX6_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
	MX6_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
	MX6_PAD_DISP0_DAT16__IPU1_DISP0_DAT_16,
	MX6_PAD_DISP0_DAT17__IPU1_DISP0_DAT_17,
	MX6_PAD_DISP0_DAT18__IPU1_DISP0_DAT_18,
	MX6_PAD_DISP0_DAT19__IPU1_DISP0_DAT_19,
	MX6_PAD_DISP0_DAT20__IPU1_DISP0_DAT_20,
	MX6_PAD_DISP0_DAT21__IPU1_DISP0_DAT_21,
	MX6_PAD_DISP0_DAT22__IPU1_DISP0_DAT_22,
	MX6_PAD_DISP0_DAT23__IPU1_DISP0_DAT_23,
};

struct display_info_t {
	int	bus;
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev);
	struct	fb_videomode mode;
};

/* LZ: not used yet
static int detect_hdmi(struct display_info_t const *dev)
{
	struct hdmi_regs *hdmi	= (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
	return readb(&hdmi->phy_stat0) & HDMI_DVI_STAT;
}

static void enable_hdmi(struct display_info_t const *dev)
{
	imx_enable_hdmi_phy();
}

static int detect_i2c(struct display_info_t const *dev)
{
	return ((0 == i2c_set_bus_num(dev->bus))
		&&
		(0 == i2c_probe(dev->addr)));
}
*/
#ifdef CONFIG_SPLASH_SCREEN
// Read splashimage from persistant memory
static int board_get_splashimage(void)
{
	char *buffer;
	char *splashimage;
        char *command;

	/* get pointer to buffer and point to splashimage in ram from env */
	buffer = (char*)simple_strtoul(getenv("loadaddr"), NULL, 16);
	splashimage = (char*)simple_strtoul(getenv("splashimage"), NULL, 16);
	
	if ( buffer < (char*)0x10000000 || splashimage < (char*)0x10000000 ) {
		printf ("Error: invalid \"loadaddr\" and \"splashimage\" env values!\n");
		return -ENOMEM;
	}	        

	gd->flags |= GD_FLG_DISABLE_CONSOLE;

	/* load splashimage file from a filesystem */
	if ((command = getenv ("load_splash")) == NULL) {
		gd->flags &= (~GD_FLG_DISABLE_CONSOLE);
		printf ("Error: \"load_splash\" not defined\n");
		return -ENOENT;
	}

	if (run_command (command, 0) != 0) {
		gd->flags &= (~GD_FLG_DISABLE_CONSOLE);
		printf ("Warning: Can't load splash bitmap\n");
		return -EIO;
	}

	// Copy bitmap to splashscreen addresss
	// Note: It is necessary to align bitmaps on a memory address with an offset of an odd multiple of +2, 
	//       since the use of a four-byte alignment will cause alignment exceptions at run-time.
	memcpy(splashimage, buffer, SPLASH_MAX_SIZE);

	gd->flags &= (~GD_FLG_DISABLE_CONSOLE);
	return 0;
}
#endif /* CONFIG_SPLASH_SCREEN */

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	u32 dividor = 0x1b; // 0x1b = 23,5MHz
	u32 pixclock_Hz = 0;
	u32 i = 0;
	
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
	writel(reg, &iomux->gpr[2]);
	//gpio_direction_output(LVDS_BACKLIGHT_GP, 1);
	
	pixclock_Hz = PICOS2KHZ(dev->mode.pixclock) * 1000; 
	
	if(pixclock_Hz < 56500000) {
	
		/* set LDB0, LDB1 clk select to 011/011 */
		reg = readl(&mxc_ccm->cs2cdr);
		reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
			 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
		reg |= (0 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
		      | (0 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
		writel(reg, &mxc_ccm->cs2cdr);
	
	
		for (i = 0; (23500000 + (i*850000)) <= pixclock_Hz; i++) {
			dividor++;
			
			if(dividor >= 54) {
				break;
			}
			
		// Dividor:
		// 0x1b = 27 --> 23,5MHz
		// ...
		// 0x36 = 54 --> 46,45MHz	
		}	
	
		reg = readl(&anatop->pll_video_clr);
		reg |= 0x2000;
		writel(reg, &anatop->pll_video_clr);
		reg = readl(&anatop->pll_video_set);
		reg &= 0xffffff80;
		reg |= dividor;
		writel(reg, &anatop->pll_video_set);
		reg = readl(&anatop->pll_video_set);
		reg |= 0x2000;
		writel(reg, &anatop->pll_video_set);
	}	

}

static void enable_rgb(struct display_info_t const *dev)
{
	imx_iomux_v3_setup_multiple_pads(
		rgb_pads,
		 ARRAY_SIZE(rgb_pads));
	//gpio_direction_output(RGB_BACKLIGHT_GP, 1);
	//gpio_direction_output(PWM_BACKLIGHT_GP, 0);
}

static struct display_info_t displays[] = {/*{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_hdmi,
	.enable	= enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} },*/ /* {
	.bus	= 2,
	.addr	= 0x4,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "Hannstar-XGA",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, */{
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_BGR24,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "lvds_24bit",
		.refresh        = 60,//78,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 40000,//32258,
		.left_margin    = 42,//40,
		.right_margin   = 86,//40,
		.upper_margin   = 10,//3,
		.lower_margin   = 33,//3,
		.hsync_len      = 128,//10,
		.vsync_len      = 2,
		.sync           = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT /*| FB_SYNC_CLK_LAT_FALL*/| FB_SYNC_EXT, 
		.vmode          = FB_VMODE_NONINTERLACED	
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_BGR24,//IPU_PIX_FMT_RGB666,
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
} } };

int board_video_skip(void)
{
	int i;
	int ret;
	char const *panel = getenv("panel");
	int iClocksPerPicture = 0;
	int iDI_TYPE = 0;
	int iDHSettingsInitialization = 0;
	
#ifdef CONFIG_SPLASH_SCREEN
	/* Copy Splash-Image to ddr3 ram - DHCOM specific */
	board_get_splashimage();
#endif	
	if (!panel) {
		iDHSettingsInitialization = 1;
		panel = displays[0].mode.name;
		printf("Using DH settings\n");
		i = 0;

		iDI_TYPE = ((gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_DI_TYPE_FLAG) >> 13);
		displays[0].mode.sync = 0;

		switch (iDI_TYPE)
		{
			case 2: // RGB
				displays[0].enable = enable_rgb;
				break;
			case 3: // LVDS0
				displays[0].enable = enable_lvds;
				displays[0].mode.sync |= FB_SYNC_EXT;
				break;
			default:
				displays[0].enable = enable_rgb;
				break;
		} 

		/* Setup display settings from DH settings file */
		displays[0].mode.xres = gd->dh_board_settings.wXResolution;
		displays[0].mode.yres = gd->dh_board_settings.wYResolution;
		displays[0].mode.pixclock = KHZ2PICOS(gd->dh_board_settings.wPixelClock);
		displays[0].mode.left_margin = gd->dh_board_settings.wHFrontPorch;
		displays[0].mode.hsync_len = gd->dh_board_settings.wHPulseWidth;
		displays[0].mode.right_margin = gd->dh_board_settings.wHBackPorch;
		displays[0].mode.upper_margin = gd->dh_board_settings.wVFrontPorch;
		displays[0].mode.vsync_len = gd->dh_board_settings.wVPulseWidth;
		displays[0].mode.lower_margin = gd->dh_board_settings.wVBackPorch;
		iClocksPerPicture = ((gd->dh_board_settings.wXResolution + gd->dh_board_settings.wHFrontPorch + gd->dh_board_settings.wHPulseWidth + gd->dh_board_settings.wHBackPorch) *
					 (gd->dh_board_settings.wYResolution + gd->dh_board_settings.wVFrontPorch + gd->dh_board_settings.wVPulseWidth + gd->dh_board_settings.wVBackPorch));
		displays[0].mode.refresh = ((gd->dh_board_settings.wPixelClock * 1000) / (iClocksPerPicture));
		if((gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_IVS_FLAG)) {
			displays[0].mode.sync |= FB_SYNC_VERT_HIGH_ACT;
		}
		if((gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_IHS_FLAG)) {
			displays[0].mode.sync |= FB_SYNC_HOR_HIGH_ACT;
		}			
		if((gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_IPC_FLAG)) {
			displays[0].mode.sync |= FB_SYNC_CLK_LAT_FALL;
		}
		if(gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_IOE_FLAG) {
			displays[0].mode.sync |= FB_SYNC_OE_LOW_ACT;
		}
		if(gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_IDATA_FLAG) {
			displays[0].mode.sync |= FB_SYNC_DATA_INVERT;
		}			
		displays[0].mode.vmode = FB_VMODE_NONINTERLACED;
	} else {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			if (!strcmp(panel, displays[i].mode.name))
				break;
		}
	}
	if (i < ARRAY_SIZE(displays)) {
		ret = ipuv3_fb_init(&displays[i].mode, 0,
				    displays[i].pixfmt);
		if (!ret) {
			displays[i].enable(displays+i);
			
			if(iDHSettingsInitialization) {
				printf("Display: DHCOM settings (%ux%u)\n",
				       displays[i].mode.xres,
				       displays[i].mode.yres);
			}
			else {
				printf("Display: %s (%ux%u)\n",
				       displays[i].mode.name,
				       displays[i].mode.xres,
				       displays[i].mode.yres);
			}
		} else
			printf("LCD %s cannot be configured: %d\n",
			       displays[i].mode.name, ret);
	} else {
		printf("unsupported panel %s\n", panel);
		return -EINVAL;
	}

	return 0;
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;
	
	enable_ipu_clock();
	//imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
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
	     |IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
	     |IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     |IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     |IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     |IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	     |IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~IOMUXC_GPR3_LVDS0_MUX_CTL_MASK)
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);

	/* backlights off until needed */
	/*imx_iomux_v3_setup_multiple_pads(backlight_pads,
					 ARRAY_SIZE(backlight_pads));
	gpio_direction_input(LVDS_BACKLIGHT_GP);
	gpio_direction_input(RGB_BACKLIGHT_GP);*/
}
#endif /* CONFIG_VIDEO_IPUV3 */

#ifdef CONFIG_MXC_SPI
iomux_v3_cfg_t const ecspi1_pads[] = {
	/* SS0 */
	MX6_PAD_EIM_EB2__GPIO_2_30   | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

void setup_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads,
					 ARRAY_SIZE(ecspi1_pads));
}
#endif

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}


int board_eth_init(bd_t *bis)
{
	int ret;

	setup_iomux_enet();
	
#ifdef  CONFIG_FEC_MXC
        setup_fec();
#endif	

	ret = cpu_eth_init(bis);
	if (ret)
		printf("FEC MXC: %s:failed\n", __func__);

	return ret;
}

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	imx_iomux_v3_setup_multiple_pads(usb_pads, ARRAY_SIZE(usb_pads));

    /* Enable USB VBUS */
    gpio_direction_output(IMX_GPIO_NR(3, 31) , 1);
    //udelay(500);
    //gpio_set_value(IMX_GPIO_NR(1, 7), 1);	
	
	
	return 0;
}

#endif

void load_dh_settings_file(void)
{
	ulong addr;
	char *command;
	uchar ucBuffer[DHCOM_DISPLAY_SETTINGS_SIZE];
	int ret_value = 0;



	//env = getenv ("loadaddr");
	addr = simple_strtoul(getenv ("loadaddr"), NULL, 16);
	
	/* initialize DH Global Data */
	gd->dh_board_settings.cLength = 		DEFAULT_SETTINGS_BLOCK_LENGTH;
	gd->dh_board_settings.cDisplayID = 		DEFAULT_SETTINGS_DISPLAY_ID;
	gd->dh_board_settings.wValidationID = 	0;
	gd->dh_board_settings.wYResolution = 	DEFAULT_SETTINGS_Y_RESOLUTION;
	gd->dh_board_settings.wXResolution = 	DEFAULT_SETTINGS_X_RESOLUTION;
	gd->dh_board_settings.wLCDConfigFlags = DEFAULT_SETTINGS_LCD_CONFIG_FLAGS;
	gd->dh_board_settings.wPixelClock = 	DEFAULT_SETTINGS_PIXEL_CLOCK;
	gd->dh_board_settings.wVPulseWidth = 	DEFAULT_SETTINGS_V_PULSE_WIDTH;
	gd->dh_board_settings.wHPulseWidth = 	DEFAULT_SETTINGS_H_PULSE_WIDTH;
	gd->dh_board_settings.wHBackPorch = 	DEFAULT_SETTINGS_H_BACK_PORCH;
	gd->dh_board_settings.wHFrontPorch = 	DEFAULT_SETTINGS_H_FRONT_PORCH;
	gd->dh_board_settings.wVBackPorch = 	DEFAULT_SETTINGS_V_BACK_PORCH;
	gd->dh_board_settings.wVFrontPorch = 	DEFAULT_SETTINGS_V_FRONT_PORCH;
	gd->dh_board_settings.cACBiasTrans = 	DEFAULT_SETTINGS_AC_BIAS_TRANS;
	gd->dh_board_settings.cACBiasFreq = 	DEFAULT_SETTINGS_AC_BIAS_FREQ;
	gd->dh_board_settings.cDatalines = 		DEFAULT_SETTINGS_DATALINES;
	gd->dh_board_settings.wGPIODir = 		DEFAULT_SETTINGS_GPIO_DIRECTION;
	gd->dh_board_settings.wGPIOState = 		DEFAULT_SETTINGS_GPIO_STATE;
	gd->dh_board_settings.wHWConfigFlags = 	DEFAULT_SETTINGS_HW_CONFIG_FLAGS;
	
	printf("Load DH settings...\n");

	// Disable console output
	gd->flags |= GD_FLG_DISABLE_CONSOLE;

	/* Load DH settings file from EXT4 Filesystem */
	if ((command = getenv ("load_settings_bin")) == NULL) 
	{
		// Enable console output	
		gd->flags &= (~GD_FLG_DISABLE_CONSOLE);	
		printf ("Warning: \"load_settings_bin\" not defined\n");
		return;
	}	
	
	if (run_command (command, 0) != 0)
	{
		// Enable console output	
		gd->flags &= (~GD_FLG_DISABLE_CONSOLE);	
		printf ("Info: Can't load DH settings file\n");		
		//return; // Don't return, because display settings needs to be loaded from eeprom in that case
	}
	
	// Enable console output	
	gd->flags &= (~GD_FLG_DISABLE_CONSOLE);	

	/* copy settingsblock from dram into the dh_board_settings structure */
	gd->dh_board_settings.wValidationID = ((readl(addr) & 0xFFFF0000) >> 16);

	// settings.bin file Valid Mask should be "DH" = 0x4844
	if(gd->dh_board_settings.wValidationID == 0x4844)
	{
		gd->dh_board_settings.cLength = (readl(addr) & 0xFF);
		gd->dh_board_settings.cDisplayID = ((readl(addr) & 0xFF00) >> 8);

		gd->dh_board_settings.wYResolution = (readl(addr+4) & 0xFFFF);
		gd->dh_board_settings.wXResolution = ((readl(addr+4) & 0xFFFF0000) >> 16);

		gd->dh_board_settings.wLCDConfigFlags = (readl(addr+8) & 0xFFFF);
		gd->dh_board_settings.wPixelClock = ((readl(addr+8) & 0xFFFF0000) >> 16);

		gd->dh_board_settings.wVPulseWidth = (readl(addr+12) & 0xFFFF);
		gd->dh_board_settings.wHPulseWidth = ((readl(addr+12) & 0xFFFF0000) >> 16);

		gd->dh_board_settings.wHBackPorch = (readl(addr+16) & 0xFFFF);
		gd->dh_board_settings.wHFrontPorch = ((readl(addr+16) & 0xFFFF0000) >> 16);

		gd->dh_board_settings.wVBackPorch = (readl(addr+20) & 0xFFFF);
		gd->dh_board_settings.wVFrontPorch = ((readl(addr+20) & 0xFFFF0000) >> 16);

		gd->dh_board_settings.cACBiasTrans = (readl(addr+24) & 0xFF);
		gd->dh_board_settings.cACBiasFreq = ((readl(addr+24) & 0xFF00) >> 8);
		gd->dh_board_settings.cDatalines = ((readl(addr+24) & 0xFFFF0000) >> 16);

		gd->dh_board_settings.wGPIODir = (readl(addr+32));
		gd->dh_board_settings.wGPIOState = (readl(addr+36));

		gd->dh_board_settings.wHWConfigFlags = (readl(addr+40) & 0xFFFF);
	}

	// settings.bin file Valid Mask should be "V2" = 0x3256
	else if(gd->dh_board_settings.wValidationID == 0x3256)
	{
		gd->dh_board_settings.cLength = (readl(addr) & 0xFF);
		gd->dh_board_settings.cDisplayID = ((readl(addr) & 0xFF00) >> 8);

		gd->dh_board_settings.wYResolution = (readl(addr+4) & 0xFFFF);
		gd->dh_board_settings.wXResolution = ((readl(addr+4) & 0xFFFF0000) >> 16);

		gd->dh_board_settings.wPixelClock = (readl(addr+8));

		gd->dh_board_settings.wVPulseWidth = (readl(addr+12) & 0xFFFF);
		gd->dh_board_settings.wHPulseWidth = ((readl(addr+12) & 0xFFFF0000) >> 16);

		gd->dh_board_settings.wHBackPorch = (readl(addr+16) & 0xFFFF);
		gd->dh_board_settings.wHFrontPorch = ((readl(addr+16) & 0xFFFF0000) >> 16);

		gd->dh_board_settings.wVBackPorch = (readl(addr+20) & 0xFFFF);
		gd->dh_board_settings.wVFrontPorch = ((readl(addr+20) & 0xFFFF0000) >> 16);

		gd->dh_board_settings.cACBiasTrans = (readl(addr+24) & 0xFF);
		gd->dh_board_settings.cACBiasFreq = ((readl(addr+24) & 0xFF00) >> 8);
		gd->dh_board_settings.cDatalines = ((readl(addr+24) & 0xFFFF0000) >> 16);

		gd->dh_board_settings.wLCDConfigFlags = (readl(addr+28));

		gd->dh_board_settings.wGPIODir = (readl(addr+32));
		gd->dh_board_settings.wGPIOState = (readl(addr+36));

		gd->dh_board_settings.wHWConfigFlags = (readl(addr+40) & 0xFFFF);
	}
	
	else
	{
		gd->dh_board_settings.wValidationID = 0;
	}
	
	/* Check and Read Display data from EEPROM if enabled */
	if((gd->dh_board_settings.wHWConfigFlags & SETTINGS_HW_EN_DISP_ADPT_EE_CHK) != 0)
	{
		/* Set i2c driver to use i2c bus 0  */
		DISABLE_PRINTF()
		run_command ("i2c dev 0", 0);
		ENABLE_PRINTF()

		i2c_init (CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
		ret_value = i2c_read(DISPLAY_ADAPTER_EEPROM_ADDR, 0, 1, &ucBuffer[0], DHCOM_DISPLAY_SETTINGS_SIZE);

		/* Set i2c driver back to use i2c bus 2 */
		DISABLE_PRINTF()
		run_command ("i2c dev 2", 0);
		ENABLE_PRINTF()

		if((ret_value == 0) && (ucBuffer[2] == 'D') && (ucBuffer[3] == 'H'))
		{
			gd->dh_board_settings.wValidationID = 0x4844; // Display settings "DH"

			gd->dh_board_settings.cDisplayID = ucBuffer[1];

			gd->dh_board_settings.wYResolution = (ucBuffer[5] << 8) | ucBuffer[4];
			gd->dh_board_settings.wXResolution = (ucBuffer[7] << 8) | ucBuffer[6];

			gd->dh_board_settings.wLCDConfigFlags = (ucBuffer[9] << 8) | ucBuffer[8];
			gd->dh_board_settings.wPixelClock = (ucBuffer[11] << 8) | ucBuffer[10];

			gd->dh_board_settings.wVPulseWidth = (ucBuffer[13] << 8) | ucBuffer[12];
			gd->dh_board_settings.wHPulseWidth = (ucBuffer[15] << 8) | ucBuffer[14];

			gd->dh_board_settings.wHBackPorch = (ucBuffer[17] << 8) | ucBuffer[16];
			gd->dh_board_settings.wHFrontPorch = (ucBuffer[19] << 8) | ucBuffer[18];

			gd->dh_board_settings.wVBackPorch = (ucBuffer[21] << 8) | ucBuffer[20];
			gd->dh_board_settings.wVFrontPorch = (ucBuffer[23] << 8) | ucBuffer[22];

			gd->dh_board_settings.cACBiasTrans = ucBuffer[24];
			gd->dh_board_settings.cACBiasFreq = ucBuffer[25];
			gd->dh_board_settings.cDatalines = (ucBuffer[27] << 8) | ucBuffer[26];
		}

		else if((ret_value == 0) && (ucBuffer[2] == 'V') && (ucBuffer[3] == '2'))
		{
			gd->dh_board_settings.wValidationID = 0x3256; // DIsplay settings "V2"

			gd->dh_board_settings.cDisplayID = ucBuffer[1];

			gd->dh_board_settings.wYResolution = (ucBuffer[5] << 8) | ucBuffer[4];
			gd->dh_board_settings.wXResolution = (ucBuffer[7] << 8) | ucBuffer[6];

			gd->dh_board_settings.wPixelClock = (ucBuffer[11] << 24) | (ucBuffer[10]  << 16) | (ucBuffer[9] << 8) | ucBuffer[8];

			gd->dh_board_settings.wVPulseWidth = (ucBuffer[13] << 8) | ucBuffer[12];
			gd->dh_board_settings.wHPulseWidth = (ucBuffer[15] << 8) | ucBuffer[14];

			gd->dh_board_settings.wHBackPorch = (ucBuffer[17] << 8) | ucBuffer[16];
			gd->dh_board_settings.wHFrontPorch = (ucBuffer[19] << 8) | ucBuffer[18];

			gd->dh_board_settings.wVBackPorch = (ucBuffer[21] << 8) | ucBuffer[20];
			gd->dh_board_settings.wVFrontPorch = (ucBuffer[23] << 8) | ucBuffer[22];

			gd->dh_board_settings.cACBiasTrans = ucBuffer[24];
			gd->dh_board_settings.cACBiasFreq = ucBuffer[25];
			gd->dh_board_settings.cDatalines = (ucBuffer[27] << 8) | ucBuffer[26];

			gd->dh_board_settings.wLCDConfigFlags = (ucBuffer[31] << 24) | (ucBuffer[30]  << 16) | (ucBuffer[29] << 8) | ucBuffer[28];
		}
		
		else
		{
			printf ("Info: Can't load DH settings file from eeprom\n");		
		}
	}
}

void generate_dh_settings_kernel_args(void)
{
	int iDI_TYPE = 0;
	uchar buf[512];
	int backlight_gpio = 0;
	int backlight_en_pol = 0;
	int backlight_pwm_pol = 0;
	int backlight_on = 0;
	int h_sync_inv = 0;
	int v_sync_inv = 0;
	int DE_inv = 0;
	int PCLK_inv = 0;	
	
	// Set ENV Linux Kernel parameter
	// DHCOM settings "V2" = 0x3256 or "DH" = 0x4844
	if((gd->dh_board_settings.wValidationID == 0x3256) || (gd->dh_board_settings.wValidationID == 0x4844))
	{	
		iDI_TYPE = ((gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_DI_TYPE_FLAG) >> 13);

		// Mask Backlight enable GPIO
		backlight_gpio = ((gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_BL_EN_GPIO_FLAG) >> 7);
		
		// Covert to struct gpio number
		backlight_gpio = backlight_gpio - 1;		

		// Mask Backlight pol flag: 0 = active high; 1 = active low
		backlight_en_pol = ((gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_IBL_FLAG) >> 11) ;

		// Mask Backlight PWM pol flag: 0 = active high; 1 = active low
		backlight_pwm_pol = ((gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_PWM_POL_FLAG) >> 6);

		// Mask Backlight ON pol flag: 0 = backlight off; 1 = backlight on
		backlight_on = ((gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_BL_ON_FLAG) >> 12);
		
		// Display control flags
		v_sync_inv = (gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_IVS_FLAG);
		h_sync_inv = ((gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_IHS_FLAG) >> 1);
		PCLK_inv = ((gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_IPC_FLAG) >> 2);
		DE_inv = ((gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_IOE_FLAG) >> 3);

		// Set Display Type to RGB for old settings file
		if(gd->dh_board_settings.wValidationID == 0x4844)
		{
			iDI_TYPE = 2; // RGB Display
			backlight_on = 1; // Set backlight_on flag
		}

		switch (iDI_TYPE)
		{
		case 0: // Ignore Display settings	
			// Delete ENV variables
			setenv("parallel_display", NULL);	
			setenv("lvds_display0", NULL);
			setenv("lvds_display1", NULL);
			setenv("pwm_bl.set", NULL);			
		   	break;
		case 1: // Headless
			sprintf((char *)buf, "parallel_display.disable");	
			setenv("parallel_display", (char *)buf);
			sprintf((char *)buf, "imx_ldb.disable0");	
			setenv("lvds_display0", (char *)buf);
			sprintf((char *)buf, "imx_ldb.disable1");	
			setenv("lvds_display1", (char *)buf);
			sprintf((char *)buf, "pwm_bl.disable");	
			setenv("backlight", (char *)buf);
		   	break;
		case 2: // RGB
			sprintf((char *)buf, "parallel_display.timings=ID:%d,PCLK:%d,XRES:%d,YRES:%d,HFP:%d,HBP:%d,HSYNC:%d,VFP:%d,VBP:%d,VSYNC:%d,HINV:%d,VINV:%d,DEINV:%d,PCLKPOL:%d",
		                 gd->dh_board_settings.cDisplayID, (gd->dh_board_settings.wPixelClock*1000), gd->dh_board_settings.wXResolution,
		                 gd->dh_board_settings.wYResolution, gd->dh_board_settings.wHFrontPorch, gd->dh_board_settings.wHBackPorch, 
		                 gd->dh_board_settings.wHPulseWidth, gd->dh_board_settings.wVFrontPorch, gd->dh_board_settings.wVBackPorch, 
		                 gd->dh_board_settings.wVPulseWidth, h_sync_inv, v_sync_inv, DE_inv, PCLK_inv);	
			setenv("parallel_display", (char *)buf);
			sprintf((char *)buf, "imx_ldb.disable0");	
			setenv("lvds_display0", (char *)buf);
			sprintf((char *)buf, "imx_ldb.disable1");	
			setenv("lvds_display1", (char *)buf);
			sprintf((char *)buf, "pwm_bl.set=BLGPIO:%d,BLINV:%d,BLON:%d,PWMINV:%d",DHCOM_gpios[backlight_gpio], backlight_en_pol, backlight_on, backlight_pwm_pol);	
			setenv("backlight", (char *)buf);
		   	break;
		case 3: // LVDS0
			sprintf((char *)buf, "parallel_display.disable");	
			setenv("parallel_display", (char *)buf);		
			sprintf((char *)buf, "imx_ldb.timings0=ID:%d,PCLK:%d,XRES:%d,YRES:%d,HFP:%d,HBP:%d,HSYNC:%d,VFP:%d,VBP:%d,VSYNC:%d,HINV:%d,VINV:%d,DEINV:%d,PCLKPOL:%d",
		                 gd->dh_board_settings.cDisplayID, (gd->dh_board_settings.wPixelClock*1000), gd->dh_board_settings.wXResolution,
		                 gd->dh_board_settings.wYResolution, gd->dh_board_settings.wHFrontPorch, gd->dh_board_settings.wHBackPorch, 
		                 gd->dh_board_settings.wHPulseWidth, gd->dh_board_settings.wVFrontPorch, gd->dh_board_settings.wVBackPorch, 
		                 gd->dh_board_settings.wVPulseWidth, h_sync_inv, v_sync_inv, DE_inv, PCLK_inv);	
			setenv("lvds_display0", (char *)buf);
			sprintf((char *)buf, "imx_ldb.disable1");	
			setenv("lvds_display1", (char *)buf);
			sprintf((char *)buf, "pwm_bl.set=BLGPIO:%d,BLINV:%d,BLON:%d,PWMINV:%d",DHCOM_gpios[backlight_gpio], backlight_en_pol, backlight_on, backlight_pwm_pol);	
			setenv("backlight", (char *)buf);
		   	break;
		case 4: // LVDS1
			sprintf((char *)buf, "parallel_display.disable");	
			setenv("parallel_display", (char *)buf);
			sprintf((char *)buf, "imx_ldb.disable0");	
			setenv("lvds_display0", (char *)buf);		
			sprintf((char *)buf, "imx_ldb.timings1=ID:%d,PCLK:%d,XRES:%d,YRES:%d,HFP:%d,HBP:%d,HSYNC:%d,VFP:%d,VBP:%d,VSYNC:%d,HINV:%d,VINV:%d,DEINV:%d,PCLKPOL:%d",
		                 gd->dh_board_settings.cDisplayID, (gd->dh_board_settings.wPixelClock*1000), gd->dh_board_settings.wXResolution,
		                 gd->dh_board_settings.wYResolution, gd->dh_board_settings.wHFrontPorch, gd->dh_board_settings.wHBackPorch, 
		                 gd->dh_board_settings.wHPulseWidth, gd->dh_board_settings.wVFrontPorch, gd->dh_board_settings.wVBackPorch, 
		                 gd->dh_board_settings.wVPulseWidth, h_sync_inv, v_sync_inv, DE_inv, PCLK_inv);	
			setenv("lvds_display1", (char *)buf);
			sprintf((char *)buf, "pwm_bl.set=BLGPIO:%d,BLINV:%d,BLON:%d,PWMINV:%d",DHCOM_gpios[backlight_gpio], backlight_en_pol, backlight_on, backlight_pwm_pol);	
			setenv("backlight", (char *)buf);
		   	break;
		case 5: // Dual LVDS
			sprintf((char *)buf, "parallel_display.disable");	
			setenv("parallel_display", (char *)buf);		
			sprintf((char *)buf, "imx_ldb.timings0=ID:%d,PCLK:%d,XRES:%d,YRES:%d,HFP:%d,HBP:%d,HSYNC:%d,VFP:%d,VBP:%d,VSYNC:%d,HINV:%d,VINV:%d,DEINV:%d,PCLKPOL:%d",
		                 gd->dh_board_settings.cDisplayID, (gd->dh_board_settings.wPixelClock*1000), gd->dh_board_settings.wXResolution,
		                 gd->dh_board_settings.wYResolution, gd->dh_board_settings.wHFrontPorch, gd->dh_board_settings.wHBackPorch, 
		                 gd->dh_board_settings.wHPulseWidth, gd->dh_board_settings.wVFrontPorch, gd->dh_board_settings.wVBackPorch, 
		                 gd->dh_board_settings.wVPulseWidth, h_sync_inv, v_sync_inv, DE_inv, PCLK_inv);	
			setenv("lvds_display0", (char *)buf);
			setenv("lvds_display1", NULL); // Delete lvds_display1 ENV variable for Dual channel LVDS display
			sprintf((char *)buf, "pwm_bl.set=BLGPIO:%d,BLINV:%d,BLON:%d,PWMINV:%d",DHCOM_gpios[backlight_gpio], backlight_en_pol, backlight_on, backlight_pwm_pol);	
			setenv("backlight", (char *)buf);
		   	break;
		default:
			break;
		  
		}
	}
	else
	{
		// Delete ENV variables
		setenv("parallel_display", NULL);	
		setenv("lvds_display0", NULL);
		setenv("lvds_display1", NULL);
		setenv("pwm_bl.set", NULL);	
	}	
}

void set_dhcom_gpios(void)
{
	int i;
	int mask = 0x1;

	// Setup alternate function and configure pads
	imx_iomux_v3_setup_multiple_pads(gpio_pads, ARRAY_SIZE(gpio_pads));
	
	
	for(i = 0; i < 23; i++) {

		if(gd->dh_board_settings.wGPIODir & mask) {
			// Set to input
			gpio_direction_input(DHCOM_gpios[i]);		
		}
		else {
		        // Set to output
			if(gd->dh_board_settings.wGPIOState & mask) {
				// Set to high
				gpio_direction_output(DHCOM_gpios[i] , 1);
			}
			else {
				// Set to low
				gpio_direction_output(DHCOM_gpios[i] , 0);
			}
		}
		mask = mask << 1;
	}
}

void set_dhcom_backlight_gpio(void)
{
	int backlight_gpio;
	int backlight_en_pol;
	int backlight_pwm_pol;
	int disable_backlight = 0;
	char const *panel = getenv("panel");
	const char *no_panel = "no_panel";

	if(panel) 
	{
		if (!strcmp(panel, no_panel)) 
		{
			disable_backlight = 1;
		}
	}
	else
	{
		// If display interface is enabled check BL_ON flag (0 = backlight off / 1 = backlight on)
		if(gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_BL_ON_FLAG)
		{
			disable_backlight = 0;
		}
		else
		{
			disable_backlight = 1;
		}
	}

	/* Init backlight PWM Pin */
	imx_iomux_v3_setup_multiple_pads(backlight_pwm_pad, ARRAY_SIZE(backlight_pwm_pad));

	// Mask Backlight enable GPIO
	backlight_gpio = ((gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_BL_EN_GPIO_FLAG) >> 7);
	
	// Check if backlight enable ist specified
	if(backlight_gpio != 0) {
		// Covert to struct gpio number
		backlight_gpio = backlight_gpio - 1;

		// Mask Backlight pol flag: 0 = active high; 1 = active low
		backlight_en_pol = (gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_IBL_FLAG);

		// Mask Backlight PWM pol flag: 0 = active high; 1 = active low
		backlight_pwm_pol = (gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_PWM_POL_FLAG);
		
		if(disable_backlight == 1) {
			// Disable backlight pin till linux is running
			if(backlight_en_pol == 0)
			{
				// Set to low
				gpio_direction_output(DHCOM_gpios[backlight_gpio] , 0);
			}
			else
			{
				// Set to high
				gpio_direction_output(DHCOM_gpios[backlight_gpio] , 1);
			}

			// Disable backlight PWM pin till linux is running
			if(backlight_pwm_pol == 0)
			{
				// Set to low
				gpio_direction_output(PWM_BACKLIGHT_GP, 0);
			}
			else
			{
				// Set to high
				gpio_direction_output(PWM_BACKLIGHT_GP, 1);
			}
		}
		else {
			// Enable backlight
			if(backlight_en_pol == 0)
			{
				// Set to low
				gpio_direction_output(DHCOM_gpios[backlight_gpio] , 1);
			}
			else
			{
				// Set to high
				gpio_direction_output(DHCOM_gpios[backlight_gpio] , 0);
			}

			// Disable backlight PWM pin till linux is running
			if(backlight_pwm_pol == 0)
			{
				// Set to low
				gpio_direction_output(PWM_BACKLIGHT_GP, 1);
			}
			else
			{
				// Set to high
				gpio_direction_output(PWM_BACKLIGHT_GP, 0);
			}
		}

	}

}
void burn_fuses(void)
{
	u32 val;
	
	if(fuse_read(0, 6, &val))
	{
		printf("Fusemap: ERROR Can't read BT_FUSE_SEL bit!\n");
		return;
	}
	
	if(!(val & BT_FUSE_SEL))
	{
		if(fuse_prog(0, 5, SPI_E_FUSE_CONFIG))
		{
			printf("Fusemap: ERROR Can't write BOOT_CFG bits!\n");
			return;
		}

		if(fuse_prog(0, 6, BT_FUSE_SEL))	
		{
			printf("Fusemap: ERROR Can't write BT_FUSE_SEL bit!\n");
			return;
		}
		printf("Fusemap: Burning BOOT_CFG bits\n");
	}
}

void init_MAC_address(void)
{
	u32 val = 0;
	unsigned char linebuf[6];
	unsigned char env_enetaddr[6];
	unsigned char default_mac[6]={0x00,0x11,0x22,0x33,0x44,0x55};
	u8 mac[6];
	int i = 0;
	
	if(fuse_read(4, 2, &val))
	{
		printf("Init MAC: ERROR Can't read lower MAC address!\n");
		return;
	}
	
	if (!eth_getenv_enetaddr("ethaddr", env_enetaddr))
	{
		printf("Init MAC: Can't find ethaddr! Generating ENV entry!\n");
		eth_setenv_enetaddr("ethaddr", default_mac);	
		for(i = 0; i < 6; i++)
		{
			env_enetaddr[i] = default_mac[i];
		}		
	}
	
	if((val == 0) || (!memcmp(env_enetaddr, default_mac, 6)))
	{
		if (i2c_set_bus_num(2) != 0)
		{
			printf("Init MAC: ERROR Can't set I2C bus number!\n");
			return;
		}
		if (i2c_read(EEPROM_I2C_ADDRESS, 0xfa, 0x1, linebuf, 0x6) != 0)
		{
			printf("Init MAC: ERROR Can't read MAC from EEPROM!\n");
			return;
		}
		val = (linebuf[5]) | (linebuf[4] << 8) | (linebuf[3] << 16) | (linebuf[2] << 24);
		if(fuse_prog(4, 2, val))
		{
			printf("Fusemap: ERROR Can't write lower MAC to fusemap!\n");
			return;
		}
		val = (linebuf[1]) | (linebuf[0] << 8);
		if(fuse_prog(4, 3, val))
		{
			printf("Fusemap: ERROR Can't write higher MAC to fusemap!\n");
			return;
		}
		printf("Fusemap: Burning MAC address\n");

		for(i = 0; i < 6; i++)
		{
			mac[i] = (u8)linebuf[i];
		}
		if (is_valid_ether_addr(mac))
		{
			eth_setenv_enetaddr("ethaddr", mac);	
		}
		saveenv();
	}
}

int board_early_init_f(void)
{
	setup_iomux_uart();
	
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

	return 0;
}

int board_init(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int reg;

	/* Enable eim_slow clocks */
	reg = __raw_readl(&mxc_ccm->CCGR6);
	reg |=  (0x1 << MXC_CCM_CCGR6_EMI_SLOW_OFFSET);
	writel(reg, &mxc_ccm->CCGR6);

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	
#ifdef CONFIG_I2C_MXC	
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);	
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
#endif
	
#ifdef DH_IMX6_NAND_VERSION
	setup_gpmi_nand();
#endif

#ifdef CONFIG_MXC_SPI
	setup_spi();
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

int board_late_init(void)
{
	struct scu_regs *scu = (struct scu_regs *)SCU_BASE_ADDR;
	u32 cfg = readl(&scu->config) & 0xFFFF;
	u32 hw_code = 0;
	uchar buf[128];
	
	// HW coed pins: Setup alternate function and configure pads
	imx_iomux_v3_setup_multiple_pads(hw_code_pads, ARRAY_SIZE(hw_code_pads));	
	
	// Set to input
	gpio_direction_input(HW_CODE_BIT_0);	
	gpio_direction_input(HW_CODE_BIT_1);	
	gpio_direction_input(HW_CODE_BIT_2);	
	
	hw_code = ((gpio_get_value(HW_CODE_BIT_2) << 2) | (gpio_get_value(HW_CODE_BIT_1) << 1) | gpio_get_value(HW_CODE_BIT_0)) + 2; // HW 100 + HW 200 = 00b; HW 300 = 01b
	
	switch (cfg)
	{
	case 0x0500: // Solo	
		sprintf((char *)buf, "imx6s-dhcom%d",hw_code);	
		setenv("dhcom", (char *)buf);		
		break;
	case 0x0501: // DualLite
		sprintf((char *)buf, "imx6dl-dhcom%d",hw_code);	
		setenv("dhcom", (char *)buf);
		break;
	case 0x5501: // Dual
		sprintf((char *)buf, "imx6d-dhcom%d",hw_code);	
		setenv("dhcom", (char *)buf);
		break;
	case 0x5503: // Quad
		sprintf((char *)buf, "imx6q-dhcom%d",hw_code);	
		setenv("dhcom", (char *)buf);
		break;	
	
	default:
		// Delete dhcom
		setenv("dhcom", NULL);		
		break;
	  
	}
	
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
