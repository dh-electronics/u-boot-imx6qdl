/* SPDX-License-Identifier: (GPL-2.0-or-later) */
/*
 * DHCOM DH-iMX6ULL PDK board support
 *
 * Copyright (C) 2018 DH electronics GmbH
 */

#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6ull_pins.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/io.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <environment.h>
#include <i2c.h>
#include <miiphy.h>
#include <linux/sizes.h>
#include <mmc.h>
#include <netdev.h>
#include <power/pmic.h>
#include <power/pfuze3000_pmic.h>
#include <usb.h>
#include <usb/ehci-ci.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_SPEED_HIGH   |                                  \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
	PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define MDIO_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST | PAD_CTL_ODE)

#define ENET_CLK_PAD_CTRL  (PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |         \
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define GPIO_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |        \
	PAD_CTL_DSE_40ohm)

#define EEPROM0_I2C_ADDRESS	0x50
#define EEPROM1_I2C_ADDRESS	0x53

#define SPI_CS			IMX_GPIO_NR(4, 26)
#define SPI_BOOT_FLASH_EN	IMX_GPIO_NR(1, 9)

#ifdef CONFIG_SYS_I2C_MXC
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
/* DHCOM I2C2 */
static struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode =  MX6_PAD_UART4_TX_DATA__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_UART4_TX_DATA__GPIO1_IO28 | PC,
		.gp = IMX_GPIO_NR(1, 28),
	},
	.sda = {
		.i2c_mode = MX6_PAD_UART4_RX_DATA__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_UART4_RX_DATA__GPIO1_IO29 | PC,
		.gp = IMX_GPIO_NR(1, 29),
	},
};
/* DHCOM I2C1 */
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode =  MX6_PAD_UART5_TX_DATA__I2C2_SCL | PC,
		.gpio_mode = MX6_PAD_UART5_TX_DATA__GPIO1_IO30 | PC,
		.gp = IMX_GPIO_NR(1, 30),
	},
	.sda = {
		.i2c_mode = MX6_PAD_UART5_RX_DATA__I2C2_SDA | PC,
		.gpio_mode = MX6_PAD_UART5_RX_DATA__GPIO1_IO31 | PC,
		.gp = IMX_GPIO_NR(1, 31),
	},
};
#endif

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#define LGA_HW_CODE_BIT_0	IMX_GPIO_NR(3, 0)
#define LGA_HW_CODE_BIT_1	IMX_GPIO_NR(3, 1)

int board_get_lga_hwcode(void)
{
	int hw_code;

	gpio_direction_input(LGA_HW_CODE_BIT_0);
	gpio_direction_input(LGA_HW_CODE_BIT_1);

	/* HW 100 = 0b00; HW 200 = 0b01; HW 300 = 0b10; HW 400 = 0b11 */
	hw_code = ( (gpio_get_value(LGA_HW_CODE_BIT_1) << 1) |
		    (gpio_get_value(LGA_HW_CODE_BIT_0) << 0)  ) + 1;

	return hw_code;
}

#define SODIMM_HW_CODE_BIT_0	IMX_GPIO_NR(4, 21)
#define SODIMM_HW_CODE_BIT_1	IMX_GPIO_NR(4, 22)
#define SODIMM_HW_CODE_BIT_2	IMX_GPIO_NR(4, 23)

int board_get_sodimm_hwcode(void)
{
	int hw_code;

	gpio_direction_input(SODIMM_HW_CODE_BIT_0);
	gpio_direction_input(SODIMM_HW_CODE_BIT_1);
	gpio_direction_input(SODIMM_HW_CODE_BIT_2);

	/* HW 100 + HW 200 = 00b; HW 300 = 01b */
	hw_code = ( (gpio_get_value(SODIMM_HW_CODE_BIT_2) << 2) |
		    (gpio_get_value(SODIMM_HW_CODE_BIT_1) << 1) |
		    (gpio_get_value(SODIMM_HW_CODE_BIT_0) << 0)  ) + 1;

	return hw_code;
}

#ifdef CONFIG_NAND_MXS

#define NAND_PAD_CTRL (PAD_CTL_DSE_48ohm | PAD_CTL_SRE_SLOW | PAD_CTL_HYS)
#define NAND_PAD_READY0_CTRL (PAD_CTL_DSE_48ohm | PAD_CTL_PUS_22K_UP)

static iomux_v3_cfg_t const gpmi_pads[] = {
	MX6_PAD_NAND_DATA00__RAWNAND_DATA00	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX6_PAD_NAND_DATA01__RAWNAND_DATA01	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX6_PAD_NAND_DATA02__RAWNAND_DATA02	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX6_PAD_NAND_DATA03__RAWNAND_DATA03	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX6_PAD_NAND_DATA04__RAWNAND_DATA04	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX6_PAD_NAND_DATA05__RAWNAND_DATA05	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX6_PAD_NAND_DATA06__RAWNAND_DATA06	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX6_PAD_NAND_DATA07__RAWNAND_DATA07	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX6_PAD_NAND_CLE__RAWNAND_CLE		| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX6_PAD_NAND_ALE__RAWNAND_ALE		| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX6_PAD_NAND_CE0_B__RAWNAND_CE0_B	| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX6_PAD_NAND_RE_B__RAWNAND_RE_B		| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX6_PAD_NAND_WE_B__RAWNAND_WE_B		| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX6_PAD_NAND_WP_B__RAWNAND_WP_B		| MUX_PAD_CTRL(NAND_PAD_CTRL),
	MX6_PAD_NAND_READY_B__RAWNAND_READY_B	| MUX_PAD_CTRL(NAND_PAD_READY0_CTRL),
};

void setup_gpmi_nand(void)
{
	imx_iomux_v3_setup_multiple_pads(gpmi_pads, ARRAY_SIZE(gpmi_pads));

	setup_gpmi_io_clk((3 << MXC_CCM_CSCDR1_BCH_PODF_OFFSET) |
			  (3 << MXC_CCM_CSCDR1_GPMI_PODF_OFFSET));
}
#endif /* CONFIG_NAND_MXS */

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[1] = {
	{USDHC1_BASE_ADDR, 0, 4}
};

#define USDHC1_CD_GPIO	IMX_GPIO_NR(1, 19)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		/* USDHC1 card detect on HW100 is active low */
		if (board_get_sodimm_hwcode() == 1)
			ret = !gpio_get_value(USDHC1_CD_GPIO);
		else
			ret = gpio_get_value(USDHC1_CD_GPIO);
		if (ret)
			printk("SD card detected\n");
		return ret;
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
	 */
	gpio_direction_input(USDHC1_CD_GPIO);

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);

	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret) {
			printf("Warning: failed to initialize mmc dev %d\n", i);
			return ret;
		}
	}

	return 0;
}
#endif

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return usb_phy_mode(port);
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	/* Set Power polarity */
	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}

int board_ehci_power(int port, int on)
{
	switch (port) {
	case 0:
		break;
	case 1:
		gpio_direction_output(IMX_GPIO_NR(1, 5), !on);
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}
#endif

#ifdef CONFIG_FEC_MXC
static int setup_fec(int fec_id)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int ret;

	if (fec_id == 0) {
		/*
		 * Use 50M anatop loopback REF_CLK1 for ENET1,
		 * clear gpr1[13], set gpr1[17].
		 */
		clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
				IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);
	} else {
		/*
		 * Use 50M anatop loopback REF_CLK2 for ENET2,
		 * clear gpr1[14], set gpr1[18].
		 */
		clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC2_MASK,
				IOMUX_GPR1_FEC2_CLOCK_MUX1_SEL_MASK);
	}

	ret = enable_fec_anatop_clock(fec_id, ENET_50MHZ);
	if (ret)
		return ret;

	enable_enet_clk(1);

	return 0;
}

static void eth_phy_reset(int fec_id)
{
	/* Reset PHY */
	if (fec_id == 0)
		gpio_direction_output(IMX_GPIO_NR(3, 23) , 0);
	else
		gpio_direction_output(IMX_GPIO_NR(3, 24) , 0);
	mdelay(1);
	if (fec_id == 0)
		gpio_direction_output(IMX_GPIO_NR(3, 23) , 1);
	else
		gpio_direction_output(IMX_GPIO_NR(3, 24) , 1);
	mdelay(10);
}

int board_eth_init(bd_t *bis)
{
	eth_phy_reset(CONFIG_FEC_ENET_DEV);

	return fecmxc_initialize_multi(bis, CONFIG_FEC_ENET_DEV,
				       CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}
#endif

#ifdef CONFIG_VIDEO_MXS
static iomux_v3_cfg_t const lcd_pads[] = {
	MX6_PAD_LCD_CLK__LCDIF_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_ENABLE__LCDIF_ENABLE | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_HSYNC__LCDIF_HSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_VSYNC__LCDIF_VSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA00__LCDIF_DATA00 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA01__LCDIF_DATA01 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA02__LCDIF_DATA02 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA03__LCDIF_DATA03 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA04__LCDIF_DATA04 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA05__LCDIF_DATA05 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA06__LCDIF_DATA06 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA07__LCDIF_DATA07 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA08__LCDIF_DATA08 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA09__LCDIF_DATA09 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA10__LCDIF_DATA10 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA11__LCDIF_DATA11 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA12__LCDIF_DATA12 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA13__LCDIF_DATA13 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA14__LCDIF_DATA14 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA15__LCDIF_DATA15 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA16__LCDIF_DATA16 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA17__LCDIF_DATA17 | MUX_PAD_CTRL(LCD_PAD_CTRL),

	/* LCD_RST */
	MX6_PAD_SNVS_TAMPER9__GPIO5_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* Use GPIO for Brightness adjustment, duty cycle = period. */
	MX6_PAD_GPIO1_IO08__GPIO1_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static int setup_lcd(void)
{
	enable_lcdif_clock(LCDIF1_BASE_ADDR, 1);

	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

	/* Reset the LCD */
	gpio_direction_output(IMX_GPIO_NR(5, 9) , 0);
	udelay(500);
	gpio_direction_output(IMX_GPIO_NR(5, 9) , 1);

	/* Set Brightness to high */
	gpio_direction_output(IMX_GPIO_NR(1, 8) , 1);

	return 0;
}
#endif

static int setup_dhcom_mac_from_fuse(int fec_id, const char *env_name)
{
	unsigned char enetaddr[6];
	int ret, eeprom_i2c_addr;

	if (fec_id == 0)
		eeprom_i2c_addr = EEPROM0_I2C_ADDRESS;
	else
		eeprom_i2c_addr = EEPROM1_I2C_ADDRESS;

	ret = eth_env_get_enetaddr(env_name, enetaddr);
	if (ret)	/* ethaddr is already set as env_name */
		return 0;

	imx_get_mac_from_fuse(fec_id, enetaddr);

	if (is_valid_ethaddr(enetaddr)) {
		eth_env_set_enetaddr(env_name, enetaddr);
		env_save();
		return 0;
	}

	ret = i2c_set_bus_num(0);
	if (ret) {
		printf("Error switching I2C bus!\n");
		return ret;
	}

	ret = i2c_read(eeprom_i2c_addr, 0xfa, 0x1, enetaddr, 0x6);
	if (ret) {
		printf("Error reading configuration EEPROM!\n");
		return ret;
	}

	if (is_valid_ethaddr(enetaddr)) {
		printf("Net:   FEC%d: Save MAC %02X:%02X:%02X:%02X:%02X:%02X "
		       "from EEPROM 0x%02X to env \"%s\"\n",
		        fec_id,
		        enetaddr[0], enetaddr[1], enetaddr[2],
		        enetaddr[3], enetaddr[4], enetaddr[5],
		        eeprom_i2c_addr, env_name);
		eth_env_set_enetaddr(env_name, enetaddr);
		env_save();
	}

	return 0;
}

int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
}

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	if (bus == 0 && cs == 0)
		return SPI_CS;
	else
		return -1;
}
#endif

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_SYS_I2C_MXC
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
#endif

#ifdef CONFIG_NAND_MXS
	setup_gpmi_nand();
#endif

#ifdef CONFIG_FEC_MXC
	setup_fec(CONFIG_FEC_ENET_DEV);
#endif

#ifdef CONFIG_VIDEO_MXS
	setup_lcd();
#endif

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd1", MAKE_CFGVAL(0x42, 0x20, 0x00, 0x00)},
	{"sd2", MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"qspi1", MAKE_CFGVAL(0x10, 0x00, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
	u32 lga_hw_code;
	u32 sodimm_hw_code;
	char buf[16];

	setup_dhcom_mac_from_fuse(0, "ethaddr");
	setup_dhcom_mac_from_fuse(1, "eth1addr");

	lga_hw_code = board_get_lga_hwcode();
	sodimm_hw_code = board_get_sodimm_hwcode();
	printf("HW:    LGA=HW%d00, SODIMM=HW%d00\n", lga_hw_code, sodimm_hw_code);
	snprintf(buf, sizeof(buf), "imx6ull-dhcom%1d%1d", lga_hw_code, sodimm_hw_code);
	env_set("dhcom", buf);

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	return 0;
}

int checkboard(void)
{
	puts("Board: DHCOM i.MX6ULL\n");
	return 0;
}

#ifdef CONFIG_SPL_BUILD
#include <linux/libfdt.h>
#include <spl.h>
#include <asm/arch/mx6-ddr.h>


static struct mx6ul_iomux_grp_regs mx6_grp_ioregs = {
	.grp_addds = 0x00000030,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_b0ds = 0x00000030,
	.grp_ctlds = 0x00000030,
	.grp_b1ds = 0x00000030,
	.grp_ddrpke = 0x00000000,
	.grp_ddrmode = 0x00020000,
	.grp_ddr_type = 0x000c0000,
};

static struct mx6ul_iomux_ddr_regs mx6_ddr_ioregs = {
	.dram_dqm0 = 0x00000030,
	.dram_dqm1 = 0x00000030,
	.dram_ras = 0x00000030,
	.dram_cas = 0x00000030,
	.dram_odt0 = 0x00000030,
	.dram_odt1 = 0x00000030,
	.dram_sdba2 = 0x00000000,
	.dram_sdclk_0 = 0x00000030,
	.dram_sdqs0 = 0x00000030,
	.dram_sdqs1 = 0x00000030,
	.dram_reset = 0x000C0030,
};

/* TODO: Conduct DDR Stress Test */
static struct mx6_mmdc_calibration dhcom_mmdc_calib_1x1G_800 = {
	.p0_mpwldectrl0 = 0x00000000,
	.p0_mpwldectrl1 = 0x001B001B,
	.p0_mpdgctrl0 = 0x013C013C,
	.p0_mpdgctrl1 = 0x00000000,
	.p0_mprddlctl = 0x40402A34,
	.p0_mpwrdlctl = 0x40403A32,
};

/* Values from DDR Stress Test v2.9.0 */
static struct mx6_mmdc_calibration dhcom_mmdc_calib_1x2G_800 = {
	.p0_mpwldectrl0 = 0x00000000,
	.p0_mpwldectrl1 = 0x001B001B,
	.p0_mpdgctrl0 = 0x013C013C,
	.p0_mpdgctrl1 = 0x00000000,
	.p0_mprddlctl = 0x40402A34,
	.p0_mpwrdlctl = 0x40403A32,
};

/* Values from DDR Stress Test v2.9.0 */
static struct mx6_mmdc_calibration dhcom_mmdc_calib_1x4G_800 = {
	.p0_mpwldectrl0 = 0x00000000,
	.p0_mpwldectrl1 = 0x001F001F,
	.p0_mpdgctrl0 = 0x012B0128,
	.p0_mpdgctrl1 = 0x00000000,
	.p0_mprddlctl = 0x40402832,
	.p0_mpwrdlctl = 0x40403F30,
};

struct mx6_ddr_sysinfo dhcom_ddr_16bit  = {
	/* width of data bus:0=16,1=32,2=64 */
	.dsize = 0,
	.cs_density = 20,
	.ncs = 1,
	.cs1_mirror = 0,
	.rtt_wr = 2,
	.rtt_nom = 1,		/* RTT_Nom = RZQ/2 */
	.walat = 0,		/* Write additional latency */
	.ralat = 5,		/* Read additional latency */
	.mif3_mode = 3,		/* Command prediction working mode */
	.bi_on = 1,		/* Bank interleaving enabled */
	.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
	.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
	.ddr_type = DDR_TYPE_DDR3,
	.refsel = 0,	/* Refresh cycles at 64KHz */
	.refr = 1,	/* 2 refresh commands per refresh cycle */
};

/* TBD */
static struct mx6_ddr3_cfg dhcom_mem_ddr_1G = {
	.mem_speed = 800,
	.density = 1,
	.width = 16,
	.banks = 8,
	.rowaddr = 13,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

/* Nanya Technology NT5CC128M16IP-DII */
static struct mx6_ddr3_cfg dhcom_mem_ddr_2G = {
	.mem_speed = 800,
	.density = 2,
	.width = 16,
	.banks = 8,
	.rowaddr = 14,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

/* Intelligent Memory IM4G16D3FABG-125I */
static struct mx6_ddr3_cfg dhcom_mem_ddr_4G = {
	.mem_speed = 800,
	.density = 4,
	.width = 16,
	.banks = 8,
	.rowaddr = 15,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	writel(0xFFFFFFFF, &ccm->CCGR0);
	writel(0xFFFFFFFF, &ccm->CCGR1);
	writel(0xFFFFFFFF, &ccm->CCGR2);
	writel(0xFFFFFFFF, &ccm->CCGR3);
	writel(0xFFFFFFFF, &ccm->CCGR4);
	writel(0xFFFFFFFF, &ccm->CCGR5);
	writel(0xFFFFFFFF, &ccm->CCGR6);
	writel(0xFFFFFFFF, &ccm->CCGR7);
}

/* Board ID */
static iomux_v3_cfg_t const hwcode_pads[] = {
	/* LGA HW code */
	MX6_PAD_LCD_CLK__GPIO3_IO00	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_LCD_ENABLE__GPIO3_IO01	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
	/* SODIMM HW code */
	MX6_PAD_CSI_DATA00__GPIO4_IO21	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_CSI_DATA01__GPIO4_IO22	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_CSI_DATA02__GPIO4_IO23	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static void setup_iomux_boardid(void)
{
	/* HW code pins: Setup alternate function and configure pads */
	imx_iomux_v3_setup_multiple_pads(hwcode_pads, ARRAY_SIZE(hwcode_pads));
}

/* DDR Code */
static iomux_v3_cfg_t const ddrcode_pads[] = {
	MX6_PAD_LCD_HSYNC__GPIO3_IO02	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_LCD_VSYNC__GPIO3_IO03	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static void setup_iomux_ddrcode(void)
{
	/* ddr code pins */
	imx_iomux_v3_setup_multiple_pads(ddrcode_pads, ARRAY_SIZE(ddrcode_pads));
}

enum dhcom_ddr3_code {
	DH_DDR3_SIZE_128MIB = 0x00,
	DH_DDR3_SIZE_256MIB = 0x01,
	DH_DDR3_SIZE_512MIB = 0x02,
	DH_DDR3_SIZE_1GIB   = 0x03,
};

#define DDR3_CODE_BIT_0   IMX_GPIO_NR(3, 2)
#define DDR3_CODE_BIT_1   IMX_GPIO_NR(3, 3)

enum dhcom_ddr3_code  dhcom_get_ddr3_size(void)
{
	enum dhcom_ddr3_code ddr3_code;

	gpio_request(DDR3_CODE_BIT_0, "DDR3_CODE_BIT_0");
	gpio_request(DDR3_CODE_BIT_1, "DDR3_CODE_BIT_1");

	gpio_direction_input(DDR3_CODE_BIT_0);
	gpio_direction_input(DDR3_CODE_BIT_1);

	/* 128MB = 0b00; 256MB = 0b01; 512MB = 0b10; 1GB = 0b11 */
	ddr3_code =   (!!gpio_get_value(DDR3_CODE_BIT_1) << 1)
	            | (!!gpio_get_value(DDR3_CODE_BIT_0) << 0);

	return ddr3_code;
}

/* ENET */
/*
 * pin conflicts for fec1 and fec2, GPIO1_IO06 and GPIO1_IO07 can only
 * be used for ENET1 or ENET2, cannot be used for both.
 */
static iomux_v3_cfg_t const fec1_pads[] = {
	MX6_PAD_GPIO1_IO06__ENET1_MDIO | MUX_PAD_CTRL(MDIO_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),

	MX6_PAD_ENET1_TX_DATA0__ENET1_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA1__ENET1_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_EN__ENET1_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_CLK__ENET1_REF_CLK1 | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL) | MUX_MODE_SION,

	MX6_PAD_ENET1_RX_DATA0__ENET1_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA1__ENET1_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_ER__ENET1_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_EN__ENET1_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* SMSC PHY Reset */
	MX6_PAD_LCD_DATA18__GPIO3_IO23 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static iomux_v3_cfg_t const fec2_pads[] = {
	MX6_PAD_GPIO1_IO06__ENET2_MDIO | MUX_PAD_CTRL(MDIO_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET2_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),

	MX6_PAD_ENET2_TX_DATA0__ENET2_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA1__ENET2_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_CLK__ENET2_REF_CLK2 | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL) | MUX_MODE_SION,
	MX6_PAD_ENET2_TX_EN__ENET2_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),

	MX6_PAD_ENET2_RX_DATA0__ENET2_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_DATA1__ENET2_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_EN__ENET2_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_ER__ENET2_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* SMSC PHY Reset */
	MX6_PAD_LCD_DATA19__GPIO3_IO24 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static void setup_iomux_fec(int fec_id)
{
	if (fec_id == 0)
		imx_iomux_v3_setup_multiple_pads(fec1_pads,
						 ARRAY_SIZE(fec1_pads));
	else
		imx_iomux_v3_setup_multiple_pads(fec2_pads,
						 ARRAY_SIZE(fec2_pads));
}

/* GPIO */
static iomux_v3_cfg_t const gpio_pads[] = {
	MX6_PAD_SNVS_TAMPER0__GPIO5_IO00 | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* A */
	MX6_PAD_SNVS_TAMPER1__GPIO5_IO01 | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* B */
	MX6_PAD_SNVS_TAMPER2__GPIO5_IO02 | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* C */
	MX6_PAD_SNVS_TAMPER3__GPIO5_IO03 | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* D */
	MX6_PAD_SNVS_TAMPER4__GPIO5_IO04 | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* E */
	MX6_PAD_SNVS_TAMPER7__GPIO5_IO07 | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* F */
	MX6_PAD_SNVS_TAMPER8__GPIO5_IO08 | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* G */
	MX6_PAD_SNVS_TAMPER9__GPIO5_IO09 | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* H */
	MX6_PAD_UART1_CTS_B__GPIO1_IO18  | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* I */
	MX6_PAD_CSI_HSYNC__GPIO4_IO20    | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* J */
	MX6_PAD_CSI_PIXCLK__GPIO4_IO18   | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* K */
	MX6_PAD_CSI_MCLK__GPIO4_IO17     | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* L */
	MX6_PAD_CSI_VSYNC__GPIO4_IO19    | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* M */
	/* CSI_DATA04..07 used for spi boot flash */
//	MX6_PAD_CSI_DATA07__GPIO4_IO28   | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* N */
//	MX6_PAD_CSI_DATA06__GPIO4_IO27   | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* O */
//	MX6_PAD_CSI_DATA05__GPIO4_IO26   | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* P */
//	MX6_PAD_CSI_DATA04__GPIO4_IO25   | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* Q */
	MX6_PAD_CSI_DATA03__GPIO4_IO24   | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* R */
	/* CSI_DATA00..02 are already muxed as GPIO by SODIMM HW code */
//	MX6_PAD_CSI_DATA02__GPIO4_IO23   | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* S */
//	MX6_PAD_CSI_DATA01__GPIO4_IO22   | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* T */
//	MX6_PAD_CSI_DATA00__GPIO4_IO21   | MUX_PAD_CTRL(GPIO_PAD_CTRL), /* U */
	/* not available */                                             /* V */
	/* not available */                                             /* W */
};

static void setup_iomux_gpio(void)
{
	imx_iomux_v3_setup_multiple_pads(gpio_pads, ARRAY_SIZE(gpio_pads));
}

/* SD */
static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),

	/* CD */
	MX6_PAD_UART1_RTS_B__GPIO1_IO19 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static void setup_iomux_sd(void)
{
	imx_iomux_v3_setup_multiple_pads(usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
}

/* SPI */
static iomux_v3_cfg_t const ecspi1_pads[] = {
	/* SS0 via GPIO and function board_spi_cs_gpio() above */
	MX6_PAD_CSI_DATA05__GPIO4_IO26	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
	/* SPI_BOOT_FLASH_EN */
	MX6_PAD_GPIO1_IO09__GPIO1_IO09	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_CSI_DATA07__ECSPI1_MISO	| MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI_DATA06__ECSPI1_MOSI	| MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI_DATA04__ECSPI1_SCLK	| MUX_PAD_CTRL(SPI_PAD_CTRL),
};

static void setup_iomux_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
	gpio_request(SPI_CS, "escpi cs");
	gpio_direction_output(SPI_CS, 0);

	gpio_request(SPI_BOOT_FLASH_EN, "spi boot flash enable");
	gpio_direction_output(SPI_BOOT_FLASH_EN, 0);
}

/* USB */
static iomux_v3_cfg_t const usb_pads[] = {
	MX6_PAD_GPIO1_IO00__ANATOP_OTG1_ID	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_GPIO1_IO05__GPIO1_IO05		| MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static void setup_iomux_usb(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_pads, ARRAY_SIZE(usb_pads));
}

/* DRAM */
/* Setup DRAM, in case of NULL it will use default calibration values */
static void dhcom_setup_dram(struct mx6_mmdc_calibration *calibration)
{
	enum dhcom_ddr3_code ddr3_code = dhcom_get_ddr3_size();

	switch (ddr3_code) {
	default:
		printf("imx6ull: unsupported ddr3 code %d\n", ddr3_code);
		printf("         choosing 128 MB\n");
		/* fall through */
	case DH_DDR3_SIZE_128MIB:
		mx6_dram_cfg(&dhcom_ddr_16bit,
				calibration ? calibration : &dhcom_mmdc_calib_1x1G_800,
				&dhcom_mem_ddr_1G);
		break;
	case DH_DDR3_SIZE_256MIB:
		mx6_dram_cfg(&dhcom_ddr_16bit,
				calibration ? calibration : &dhcom_mmdc_calib_1x2G_800,
				&dhcom_mem_ddr_2G);
		break;
	case DH_DDR3_SIZE_512MIB:
		mx6_dram_cfg(&dhcom_ddr_16bit,
				calibration ? calibration : &dhcom_mmdc_calib_1x4G_800,
				&dhcom_mem_ddr_4G);
		break;
	}
}

static void dhcom_spl_dram_init(void)
{
#if defined(CONFIG_MX6_DDRCAL)
	int errs;
	struct mx6_mmdc_calibration calibration = {0};

	mx6ul_dram_iocfg(16, &mx6_ddr_ioregs, &mx6_grp_ioregs);

	/* Set leveling calibration defaults */
	calibration.p0_mpwrdlctl = 0x40404040;

	dhcom_setup_dram(&calibration);

	/* Perform DRAM auto calibration */
	udelay(100);
	printf("DRAM: Auto calibration...");
	errs = mmdc_do_write_level_calibration(&dhcom_ddr_16bit);
	if (!errs) {
		errs = mmdc_do_dqs_calibration(&dhcom_ddr_16bit);
	}

	if (!errs) {
		printf("successful\n");
#if defined(DEBUG)
		mmdc_read_calibration(&dhcom_ddr_16bit, &calibration);
		printf(" MPDGCTRL0\t= 0x%08X\n", calibration.p0_mpdgctrl0);
		printf(" MPDGCTRL1\t= 0x%08X\n", calibration.p0_mpdgctrl1);
		printf(" MPRDDLCTL\t= 0x%08X\n", calibration.p0_mprddlctl);
		printf(" MPWRDLCTL\t= 0x%08X\n", calibration.p0_mpwrdlctl);
		printf(" MPWLDECTRL0\t= 0x%08X\n", calibration.p0_mpwldectrl0);
		printf(" MPWLDECTRL1\t= 0x%08X\n", calibration.p0_mpwldectrl1);
#endif /* DEBUG */
	} else {
		/* Use default values if auto calibration is failed */
		printf("failed, using default values\n");
		dhcom_setup_dram(NULL);
	}
#else /* CONFIG_MX6_DDRCAL */
	mx6ul_dram_iocfg(16, &mx6_ddr_ioregs, &mx6_grp_ioregs);

	/* Using default calibration values */
	dhcom_setup_dram(NULL);
#endif /* CONFIG_MX6_DDRCAL */
}

void board_init_f(ulong dummy)
{
	ccgr_init();

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* board muxing */
	setup_iomux_boardid();
	setup_iomux_ddrcode();
	setup_iomux_fec(CONFIG_FEC_ENET_DEV);
	setup_iomux_gpio();
	setup_iomux_sd();
	setup_iomux_spi();
	setup_iomux_usb();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* DDR initialization */
	dhcom_spl_dram_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif
