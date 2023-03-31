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
#include <nand.h>
#include <fuse.h>

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
#define PMIC_I2C_ADDRESS	0x58

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
#endif /* CONFIG_SYS_I2C_MXC */

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

/*
 * Important: LCD pins are used for lga hardware code. So call this function
 *            once before setup_lcd() to save the value into a static variable.
 */
int board_get_lga_hwcode(void)
{
	static int hw_code = (-1);

	if ( hw_code == (-1) ) {
		gpio_direction_input(LGA_HW_CODE_BIT_0);
		gpio_direction_input(LGA_HW_CODE_BIT_1);

		/* HW 100 = 0b00; HW 200 = 0b01; HW 300 = 0b10; HW 400 = 0b11 */
		hw_code = ( (gpio_get_value(LGA_HW_CODE_BIT_1) << 1) |
			    (gpio_get_value(LGA_HW_CODE_BIT_0) << 0)  ) + 1;
	}

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

	/* HW 100 = 0b00; HW 200 = 0b01; HW 300 = 0b10; HW 400 = 0b11; ... */
	hw_code = ( (gpio_get_value(SODIMM_HW_CODE_BIT_2) << 2) |
		    (gpio_get_value(SODIMM_HW_CODE_BIT_1) << 1) |
		    (gpio_get_value(SODIMM_HW_CODE_BIT_0) << 0)  ) + 1;

	return hw_code;
}

#ifdef CONFIG_NAND_MXS
/* NAND (on module) */
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
#else /* CONFIG_NAND_MXS */
#ifdef CONFIG_FSL_ESDHC
/* eMMC (on module) */
static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_NAND_RE_B__USDHC2_CLK		| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_WE_B__USDHC2_CMD		| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA00__USDHC2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA01__USDHC2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA02__USDHC2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA03__USDHC2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA04__USDHC2_DATA4	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA05__USDHC2_DATA5	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA06__USDHC2_DATA6	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA07__USDHC2_DATA7	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_ALE__USDHC2_RESET_B	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static void setup_emmc(void)
{
	imx_iomux_v3_setup_multiple_pads(usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
}
#endif /* CONFIG_FSL_ESDHC */
#endif /* CONFIG_NAND_MXS */

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[CONFIG_SYS_FSL_USDHC_NUM] = {
	{0},
	{USDHC1_BASE_ADDR}, /* uSD card (on module) or uSD/SD/MMC card interface (external) */
#ifdef CONFIG_NAND_MXS
	{0},
#else
	{USDHC2_BASE_ADDR}, /* eMMC (on module) */
#endif
};

#define USDHC1_CD_GPIO	IMX_GPIO_NR(1, 19)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		/*
		 * uSD card (on module) or uSD/SD/MMC card interface (external)
		 * card detect is active low
		 */
		ret = !gpio_get_value(USDHC1_CD_GPIO);
		if (ret)
			printf("SD card detected\n");
		return ret;
	case USDHC2_BASE_ADDR:
		/*
		 * eMMC (on module)
		 * card detect is always present
		 */
		return 1;
	}

	return 0;
}

int board_mmc_init(bd_t *bis)
{
	int i, ret;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    not available
	 * mmc1                    USDHC1
	 * mmc2                    USDHC2
	 */

	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			continue;
		case 1:
			/* uSD card (on module) or uSD/SD/MMC card interface (external) */
			usdhc_cfg[i].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			usdhc_cfg[i].max_bus_width = 4;
			gpio_request(USDHC1_CD_GPIO, "usdhc1_cd");
			gpio_direction_input(USDHC1_CD_GPIO);
			break;
		case 2:
#ifdef CONFIG_NAND_MXS
			continue;
#else
			/* eMMC (on module) */
			usdhc_cfg[i].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			usdhc_cfg[i].max_bus_width = 8;
			break;
#endif /* CONFIG_NAND_MXS */
		default:
			printf("Warning: you configured more USDHC controllers (%d) than supported by the board\n", i + 1);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret) {
			printf("Warning: failed to initialize mmc dev %d\n", i);
			return ret;
		}
	}

	return 0;
}
#endif /* CONFIG_FSL_ESDHC */

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
#endif /* CONFIG_USB_EHCI_MX6 */

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
	/*
	 * On the i.MX6ULL VIO doesn't need to be enabled,
	 * because this is directly connected to the supply.
	 */

	/* Reset PHY */
	if (fec_id == 0)
		gpio_direction_output(IMX_GPIO_NR(3, 23) , 0);
	else
		gpio_direction_output(IMX_GPIO_NR(3, 24) , 0);
	udelay(500);
	if (fec_id == 0)
		gpio_direction_output(IMX_GPIO_NR(3, 23) , 1);
	else
		gpio_direction_output(IMX_GPIO_NR(3, 24) , 1);
	udelay(500);
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
		printf("Net:   FEC%d: Save MAC %02X:%02X:%02X:%02X:%02X:%02X "
		       "from fuse to env \"%s\"\n",
		       fec_id,
		       enetaddr[0], enetaddr[1], enetaddr[2],
		       enetaddr[3], enetaddr[4], enetaddr[5],
		       env_name);
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
		printf("Net:   FEC%d: EEPROM 0x%02X not available\n",
		       fec_id, eeprom_i2c_addr);
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
#endif /* CONFIG_FEC_MXC */

#define DA9061_PAGE_CON				0x000
#define DA9061_CONTROL_D			0x011
#define DA9061_BUCK1_CFG			0x09E
#define DA9061_BUCK2_CFG			0x0A0
#define DA9061_BUCK3_CFG			0x09F
#define DA9061_CONFIG_C				0x108
#define DA9061_CONFIG_H				0x10D
#define DA9061_VARIANT_ID			0x182
#define DA9061_CONFIG_C_BUCK_ACTV_DISCHRG	0x04
#define DA9061_CONFIG_C_BUCK1_CLK_INV		0x08
#define DA9061_CONFIG_C_BUCK2_CLK_INV		0x40
#define DA9061_CONFIG_C_BUCK3_CLK_INV		0x10
#define DA9061_BUCKX_CFG_PFM			0x40
#define DA9061_BUCKX_CFG_PWM			0x80
#define DA9061_BUCKX_CFG_AUTO			0xC0
#define DA9061_BUCKX_CFG_MASK			0xC0
static int da9061_read(int reg, unsigned char *val)
{
	int ret;
	unsigned char page_con;

	/* Adjust PAGE_CON if necessary */
	if (reg > 0xFF) {
		page_con = 0x02;
		ret = i2c_write(PMIC_I2C_ADDRESS, DA9061_PAGE_CON, 1, &page_con, 1);
		if (ret)
			return ret;
	}

	ret = i2c_read(PMIC_I2C_ADDRESS, reg, 1, val, 1);
	if (ret)
		return ret;

	/* Reset PAGE_CON, if necessary */
	if (reg > 0xFF) {
		page_con = 0x00;
		ret = i2c_write(PMIC_I2C_ADDRESS, DA9061_PAGE_CON, 1, &page_con, 1);
		if (ret)
			return ret;
	}

	return 0;
}

static int da9061_write(int reg, unsigned char val)
{
	int ret;
	unsigned char page_con;

	/* Adjust PAGE_CON if necessary */
	if (reg > 0xFF) {
		val &= 0xFF;
		page_con = 0x02;
		ret = i2c_write(PMIC_I2C_ADDRESS, DA9061_PAGE_CON, 1, &page_con, 1);
		if (ret)
			return ret;
	}

	ret = i2c_write(PMIC_I2C_ADDRESS, reg, 1, &val, 1);
	if (ret)
		return ret;

	/* Reset PAGE_CON, if necessary */
	if (reg > 0xFF) {
		page_con = 0x00;
		ret = i2c_write(PMIC_I2C_ADDRESS, DA9061_PAGE_CON, 1, &page_con, 1);
		if (ret)
			return ret;
	}

	return 0;
}

static void pmic_adjustments(void)
{
#ifdef CONFIG_SYS_I2C_MXC
	int ret, ret_b1, ret_b2, ret_b3;
	unsigned char val;
	char *var;
	const char *buck_cfg;
	const char *config_c;
	int value;
	const char buck_cfg_str[][7] = {"SL_A/B", "PFM", "PWM", "AUTO"};

	ret = i2c_set_bus_num(0);
	if (ret) {
		printf("Error switching I2C bus!\n");
		return;
	}

#if defined(DEBUG)
	#define REG 0x011
	da9061_read(REG, &val); printf("PMIC:  [0x%03X]=0x%02X\n", REG, val);
	#undef REG
	#define REG 0x013
	da9061_read(REG, &val); printf("PMIC:  [0x%03X]=0x%02X\n", REG, val);
	#undef REG
	#define REG 0x09E
	da9061_read(REG, &val); printf("PMIC:  [0x%03X]=0x%02X\n", REG, val);
	#undef REG
	#define REG 0x0A0
	da9061_read(REG, &val); printf("PMIC:  [0x%03X]=0x%02X\n", REG, val);
	#undef REG
	#define REG 0x09F
	da9061_read(REG, &val); printf("PMIC:  [0x%03X]=0x%02X\n", REG, val);
	#undef REG
	#define REG 0x108
	da9061_read(REG, &val); printf("PMIC:  [0x%03X]=0x%02X\n", REG, val);
	#undef REG
	#define REG 0x10D
	da9061_read(REG, &val); printf("PMIC:  [0x%03X]=0x%02X\n", REG, val);
	#undef REG
	#define REG 0x10E
	da9061_read(REG, &val); printf("PMIC:  [0x%03X]=0x%02X\n", REG, val);
	#undef REG
	#define REG 0x112
	da9061_read(REG, &val); printf("PMIC:  [0x%03X]=0x%02X\n", REG, val);
	#undef REG
	#define REG 0x182
	da9061_read(REG, &val); printf("PMIC:  [0x%03X]=0x%02X\n", REG, val);
	#undef REG
#endif

	/* PMIC Variant */
	ret = da9061_read(DA9061_VARIANT_ID, &val);
	if (ret == 0) {
		val = (val & 0xF0) >> 4;
		switch (val) {
		case 1:
			var = "-AA";
			break;
		case 2:
			var = "-AB";
			break;
		case 3:
			var = "-BA";
			break;
		default:
			var = "";
			break;
		}
		printf("PMIC:  DA9061%s\n", var);
	}

	/* PMIC CONTROL_D (watchdog) */
	ret = da9061_write(DA9061_CONTROL_D, 0x00);
	if (ret == 0)
		printf("PMIC:  Disabled WDT\n");

	/* PMIC BUCKX_CFG */
	value = DA9061_BUCKX_CFG_PFM << 16 | /* BUCK1 */
		DA9061_BUCKX_CFG_PWM << 8  | /* BUCK2 */
		DA9061_BUCKX_CFG_PWM << 0;   /* BUCK3 */
	buck_cfg = env_get("pmic_buck_cfg"); /* Check for overwriting by environment */
	if (buck_cfg != NULL)
		value = simple_strtol(buck_cfg, NULL, 16);
	ret_b1 = da9061_write(DA9061_BUCK1_CFG, (value >> 16) & 0xFF);
	ret_b2 = da9061_write(DA9061_BUCK2_CFG, (value >>  8) & 0xFF);
	ret_b3 = da9061_write(DA9061_BUCK3_CFG, (value >>  0) & 0xFF);
	if ((ret_b1 == 0) && (ret_b2 == 0) && (ret_b3 == 0))
		printf("PMIC:  BUCK_CFG=0x%06X (BUCK1=%s, BUCK2=%s, BUCK3=%s)\n",
		       value,
		       buck_cfg_str[((value >> 16) & DA9061_BUCKX_CFG_MASK) >> 6],
		       buck_cfg_str[((value >>  8) & DA9061_BUCKX_CFG_MASK) >> 6],
		       buck_cfg_str[((value >>  0) & DA9061_BUCKX_CFG_MASK) >> 6]);

	/* PMIC CONFIG_C (BUCK config) */
	value = DA9061_CONFIG_C_BUCK2_CLK_INV |
		DA9061_CONFIG_C_BUCK_ACTV_DISCHRG;
	config_c = env_get("pmic_config_c"); /* Check for overwriting by environment */
	if (config_c != NULL)
		value = simple_strtol(config_c, NULL, 16);
	ret = da9061_write(DA9061_CONFIG_C, value);
	if (ret == 0)
		printf("PMIC:  CONFIG_C=0x%02X (BUCK1=%s, BUCK2=%s, BUCK3=%s, ACTV_DISCHRG=%s)\n",
		       value,
		       (value & DA9061_CONFIG_C_BUCK1_CLK_INV) ? "INV" : "NORM",
		       (value & DA9061_CONFIG_C_BUCK2_CLK_INV) ? "INV" : "NORM",
		       (value & DA9061_CONFIG_C_BUCK3_CLK_INV) ? "INV" : "NORM",
		       (value & DA9061_CONFIG_C_BUCK_ACTV_DISCHRG) ? "ON" : "OFF");

	/* PMIC CONFIG_H (BUCK current mode) */
	ret = da9061_write(DA9061_CONFIG_H, 0x00);
	if (ret == 0)
		printf("PMIC:  Enable half-current mode\n");
#endif
}

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

	/* Use GPIO for Brightness adjustment, duty cycle = period. */
	MX6_PAD_GPIO1_IO08__GPIO1_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static int setup_lcd(void)
{
	enable_lcdif_clock(LCDIF1_BASE_ADDR, 1);

	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

	/* Set Brightness to high */
	gpio_direction_output(IMX_GPIO_NR(1, 8) , 1);

	return 0;
}
#endif /* CONFIG_VIDEO_MXS */

static void handle_hw_revision(void)
{
	u32 cpu_sn_high;
	u32 cpu_sn_low;
	u32 lga_hw_code;
	u32 sodimm_hw_code;
	u32 sw_compatibility;
	char buf[24];
	const char *env_value;

	fuse_sense(0, 1, &cpu_sn_low);
	fuse_sense(0, 2, &cpu_sn_high);
	snprintf(buf, sizeof(buf), "%08X%08X", cpu_sn_high, cpu_sn_low);
	printf("CPUSN: %s\n", buf);
	env_set("serial#", buf);

	lga_hw_code = board_get_lga_hwcode();
	sodimm_hw_code = board_get_sodimm_hwcode();
	printf("HW:    LGA=HW%d00, SODIMM=HW%d00\n", lga_hw_code, sodimm_hw_code);

	snprintf(buf, sizeof(buf), "imx6ull-dhcom%1d%1d", lga_hw_code, sodimm_hw_code);
	env_set("dhcom", buf);

	env_value = env_get("SN");
	printf("SN:    %s\n", env_value != NULL ? env_value : "not available");

	env_value = env_get("PSN");
	if (env_value != NULL)
		printf("PSN:   %s\n", env_value);

	/*
	 * If software (device tree) must be changed cased by hardware modifications
	 * derive next sw_compatibility from lga_hw_code and sodimm_hw_code.
	 * First version starts with a fixed value of 1.
	 */
	sw_compatibility = 1;
	snprintf(buf, sizeof(buf), "imx6ull-v%1d", sw_compatibility);
	env_set("dhsw", buf);
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
#endif /* CONFIG_MXC_SPI */

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
#else
#ifdef CONFIG_FSL_ESDHC
	setup_emmc();
#endif
#endif /* CONFIG_NAND_MXS */

#ifdef CONFIG_FEC_MXC
	setup_fec(CONFIG_FEC_ENET_DEV);
#endif

	/* Read hw code into static variable */
	board_get_lga_hwcode();

#ifdef CONFIG_VIDEO_MXS
	setup_lcd();
#endif

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	{"spi", MAKE_CFGVAL(0x30, 0x00, 0x00, 0x08)},
	{"emmc", MAKE_CFGVAL(0x60, 0x48, 0x00, 0x00)},
	{"usd", MAKE_CFGVAL(0x40, 0x20, 0x00, 0x00)},
	{NULL,	 0},
};
#endif /* CONFIG_CMD_BMODE */

int board_late_init(void)
{
	pmic_adjustments();

#ifdef CONFIG_FEC_MXC
	setup_dhcom_mac_from_fuse(0, "ethaddr");
	setup_dhcom_mac_from_fuse(1, "eth1addr");
#endif

#ifdef CONFIG_NAND_MXS
	if (nand_size() != 0) {
		env_set("dhblloc", "spiflash");
		env_set("dhenvloc", "spiflash");
		env_set("dhstoragetype", "nand");
	} else {
		printf("NAND:  not available => Assuming eMMC for Update Kernel\n");
		env_set("dhblloc", "emmc");
		env_set("dhenvloc", "emmc");
		env_set("dhstoragetype", "emmc");
	}
#else /* CONFIG_NAND_MXS */
#ifdef CONFIG_FSL_ESDHC
	env_set("dhblloc", "emmc");
	env_set("dhenvloc", "emmc");
	env_set("dhstoragetype", "emmc");
#endif /* CONFIG_FSL_ESDHC */
#endif /* CONFIG_NAND_MXS */

	handle_hw_revision();

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

/* Preparations before starting the linux kernel with bootm */
int board_prep_linux(bootm_headers_t *images)
{
	/* Set SPI CS as input, because pin is used as DHCOM GPIO P */
	gpio_direction_input(SPI_CS);
#ifdef CONFIG_FEC_MXC
	eth_phy_reset(CONFIG_FEC_ENET_DEV);
#endif
	return 0;
}

/* Preparations before starting the linux kernel with bootz */
int bootz_board_prep_linux()
{
	/* Set SPI CS as input, because pin is used as DHCOM GPIO P */
	gpio_direction_input(SPI_CS);
#ifdef CONFIG_FEC_MXC
	eth_phy_reset(CONFIG_FEC_ENET_DEV);
#endif
	return 0;
}

#ifdef CONFIG_SPL_BUILD
#include <linux/libfdt.h>
#include <spl.h>
#include <asm/arch/mx6-ddr.h>
#include <fuse.h>

/* For consistency use the same order as the DHCOM i.MX6 */
#define DH_BOOT_DEVICE_SPI	BOOT_DEVICE_SPI
#define DH_BOOT_DEVICE_SD	BOOT_DEVICE_MMC1   /* mmc0, but isn't available on i.MX6ULL */
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
	 * - Default order = SPI, eMMC, uSD
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
		printf(", device=ECSPI%d:SS%d (set order: SPI, uSD)\n", index, cs);
		spl_boot_list[0] = DH_BOOT_DEVICE_SPI;
		spl_boot_list[1] = DH_BOOT_DEVICE_USD;
		break;
	case BOOT_DEVICE_MMC1: /* SD, eSD, MMC, eMMC */
		reg = imx6_src_get_boot_mode();
		index = ((reg & IMX6_BMODE_USDHC_MASK) >> IMX6_BMODE_USDHC_SHIFT) + 1;
		printf(", device=uSDHC%d", index);
		if (index == 1) { /* Could only boot from uSDHC1 */
			printf("\n");
			spl_boot_list[0] = DH_BOOT_DEVICE_USD;
		} else {
			printf(" (unsupported => set order: SPI, uSD)\n");
			spl_boot_list[0] = DH_BOOT_DEVICE_SPI;
			spl_boot_list[1] = DH_BOOT_DEVICE_USD;
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
		break;
	}
#else /* CONFIG_NAND_MXS */
#ifdef CONFIG_FSL_ESDHC
	switch (boot_dev) {
	case BOOT_DEVICE_SPI: /* SERIAL_ROM SPI */
		reg = imx6_src_get_boot_mode();
		index = ((reg & IMX6_BMODE_SERIAL_ROM_MASK) >> IMX6_BMODE_SERIAL_ROM_SHIFT) + 1;
		cs = ((reg & IMX6_BMODE_SERIAL_ROM_CS_MASK) >> IMX6_BMODE_SERIAL_ROM_CS_SHIFT);
		printf(", device=ECSPI%d:SS%d (set order: SPI, eMMC, uSD)\n", index, cs);
		spl_boot_list[0] = DH_BOOT_DEVICE_SPI;
		spl_boot_list[1] = DH_BOOT_DEVICE_EMMC;
		spl_boot_list[2] = DH_BOOT_DEVICE_USD;
		break;
	case BOOT_DEVICE_MMC1: /* SD, eSD, MMC, eMMC */
		reg = imx6_src_get_boot_mode();
		index = ((reg & IMX6_BMODE_USDHC_MASK) >> IMX6_BMODE_USDHC_SHIFT) + 1;
		printf(", device=uSDHC%d", index);
		switch (index) {
		case 1: /* uSDHC1 */
			printf("\n");
			spl_boot_list[0] = DH_BOOT_DEVICE_USD;
			break;
		case 2: /* uSDHC2 */
			printf(" (set order: eMMC, SPI, uSD)\n");
			spl_boot_list[0] = DH_BOOT_DEVICE_EMMC;
			spl_boot_list[1] = DH_BOOT_DEVICE_SPI;
			spl_boot_list[2] = DH_BOOT_DEVICE_USD;
			break;
		default:
			printf(" (unsupported => set order: SPI, eMMC, uSD)\n");
			spl_boot_list[0] = DH_BOOT_DEVICE_SPI;
			spl_boot_list[1] = DH_BOOT_DEVICE_EMMC;
			spl_boot_list[2] = DH_BOOT_DEVICE_USD;
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
	.p0_mpdgctrl0 = 0x011C011C,
	.p0_mpdgctrl1 = 0x00000000,
	.p0_mprddlctl = 0x40402831,
	.p0_mpwrdlctl = 0x40403B2D,
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
	.refr = 3,	/* 4 refresh commands per refresh cycle */
};

/*
 * DDR3 128MB:
 * TBD
 * Values are taken from JEDEC DDR3-800E
 */
static struct mx6_ddr3_cfg dhcom_mem_ddr_1G = {
	.mem_speed = 800,
	.density = 1,
	.width = 16,
	.banks = 8,
	.rowaddr = 13,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1500,
	.trcmin = 5250,
	.trasmin = 3750,
};

/*
 * DDR3 256MB:
 * Nanya Technology NT5CC128M16IP-DII
 * Values are taken from JEDEC DDR3-800E
 */
static struct mx6_ddr3_cfg dhcom_mem_ddr_2G = {
	.mem_speed = 800,
	.density = 2,
	.width = 16,
	.banks = 8,
	.rowaddr = 14,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1500,
	.trcmin = 5250,
	.trasmin = 3750,
};

/*
 * DDR3 512MB:
 * Intelligent Memory IM4G16D3FABG-125(I) / IM4G16D3FDBG-107(I)
 * Values are taken from JEDEC DDR3-800E
 */
static struct mx6_ddr3_cfg dhcom_mem_ddr_4G = {
	.mem_speed = 800,
	.density = 4,
	.width = 16,
	.banks = 8,
	.rowaddr = 15,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1500,
	.trcmin = 5250,
	.trasmin = 3750,
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

#ifdef CONFIG_FEC_MXC
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
#endif /* CONFIG_FEC_MXC */

/* Unused uart pins */
static iomux_v3_cfg_t const uart1_rtscts_pads[] = {
	MX6_PAD_UART1_RTS_B__GPIO1_IO19		| MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_UART1_CTS_B__GPIO1_IO18		| MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static iomux_v3_cfg_t const uart2_pads[] = {
	MX6_PAD_UART2_TX_DATA__GPIO1_IO20	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_UART2_RX_DATA__GPIO1_IO21	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_UART3_RX_DATA__GPIO1_IO25	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_UART3_TX_DATA__GPIO1_IO24	| MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static void setup_iomux_unused_uart_pins_as_gpio_input(void)
{
	/*
	 * For now uart1 rts/cts is muxed as gpios input, because if used as
	 * rs485 it would not enable Tx to the bus and therefore not block it.
	 * Later uart1 rts will be used as SD card detect, uart1 cts as GPIO I.
	 */
	imx_iomux_v3_setup_multiple_pads(uart1_rtscts_pads, ARRAY_SIZE(uart1_rtscts_pads));
	gpio_direction_input(IMX_GPIO_NR(1, 19));
	gpio_direction_input(IMX_GPIO_NR(1, 18));

	/* All uart2 pins are muxed as gpio input to avoid blocking if used as rs485 */
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
	gpio_direction_input(IMX_GPIO_NR(1, 20));
	gpio_direction_input(IMX_GPIO_NR(1, 21));
	gpio_direction_input(IMX_GPIO_NR(1, 25));
	gpio_direction_input(IMX_GPIO_NR(1, 24));
}

/* GPIO */
static iomux_v3_cfg_t const gpio_pads[] = {
	MX6_PAD_SNVS_TAMPER0__GPIO5_IO00 | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* A */
	MX6_PAD_SNVS_TAMPER1__GPIO5_IO01 | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* B */
	MX6_PAD_SNVS_TAMPER2__GPIO5_IO02 | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* C */
	MX6_PAD_SNVS_TAMPER3__GPIO5_IO03 | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* D */
	MX6_PAD_SNVS_TAMPER4__GPIO5_IO04 | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* E */
	MX6_PAD_SNVS_TAMPER7__GPIO5_IO07 | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* F */
	MX6_PAD_SNVS_TAMPER8__GPIO5_IO08 | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* G */
	MX6_PAD_SNVS_TAMPER9__GPIO5_IO09 | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* H */
	MX6_PAD_UART1_CTS_B__GPIO1_IO18  | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* I */
	MX6_PAD_CSI_HSYNC__GPIO4_IO20    | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* J */
	MX6_PAD_CSI_PIXCLK__GPIO4_IO18   | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* K */
	MX6_PAD_CSI_MCLK__GPIO4_IO17     | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* L */
	MX6_PAD_CSI_VSYNC__GPIO4_IO19    | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* M */
	/* CSI_DATA04..07 used for spi boot flash */
//	MX6_PAD_CSI_DATA07__GPIO4_IO28   | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* N */
//	MX6_PAD_CSI_DATA06__GPIO4_IO27   | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* O */
//	MX6_PAD_CSI_DATA05__GPIO4_IO26   | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* P */
//	MX6_PAD_CSI_DATA04__GPIO4_IO25   | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* Q */
	MX6_PAD_CSI_DATA03__GPIO4_IO24   | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* R */
	/* CSI_DATA00..02 are already muxed as GPIO by SODIMM HW code */
//	MX6_PAD_CSI_DATA02__GPIO4_IO23   | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* S */
//	MX6_PAD_CSI_DATA01__GPIO4_IO22   | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* T */
//	MX6_PAD_CSI_DATA00__GPIO4_IO21   | MUX_PAD_CTRL(GPIO_PAD_CTRL) | MUX_MODE_SION, /* U */
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

#define SNVS_LPSR		0x020CC04C
#define SNVS_LPPGDR		0x020CC064
#define SNVS_LPPGDR_INIT	0x41736166
static void imx6ull_snvs_lp_init(void)
{
	/* Initialize glitch detect */
	writel(SNVS_LPPGDR_INIT, (void *)SNVS_LPPGDR);

	/* Clear interrupt status */
	writel(0xFFFFFFFF, (void *)SNVS_LPSR);
}

void board_init_f(ulong dummy)
{
	ccgr_init();

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	/* iomux and setup of i2c */
	board_early_init_f();
	setup_iomux_unused_uart_pins_as_gpio_input();

	/* setup GP timer */
	timer_init();

	/* board muxing */
	setup_iomux_boardid();
	setup_iomux_ddrcode();
#ifdef CONFIG_FEC_MXC
	setup_iomux_fec(CONFIG_FEC_ENET_DEV);
#endif
	setup_iomux_gpio();
	setup_iomux_sd();
	setup_iomux_spi();
	setup_iomux_usb();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* DDR initialization */
	dhcom_spl_dram_init();

	/* Init i.MX6ULL SNVS_LP (for writing to SNVS_LPGPR) */
	imx6ull_snvs_lp_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif /* CONFIG_SPL_BUILD */
