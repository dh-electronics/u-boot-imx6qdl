// SPDX-License-Identifier: GPL-2.0+
/*
 * DHCOM DH-iMX6 PDK board support
 *
 * Copyright (C) 2022 Christoph Niedermaier <cniedermaier@dh-electronics.com>
 */

#include <common.h>
#include <asm/arch/clock.h>
#include <asm/gpio.h>
#include <fsl_esdhc.h>

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
