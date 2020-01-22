/* SPDX-License-Identifier: (GPL-2.0-or-later) */
/*
 * DHCOM DH-iMX6ULL PDK board support
 *
 * Copyright (C) 2018 DH electronics GmbH
 */

#ifndef __DH_IMX6ULL_CONFIG_H
#define __DH_IMX6ULL_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/mach-imx/gpio.h>

/*
 * SPI NOR flash / eMMC layout:
 *
 * 0x00_0000-0x00_03ff ...     1.024 ... UNUSED
 * 0x00_0400-0x0f_ffff ... 1.047.552 ... SPL + U-Boot
 *
 * 0x10_0000-0x10_3fff ...    16.384 ... U-Boot env #1
 * 0x10_4000-0x10_ffff ...    49.152     UNUSED
 *
 * 0x11_0000-0x11_3fff ...    16.384 ... U-Boot env #2
 * 0x11_4000-0x11_ffff ...    49.152     UNUSED
 *
 * 0x12_0000-0x1f_ffff ...   917.504     UNUSED
 */

/* SPL options */
#include "imx6_spl.h"			/* common IMX6 SPL configuration */
#define CONFIG_SYS_SPI_SPL_OFFS		0x00400
#define CONFIG_SYS_SPI_U_BOOT_OFFS	0x11400 /* 0x8a * 512 */
#define CONFIG_SPL_TARGET		"u-boot-with-spl-env.imx"

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(4 * SZ_1M)

/* DHCOM GPIOS */
#define DHCOM_GPIO_A	IMX_GPIO_NR(5, 0)
#define DHCOM_GPIO_B	IMX_GPIO_NR(5, 1)
#define DHCOM_GPIO_C	IMX_GPIO_NR(5, 2)
#define DHCOM_GPIO_D	IMX_GPIO_NR(5, 3)
#define DHCOM_GPIO_E	IMX_GPIO_NR(5, 4)
#define DHCOM_GPIO_F	IMX_GPIO_NR(5, 7)
#define DHCOM_GPIO_G	IMX_GPIO_NR(5, 8)
#define DHCOM_GPIO_H	IMX_GPIO_NR(5, 9)
#define DHCOM_GPIO_I	IMX_GPIO_NR(1, 18)
#define DHCOM_GPIO_J	IMX_GPIO_NR(4, 20)
#define DHCOM_GPIO_K	IMX_GPIO_NR(4, 18)
#define DHCOM_GPIO_L	IMX_GPIO_NR(4, 17)
#define DHCOM_GPIO_M	IMX_GPIO_NR(4, 19)
#define DHCOM_GPIO_N	IMX_GPIO_NR(4, 28)
#define DHCOM_GPIO_O	IMX_GPIO_NR(4, 27)
#define DHCOM_GPIO_P	IMX_GPIO_NR(4, 26)
#define DHCOM_GPIO_Q	IMX_GPIO_NR(4, 25)
#define DHCOM_GPIO_R	IMX_GPIO_NR(4, 24)
#define DHCOM_GPIO_S	IMX_GPIO_NR(4, 23)
#define DHCOM_GPIO_T	IMX_GPIO_NR(4, 22)
#define DHCOM_GPIO_U	IMX_GPIO_NR(4, 21)
/* GPIO V is not available */
/* GPIO W is not available */

/* UART */
#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE
#define CONFIG_BAUDRATE			115200

/* I2C configs */
#ifdef CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_SPEED		100000
#endif

/* Watchdog */
#define CONFIG_HW_WATCHDOG
#define CONFIG_IMX_WATCHDOG
#define CONFIG_WATCHDOG_TIMEOUT_MSECS	60000

/* MMC Configs */
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	0
#define MMC_START_INDEX			1
#define CONFIG_SYS_FSL_USDHC_NUM	3 /* 0=dummy, 1=uSD/SD/MMC, 2=eMMC */
#define CONFIG_SUPPORT_EMMC_BOOT	/* eMMC specific */

/* NAND stuff */
#ifdef CONFIG_NAND_MXS
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION
#define CONFIG_SYS_NAND_USE_FLASH_BBT

#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS
#endif

/* SPI Flash Configs */
#ifdef CONFIG_CMD_SF
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED		40000000
#define CONFIG_SF_DEFAULT_MODE		(SPI_MODE_0)
#endif

#ifndef CONFIG_SPL_BUILD
#define EXTRA_ENV_SETTINGS \
	"console=ttymxc0,115200\0" \
	"bootenv_file=uLinuxEnv.txt\0" \
	"bootlinux=if run load_bootenv; then run importbootenv;fi; " \
		"setenv set_rootfs setenv rootfs ${rootfs}; run set_rootfs; " \
		"setenv set_fdt_file setenv fdt_file ${fdt_file}; run set_fdt_file; run load_fdt; " \
		"run load_zimage; run linuxargs; bootz ${loadaddr} - ${fdt_addr};\0" \
	"importbootenv=echo Importing environment from ${bootenv_file}...; env import -t ${loadaddr} ${filesize}\0" \
	"linuxargs=setenv bootargs " \
		"console=${console} ${rootfs} ${mtdparts} fbcon=${fbcon} ${videoargs} ${backlight} ${optargs} " \
		"dhblloc=${dhblloc} dhenvloc=${dhenvloc} dhcom=${dhcom} dhsw=${dhsw} SN=${SN}\0" \
	"fdt_addr=0x83000000\0" \
	"fdt_high=0xffffffff\0" \
	"enable_watchdog_128s=mw.w 20bc000 0xffb7; run serv_watchdog\0" \
	"serv_watchdog=mw.w 0x020bc002 0x5555; mw.w 0x020bc002 0xaaaa\0" \
	"setupdateargs=setenv bootargs " \
		"console=${console} src_intf=${src_intf} src_dev_part=${src_dev_part} ${upd_extra_args} ${mtdparts} " \
		"vt.global_cursor_default=0 consoleblank=0 ${backlight} dhblloc=${dhblloc} dhenvloc=${dhenvloc} dhcom=${dhcom} dhsw=${dhsw} SN=${SN}\0" \
	"load_update_kernel=load ${src_intf} ${src_dev_part} ${loadaddr} zImage_${dhsw}_${dhstoragetype}.update; run setupdateargs; bootz ${loadaddr}\0" \
	"tftp_update=setenv src_intf tftp; setenv src_dev_part ${tftp_path}; setenv upd_extra_args tftp_blocksize=65464; " \
		"if tftp ${loadaddr} ${serverip}:${tftp_path}/zImage_${dhsw}_${dhstoragetype}.update; " \
		"then run setupdateargs; bootz ${loadaddr}; fi;\0" \

#ifndef CONFIG_NAND_MXS /* eMMC default args */

#define CONFIG_EXTRA_ENV_SETTINGS \
	EXTRA_ENV_SETTINGS \
	"load_bootenv=echo Loading u-boot environment ${bootenv_file}...; " \
		"load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${bootenv_file};\0" \
	"load_fdt=echo Loading device tree ${fdt_file}...; " \
		"load mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"load_zimage=echo Loading linux kernel ${zImage_file}...; " \
		"load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${zImage_file}\0" \
	"mmcdev=1\0" \
	"mmcpart=1\0" \
	"mmc_rootfs_part=2\0" \
	""

#define CONFIG_BOOTCOMMAND \
	"update auto; " \
	"if test ${mmcdev} -eq 1; then echo Trying to boot from uSD/SD/MMC card...; fi; " \
	"if test ${mmcdev} -eq 2; then echo Trying to boot from eMMC...; fi; " \
	"mmc dev ${mmcdev}; " \
	"if mmc rescan; then run bootlinux; " \
	"else echo Boot failed, because mmc${mmcdev} not found!; fi;"

#else /* NAND default args */

#define MTDIDS_DEFAULT			"nand0=gpmi-nand"
#define MTDPARTS_PART_NAME		"main-nand"
#define MTDPARTS_DEFAULT		"mtdparts=gpmi-nand:-("MTDPARTS_PART_NAME")"

#define CONFIG_EXTRA_ENV_SETTINGS \
	EXTRA_ENV_SETTINGS \
	"mtdids=nand0=gpmi-nand\0" \
	"mtdparts="MTDPARTS_DEFAULT"\0" \
	"load_bootenv=echo Loading u-boot env file ${bootenv_file}...; " \
		"ubifsload ${loadaddr} ${bootenv_file};\0" \
	"load_fdt=echo Loading device tree ${fdt_file}...; " \
		"ubifsload ${fdt_addr} ${fdt_file}\0" \
	"load_zimage=echo Loading linux ${zImage_file}...; " \
		"ubifsload ${loadaddr} ${zImage_file}\0" \
	""

#define CONFIG_BOOTCOMMAND \
	"update auto; " \
	"if mtdparts; then echo Starting nand boot...; " \
	"else mtdparts default; fi; " \
	"ubi part "MTDPARTS_PART_NAME"; ubifsmount ubi0:boot; run bootlinux"

#endif
#endif

/* Miscellaneous configurable options */
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x8000000)

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment */
#define CONFIG_ENV_SIZE			(16 * 1024)
#define CONFIG_ENV_RESERVED_SIZE	CONFIG_ENV_SECT_SIZE /* Reserved size for one env */
#define CONFIG_SYS_REDUNDAND_ENVIRONMENT
#define CONFIG_ENV_OFFSET		(1024 * 1024)
#define CONFIG_ENV_SECT_SIZE		(64 * 1024)
#define CONFIG_ENV_OFFSET_REDUND	\
	(CONFIG_ENV_OFFSET + CONFIG_ENV_SECT_SIZE)
#define CONFIG_ENV_SIZE_REDUND		CONFIG_ENV_SIZE

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV		2
#define CONFIG_SYS_MMC_ENV_PART		1 /* PART1 = mmcblkXboot0 */
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#endif

#ifndef CONFIG_SYS_DCACHE_OFF
#endif

#ifdef CONFIG_FSL_QSPI
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED	40000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#define FSL_QSPI_FLASH_NUM		1
#define FSL_QSPI_FLASH_SIZE		SZ_32M
#endif

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS   0
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#endif

#ifdef CONFIG_CMD_NET
#define CONFIG_MII
#define CONFIG_FEC_ENET_DEV		0

#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR		0x0
#define CONFIG_FEC_XCV_TYPE		RMII
#elif (CONFIG_FEC_ENET_DEV == 1)
#define IMX_FEC_BASE			ENET2_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR		0x1
#define CONFIG_FEC_XCV_TYPE		RMII
#endif
#define CONFIG_ETHPRIME			"FEC"
#endif

#define CONFIG_IOMUX_LPSR

#ifndef CONFIG_SPL_BUILD
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_MXS
#define CONFIG_VIDEO_LOGO
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
#define MXS_LCDIF_BASE MX6UL_LCDIF1_BASE_ADDR
#endif
#endif

#endif
