/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * DHCOM DH-iMX6 PDK board configuration
 *
 * Copyright (C) 2017 Marek Vasut <marex@denx.de>
 */

#ifndef __DH_IMX6_CONFIG_H
#define __DH_IMX6_CONFIG_H

#include <asm/arch/imx-regs.h>

#include "mx6_common.h"

/*
 * SPI NOR layout:
 * 0x00_0000-0x00_ffff ... U-Boot SPL
 * 0x01_0000-0x0f_ffff ... U-Boot
 * 0x10_0000-0x10_ffff ... U-Boot env #1
 * 0x11_0000-0x11_ffff ... U-Boot env #2
 * 0x12_0000-0x1f_ffff ... UNUSED
 */

/* SPL */
#include "imx6_spl.h"			/* common IMX6 SPL configuration */
#define CONFIG_SYS_SPI_SPL_OFFS		0x00400
#define CONFIG_SYS_SPI_U_BOOT_OFFS	0x11400
#define CONFIG_SPL_TARGET		"u-boot-with-spl.imx"

/* Miscellaneous configurable options */

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#define CONFIG_BOUNCE_BUFFER
#define CONFIG_BZIP2

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(4 * SZ_1M)

/* Bootcounter */
#define CONFIG_SYS_BOOTCOUNT_BE

/* FEC ethernet */
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		0
#define CONFIG_ARP_TIMEOUT		200UL

/* Fuses */
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

/* DHCOM GPIOS */
#define DHCOM_GPIO_A	IMX_GPIO_NR(1, 2)
#define DHCOM_GPIO_B	IMX_GPIO_NR(1, 4)
#define DHCOM_GPIO_C	IMX_GPIO_NR(1, 5)
#define DHCOM_GPIO_D	IMX_GPIO_NR(6, 3)
#define DHCOM_GPIO_E	IMX_GPIO_NR(4, 5)
#define DHCOM_GPIO_F	IMX_GPIO_NR(4, 20)
#define DHCOM_GPIO_G	IMX_GPIO_NR(3, 27)
#define DHCOM_GPIO_H	IMX_GPIO_NR(4, 7)
#define DHCOM_GPIO_I	IMX_GPIO_NR(4, 8)
#define DHCOM_GPIO_J	IMX_GPIO_NR(6, 14)
#define DHCOM_GPIO_K	IMX_GPIO_NR(6, 15)
#define DHCOM_GPIO_L	IMX_GPIO_NR(4, 9)
#define DHCOM_GPIO_M	IMX_GPIO_NR(7, 0)
#define DHCOM_GPIO_N	IMX_GPIO_NR(7, 1)
#define DHCOM_GPIO_O	IMX_GPIO_NR(5, 21)
#define DHCOM_GPIO_P	IMX_GPIO_NR(7, 13)
#define DHCOM_GPIO_Q	IMX_GPIO_NR(1, 18)
#define DHCOM_GPIO_R	IMX_GPIO_NR(1, 16)
#define DHCOM_GPIO_S	IMX_GPIO_NR(1, 17)
#define DHCOM_GPIO_T	IMX_GPIO_NR(1, 19)
#define DHCOM_GPIO_U	IMX_GPIO_NR(1, 20)
#define DHCOM_GPIO_V	IMX_GPIO_NR(5, 18)
#define DHCOM_GPIO_W	IMX_GPIO_NR(5, 19)

/* GPIO for backlight pwm (init in gpio mode) */
#define PWM_BACKLIGHT_GP IMX_GPIO_NR(1, 21)

/* PCI express */
/* call imx_pcie_remove() on reboot to avoid hang on watchdog reset */
#define CONFIG_PCIE_IMX

/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		100000

/* RTC Configs */
#ifdef CONFIG_RTC_RV3029
#define CONFIG_SYS_RTC_BUS_NUM         2
#define CONFIG_SYS_I2C_RTC_ADDR                0x56
#endif

/* MMC Configs */
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	0
#define CONFIG_SYS_FSL_USDHC_NUM	3
#define CONFIG_SYS_MMC_ENV_DEV		2 /* 1 = SDHC3, 2 = SDHC4 (eMMC) */

/* NAND stuff */
#ifdef CONFIG_NAND_MXS
#define CONFIG_SYS_MAX_NAND_DEVICE     1
#define CONFIG_SYS_NAND_BASE           0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION

#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS
#endif

/* SATA Configs */
#ifdef CONFIG_CMD_SATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#endif

/* SPI Flash Configs */
#ifdef CONFIG_CMD_SF
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED		25000000
#define CONFIG_SF_DEFAULT_MODE		(SPI_MODE_0)
#endif

/* UART */
#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE
#define CONFIG_BAUDRATE			115200

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2 /* Enabled USB controller number */

/* USB Gadget (DFU, UMS) */
#if defined(CONFIG_CMD_DFU) || defined(CONFIG_CMD_USB_MASS_STORAGE)
#define CONFIG_SYS_DFU_DATA_BUF_SIZE	(16 * 1024 * 1024)
#define DFU_DEFAULT_POLL_TIMEOUT	300

/* USB IDs */
#define CONFIG_G_DNL_UMS_VENDOR_NUM	0x0525
#define CONFIG_G_DNL_UMS_PRODUCT_NUM	0xA4A5
#endif
#endif

/* Watchdog */
#define CONFIG_HW_WATCHDOG
#define CONFIG_IMX_WATCHDOG
#define CONFIG_WATCHDOG_TIMEOUT_MSECS	60000

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE

#define CONFIG_LOADADDR			0x12000000
#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

#ifndef CONFIG_SPL_BUILD
#define EXTRA_ENV_SETTINGS	\
	"usb_pgood_delay=1000\0" \
	"panel=no_panel\0" 		\
	"console=ttymxc0,115200\0"	\
	"splashimage=0x10000002\0" 	\
	"splashpos=m,m\0" 		\
	"settings_bin_file=settings.bin\0" \
	"splash_file=splash.bmp\0" 	\
	"setupdateargs=setenv bootargs "\
	        "console=${console} src_intf=${src_intf} src_dev_part=${src_dev_part} dhcom=${dhcom} " \
	        "${backlight} ${parallel_display} ${lvds_display0} ${lvds_display1} SN=${SN} PSN=${PSN} vt.global_cursor_default=0\0" \
	"load_update_kernel=load ${src_intf} ${src_dev_part} ${loadaddr} zImage_${dhcom}.update; run setupdateargs; bootz ${loadaddr}\0" \
	"bootenv_file=uLinuxEnv.txt\0" 	\
	"bootlinux=if run load_bootenv; then run importbootenv;fi;" \
                " setenv set_rootfs setenv rootfs ${rootfs}; run set_rootfs;" \
                " setenv set_fdt_file setenv fdt_file ${fdt_file}; run set_fdt_file; run load_fdt;" \
                " run load_zimage; run linuxargs; bootz ${loadaddr} - ${fdt_addr};\0" \
	"importbootenv=echo Importing environment from ${bootenv_file}...; env import -t ${loadaddr} ${filesize}\0" \
	"linuxargs=setenv bootargs " 	\
	        "console=${console} ${rootfs} fbcon=${fbcon} ${videoargs} ${optargs} dhcom=${dhcom} " \
	        "${backlight} ${parallel_display} ${lvds_display0} ${lvds_display1} SN=${SN} PSN=${PSN}\0" \
	"fdt_addr=0x18000000\0"		\
	"fdt_high=0xffffffff\0"		\
	"enable_watchdog_128s=mw.w 20bc000 0xffb7; run serv_watchdog\0" \
	"serv_watchdog=mw.w 0x020bc002 0x5555; mw.w 0x020bc002 0xaaaa\0" \
	"wec_image_addr=0x10200000\0" 	\
	"eboot_flash_offset=0x100000\0" \
	"eboot_image_addr=0x10041000\0" \
	"initrd_high=0xffffffff\0"	\
	"kernel_addr_r=0x10008000\0"	\
	"fdt_addr_r=0x13000000\0"	\
	"ramdisk_addr_r=0x18000000\0"	\
	"scriptaddr=0x14000000\0"	\
	"fdtfile=imx6q-dhcom-pdk2.dtb\0"

#ifndef CONFIG_NAND_MXS /* eMMC default args */

#define CONFIG_EXTRA_ENV_SETTINGS	\
	EXTRA_ENV_SETTINGS 		\
	"load_settings_bin=load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${settings_bin_file}\0" \
	"load_splash=load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${splash_file}\0" \
	"load_bootenv=echo Loading u-boot environment ${bootenv_file}...;" \
		" load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${bootenv_file};\0" \
	"load_fdt=echo Loading device tree ${fdt_file}...;" \
		" load mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"load_zimage=echo Loading linux ${zImage_file}...;" \
		" load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${zImage_file}\0" \
	"mmcdev=1\0" 			\
	"mmcpart=1\0" 			\
	"mmc_rootfs_part=2\0" 		\
	""

#define CONFIG_BOOTCOMMAND \
        "update auto; mmc dev ${mmcdev};" \
		" if mmc rescan; then run bootlinux;" \
		" else echo Boot failed, because mmc${mmcdev} not found!;fi;"

#else /* NAND default args */

#define MTDIDS_DEFAULT          "nand0=gpmi-nand"
#define MTDPARTS_DEFAULT        "mtdparts=gpmi-nand:-(gpmi-nand)"

#define CONFIG_EXTRA_ENV_SETTINGS	\
	EXTRA_ENV_SETTINGS 		\
	"mtdids=nand0=gpmi-nand\0" \
	"mtdparts=mtdparts=gpmi-nand:-(gpmi-nand)\0" \
	"load_settings_bin=ubi part gpmi-nand; ubifsmount ubi0:boot; ubifsload ${loadaddr} ${settings_bin_file}\0" \
	"load_splash=ubifsload ${loadaddr} ${splash_file}\0" \
	"load_bootenv=echo Loading u-boot env file ${bootenv_file}...; ubifsload ${loadaddr} ${bootenv_file};\0" \
	"load_fdt=echo Loading device tree ${fdt_file}...; ubifsload ${fdt_addr} ${fdt_file}\0" \
	"load_zimage=echo Loading linux ${zImage_file}...; ubifsload ${loadaddr} ${zImage_file}\0" \
	""

#define CONFIG_BOOTCOMMAND \
        "update auto; run bootlinux;"

#endif
#endif

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

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		0x20000000
#define CONFIG_SYS_MEMTEST_SCRATCH	0x10800000

/* Environment */
#define CONFIG_ENV_SIZE			(256 * 1024)
#define CONFIG_ENV_RESERVED_SIZE	CONFIG_ENV_SIZE /* Reserved size for one env */

#if defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		(768 * 1024)
#define CONFIG_ENV_SECT_SIZE		(64 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#endif

/* Framebuffer */
#ifdef CONFIG_VIDEO_IPUV3
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_BMP_24BPP
#define CONFIG_BMP_32BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP
#endif

#endif	/* __DH_IMX6_CONFIG_H */
