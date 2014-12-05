/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6QSABRE_COMMON_CONFIG_H
#define __MX6QSABRE_COMMON_CONFIG_H

#define CONFIG_MX6

#include "mx6_common.h"

#define DH_IMX6_EMMC_VERSION
//#define DH_IMX6_NAND_VERSION
#define CONFIG_DHCOM

#define UBOOT_DH_VERSION "0.3.0.0" 	/* DH - Version of U-Boot e.g. 1.4.0.1 */

#define BOOTLOADER_FLASH_OFFSET				0x400
/*
 * Default settings
 */
#define DEFAULT_SETTINGS_BLOCK_LENGTH       0x2C        /* 44 Byte */
#define DEFAULT_SETTINGS_DISPLAY_ID         0x01
#define DEFAULT_SETTINGS_Y_RESOLUTION       272
#define DEFAULT_SETTINGS_X_RESOLUTION       480
#define DEFAULT_SETTINGS_LCD_CONFIG_FLAGS   0x03E7      /* includes GPIO G for BE enable */

#define DEFAULT_SETTINGS_PIXEL_CLOCK        11100
#define DEFAULT_SETTINGS_V_PULSE_WIDTH      11
#define DEFAULT_SETTINGS_H_PULSE_WIDTH      42
#define DEFAULT_SETTINGS_H_BACK_PORCH       2
#define DEFAULT_SETTINGS_H_FRONT_PORCH      3
#define DEFAULT_SETTINGS_V_BACK_PORCH       2
#define DEFAULT_SETTINGS_V_FRONT_PORCH      3
#define DEFAULT_SETTINGS_AC_BIAS_TRANS      0
#define DEFAULT_SETTINGS_AC_BIAS_FREQ       0
#define DEFAULT_SETTINGS_DATALINES          24

#define DEFAULT_SETTINGS_GPIO_DIRECTION     0x1FF       /* 1 = Input, 0 = Output --> bits 0 - 8 = GPIO A - I */
#define DEFAULT_SETTINGS_GPIO_STATE         0x055       /* 0 = Low, 1 = High --> bits 0 - 8 = GPIO A - I */

#define DEFAULT_SETTINGS_HW_CONFIG_FLAGS    0x027E     	/* SILENT_MODE = off , UPDATE_DEV = all , USE_DA_EEPROM_SETTINGS = on */

#define DEFAULT_BACKLIGHT_PWM				57  		/* DHCOM GPIO_PWM - this is fix and not to change with settings.bin functionality */

/*
 * DHCOM GPIOS
 */
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
 
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#include <asm/arch/imx-regs.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART

#define CONFIG_CMD_SETTINGS_INFO
#define CONFIG_CMD_DHCOM_UPDATE

/* Fuses */
#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

/* SPI Flash Configs */
#define CONFIG_CMD_SF
#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_SPANSION
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS  0
#define CONFIG_SF_DEFAULT_CS   (0|(IMX_GPIO_NR(2, 30)<<8))
#define CONFIG_SF_DEFAULT_SPEED 25000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
#define CONFIG_SF_PAGE_SIZE	256
#endif

/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR      0

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_FS_FAT
#define CONFIG_FS_EXT4
#define CONFIG_CMD_FS_GENERIC			/* Generic load commands */
#define CONFIG_CMD_BOOTZ			/* bootz zImage support */
#define CONFIG_SUPPORT_RAW_INITRD		/* bootz raw initrd support */
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION
#define CONFIG_BZIP2
#define CONFIG_CMD_UNZIP
#define CONFIG_CMD_TESTROUTINES

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE                    ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE             RMII
#define CONFIG_ETHPRIME                 "FEC"
#define CONFIG_FEC_MXC_PHYADDR		0

#define CONFIG_PHYLIB
#define CONFIG_PHY_SMSC

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_CMD_FAT
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE

#define CONFIG_MXC_USB_PORT	1
#define CONFIG_MXC_USB_PORTSC	(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS	0

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX              1
#define CONFIG_BAUDRATE                115200

/* Command definition */
#define CONFIG_CMD_BMODE
#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_SETEXPR
#undef CONFIG_CMD_IMLS

 /* Only interrupt autoboot if <del> is pressed. Otherwise, garbage
  * data on the serial line may interrupt the boot sequence.
  */
#define CONFIG_BOOTDELAY		0
#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_AUTOBOOT
#define CONFIG_AUTOBOOT_KEYED
#define CONFIG_AUTOBOOT_PROMPT		\
	"Press DEL to abort autoboot\n"
#define CONFIG_AUTOBOOT_STOP_STR	"\x7f"

#define CONFIG_LOADADDR                0x12000000
#define CONFIG_SYS_TEXT_BASE           0x17800000

/*
 * Flash Update defines (Command do_update())
 */
#define UPDATE_DHUPDATE_INI_SDRAM_ADDRESS           "10000000"  /* should equal os base */
#define UPDATE_BMP_SDRAM_ADDRESS                    "10100002"


#define CONFIG_EXTRA_ENV_SETTINGS \
        "splashimageflashaddr=0x0\0" \
        "dhsettingsflashaddr=0x100000\0" \
		"splashimage=0x10000002\0" \
		"splashpos=m,m\0" \
        "script=boot.scr\0" \
        "uimage=uImage\0" \
        "fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
        "fdt_addr=0x11000000\0" \
        "boot_fdt=try\0" \
        "ip_dyn=yes\0" \
        "console=ttymxc0,115200\0" \
        "optargs=\0" \
        "video=\0" \
        "fdt_high=0xffffffff\0"   \
        "initrd_high=0xffffffff\0" \
		"settings_bin_name=13_DataImage_7inch_FG0700G3DSSW.bin\0" \
		"splash_name=MX6_480x272.bmp\0" \
		"load_settings_bin=ext2load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${settings_bin_name}\0" \
		"load_splash=ext2load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${splash_name}\0" \
		"load_update_kernel=dummy\0" \
        "mmcdev=" __stringify(CONFIG_SYS_DEFAULT_MMC_DEV) "\0" \
        "mmcpart=1\0" \
		"ethaddr=00:11:22:33:44:55\0" \
		"ipaddr=10.64.31.252\0" \
		"bootlinux=update auto; bootm\0"

#define CONFIG_BOOTCOMMAND \
        "run bootlinux"

#define CONFIG_ARP_TIMEOUT     200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_SYS_PROMPT              "U-Boot > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE              256

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS             16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END         0x20000000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR
#define CONFIG_SYS_HZ                  1000

#define CONFIG_CMDLINE_EDITING
#define CONFIG_STACKSIZE               (128 * 1024)

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
	
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(256 * 1024)

#define CONFIG_ENV_IS_IN_SPI_FLASH
//#define CONFIG_ENV_IS_IN_NVRAM
//#define CONFIG_ENV_ADDR 0x30000000
//#define CONFIG_SYS_ICACHE_OFF


#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		(768 * 1024)
#define CONFIG_ENV_SECT_SIZE	(4 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ	CONFIG_SF_DEFAULT_SPEED
#endif

#define CONFIG_OF_LIBFDT

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

#endif                         /* __MX6QSABRE_COMMON_CONFIG_H */
