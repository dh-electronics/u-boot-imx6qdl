/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6QSABRE_COMMON_CONFIG_H
#define __MX6QSABRE_COMMON_CONFIG_H

#include "mx6_common.h"

#define DH_IMX6_EMMC_VERSION
/* #define DH_IMX6_NAND_VERSION */
#define CONFIG_DHCOM
#define BOOT_CFI2

#define UBOOT_DH_VERSION "1.0.0.2" 	/* DH - Version of U-Boot e.g. 1.4.0.1 */

#define BOOTLOADER_FLASH_OFFSET				0x400

/* Silent Mode */
#define CONFIG_SILENT_CONSOLE		1
#define CONFIG_SYS_DEVICE_NULLDEV
#define CONFIG_SILENT_CONSOLE_UPDATE_ON_RELOC

/*
 * Default settings
 */
#define DEFAULT_SETTINGS_BLOCK_LENGTH       0x2C        /* 44 Byte */
#define DEFAULT_SETTINGS_DISPLAY_ID         0xFF
#define DEFAULT_SETTINGS_Y_RESOLUTION       0xFFFF
#define DEFAULT_SETTINGS_X_RESOLUTION       0xFFFF
#define DEFAULT_SETTINGS_LCD_CONFIG_FLAGS   0xFFFFFFFF      /* includes GPIO G for BE enable */

#define DEFAULT_SETTINGS_PIXEL_CLOCK        0xFFFFFFFF
#define DEFAULT_SETTINGS_V_PULSE_WIDTH      0xFFFF
#define DEFAULT_SETTINGS_H_PULSE_WIDTH      0xFFFF
#define DEFAULT_SETTINGS_H_BACK_PORCH       0xFFFF
#define DEFAULT_SETTINGS_H_FRONT_PORCH      0xFFFF
#define DEFAULT_SETTINGS_V_BACK_PORCH       0xFFFF
#define DEFAULT_SETTINGS_V_FRONT_PORCH      0xFFFF
#define DEFAULT_SETTINGS_AC_BIAS_TRANS      0xFF
#define DEFAULT_SETTINGS_AC_BIAS_FREQ       0xFF
#define DEFAULT_SETTINGS_DATALINES          0xFFFF

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

/* WEC Support */
#define CONFIG_CMD_WINCE
#define WEC_PARTITION_SIZE		0x6400000 /* (100MB) */
#define EBOOT_PARTITION_SIZE		0x100000 /* (1MB) */ 
#define WEC_IMAGE_FLASH_ADDRESS		0
#define BOOTCE_ARGUMENTS_SDRAM_ADDRESS	0x10001044

/* Fuses */
#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

/* SPI Flash Configs */
#define CONFIG_CMD_SF
#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH_SPANSION
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS  0
#define CONFIG_SF_DEFAULT_CS   0
#define CONFIG_SF_DEFAULT_SPEED 25000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
#define CONFIG_SF_PAGE_SIZE	256
#endif

/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		  100000

#ifdef BOOT_CFI2
#define CONFIG_SYS_I2C_EEPROM_ADDR 0x50
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN 1 /* Bosch CFI2 EEPROM */
#define EEPROM_I2C_BUS_NUM 0
#endif

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
#ifdef BOOT_CFI2
#define CONFIG_CMD_CFI2BOOT
#define CONFIG_CMD_CFI2DIP
#endif

#define CONFIG_CMD_GPIO
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE                    ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE             RMII
#define CONFIG_ETHPRIME                 "FEC"
#define CONFIG_FEC_MXC_PHYADDR		0

#define CONFIG_PHYLIB
#define CONFIG_PHY_SMSC

/* NAND Configs */
#ifdef DH_IMX6_NAND_VERSION
	#define CONFIG_CMD_NAND_TRIMFFS
	#define CONFIG_CMD_TIME
        #define CONFIG_CMD_NAND

	/* NAND stuff */
	#define CONFIG_NAND_MXS
	#define CONFIG_SYS_MAX_NAND_DEVICE     1
	#define CONFIG_SYS_NAND_MAX_CHIPS	1
 	#define CONFIG_SYS_NAND_BASE           0x40000000
	#define CONFIG_SYS_NAND_5_ADDR_CYCLE
	#define CONFIG_SYS_NAND_ONFI_DETECTION
	 
	/* DMA stuff, needed for GPMI/MXS NAND support */
	#define CONFIG_APBH_DMA
	#define CONFIG_APBH_DMA_BURST
	#define CONFIG_APBH_DMA_BURST8

	/* UBI/UBIFS config options */
	#define CONFIG_LZO
	#define CONFIG_MTD_DEVICE
	#define CONFIG_MTD_PARTITIONS
	#define CONFIG_RBTREE
	#define CONFIG_CMD_MTDPARTS
	#define CONFIG_CMD_UBI
	#define CONFIG_CMD_UBIFS
#endif

/* USB Configs */
#define CONFIG_CMD_USB
#ifdef CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC          (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS           0
#define CONFIG_USB_MAX_CONTROLLER_COUNT        2 /* Enabled USB controller number */
#endif

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX              1
#define CONFIG_BAUDRATE                115200

/* Command definition */
#define CONFIG_CMD_BMODE
#define CONFIG_CMD_BOOTZ
#undef CONFIG_CMD_IMLS

 /* Only interrupt autoboot if <del> is pressed. Otherwise, garbage
  * data on the serial line may interrupt the boot sequence.
  */
#ifdef CONFIG_BOOTDELAY
#undef CONFIG_BOOTDELAY
#endif
#define CONFIG_BOOTDELAY 0
#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_AUTOBOOT

#define CONFIG_LOADADDR                0x12000000
#define CONFIG_SYS_TEXT_BASE           0x17800000

/*
 * Flash Update defines (Command do_update())
 */
#define UPDATE_DHUPDATE_INI_SDRAM_ADDRESS           "10000000"  /* should equal os base */
#define UPDATE_BMP_SDRAM_ADDRESS                    "10100002"

#ifdef DH_IMX6_NAND_VERSION
	#define MTDIDS_DEFAULT		"nand0=gpmi-nand"
	#define MTDPARTS_DEFAULT	"mtdparts=gpmi-nand:-(gpmi-nand)"
#endif

#define CONFIG_EXTRA_ENV_SETTINGS_BASE \
		"panel=no_panel\0" \
		"console=ttymxc0,115200\0" \
		"splashimage=0x10000002\0" \
		"splashpos=m,m\0" \
		"settings_bin_file=default_settings.bin\0" \
		"splash_file=splash.bmp\0" \
		"setupdateargs=setenv bootargs " \
		        "console=${console} src_intf=${src_intf} src_dev_part=${src_dev_part} dhcom=${dhcom} " \
		        "${backlight} ${parallel_display} ${lvds_display0} ${lvds_display1} vt.global_cursor_default=0\0" \
		"load_update_kernel=load ${src_intf} ${src_dev_part} ${loadaddr} zImage_${dhcom}.update; run setupdateargs; bootz ${loadaddr}\0" \
		"ethaddr=00:11:22:33:44:55\0" \
		"ipaddr=10.64.31.252\0" \
		"bootenv_file=uLinuxEnv.txt\0" \
		"bootlinux=if run load_bootenv; then run importbootenv;fi;" \
                        " setenv set_rootfs setenv rootfs ${rootfs}; run set_rootfs;" \
                        " setenv set_fdt_file setenv fdt_file ${fdt_file}; run set_fdt_file; run load_fdt;" \
                        " run load_zimage; run linuxargs; bootz ${loadaddr} - ${fdt_addr};\0" \
		"importbootenv=echo Importing environment from ${bootenv_file}...; env import -t ${loadaddr} ${filesize}\0" \
		"linuxargs=setenv bootargs " \
		        "console=${console} ${rootfs} fbcon=${fbcon} ${videoargs} ${optargs} dhcom=${dhcom} " \
		        "${backlight} ${parallel_display} ${lvds_display0} ${lvds_display1} SN=${SN}\0" \
		"fdt_addr=0x18000000\0" \
		"fdt_high=0xffffffff\0" \
		"enable_watchdog_128s=mw.w 20bc000 0xffb7; run serv_watchdog\0" \
		"serv_watchdog=mw.w 0x020bc002 0x5555; mw.w 0x020bc002 0xaaaa\0" \
		"wec_image_addr=0x10200000\0" \
		"eboot_flash_offset=0x100000\0" \
		"eboot_image_addr=0x10041000\0"
		
#if defined(DH_IMX6_NAND_VERSION)
	#define CONFIG_EXTRA_ENV_SETTINGS_SELECT \
		"mtdids=" MTDIDS_DEFAULT "\0" \
		"mtdparts=" MTDPARTS_DEFAULT "\0" \
		"load_settings_bin=ubi part gpmi-nand; ubifsmount ubi0:boot; ubifsload ${loadaddr} ${settings_bin_file}\0" \
		"load_splash=ubifsload ${loadaddr} ${splash_file}\0" \
		"load_bootenv=echo Loading u-boot env file ${bootenv_file}...; ubifsload ${loadaddr} ${bootenv_file};\0" \
		"load_fdt=echo Loading device tree ${fdt_file}...; ubifsload ${fdt_addr} ${fdt_file}\0" \
		"load_zimage=echo Loading linux ${zImage_file}...; ubifsload ${loadaddr} ${zImage_file}\0" \
		"wec_nand_flash_address=0x0\0" \
		"wec_nand_partition_size=0x6400000\0" \
		"wec_ram_buffer=0x18000000\0" \
		"wec_image_size=0x5E00000\0" \
		"wec_unzip_nk_gz=unzip ${wec_ram_buffer} ${wec_image_addr}\0" \
		"copy_wec_image= " \
			"if test -n ${wec_image_type_gz}; then " \
				"echo --> BootWinCE INFO: Unzip WEC image (nk.gz) file;" \
				"unzip ${wec_ram_buffer} ${wec_image_addr};" \
			"elif test -n ${wec_image_type_nb0}; then " \
				"echo --> BootWinCE INFO: Copy WEC image (nk.nb0) file;" \
				"cp.b ${wec_ram_buffer} ${wec_image_addr} ${wec_image_size};" \
			"else " \
				"echo --> BootWinCE ERROR: No WEC image type defined;" \
			"fi;\0" \
		"copy_and_start_eboot_image= " \
			"if test -n ${eboot_image}; then " \
				"echo --> BootWinCE INFO: Copy eboot image;" \
				"sf probe; sf read ${eboot_image_addr} ${eboot_flash_offset} 80000; go 10041000;" \
			"else " \
				"echo --> BootWinCE ERROR: No eboot image stored in flash;" \
			"fi;\0" \
		""
#elif defined(DH_IMX6_EMMC_VERSION)
	#define CONFIG_EXTRA_ENV_SETTINGS_SELECT \
		"load_settings_bin=load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${settings_bin_file}\0" \
		"load_splash=load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${splash_file}\0" \
		"load_bootenv=echo Loading u-boot environment file ${bootenv_file}...; load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${bootenv_file};\0" \
		"load_fdt=echo Loading device tree ${fdt_file}...; load mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
		"load_zimage=echo Loading linux ${zImage_file}...; load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${zImage_file}\0" \
		"mmcdev=" __stringify(CONFIG_SYS_DEFAULT_MMC_DEV) "\0" \
		"mmcpart=1\0" \
		"mmc_rootfs_part=2\0" \
		""
#endif	
	
#define	CONFIG_EXTRA_ENV_SETTINGS CONFIG_EXTRA_ENV_SETTINGS_BASE CONFIG_EXTRA_ENV_SETTINGS_SELECT

#if defined(DH_IMX6_NAND_VERSION)
        #define CONFIG_BOOTCOMMAND \
                "update auto; run bootlinux;"
#elif defined(DH_IMX6_EMMC_VERSION)		
        #define CONFIG_BOOTCOMMAND \
                "update auto; mmc dev ${mmcdev}; if mmc rescan; then run bootlinux; else echo Linux start failed, because mmc${mmcdev} was not found!;fi;"
#endif	

#define CONFIG_ARP_TIMEOUT     200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_AUTO_COMPLETE

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
