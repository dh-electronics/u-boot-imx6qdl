/*
 * File: cmd_settings_info.c
 *
 * (C) Copyright 2011-2012
 * Andreas Geisreiter , DH electronics GmbH , ageisreiter@dh-electronics.de
 * Ludwig Zenz, DH electronics GmbH , lzenz@dh-electronics.de
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 *  Version:  1.1
 *
 *  Abstract: This file contains the source code for the "lcdinfo" command line function.
 *            This function lists the act. i.MX25 display controller settings.
 *
 *  Notes:
 *  	Created: 		May 10, 2011 by Andreas Geisreiter (ageisreiter@dh-electronics.de)
 *  	Modified: 		Jan 25, 2012 by Ludwig Zenz (lzenz@dh-electronics.de)
 */
 
#include <common.h>
#include <command.h>
#include <errno.h>
#include <lcd.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <i2c.h>


#include <asm/mach-imx/video.h>

/* Special MXCFB sync flags are here. */
#include "../drivers/video/mxcfb.h"

DECLARE_GLOBAL_DATA_PTR;

static bool settings_loaded = false; // to remember if settings cmd was executed

/* Default settings */
#define DEFAULT_BLOCK_LENGTH       0x2C        /* 44 Byte */
#define DEFAULT_DISPLAY_ID         0xFF
#define DEFAULT_Y_RESOLUTION       0xFFFF
#define DEFAULT_X_RESOLUTION       0xFFFF
#define DEFAULT_LCD_CONFIG_FLAGS   0xFFFFFFFF  /* includes GPIO G for BE enable */

#define DEFAULT_PIXEL_CLOCK        0xFFFFFFFF
#define DEFAULT_V_PULSE_WIDTH      0xFFFF
#define DEFAULT_H_PULSE_WIDTH      0xFFFF
#define DEFAULT_H_BACK_PORCH       0xFFFF
#define DEFAULT_H_FRONT_PORCH      0xFFFF
#define DEFAULT_V_BACK_PORCH       0xFFFF
#define DEFAULT_V_FRONT_PORCH      0xFFFF
#define DEFAULT_AC_BIAS_TRANS      0xFF
#define DEFAULT_AC_BIAS_FREQ       0xFF
#define DEFAULT_DATALINES          0xFFFF

#define DEFAULT_GPIO_DIRECTION     0x1FF      /* 1 = Input, 0 = Output --> bits 0 - 8 = GPIO A - I */
#define DEFAULT_GPIO_STATE         0x055      /* 0 = Low, 1 = High --> bits 0 - 8 = GPIO A - I */

#define DEFAULT_HW_CONFIG_FLAGS    0x027E     /* SILENT_MODE = off , UPDATE_DEV = all , USE_DA_EEPROM_SETTINGS = on */

// TODO: This is i.MX6 specific -> DIRTY
extern int board_get_hwcode(void);



/* copy settingsblock from buffer into the dh_board_settings structure */
int settings_bin_to_struct(ulong addr, bool is_DA_eeprom)
{
	volatile settingsinfo_t  *ptr = &gd->dh_board_settings;

	u16 valid_ID = ((readl(addr) & 0xFFFF0000) >> 16);

	// settings.bin file Valid Mask should be "DH" = 0x4844
	if(valid_ID == 0x4844) {
		ptr->wValidationID = valid_ID;
		ptr->cLength      = (readl(addr) & 0xFF);
		ptr->cDisplayID   = ((readl(addr) & 0xFF00) >> 8);
		ptr->wYResolution = (readl(addr+4) & 0xFFFF);
		ptr->wXResolution = ((readl(addr+4) & 0xFFFF0000) >> 16);
		ptr->wLCDConfigFlags = (readl(addr+8) & 0xFFFF);
		ptr->wPixelClock  = ((readl(addr+8) & 0xFFFF0000) >> 16);
		ptr->wVPulseWidth = (readl(addr+12) & 0xFFFF);
		ptr->wHPulseWidth = ((readl(addr+12) & 0xFFFF0000) >> 16);
		ptr->wHBackPorch  = (readl(addr+16) & 0xFFFF);
		ptr->wHFrontPorch = ((readl(addr+16) & 0xFFFF0000) >> 16);
		ptr->wVBackPorch  = (readl(addr+20) & 0xFFFF);
		ptr->wVFrontPorch = ((readl(addr+20) & 0xFFFF0000) >> 16);
		ptr->cACBiasTrans = (readl(addr+24) & 0xFF);
		ptr->cACBiasFreq  = ((readl(addr+24) & 0xFF00) >> 8);
		ptr->cDatalines   = ((readl(addr+24) & 0xFFFF0000) >> 16);
		if (!is_DA_eeprom) { // skip in DA eeprom case
			ptr->wGPIODir     = (readl(addr+32));
			ptr->wGPIOState   = (readl(addr+36));
			ptr->wHWConfigFlags = (readl(addr+40) & 0xFFFF);
		}
	}
	// settings.bin file Valid Mask should be "V2" = 0x3256
	else if(valid_ID == 0x3256) {
		ptr->wValidationID = valid_ID;
		ptr->cLength      = (readl(addr) & 0xFF);
		ptr->cDisplayID   = ((readl(addr) & 0xFF00) >> 8);
		ptr->wYResolution = (readl(addr+4) & 0xFFFF);
		ptr->wXResolution = ((readl(addr+4) & 0xFFFF0000) >> 16);
		ptr->wPixelClock  = (readl(addr+8));
		ptr->wVPulseWidth = (readl(addr+12) & 0xFFFF);
		ptr->wHPulseWidth = ((readl(addr+12) & 0xFFFF0000) >> 16);
		ptr->wHBackPorch  = (readl(addr+16) & 0xFFFF);
		ptr->wHFrontPorch = ((readl(addr+16) & 0xFFFF0000) >> 16);
		ptr->wVBackPorch  = (readl(addr+20) & 0xFFFF);
		ptr->wVFrontPorch = ((readl(addr+20) & 0xFFFF0000) >> 16);
		ptr->cACBiasTrans = (readl(addr+24) & 0xFF);
		ptr->cACBiasFreq  = ((readl(addr+24) & 0xFF00) >> 8);
		ptr->cDatalines   = ((readl(addr+24) & 0xFFFF0000) >> 16);
		ptr->wLCDConfigFlags = (readl(addr+28));
		if (!is_DA_eeprom) { // skip in DA eeprom case
			ptr->wGPIODir     = (readl(addr+32));
			ptr->wGPIOState   = (readl(addr+36));
			ptr->wHWConfigFlags = (readl(addr+40) & 0xFFFF);
		}
	} else
		return -EFAULT;

	return 0;
}

int settings_load(void)
{
	int ret;
	int got_settings = 0; // 0 = OK, -1 = no settings
	ulong addr;
	char *command;
	uchar ucBuffer[DHCOM_DISPLAY_SETTINGS_SIZE];
	unsigned int old_bus = 1;

	volatile settingsinfo_t  *ptr = &gd->dh_board_settings;

	addr = simple_strtoul(env_get("loadaddr"), NULL, 16);
	
	/* initialize DH Global Data */
	ptr->cLength = 		DEFAULT_BLOCK_LENGTH;
	ptr->cDisplayID = 	DEFAULT_DISPLAY_ID;
	ptr->wValidationID = 	0;
	ptr->wYResolution = 	DEFAULT_Y_RESOLUTION;
	ptr->wXResolution = 	DEFAULT_X_RESOLUTION;
	ptr->wLCDConfigFlags = 	DEFAULT_LCD_CONFIG_FLAGS;
	ptr->wPixelClock = 	DEFAULT_PIXEL_CLOCK;
	ptr->wVPulseWidth = 	DEFAULT_V_PULSE_WIDTH;
	ptr->wHPulseWidth = 	DEFAULT_H_PULSE_WIDTH;
	ptr->wHBackPorch = 	DEFAULT_H_BACK_PORCH;
	ptr->wHFrontPorch = 	DEFAULT_H_FRONT_PORCH;
	ptr->wVBackPorch = 	DEFAULT_V_BACK_PORCH;
	ptr->wVFrontPorch = 	DEFAULT_V_FRONT_PORCH;
	ptr->cACBiasTrans = 	DEFAULT_AC_BIAS_TRANS;
	ptr->cACBiasFreq = 	DEFAULT_AC_BIAS_FREQ;
	ptr->cDatalines = 	DEFAULT_DATALINES;
	ptr->wGPIODir = 	DEFAULT_GPIO_DIRECTION;
	ptr->wGPIOState = 	DEFAULT_GPIO_STATE;
	ptr->wHWConfigFlags = 	DEFAULT_HW_CONFIG_FLAGS;
	
	printf("Load DH settings...\n");

	/* Load DH settings file from filesystem */
	if ((command = env_get("load_settings_bin")) == NULL) {
		printf ("\"load_settings_bin\" not defined\n");
		return -ENOENT;
	}	
	
	if (run_command(command, 0) != 0) {
		printf("Can't load DH settings from filesystem\n");
		got_settings = -1;
	}
	
	ret = settings_bin_to_struct(addr, false);
	if (ret < 0)
		ptr->wValidationID = 0; // set to invalid on error
	
	/* Check and Read Display data from EEPROM if enabled */
	if((ptr->wHWConfigFlags & SETTINGS_HW_EN_DISP_ADPT_EE_CHK) != 0) {
		/* TODO: DIRTY (imx6-specific) Set i2c driver to use i2c bus 0  */
                old_bus = I2C_GET_BUS();
                if (board_get_hwcode() < 3)
                        I2C_SET_BUS(0);
                else
                        I2C_SET_BUS(1);

		i2c_init (CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
		i2c_read(DISPLAY_ADAPTER_EEPROM_ADDR, 0, 1, &ucBuffer[0], DHCOM_DISPLAY_SETTINGS_SIZE);

		I2C_SET_BUS(old_bus);

		ret = settings_bin_to_struct((ulong)ucBuffer, true);
		if (ret == 0) {
			printf ("Using settings of DA eeprom\n");
			got_settings = 0;
		}
	}

	// SETTINGS_LCD_BL_ON_FLAG must be set for function set_dhcom_backlight_gpio(),
	// but isn't defined in the old DHCOM settings (0x4844)
	// ==> We have to set a default value (ON)
	if(ptr->wValidationID == 0x4844)
		ptr->wLCDConfigFlags |= SETTINGS_LCD_BL_ON_FLAG;

	return got_settings;
}	

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

void set_dhcom_gpios(void)
{
	int i;
	int mask = 0x1;

	for(i = 0; i < 23; i++) {
		if(gd->dh_board_settings.wGPIODir & mask) {
			// Set to input
			gpio_direction_input(DHCOM_gpios[i]);		
		}
		else {
		        // Set to output
			if(gd->dh_board_settings.wGPIOState & mask)
				gpio_direction_output(DHCOM_gpios[i] , 1);
			else
				gpio_direction_output(DHCOM_gpios[i] , 0);
		}
		mask = mask << 1;
	}
}

void set_dhcom_backlight_gpio(void)
{
	int bl_gpio;
	int bl_en_pol; 	// 0 = active high; 1 = active low
	int bl_pwm_pol; // 0 = active high; 1 = active low
	int bl_on = 1;

	volatile settingsinfo_t  *ptr = &gd->dh_board_settings;

	char const *panel = env_get("panel");

	if (panel && !strcmp(panel, "no_panel"))
		bl_on = 0;
	else
		/*
		 * If display interface is enabled:
                 *   check BL_ON flag (0 = backlight off / 1 = backlight on)
		 */
		bl_on = (ptr->wLCDConfigFlags & SETTINGS_LCD_BL_ON_FLAG) ? 1 : 0;

	// mask backlight enable GPIO
	bl_gpio = (ptr->wLCDConfigFlags & SETTINGS_LCD_BL_EN_GPIO_FLAG) >> 7;
	if (bl_gpio == 0)
	        return; // no backlight enable is specified

	bl_gpio--; // convert to struct gpio index

	bl_en_pol  = (ptr->wLCDConfigFlags & SETTINGS_LCD_IBL_FLAG);
	bl_pwm_pol = (ptr->wLCDConfigFlags & SETTINGS_LCD_PWM_POL_FLAG);
		
	if (bl_on == 0) {
		// disable backlight till linux is running
		gpio_direction_output(DHCOM_gpios[bl_gpio], (bl_en_pol==0) ? 0 : 1);
		gpio_direction_output(PWM_BACKLIGHT_GP, (bl_pwm_pol==0) ? 0 : 1);
	} else {
		// enable backlight
		gpio_direction_output(DHCOM_gpios[bl_gpio], (bl_en_pol==0) ? 1 : 0);
		gpio_direction_output(PWM_BACKLIGHT_GP, (bl_pwm_pol==0) ? 1 : 0);
	}
}

void settings_gen_kernel_args(void)
{
	int iDI_TYPE = 0;
	uchar buf[512];
	int bl_gpio = 0;
	int bl_en_pol = 0;
	int bl_pwm_pol = 0;
	int bl_on = 0;
	int h_sync_inv = 0;
	int v_sync_inv = 0;
	int DE_inv = 0;
	int PCLK_inv = 0;	

	volatile settingsinfo_t  *ptr = &gd->dh_board_settings;
	
	// Set ENV Linux Kernel parameter
	// DHCOM settings "V2" = 0x3256 or "DH" = 0x4844
	if(!(ptr->wValidationID == 0x3256) && !(ptr->wValidationID == 0x4844)) {
		// Delete ENV variables
		env_set("parallel_display", NULL);	
		env_set("lvds_display0", NULL);
		env_set("lvds_display1", NULL);
		env_set("pwm_bl.set", NULL);

		return;
	}

	iDI_TYPE = (ptr->wLCDConfigFlags & SETTINGS_LCD_DI_TYPE_FLAG) >> 13;

	// Mask Backlight enable GPIO
	bl_gpio = (ptr->wLCDConfigFlags & SETTINGS_LCD_BL_EN_GPIO_FLAG) >> 7;
	
	// Covert to struct gpio number
	bl_gpio = bl_gpio - 1;		

	// Mask Backlight pol flag: 0 = active high; 1 = active low
	bl_en_pol = (ptr->wLCDConfigFlags & SETTINGS_LCD_IBL_FLAG) >> 11;

	// Mask Backlight PWM pol flag: 0 = active high; 1 = active low
	bl_pwm_pol = (ptr->wLCDConfigFlags & SETTINGS_LCD_PWM_POL_FLAG) >> 6;

	// Mask Backlight ON pol flag: 0 = backlight off; 1 = backlight on
	bl_on = (ptr->wLCDConfigFlags & SETTINGS_LCD_BL_ON_FLAG) >> 12;
	
	// Display control flags
	v_sync_inv =  ptr->wLCDConfigFlags & SETTINGS_LCD_IVS_FLAG;
	h_sync_inv = (ptr->wLCDConfigFlags & SETTINGS_LCD_IHS_FLAG) >> 1;
	PCLK_inv   = (ptr->wLCDConfigFlags & SETTINGS_LCD_IPC_FLAG) >> 2;
	DE_inv     = (ptr->wLCDConfigFlags & SETTINGS_LCD_IOE_FLAG) >> 3;

	// Set Display Type to RGB for old settings file
	if(ptr->wValidationID == 0x4844) {
		iDI_TYPE = 2; // RGB Display
		bl_on = 1; // Set bl_on flag
	}

	switch (iDI_TYPE) {
	case 0: // Ignore Display settings	
		// Delete ENV variables
		env_set("parallel_display", NULL);	
		env_set("lvds_display0", NULL);
		env_set("lvds_display1", NULL);
		env_set("pwm_bl.set", NULL);			
	   	break;
	case 1: // Headless
		sprintf((char *)buf, "parallel_display.disable");	
		env_set("parallel_display", (char *)buf);
		sprintf((char *)buf, "imx_ldb.disable0");	
		env_set("lvds_display0", (char *)buf);
		sprintf((char *)buf, "imx_ldb.disable1");	
		env_set("lvds_display1", (char *)buf);
		sprintf((char *)buf, "pwm_bl.disable");	
		env_set("backlight", (char *)buf);
	   	break;
	case 2: // RGB
		sprintf((char *)buf, "parallel_display.timings=ID:%d,PCLK:%d,XRES:%d,YRES:%d,HFP:%d,HBP:%d,HSYNC:%d,VFP:%d,VBP:%d,VSYNC:%d,HINV:%d,VINV:%d,DEINV:%d,PCLKPOL:%d",
	                 ptr->cDisplayID, (ptr->wPixelClock*1000), ptr->wXResolution,
	                 ptr->wYResolution, ptr->wHFrontPorch, ptr->wHBackPorch, 
	                 ptr->wHPulseWidth, ptr->wVFrontPorch, ptr->wVBackPorch, 
	                 ptr->wVPulseWidth, h_sync_inv, v_sync_inv, DE_inv, PCLK_inv);	
		env_set("parallel_display", (char *)buf);
		sprintf((char *)buf, "imx_ldb.disable0");	
		env_set("lvds_display0", (char *)buf);
		sprintf((char *)buf, "imx_ldb.disable1");	
		env_set("lvds_display1", (char *)buf);
		sprintf((char *)buf, "pwm_bl.set=BLGPIO:%d,BLINV:%d,BLON:%d,PWMINV:%d",DHCOM_gpios[bl_gpio], bl_en_pol, bl_on, bl_pwm_pol);	
		env_set("backlight", (char *)buf);
	   	break;
	case 3: // LVDS0
		sprintf((char *)buf, "parallel_display.disable");	
		env_set("parallel_display", (char *)buf);		
		sprintf((char *)buf, "imx_ldb.timings0=ID:%d,PCLK:%d,XRES:%d,YRES:%d,HFP:%d,HBP:%d,HSYNC:%d,VFP:%d,VBP:%d,VSYNC:%d,HINV:%d,VINV:%d,DEINV:%d,PCLKPOL:%d",
	                 ptr->cDisplayID, (ptr->wPixelClock*1000), ptr->wXResolution,
	                 ptr->wYResolution, ptr->wHFrontPorch, ptr->wHBackPorch, 
	                 ptr->wHPulseWidth, ptr->wVFrontPorch, ptr->wVBackPorch, 
	                 ptr->wVPulseWidth, h_sync_inv, v_sync_inv, DE_inv, PCLK_inv);	
		env_set("lvds_display0", (char *)buf);
		sprintf((char *)buf, "imx_ldb.disable1");	
		env_set("lvds_display1", (char *)buf);
		sprintf((char *)buf, "pwm_bl.set=BLGPIO:%d,BLINV:%d,BLON:%d,PWMINV:%d",DHCOM_gpios[bl_gpio], bl_en_pol, bl_on, bl_pwm_pol);	
		env_set("backlight", (char *)buf);
	   	break;
	case 4: // LVDS1
		sprintf((char *)buf, "parallel_display.disable");	
		env_set("parallel_display", (char *)buf);
		sprintf((char *)buf, "imx_ldb.disable0");	
		env_set("lvds_display0", (char *)buf);		
		sprintf((char *)buf, "imx_ldb.timings1=ID:%d,PCLK:%d,XRES:%d,YRES:%d,HFP:%d,HBP:%d,HSYNC:%d,VFP:%d,VBP:%d,VSYNC:%d,HINV:%d,VINV:%d,DEINV:%d,PCLKPOL:%d",
	                 ptr->cDisplayID, (ptr->wPixelClock*1000), ptr->wXResolution,
	                 ptr->wYResolution, ptr->wHFrontPorch, ptr->wHBackPorch, 
	                 ptr->wHPulseWidth, ptr->wVFrontPorch, ptr->wVBackPorch, 
	                 ptr->wVPulseWidth, h_sync_inv, v_sync_inv, DE_inv, PCLK_inv);	
		env_set("lvds_display1", (char *)buf);
		sprintf((char *)buf, "pwm_bl.set=BLGPIO:%d,BLINV:%d,BLON:%d,PWMINV:%d",DHCOM_gpios[bl_gpio], bl_en_pol, bl_on, bl_pwm_pol);	
		env_set("backlight", (char *)buf);
	   	break;
	case 5: // Dual LVDS
		sprintf((char *)buf, "parallel_display.disable");	
		env_set("parallel_display", (char *)buf);		
		sprintf((char *)buf, "imx_ldb.timings0=DUAL:1,ID:%d,PCLK:%d,XRES:%d,YRES:%d,HFP:%d,HBP:%d,HSYNC:%d,VFP:%d,VBP:%d,VSYNC:%d,HINV:%d,VINV:%d,DEINV:%d,PCLKPOL:%d",
	                 ptr->cDisplayID, (ptr->wPixelClock*1000), ptr->wXResolution,
	                 ptr->wYResolution, ptr->wHFrontPorch, ptr->wHBackPorch, 
	                 ptr->wHPulseWidth, ptr->wVFrontPorch, ptr->wVBackPorch, 
	                 ptr->wVPulseWidth, h_sync_inv, v_sync_inv, DE_inv, PCLK_inv);	
		env_set("lvds_display0", (char *)buf);
		env_set("lvds_display1", NULL); // Delete lvds_display1 ENV variable for Dual channel LVDS display
		sprintf((char *)buf, "pwm_bl.set=BLGPIO:%d,BLINV:%d,BLON:%d,PWMINV:%d",DHCOM_gpios[bl_gpio], bl_en_pol, bl_on, bl_pwm_pol);	
		env_set("backlight", (char *)buf);
	   	break;
	default:
		/* do nothing */
		break;
	}	
}

int apply_display_settings(int got_settings)
{
	int index = 0;
	int type;
	int clksonframe;

	volatile settingsinfo_t *ptr = &gd->dh_board_settings;

	type = (gd->dh_board_settings.wLCDConfigFlags & SETTINGS_LCD_DI_TYPE_FLAG) >> 13;

	switch (type) {
		case 1: // Headless
			env_set("panel", "no_panel");
			return 0; // nothing to do
		case 2: // RGB
			env_set("panel", "RGB");
			index = 0;
			break;
		case 3: // LVDS0
			env_set("panel", "lvds_24bit");
			index = 1;
			displays[index].mode.sync |= FB_SYNC_EXT;
			break;
		default:
			if (got_settings < 0) // no settings file -> headless
				env_set("panel", "no_panel");
			else // settings file v1 -> parallel RGB display
				env_set("panel", "RGB");
			index = 0;
			break;
	}

	/* Setup display settings from DH settings file */
	displays[index].mode.xres 		= ptr->wXResolution;
	displays[index].mode.yres 		= ptr->wYResolution;
	displays[index].mode.pixclock 		= KHZ2PICOS(ptr->wPixelClock);
	displays[index].mode.left_margin 	= ptr->wHBackPorch;
	displays[index].mode.hsync_len 		= ptr->wHPulseWidth;
	displays[index].mode.right_margin 	= ptr->wHFrontPorch;
	displays[index].mode.upper_margin 	= ptr->wVBackPorch;
	displays[index].mode.vsync_len 		= ptr->wVPulseWidth;
	displays[index].mode.lower_margin 	= ptr->wVFrontPorch;

	clksonframe = ((ptr->wXResolution + ptr->wHFrontPorch +
			ptr->wHPulseWidth + ptr->wHBackPorch) *
		       (ptr->wYResolution + ptr->wVFrontPorch +
			ptr->wVPulseWidth + ptr->wVBackPorch));
	displays[index].mode.refresh = ((ptr->wPixelClock*1000)/clksonframe);

	if(!(ptr->wLCDConfigFlags & SETTINGS_LCD_IVS_FLAG))
		displays[index].mode.sync |= FB_SYNC_VERT_HIGH_ACT;

	if(!(ptr->wLCDConfigFlags & SETTINGS_LCD_IHS_FLAG))
		displays[index].mode.sync |= FB_SYNC_HOR_HIGH_ACT;

	if((ptr->wLCDConfigFlags & SETTINGS_LCD_IPC_FLAG))
		displays[index].mode.sync |= FB_SYNC_CLK_LAT_FALL;

	if(ptr->wLCDConfigFlags & SETTINGS_LCD_IOE_FLAG)
		displays[index].mode.sync |= FB_SYNC_OE_LOW_ACT;

	if(ptr->wLCDConfigFlags & SETTINGS_LCD_IDATA_FLAG)
		displays[index].mode.sync |= FB_SYNC_DATA_INVERT;

	displays[index].mode.vmode = FB_VMODE_NONINTERLACED;

	return 0;
}

static int do_settings( cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret;
	settings_loaded = true;

        ret = settings_load();
	apply_display_settings(ret);

	if (ret < 0)
		return 0;

        settings_gen_kernel_args();
        set_dhcom_gpios();
        return 0;
}

U_BOOT_CMD(
	settings,   1,   1,     do_settings,
	"load and apply DHCOM settings (display and gpios)",
	"\n"
);

static int do_backlight( cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	/* if settings are not loaded (uninitialized) -> load settings */
	if (!settings_loaded)
		run_command("settings", 0);

        set_dhcom_backlight_gpio();
        return 0;
}

U_BOOT_CMD(
	backlight,   1,   1,     do_backlight,
	"switch DHCOM backlight pwm and gpio (based on settings)",
	"\n"
);

#ifdef CONFIG_SPLASH_SCREEN

#define SPLASH_MAX_SIZE 0x69788A // Max WUXGA 1920x1200 24bpp (1920x1200x3 + 138 BMP Header)
static int do_splash( cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char *buffer;
	char *splashimage;
        char *command;
	char *panel; 
	unsigned long splash_size;

	volatile settingsinfo_t *ptr = &gd->dh_board_settings;

        /* check for headless system */
        if (ptr->wValidationID == 0x3256) { // "V2" = 0x3256
		int type = 0;

		type = (ptr->wLCDConfigFlags & SETTINGS_LCD_DI_TYPE_FLAG) >> 13;
		/* skip loading splash on headless systems */
		if (type == 1)
			return 0;
        }

	panel = env_get("panel");
	if (panel) {
		/* skip loading splash on headless systems */
		if (!strcmp(panel, "no_panel"))
			return 0;
	}

	/* get pointer to buffer and point to splashimage in ram from env */
	buffer      = (char*)simple_strtoul(env_get("loadaddr"), NULL, 16);
	splashimage = (char*)simple_strtoul(env_get("splashimage"), NULL, 16);

	if ( buffer < (char*)0x10000000 || splashimage < (char*)0x10000000 ) {
		printf ("Error: invalid \"loadaddr\" or \"splashimage\"!\n");
		return -ENOMEM;
	}

	printf("Load splash...\n");

	/* load splashimage file from a filesystem */
	if ((command = env_get("load_splash")) == NULL) {
		printf ("Error: \"load_splash\" not defined\n");
		return -ENOENT;
	}

	if (run_command(command, 0) != 0) {
		printf ("Warning: Can't load splash bitmap\n");
		return -EIO;
	}

	/* get filesize of bmp from env */
	splash_size = simple_strtoul(env_get("filesize"), NULL, 16);
	if (splash_size > SPLASH_MAX_SIZE) {
		printf("Warning: Crop spashimage, \"filesize\" max %d bytes!\n",
							 SPLASH_MAX_SIZE);
		splash_size = SPLASH_MAX_SIZE;
	}

	/*
	 * Copy bitmap to splashscreen addresss
         *
	 *   Note: It is necessary to align bitmaps on a memory address with an
	 *   offset of an odd multiple of +2, since the use of a four-byte
	 *   alignment will cause alignment exceptions at run-time.
	 */

	memcpy(splashimage, buffer, splash_size);
	return 0;
}
U_BOOT_CMD(
	splash,   1,   1,     do_splash,
	"load splash image to splashimage addr",
	"\n"
);
#endif

bool settings_get_microSD(void)
{
	/* if settings are not loaded (uninitialized) -> load settings */
	if (!settings_loaded)
		run_command("settings", 0);

	if (gd->dh_board_settings.wHWConfigFlags & UPDATE_VIA_MICROSD_SLOT)
		return true;

	printf ("\nsettings: on module microSD slot is disabled!\n");
	return false;
}

bool settings_get_extSD(void)
{
	/* if settings are not loaded (uninitialized) -> load settings */
	if (!settings_loaded)
		run_command("settings", 0);

	if (gd->dh_board_settings.wHWConfigFlags & UPDATE_VIA_SD_MMC_SLOT)
		return true;

	printf ("\nsettings: external SD slot (baseboard) is disabled!\n");
	return false;
}

bool settings_get_usbhost1(void)
{
	/* if settings are not loaded (uninitialized) -> load settings */
	if (!settings_loaded)
		run_command("settings", 0);

	if (gd->dh_board_settings.wHWConfigFlags & UPDATE_VIA_USB_HOST_1_PORT)
		return true;

	printf ("\nsettings: usb host1 is disabled!\n");
	return false;
}

bool settings_get_usbotg(void)
{
	/* if settings are not loaded (uninitialized) -> load settings */
	if (!settings_loaded)
		run_command("settings", 0);

	if (gd->dh_board_settings.wHWConfigFlags & UPDATE_VIA_USB_OTG_PORT)
		return true;

	printf ("\nsettings: usb otg is disabled!\n");
	return false;
}

static int do_settings_info( cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	volatile settingsinfo_t  *ptr = &gd->dh_board_settings;

	printf("  VALIDATION_ID:    \"%c%c\"\n", (char)ptr->wValidationID, (char)(ptr->wValidationID >> 8));
	printf("  DISPLAY_ID:       0x%02x\n", ptr->cDisplayID);
	printf("  LENGTH:           0x%02x\n", ptr->cLength);
	printf("  X_RESOLUTION:     %d pixel\n", ptr->wXResolution);
	printf("  Y_RESOLUTION:     %d pixel\n", ptr->wYResolution);
	printf("  PIXEL_CLOCK:      %d kHz\n", ptr->wPixelClock);   
	printf("  LCD_CONFIG_FLAGS: 0x%x\n",  ptr->wLCDConfigFlags);  
	printf("        IVS:        0x%x\n", (ptr->wLCDConfigFlags & SETTINGS_LCD_IVS_FLAG) ? 1 : 0);
	printf("        IHS:        0x%x\n", (ptr->wLCDConfigFlags & SETTINGS_LCD_IHS_FLAG) ? 1 : 0);
	printf("        IPC:        0x%x\n", (ptr->wLCDConfigFlags & SETTINGS_LCD_IPC_FLAG) ? 1 : 0);
	printf("        IOE:        0x%x\n", (ptr->wLCDConfigFlags & SETTINGS_LCD_IOE_FLAG) ? 1 : 0);
	printf("        IDATA:      0x%x\n", (ptr->wLCDConfigFlags & SETTINGS_LCD_IDATA_FLAG) ? 1 : 0);
	printf("        ACT_PAS:    0x%x\n", (ptr->wLCDConfigFlags & SETTINGS_LCD_ACT_PAS_FLAG) ? 1 : 0);
	printf("        PWM_POL:    0x%x\n", (ptr->wLCDConfigFlags & SETTINGS_LCD_PWM_POL_FLAG) ? 1 : 0);
	printf("        BL_EN_GPIO: 0x%x\n", (ptr->wLCDConfigFlags & SETTINGS_LCD_BL_EN_GPIO_FLAG) >> 7);
	printf("        IBL:        0x%x\n", (ptr->wLCDConfigFlags & SETTINGS_LCD_IBL_FLAG) ? 1 : 0);
	// settings.bin file Valid Mask should be "V2" = 0x3256
	if(ptr->wValidationID == 0x3256) {
		printf("        BL_ON:      0x%x\n", (ptr->wLCDConfigFlags & SETTINGS_LCD_BL_ON_FLAG) ? 1 : 0);
		printf("        DI_TYPE:    0x%x\n", (ptr->wLCDConfigFlags & SETTINGS_LCD_DI_TYPE_FLAG) ? 1 : 0);
		printf("        NEXT_DI:    0x%x\n", (ptr->wLCDConfigFlags & SETTINGS_LCD_NEXT_DI_FLAG) ? 1 : 0);
	}
	printf("  HSW:              %d pixel clocks\n", ptr->wHPulseWidth);  
	printf("  VSW:              %d line clocks\n", ptr->wVPulseWidth);      
	printf("  HFP:              %d pixel clocks\n", ptr->wHFrontPorch);   
	printf("  HBP:              %d pixel clocks\n", ptr->wHBackPorch);       
	printf("  VFP:              %d line clocks\n", ptr->wVFrontPorch);     
	printf("  VBP:              %d line clocks\n", ptr->wVBackPorch);          
	printf("  DATALINES:        %d\n", ptr->cDatalines); 
	printf("  ACB:              %d\n", ptr->cACBiasFreq); 
	printf("  ACBI:             %d\n\n", ptr->cACBiasTrans);     
    
	printf("  GPIO_DIR:         0x%08x\n", ptr->wGPIODir);
	printf("  GPIO_STATE:       0x%08x\n", ptr->wGPIOState);
	printf("  HW_CONFIG_FLAGS:  0x%04x\n\n", ptr->wHWConfigFlags);    
    
    return 0;
}

U_BOOT_CMD(
	settings_info,   1,   1,     do_settings_info,
	"shows the DHCOM settings",
	"\n"
);

