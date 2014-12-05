/*
 * File: dh_settings.h
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
 *  Version:  1.2
 *
 *  Abstract: This file contains DH electronics specific header file stuff
 *
 *  Changes:
 *  	Created: 		May 13, 2011 by Andreas Geisreiter (ageisreiter@dh-electronics.de)
 *  	Modified: 		Jan 20, 2012 by Ludwig Zenz (lzenz@dh-electronics.de)
 */

#ifndef DH_SETTINGS_H
#define DH_SETTINGS_H

typedef struct settingsinfo {

    u8      cLength;            // Settings block length
    u8      cDisplayID;         // Display unique ID
    u16     wValidationID;      // Settings Block validation ID

    u16     wYResolution;       // Display y resolution
    u16     wXResolution;       // Display x resolution
    
    u16     wLCDConfigFlags;    // Display configuration flags
    u16     wPixelClock;        // Display pixel clock

    u16     wVPulseWidth;       // VSYNC pulse width
    u16     wHPulseWidth;       // HSYNC pulse width

    u16     wHBackPorch;        // HSYNC back porch
    u16     wHFrontPorch;       // HSYNC front porch
    
    u16     wVBackPorch;        // VSYNC back porch
    u16     wVFrontPorch;       // VSYNC front porch   
    
    u8      cACBiasTrans;       // AC Bias transitions per interrupt
    u8      cACBiasFreq;        // AC Bias frequency
    u16     cDatalines;         // Number of display datalines
    
    u32     wGPIODir;           // DHCOM GPIO default direction
    u32     wGPIOState;         // DHCOM GPIO default state

    u16     wHWConfigFlags;     // Hardware configuration flags    
} settingsinfo_t;

#define DHCOM_DISPLAY_SETTINGS_SIZE     0x20

#define SETTINGS_LCD_IVS_FLAG           (0x1 << 0)
#define SETTINGS_LCD_IHS_FLAG           (0x1 << 1)
#define SETTINGS_LCD_IPC_FLAG           (0x1 << 2)
#define SETTINGS_LCD_IOE_FLAG           (0x1 << 3)
#define SETTINGS_LCD_IDATA_FLAG         (0x1 << 4)
#define SETTINGS_LCD_ACT_PAS_FLAG       (0x1 << 5)
#define SETTINGS_LCD_PWM_POL_FLAG       (0x1 << 6)
#define SETTINGS_LCD_BL_EN_GPIO_FLAG    (0xF << 7)
#define SETTINGS_LCD_IBL_FLAG           (0x1 << 11)

#define SETTINGS_HW_SILENT_MODE_FLAG    (0x1 << 0)

#define UPDATE_VIA_MICROSD_SLOT         (0x1 << 1)
#define UPDATE_VIA_SD_MMC_SLOT          (0x1 << 2)
#define UPDATE_VIA_USB_HOST_1_PORT      (0x1 << 3)
#define UPDATE_VIA_USB_OTG_PORT         (0x1 << 4)
#define UPDATE_VIA_USB_HOST_2_PORT      (0x1 << 5)
#define UPDATE_VIA_ETHERNET             (0x1 << 6)

#define SETTINGS_HW_LCD_MODE_FLAG       (0x1 << 7)
#define SETTINGS_HW_DIS_CON_FLAG        (0x1 << 8)
#define SETTINGS_HW_EN_DISP_ADPT_EE_CHK (0x1 << 9)

#define DISPLAY_ADAPTER_EEPROM_ADDR     0x50

#endif /* DH_SETTINGS_H */
