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
#include <lcd.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/errno.h>

DECLARE_GLOBAL_DATA_PTR;

int do_settings_info ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{                                                                   
    printf("  VALIDATION_ID:    \"%c%c\"\n", (char)gd->dh_board_settings.wValidationID, (char)(gd->dh_board_settings.wValidationID >> 8));
    printf("  DISPLAY_ID:       0x%02x\n", gd->dh_board_settings.cDisplayID);
    printf("  LENGTH:           0x%02x\n", gd->dh_board_settings.cLength);
    printf("  X_RESOLUTION:     %d pixel\n", gd->dh_board_settings.wXResolution);
    printf("  Y_RESOLUTION:     %d pixel\n", gd->dh_board_settings.wYResolution);
    printf("  PIXEL_CLOCK:      %d kHz\n", gd->dh_board_settings.wPixelClock);   
    printf("  LCD_CONFIG_FLAGS: 0x%x\n", gd->dh_board_settings.wLCDConfigFlags);  
    printf("  HSW:              %d pixel clocks\n", gd->dh_board_settings.wHPulseWidth);  
    printf("  VSW:              %d line clocks\n", gd->dh_board_settings.wVPulseWidth);      
    printf("  HFP:              %d pixel clocks\n", gd->dh_board_settings.wHFrontPorch);   
    printf("  HBP:              %d pixel clocks\n", gd->dh_board_settings.wHBackPorch);       
    printf("  VFP:              %d line clocks\n", gd->dh_board_settings.wVFrontPorch);     
    printf("  VBP:              %d line clocks\n", gd->dh_board_settings.wVBackPorch);          
    printf("  DATALINES:        %d\n", gd->dh_board_settings.cDatalines); 
    printf("  ACB:              %d\n", gd->dh_board_settings.cACBiasFreq); 
    printf("  ACBI:             %d\n\n", gd->dh_board_settings.cACBiasTrans);     
    
    printf("  GPIO_DIR:         0x%08x\n", gd->dh_board_settings.wGPIODir);
    printf("  GPIO_STATE:       0x%08x\n", gd->dh_board_settings.wGPIOState);
    printf("  HW_CONFIG_FLAGS:  0x%04x\n\n", gd->dh_board_settings.wHWConfigFlags);    
    
    return 0;
}

U_BOOT_CMD(
	settings,   1,   1,     do_settings_info,
	"shows the DHCOM settings",
	"\n"
);

