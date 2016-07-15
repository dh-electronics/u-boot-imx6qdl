/*
 * File: dh_update.h
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
 *
 *  Version:  1.2
 *
 *  Abstract: This file contains DH electronics specific header file stuff
 *
 *  Notes:
 *  	Created: 		December 05, 2011 by Andreas Geisreiter (ageisreiter@dh-electronics.de)
 *  	Modified: 		Jan 20, 2012 by Ludwig Zenz (lzenz@dh-electronics.de)
 */

#ifndef DH_UPDATE_H
#define DH_UPDATE_H

/*
 * Flash Update defines (Command do_update())
 */
#define UPDATE_FILE_TYPE_NB0                        '1'
#define UPDATE_FILE_TYPE_GZ                         '2'

#define LOAD_UPDATE_KERNEL_LATER                    '0'
#define BOOTLOADER_FLASH_UPDATE                     '1'
#define REFRESH_DH_SETTINGS                         '2'
#define EEPROM_SETTINGS_UPDATE                      '3'
#define EXECUTE_RESET				    '4'
#define EXECUTE_BOOTLOADER_SCRIPT                   '5'
#define WINCE_IMAGE_UPDATE                          '6'
#define EBOOT_IMAGE_UPDATE                          '7'

#define UPDATEINI_ID         "##DHCOMupdate##"
#define UPDATEINI_DISPLAY    "display"
#define UPDATEINI_LED        "led"
#define UPDATEINI_UPDATE     "update"
#define UPDATEINI_END        "end"
#define DHUPDATEINI_MAX_UPDATES 10

typedef struct {
    char *p_cUpdateType;
    char *p_cFilename;
} dhupdates_t;

typedef struct {
    // Valid Marker
    char *p_cValidMarker;
    
    // Display update bmp's
    int iDisplayInfo;
    char *p_cFileNameProgressBmp;
    char *p_cFileNameOkBmp;
    char *p_cFileNameErrorBmp;
    
    // Update GPIO settings
    int iLedInfo;
    char *p_cUpdateGpioName;
    int iUpdateGpioActiveState;  // 0 = low active, 1 = high active
    
    // Updates
    int iUpdateCounter;    
    dhupdates_t stDHUpdateInfo[DHUPDATEINI_MAX_UPDATES];
} updateinfo_t;

enum BitmapTypeEnum
{
   PROGRESS_BITMAP,
   END_BITMAP,
   ERROR_BITMAP
};

enum UpdateGPIOEnum
{
   GPIO_A,
   GPIO_B,
   GPIO_C,
   GPIO_D,
   GPIO_E,
   GPIO_F,
   GPIO_G,
   GPIO_H,
   GPIO_I,
   GPIO_NOT_DEFINED
};

enum UpdateErrorEnum
{
   DHUPDATE_INI_ERROR,
   FILE_NOT_FOUND_ERROR,
   FLASH_ERROR,
   IMAGE_TYPE_ERROR,
   INVALID_FILE_ERROR,
   IMAGE_SIZE_ERROR,
   CANT_LOAD_UPDATE_KERNEL,
   NO_ERROR
};

#endif /* DH_UPDATE_H */


