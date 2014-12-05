/*
 * File: mcp7941x_rtc.c
 *
 * (C) Copyright 2011-2012
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
 * brief:		defines for using mcp7941x rtc's
 */

#ifndef RV_3029_RTC_H
#define RV_3029_RTC_H

//=================================================================================
//                    GLOBAL CONSTANTS RTCC - ADDRESSES
//=================================================================================
     #define  DEVICE_ADDR_RTC   0x56       //  DEVICE ADDR
//.................................................................................
     #define  ADDR_CONTROL_1    0x00       //  RTC Control register
//.................................................................................
     #define  ADDR_SEC          0x08       //  address of SECONDS      register
     #define  ADDR_MIN          0x09       //  address of MINUTES      register
     #define  ADDR_HOUR         0x0A       //  address of HOURS        register
     #define  ADDR_DAY_WE       0x0C       //  address of DAY OF WK    register
     #define  ADDR_DAY          0x0B       //  address of DATE         register
     #define  ADDR_MNTH         0x0D       //  address of MONTH        register
     #define  ADDR_YEAR         0x0E       //  address of YEAR         register

//.................................................................................

//=================================================================================
//                  GLOBAL CONSTANTS RTCC - INITIALIZATION
//..................................................................................


#endif

//-->> end of file
