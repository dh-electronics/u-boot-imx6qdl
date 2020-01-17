/*

  Copyright 2020 nymea GmbH <developer@nymea.io>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

*/

#ifndef HEADER_ENV_MAVEO_H
#define HEADER_ENV_MAVEO_H

#define MAVEO_ENV_SETTINGS                                              \
    "maveo_recoveryargs="                                               \
    "setenv bootargs console=${console} "                               \
    "src_intf=mmc src_dev_part=${mmcdev}:1 "                            \
    "vt.global_cursor_default=0 consoleblank=0 dhcom=${dhcom} "         \
    "dhsw=${dhsw} SN=${SN}\0"                                           \
                                                                        \
    "boot_recovery=0\0"                                                 \
                                                                        \
    "auto_update=0\0"                                                   \
                                                                        \
    "flash_counter=0\0"                                                 \
                                                                        \
    "maveo_recoveryboot="                                               \
    "echo --> Boot from recovery partition mmc ${mmcdev}:1; "           \
    "run maveo_recoveryargs; "                                          \
    "setenv boot_recovery 0; saveenv; "                                 \
    "echo --> Boot arguments: ${bootargs}; "                            \
    "mmc dev ${mmcdev}:1; "                                             \
    "load mmc ${mmcdev}:1 ${loadaddr} zImage_imx6ull-v1_emmc.update; "  \
    "bootz ${loadaddr};\0"                                              \
                                                                        \
    "maveo_autoupdate="                                                 \
    "setenv auto_update 0; saveenv; "                                   \
    "update auto;\0"                                                    \
                                                                        \
    "maveo_resetbuttoncheck="                                           \
    "if gpio input 8; then"                                             \
    " echo --> Reset button pressed;"                                   \
    " echo --> Try to start update auto;"                               \
    " run maveo_autoupdate; "                                           \
    " echo --> Try to start recovery auto;"                             \
    " run maveo_recoveryboot; "                                         \
    "fi;\0"                                                             \
                                                                        \
    "maveo_autoupdatecheck="                                            \
    "if test ${auto_update} -eq 1; then"                                \
    " echo --> Auto update enabled;"                                    \
    " run maveo_autoupdate; "                                           \
    "else"                                                              \
    " echo --> Auto update disabled; "                                  \
    "fi;\0"                                                             \
                                                                        \
    "maveo_autorecoverycheck="                                          \
    "if test ${boot_recovery} -eq 1; then"                              \
    " echo --> Auto boot recovery enabled;"                             \
    " setenv boot_recovery 0; saveenv;"                                 \
    " run maveo_recoveryboot; "                                         \
    "else"                                                              \
    " echo --> Auto boot recovery disabled; "                           \
    "fi;\0"                                                             \
                                                                        \
    "maveoboot="                                                        \
    "echo --> Run maveo boot command; "                                 \
    "setenv bootenv_file /boot/u-boot.env; "                            \
    "setenv fdt_addr_r ${fdt_addr}; "                                   \
    "setenv kernel_addr_r ${loadaddr}; "                                \
    "setenv mmcdev 2; "                                                 \
    "mmc dev ${mmcdev}; "                                               \
    "run maveo_resetbuttoncheck; "                                      \
    "run maveo_autoupdatecheck; "                                       \
    "run maveo_autorecoverycheck;\0"                                    \

#endif /* HEADER_ENV_MAVEO_H */
