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

#define MAVEO_RECOVERY_BOOTARGS                                         \
    "setenv recoveryargs setenv bootargs console=${console} "           \
    "src_intf=mmc src_dev_part=${mmcdev}:1 "                            \
    "vt.global_cursor_default=0 consoleblank=0 dhcom=${dhcom} "         \
    "dhsw=${dhsw} SN=${SN}; "


#define MAVEO_RECOVERY_BOOT                                             \
    "setenv recoveryboot "                                              \
    "echo --> Boot from recovery partition mmc ${mmcdev}:1; "           \
    MAVEO_RECOVERY_BOOTARGS                                             \
    "run recoveryargs; "                                                \
    "echo --> Boot arguments: ${bootargs}; "                            \
    "mmc dev ${mmcdev}:1; "                                             \
    "load mmc ${mmcdev}:1 ${loadaddr} zImage_imx6ull-v1_emmc.update; "  \
    "bootz ${loadaddr}; "


#define MAVEO_BOOTCOMMAND                                               \
    "echo --> Run maveo boot command; "                                 \
    "setenv bootenv_file /boot/u-boot.env; "                            \
    "setenv fdt_addr_r ${fdt_addr}; "                                   \
    "setenv kernel_addr_r ${loadaddr}; "                                \
    "setenv mmcdev 2; "                                                 \
    MAVEO_RECOVERY_BOOT                                                 \
    "mmc dev ${mmcdev}; "                                               \
    "if gpio input 8; then"                                             \
    " echo --> Reset button pressed;"                                   \
    " echo --> Try to start update auto;"                               \
    " setenv auto_update 0; saveenv;"                                   \
    " update auto; "                                                    \
    " echo --> Error: Auto update failed...; "                          \
    " setenv boot_recovery 0; saveenv;"                                 \
    " run recoveryboot; "                                               \
    "fi; "                                                              \
    "if test ${auto_update} -eq 1; then"                                \
    " echo --> Auto update enabled;"                                    \
    " setenv auto_update 0; saveenv;"                                   \
    " update auto; "                                                    \
    "fi; "                                                              \
    "if test ${boot_recovery} -eq 1; then"                              \
    " echo --> Auto boot recovery enabled;"                             \
    " setenv boot_recovery 0; saveenv;"                                 \
    " run recoveryboot; "                                               \
    "fi; "

#endif /* HEADER_ENV_MAVEO_H */
