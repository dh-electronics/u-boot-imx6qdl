/*
 * Control GPIO pins on the fly
 *
 * Copyright (c) 2008-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <common.h>
#include <command.h>

#include <asm/gpio.h>


#define CFI2DIP_NUM_BITS 8 
static const int cfi2dip_bits[CFI2DIP_NUM_BITS] = { 20, 147, 146, 19, 18, 17, 16, 205 };


static int
do_cfi2dip(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[])
{
    int i;
    int err = 0;
    int dipSwitchValue = 0;
    char buffer[64];

    if (argc != 1) {
        return CMD_RET_USAGE;
    }

    for (i = 0; i < CFI2DIP_NUM_BITS; ++i) {
        int bit;

        err = gpio_request(i, "cmd_cfi2dip");
        if (err) {
            printf("cfi2dip: Requesting GPIO pin %u failed!\n", i);
            goto out;
        }

        gpio_direction_input(cfi2dip_bits[i]);
        bit = gpio_get_value(cfi2dip_bits[i]);
        dipSwitchValue |= bit << i;
    }

out:

    for (i = 0; i < CFI2DIP_NUM_BITS; ++i) {
        gpio_free(i);
    }

    printf("DIP Switch Value : %i\n", dipSwitchValue);
    snprintf(buffer, sizeof(buffer), "setenv cfi2_dip %i", dipSwitchValue);
    run_command(buffer, 0);

    return err;
}


U_BOOT_CMD(cfi2dip, 1, 0, do_cfi2dip,
           "Print the CI2 DIP switch value to stdout and set env variable cfi2_dip",
           "\n");
