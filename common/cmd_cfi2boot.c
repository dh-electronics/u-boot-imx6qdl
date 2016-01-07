/*
 * File: cmd_cfi2boot.c
 *
 * (C) Copyright 2011-2014
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
 *
 */
#include <common.h>

#if defined(CONFIG_CMD_CFI2BOOT)
#include <command.h>
#include <environment.h>
#include <malloc.h>
#include <i2c.h>

#include <linux/ctype.h>
#include <linux/stddef.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/errno.h>

#define HW_CODE_BIT_0	IMX_GPIO_NR(2, 19)
#define HW_CODE_BIT_1	IMX_GPIO_NR(6, 6)
#define HW_CODE_BIT_2	IMX_GPIO_NR(2, 16)

#define BOOTCFG_EEPROM_ADDR            0x50
#define BOOTCFG_EEPROM_SIZE            0x0400 /* 8 kbit */
#define BOOTCFG_EEPROM_OFFSET          0
#define BOOTCFG_EEPROM_BLOCKS          4 /* The number of blocks located on the EEPROM. */
#define BOOTCFG_EEPROM_BLK_SIZE        256 /* eeprom got blocks with 256 bytes, each block got its own i2c addr */
#define BOOTCFG_EEPROM_PAGE_SIZE       16 /**< The EEPROM has pages made of 16 bytes (only relevent for page writes). */
#define BOOTCFG_READBLOCK              64
#define MAX_WRITE_TRYS                 5 /* max number of trys if writing to eeprom fails */

#define BOOTCFG_DEFAULT                "\
active_set=0 state_0=new state_1=app \
md5sum_kernel_0=123456789 md5sum_kernel_1=123456789 \
md5sum_rootfs_0=123456789 md5sum_rootfs_1=123456789 \
md5sum_localfs_0=123456789 md5sum_localfs_1=123456789 \
md5sum_optfs_0=123456789 md5sum_optfs_1=123456789 \
fw_version_0=0 fw_version_1=0"

#define KEY_ACTIVE_SET                 "active_set"
#define KEY_STATE                      "state_"
#define MAX_KEY_LEN                    0x20
#define STATE_VAL_LEN                  0x03
#define KEY_ACTIVE_VAL_LEN             0x01

enum part_state {
    empty = 0,
    new = 1,
    try = 2,
    approved = 3,
    failed = 4,
    defect = 5,
    set_default = failed
};

const char * STATE_STR[6] = { "emp", "new", "try", "app", "fai", "def" };
const char * ACTIVE_SET_STR[2] = { "0", "1" };

struct bootcfg {
    uchar active_set;
    enum part_state state[2];
    char * buffer;
    int32_t length;
};

#define STR_LEN_TERMINATED(len) len + 1

static unsigned int cfi2_i2cbus = 1;

void
determine_i2cbus(void)
{
        /*
         * DHCOM I2C ports are swapped between
         * hardware versions 200 and 300.
         *
         * Get hw version an select i2c bus which
         * is connected to the cfi2 boot eeprom.
         */
        u32 hw_code = 0;

	gpio_direction_input(HW_CODE_BIT_0);
	gpio_direction_input(HW_CODE_BIT_1);
	gpio_direction_input(HW_CODE_BIT_2);

	// i.MX6: HW 100 + HW 200 = 00b; HW 300 = 01b
	hw_code = ((gpio_get_value(HW_CODE_BIT_2) << 2) |
	           (gpio_get_value(HW_CODE_BIT_1) << 1) |
	            gpio_get_value(HW_CODE_BIT_0)) + 2;

	if (hw_code < 3) {
	        // HW 100 + HW 200
	        cfi2_i2cbus = 0;
	} else {
	        // HW 300 and further
	        cfi2_i2cbus = 1;
	}
}

int
read_bootcfg(struct bootcfg * p_cfg, int check_crc)
{
    int32_t old_bus = 0;
    int32_t idx, error = 0;
    uint32_t crc32_read, crc32_calc;
    uchar block_id = 0;

    old_bus = I2C_GET_BUS();
    I2C_SET_BUS(cfi2_i2cbus);

    /* read string from eeprom until \0 */
    for (idx = 0; idx < BOOTCFG_EEPROM_SIZE; idx += BOOTCFG_READBLOCK) {
        block_id = 0x07 & (uchar)((BOOTCFG_EEPROM_OFFSET + idx) / BOOTCFG_EEPROM_BLK_SIZE);
        error = i2c_read(BOOTCFG_EEPROM_ADDR | block_id,
                       (BOOTCFG_EEPROM_OFFSET + idx) - (block_id * BOOTCFG_EEPROM_BLK_SIZE),
                       1,
                       (uchar*)&p_cfg->buffer[idx],
                       BOOTCFG_READBLOCK);
        if (error) {
            printf("In i2c_read(): %i\n", error);
            goto exit;
        }

        if (strnlen(&p_cfg->buffer[idx], BOOTCFG_READBLOCK) != BOOTCFG_READBLOCK) {
            break;
        }
    }

    /* check if no termination was found */
    if (idx >= BOOTCFG_EEPROM_SIZE) {
        printf("No 0-termination found in EEPROM content!\n");
        error = -EINVAL;
        goto exit;
    }

    /* read crc32 from eeprom */
    p_cfg->length = strnlen(p_cfg->buffer, BOOTCFG_EEPROM_SIZE);
    block_id = 0x07 & (uchar)((BOOTCFG_EEPROM_OFFSET + STR_LEN_TERMINATED(p_cfg->length)) / BOOTCFG_EEPROM_BLK_SIZE);
    error = i2c_read(BOOTCFG_EEPROM_ADDR | block_id,
                     BOOTCFG_EEPROM_OFFSET + STR_LEN_TERMINATED(p_cfg->length),
                     1,
                     (uint8_t*)&crc32_read,
                     sizeof(uint32_t));
    if (error) {
        printf("In i2c_read(): %i\n", error);
        goto exit;
    }

    crc32_calc = crc32( 0, (uint8_t*)p_cfg->buffer, STR_LEN_TERMINATED(p_cfg->length));
    if  (crc32_calc != crc32_read)  {
        printf("Calculated CRC 0x%08X differs from read CRC 0x%08X!\n", crc32_calc, crc32_read);
        error = check_crc ? -EINVAL : 0;
        goto exit;
    }

exit:

    I2C_SET_BUS(old_bus);

    return error;
}


/*
 * search key-string in buffer-string
 * and return pointer to value-string
 * */
char *
get_value_of_key(char * buffer, char * key)
{
    char * pkey = NULL;

    /* search substring in configuration string */
    pkey = strstr(buffer, key);
    if (pkey == NULL) {
        return NULL;
    }

    /* go 'key=' characters ahead to get gointer to matching value */
    pkey += (strlen(key) + 1);

    return pkey;
}


/**
 * @brief This function writes as much bytes as possible to one single page.
 *        The EEPROM is able to accept page writes with up to pagesize bytes.
 *        The page size of the 24AA08 EEPROM is 16 bytes. Pagesize bytes can
 *        only be written from the start of a page.  Writing to the middle of
 *        a page reduces the maximum number of bytes wich can be written to
 *        pagesize minus page offset.
 *
 * @param block The index of the EEPROM block to write to.
 * @param offset The offset within the EEPROM block to write to.
 * @param data The data to write to the EEPROM.
 * @param length The length of the data.
 * @return The number of bytes written on success, negative error number on failure.
 */
static int
cfi2_eeprom_write_to_page(uint32_t block, uint32_t offset, uint8_t * data, uint32_t length)
{
    int err = 0;

    printf("block=%u offset=%u data=%p length=%u\n", block, offset, data, length);

    if (block >= BOOTCFG_EEPROM_BLOCKS) {
        printf("Block index %u is exceeds maximum number of blocks %u!\n", block, BOOTCFG_EEPROM_BLOCKS);
        return -EINVAL;
    }

    if (offset >= BOOTCFG_EEPROM_BLK_SIZE) {
        printf("Block offset %u is greater than block size %u!\n", offset, BOOTCFG_EEPROM_BLK_SIZE);
        return -EINVAL;
    }

    uint32_t page = offset / BOOTCFG_EEPROM_PAGE_SIZE; /* The number of the page to write to. */
    uint32_t space = (page + 1) * BOOTCFG_EEPROM_PAGE_SIZE - offset; /* Space to the end of the page. */
    uint32_t truncated = min(length, space); /* Limit either to data length or page space. */
    uint8_t buffer[truncated];
    int attempts = 5;

    int old_bus = I2C_GET_BUS();
    I2C_SET_BUS(cfi2_i2cbus);

    do {
        printf("i2c_write(%i, %i, %i, %p, %i)\n", BOOTCFG_EEPROM_ADDR | block, offset, 1, data, truncated);
        err = i2c_write(BOOTCFG_EEPROM_ADDR | block, offset, 1, data, truncated);
        udelay(50000); /* The EEPROM stores the data asynchronosly, thus give it some time to do the job. */
        if (err) {
            printf("In i2c_write(%i, %i, %i, %p, %i): %i!\n", BOOTCFG_EEPROM_ADDR | block, offset, 1, data, truncated, err);
            continue;
        }

        printf("i2c_read(%i, %i, %i, %p, %i)\n", BOOTCFG_EEPROM_ADDR | block, offset, 1, buffer, truncated);
        err = i2c_read(BOOTCFG_EEPROM_ADDR | block, offset, 1, buffer, truncated);
        if (err) {
            printf("In i2c_read(%i, %i, %i, %p, %i): %i!\n", BOOTCFG_EEPROM_ADDR | block, offset, 1, buffer, truncated, err);
            continue;
        }

        int not_equal = memcmp(buffer, data, truncated);
        if (not_equal) {
            int i;

            printf("Data read back (");
            for (i = 0; i < truncated; ++i) {
                printf("%c", buffer[i]);
            }
            printf(") does not match data written (");
            for (i = 0; i < truncated; ++i) {
                printf("%c", data[i]);
            }
            printf(")!\n");

            err = -EIO;
            continue;
        }

        err = truncated; /* Return the number of bytes actually written. */
        break;
    } while (--attempts);

    I2C_SET_BUS(old_bus);

    return err;
}


/**
 * @brief Writes to the EEPROM block.
 *
 * @param data The data to write.
 * @param offset The EEPROM offset to write to.
 * @param length The length of data.
 * @return The number of bytes written on success, negative number on failure.
 */
static int
cfi2_eeprom_write(uint8_t * data, uint32_t offset, uint32_t length)
{
    if (offset >= BOOTCFG_EEPROM_SIZE) {
        printf("Offset %u exceeds EEPROM size %u!\n", offset, BOOTCFG_EEPROM_SIZE);
        return -EINVAL;
    }

    if ((offset + length) > BOOTCFG_EEPROM_SIZE) {
        printf("Buffer length %u at offset %u exceeds EEPROM size %u!\n", length, offset, BOOTCFG_EEPROM_SIZE);
        return -EINVAL;
    }

    uint32_t pointer = 0;
    uint32_t remaining = length;

    while (remaining) {
        uint32_t block = (offset + pointer) / BOOTCFG_EEPROM_BLK_SIZE;
        uint32_t block_offset = (offset + pointer) % BOOTCFG_EEPROM_BLK_SIZE;

        int err = cfi2_eeprom_write_to_page(block, block_offset, &data[pointer], remaining);
        if (err < 0) {
            return err;
        }

        pointer += err;
        remaining -= err;
    }

    return 0;
}


static int
cfi2_eeprom_update_keyval(struct bootcfg * p_cfg, char * pValue, const char * newValue, int32_t length)
{
    uint32_t crc32_calc;
    struct bootcfg bootcfg_verify;
    int compare;
    int err;

    /* update value in RAM data */
    memcpy(pValue, newValue, length);

    /* calculate crc32 and update RAM data */
    crc32_calc = crc32(0, (uint8_t*)p_cfg->buffer, STR_LEN_TERMINATED(p_cfg->length));
    printf("New CRC id 0x%08X\n", crc32_calc);

    memcpy(&p_cfg->buffer[STR_LEN_TERMINATED(p_cfg->length)], &crc32_calc, sizeof(uint32_t));

    /* write new value */
    err = cfi2_eeprom_write((uint8_t *)pValue, pValue - p_cfg->buffer, length);
    if (err) {
        return err;
    }

    /* update crc32 */
    err = cfi2_eeprom_write((uint8_t *)&p_cfg->buffer[STR_LEN_TERMINATED(p_cfg->length)], STR_LEN_TERMINATED(p_cfg->length), sizeof(uint32_t));
    if (err) {
        return err;
    }

    /* read back, compare */
    bootcfg_verify.buffer = p_cfg->buffer + BOOTCFG_EEPROM_SIZE;

    if (read_bootcfg(&bootcfg_verify, 1)) {
        return -1;
    }

    compare = memcmp(p_cfg->buffer, bootcfg_verify.buffer, p_cfg->length);
    if (compare) {
        printf  ("Error in write validation at %d! Risk of corrupted data!!\n", compare);
        return -1;
    }

    return 0;
}


int
handle_state(struct bootcfg * p_cfg)
{
    char * pValue = NULL;
    char tmpkey[MAX_KEY_LEN];

    /* build key-string of active_set state_[01] */
    sprintf(tmpkey, "%s%d", KEY_STATE, p_cfg->active_set);

    /* search key-string in read config-string and get pointer to value-string*/
    pValue = get_value_of_key(p_cfg->buffer, tmpkey);
    if (pValue == NULL) {
        printf("ERROR: Key \"%s\" not found!\n", tmpkey);
        return -1;
    }

    /* check for state 'approved' */
    if (0 == strncmp(pValue, STATE_STR[approved], strnlen(STATE_STR[approved], MAX_KEY_LEN))) {
        /* nothing to do - boot system without change of bootcfg */
        return 0;
    }

    /* check for state 'new' */
    if (0 == strncmp(pValue, STATE_STR[new], strnlen(STATE_STR[new], MAX_KEY_LEN))) {
        /* updated bootcfg - change state to 'try' */
        p_cfg->state[p_cfg->active_set] = try;
        return cfi2_eeprom_update_keyval(p_cfg, pValue, STATE_STR[p_cfg->state[p_cfg->active_set]], STATE_VAL_LEN);
    }

    /* check for state 'try' */
    if (0 == strncmp(pValue, STATE_STR[try], strnlen(STATE_STR[try], MAX_KEY_LEN))) {
        /* already state try - there must have been a failed boot-process - set to failded */
        p_cfg->state[p_cfg->active_set] = failed;
        int err = cfi2_eeprom_update_keyval(p_cfg, pValue, STATE_STR[p_cfg->state[p_cfg->active_set]], STATE_VAL_LEN);        
        if (err) { return err; }

        /* switch active partition set - see processing below */
    }

    /* other states then approved or new: -> switch active partition set */
    if (p_cfg->active_set == 0) {
        p_cfg->active_set = 1;
    } else {
        p_cfg->active_set = 0;
    }

    /* build key of active_set state_[01] */
    sprintf (tmpkey, "%s%d", KEY_STATE, p_cfg->active_set);
    pValue = get_value_of_key(p_cfg->buffer, tmpkey);

    /* check for state 'approved' */
    if (0 == strncmp(pValue, STATE_STR[approved], strnlen(STATE_STR[approved], MAX_KEY_LEN))) {

        /* write that we have switched the active_set */

        /* search key-string in read config-string and get pointer to value-string*/
        pValue = get_value_of_key(p_cfg->buffer, KEY_ACTIVE_SET);
        if (pValue == NULL) {
            printf("ERROR: Key \"%s\" not found!\n", tmpkey);
            return -1;
        }

        /* write new active_set value to eeprom */
        return cfi2_eeprom_update_keyval(p_cfg, pValue, ACTIVE_SET_STR[p_cfg->active_set], KEY_ACTIVE_VAL_LEN);        
    }

    printf("ERROR: No approved partition set found, do not boot!\n");

    return -1;
}


int
write_default_bootcfg(void)
{
    uint32_t crc32_calc, length;
    uchar * buffer;

    /* get pointer to buffer in RAM */
    buffer = (uchar*)simple_strtoul(getenv("loadaddr"), NULL, 16);
    if (NULL == buffer) {
        printf("No buffer address defined (loadaddr)!\n");
        return -1;
    }

    length = strnlen(BOOTCFG_DEFAULT , BOOTCFG_EEPROM_SIZE);
    memcpy(buffer, BOOTCFG_DEFAULT, STR_LEN_TERMINATED(length));

    crc32_calc = crc32(0, buffer, STR_LEN_TERMINATED(length));
    memcpy(&buffer[STR_LEN_TERMINATED(length)], &crc32_calc, sizeof(uint32_t));

    return cfi2_eeprom_write(buffer, BOOTCFG_EEPROM_OFFSET, (STR_LEN_TERMINATED(length) + sizeof(uint32_t)));
}


int
process_bootcfg_data(struct bootcfg * tmp_bootcfg)
{
    char* pValue = NULL;

    pValue = get_value_of_key(tmp_bootcfg->buffer, KEY_ACTIVE_SET);
    if (pValue == NULL)    {
        printf  ("Key '%s' is not available!\n", KEY_ACTIVE_SET);
        return -1; /* do not boot - goto uboot console */
    }

    switch (*pValue) {
        case '0':
            tmp_bootcfg->active_set = 0;
            break;
        case '1':
            tmp_bootcfg->active_set = 1;
            break;
        default:
            return -1; /* do not boot - go to uboot console */
    }

    if (0 != handle_state(tmp_bootcfg))
        return -1; /* do not boot - go to uboot console */
    return 0;
}


int
prepare_boot(struct bootcfg * p_cfg)
{
    switch (p_cfg->active_set) {
        case 0:
            run_command("setenv mmc_boot_part 1",0);
            run_command("setenv mmc_rootfs_part 2",0);
            run_command("setenv booted_set 0",0);
            break;
        case 1:
            run_command("setenv mmc_boot_part 3",0);
            run_command("setenv mmc_rootfs_part 4",0);
            run_command("setenv booted_set 1",0);
            break;
        default:
            return -1;
    }

    return 0;
}


int
display_bootcfg(void)
{
    uint32_t error = 0;
    struct bootcfg tmp_bootcfg;
    unsigned int i;

    /* get pointer to buffer in RAM */
    tmp_bootcfg.buffer = (char*)simple_strtoul(getenv("loadaddr"), NULL, 16);

    if (NULL == tmp_bootcfg.buffer) {
        printf  ("No address defined (loadaddr)!\n");
        error = -1;
        goto exit;
    }

    error = read_bootcfg(&tmp_bootcfg, 0);
    if (error)
        goto exit;

    for (i = 0; i < strlen(tmp_bootcfg.buffer); ++i) {
        printf("%c", tmp_bootcfg.buffer[i]);
    }
    printf("\n");

exit:

    return error;
}


int
load_bootcfg(void)
{
    uint32_t error = 0;
    struct bootcfg tmp_bootcfg;

    /* get pointer to buffer in RAM */
    tmp_bootcfg.buffer = (char*)simple_strtoul(getenv("loadaddr"), NULL, 16);

    if (NULL == tmp_bootcfg.buffer) {
        printf("No address defined (loadaddr)!\n");
        error = -1;
        goto exit;
    }

    error = read_bootcfg(&tmp_bootcfg, 1);
    if (error)
        goto exit;

    /* data is valid -> process data */
    if (0 == process_bootcfg_data(&tmp_bootcfg)) {
        error = prepare_boot(&tmp_bootcfg);
    } else {
        error = -1;
    }

exit:

    return error;
}


int
do_cfi2config(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[])
{
    /* at least two arguments please */
    if (argc < 2)
        goto usage;

    /* determine i2cbus of boot eeprom, differs on hw version */
    determine_i2cbus();

    /* load boot configuration from i2c eeprom */
    if (strcmp(argv[1], "load") == 0)
        return load_bootcfg();

    /* display boot configuration content of i2c eeprom */
    if (strcmp(argv[1], "display") == 0)
        return display_bootcfg();

    /* save boot configuration to i2c eeprom */
    if (strcmp(argv[1], "default") == 0)
        return write_default_bootcfg();

usage:

    cmd_usage(cmdtp);

    return 1;
}


/* ====================================================================== */

U_BOOT_CMD(
        cfi2config, 3, 1, do_cfi2config,
        "cfi2config",
        "---------------------------\n\n"
        "cfi2config load    - load bootconfig from eeprom\n"
        "cfi2config display - show bootconfig stored in eeprom\n"
        "cfi2config default - save default bootconfig into eeprom\n"
        );

#endif /* CONFIG_CMD_CFI2BOOT */
