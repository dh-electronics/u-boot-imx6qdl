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
#include <i2c.h>

#include <linux/ctype.h>
#include <linux/stddef.h>

#include <asm/io.h>
#include <asm/errno.h>

#define BOOTCFG_EEPROM_ADDR 	0x50
#define BOOTCFG_EEPROM_SIZE	0x0400  /* 8 kbit */
#define BOOTCFG_EEPROM_OFFSET	0
#define BOOTCFG_EEPROM_BLK_SIZE 256  /* eeprom got blocks with 256 bytes,
                                        each block got its own i2c addr */
#define BOOTCFG_READBLOCK	64


#define BOOTCFG_DEFAULT		 "\
active_set=0 state_0=new state_1=app \
md5sum_kernel_0=123456789 md5sum_kernel_1=123456789 \
md5sum_rootfs_0=123456789 md5sum_rootfs_1=123456789 \
md5sum_localfs_0=123456789 md5sum_localfs_1=123456789 \
md5sum_optfs_0=123456789 md5sum_optfs_1=123456789 \
fw_version_0=0 fw_version_1=0"

#define KEY_ACTIVE_SET		"active_set"
#define KEY_STATE		"state_"
#define MAX_KEY_LEN		0x20
#define STATE_VAL_LEN		0x03

enum 	part_state { empty=0, new, try, approved, failed, defect, set_default=failed };
char*  	state_str[6] = { "emp", "new", "try", "app", "fai", "def" };

struct bootcfg {
	uchar	active_set;
	enum part_state state[2];
	char* 	buffer;
	int32_t length;
};

#define STR_LEN_TERMINATED(len) 	len + 1

int read_bootcfg(struct bootcfg* p_cfg);

char* get_value_of_key(char* buffer, char* key)
{
	char* pkey = NULL;

	/* search substring in configuration string */
	pkey = strstr(buffer, key);
	if (pkey == NULL)
		return NULL;

	/* go 'key=' characters ahead to get gointer to matching value */
	pkey += (strlen(key) + 1);
	return pkey;
}

int i2c_write_eeprom(uchar* src_data, uint32_t dest_offset, uint32_t length)
{
	int32_t old_bus = 0;
	int32_t write_index, tx_len, error;
	int32_t ee_addr;
	uchar block_id = 0;

	old_bus = I2C_GET_BUS();
	I2C_SET_BUS(EEPROM_I2C_BUS_NUM);

	/* max 16 byte on one page write */
	for (write_index = 0; write_index < length; write_index +=16) {

	        /* get block ee_addr of eeprom */
	        block_id = 0x07 & (uchar)((dest_offset + write_index) / BOOTCFG_EEPROM_BLK_SIZE);

	        /* setup length of next i2c write */
		if ((length - write_index) < 16) {
			tx_len = length - write_index;
		} else {
		        tx_len = 16;
		}

		/* write only in one block (only to one eeprom ee_addr) */
		if (block_id != (0x07 & ((uchar)((dest_offset + write_index + tx_len - 1 ) / BOOTCFG_EEPROM_BLK_SIZE)))) {

		        /* write only until the end of the block */
		        tx_len = BOOTCFG_EEPROM_BLK_SIZE - ((dest_offset + write_index) % BOOTCFG_EEPROM_BLK_SIZE);
		}


		/* calculate eeprom addr within block */
		ee_addr = dest_offset + write_index;
		ee_addr -= block_id * BOOTCFG_EEPROM_BLK_SIZE; /* substract Block offset (done via i2c ee_addr) */


		error = i2c_write( BOOTCFG_EEPROM_ADDR | block_id,
		                ee_addr,
				1,
				(uchar*)src_data + write_index,
				tx_len);

		if (error) {
		        printf("Error during I2C write operation!\n");
			return -1;
		}

		udelay(5000);
	}

	I2C_SET_BUS(old_bus);
	return 0;
}

int write_bootcfg_chg(struct bootcfg* p_cfg, char* pValue, char* newValue, int32_t length)
{
	uint32_t crc32_calc;
	struct bootcfg bootcfg_verify;
	int compare;

	/* update value in RAM data */
	memcpy(pValue, newValue, length);

	/* calculate crc32 and update RAM data */
	crc32_calc = crc32(0, (uint8_t*)p_cfg->buffer, STR_LEN_TERMINATED(p_cfg->length));
	memcpy(&p_cfg->buffer[STR_LEN_TERMINATED(p_cfg->length)], &crc32_calc, sizeof(uint32_t));

	/* write new value */
	i2c_write_eeprom((uchar*)pValue, (pValue-p_cfg->buffer), length);

	/* update crc32 */
	i2c_write_eeprom((uchar*)&p_cfg->buffer[STR_LEN_TERMINATED(p_cfg->length)], STR_LEN_TERMINATED(p_cfg->length), sizeof(uint32_t));

	/* read back, compare */
	bootcfg_verify.buffer = p_cfg->buffer + BOOTCFG_EEPROM_SIZE;

	if (read_bootcfg(&bootcfg_verify)) {
		return -1;
	}

	compare = memcmp(p_cfg->buffer, bootcfg_verify.buffer, p_cfg->length);
	if (compare) {
		printf  ("Error in write validation at %d! Risk of corrupted data!!\n", compare);
		return -1;
	}
	return 0;
}

int read_bootcfg(struct bootcfg* p_cfg)
{
        int32_t old_bus = 0;
        int32_t idx, error = 0;
        uint32_t crc32_read, crc32_calc;
        uchar block_id = 0;

        old_bus = I2C_GET_BUS();
        I2C_SET_BUS(EEPROM_I2C_BUS_NUM);

        /* read string from eeprom until \0 */
        for (idx = 0; idx < BOOTCFG_EEPROM_SIZE; idx += BOOTCFG_READBLOCK) {

                block_id = 0x07 & (uchar)((BOOTCFG_EEPROM_OFFSET + idx) / BOOTCFG_EEPROM_BLK_SIZE);
                i2c_read(BOOTCFG_EEPROM_ADDR | block_id,
                         (BOOTCFG_EEPROM_OFFSET + idx) - (block_id * BOOTCFG_EEPROM_BLK_SIZE),
                         1,
                         (uchar*)&p_cfg->buffer[idx],
                         BOOTCFG_READBLOCK);

                udelay(50000);

                if (strnlen(&p_cfg->buffer[idx], BOOTCFG_READBLOCK) != BOOTCFG_READBLOCK) {
                        break;
                }
        }

        /* check if no termination was found */
        if (idx >= BOOTCFG_EEPROM_SIZE) {
                printf  ("No 0-termination of string found!\n");
                error = -1;
                goto exit;
        }

        // printf  ("read data = %s\n", p_cfg->buffer);

        /* read crc32 from eeprom */
        p_cfg->length = strnlen(p_cfg->buffer, BOOTCFG_EEPROM_SIZE);
        block_id = 0x07 & (uchar)((BOOTCFG_EEPROM_OFFSET + STR_LEN_TERMINATED(p_cfg->length)) / BOOTCFG_EEPROM_BLK_SIZE);
        i2c_read(   BOOTCFG_EEPROM_ADDR | block_id,
                        BOOTCFG_EEPROM_OFFSET + STR_LEN_TERMINATED(p_cfg->length),
                        1,
                        (uint8_t*)&crc32_read,
                        sizeof(uint32_t));
        udelay(50000);
        crc32_calc = crc32( 0, (uint8_t*)p_cfg->buffer, STR_LEN_TERMINATED(p_cfg->length));
        if  (crc32_calc != crc32_read)  {
                printf  ("CRC32 compare failed!\n");
                error = -1;
                goto exit;
        }

exit:
        I2C_SET_BUS(old_bus);
        return error;
}

int handle_state(struct bootcfg* p_cfg)
{
	char* 	pValue = NULL;
	char 	tmpkey[MAX_KEY_LEN];

	/* build key of active_set state_[01] */
	sprintf (tmpkey, "%s%d", KEY_STATE, p_cfg->active_set);
	pValue = get_value_of_key(p_cfg->buffer, tmpkey);

	/* check for state 'approved' */
	if (0 == strncmp(pValue, state_str[approved], strnlen(state_str[approved], MAX_KEY_LEN)))
		return 0; /* nothing to do */

	/* check for state 'new' */
	if (0 == strncmp(pValue, state_str[new], strnlen(state_str[new], MAX_KEY_LEN))) {
		p_cfg->state[p_cfg->active_set] = try;
		write_bootcfg_chg(	p_cfg,
					pValue,
					state_str[p_cfg->state[p_cfg->active_set]],
					STATE_VAL_LEN);
		return 0;
	}

	/* check for state 'try' */
	if (0 == strncmp(pValue, state_str[try], strnlen(state_str[try], MAX_KEY_LEN))) {
		p_cfg->state[p_cfg->active_set] = failed;
		write_bootcfg_chg(	p_cfg,
					pValue,
					state_str[p_cfg->state[p_cfg->active_set]],
					STATE_VAL_LEN);
		/* switch active partition set - see processing below */
	}

	/* other states then approved or new:
	   -> switch active partition set 		*/
	if (p_cfg->active_set == 0)
		p_cfg->active_set = 1;
	else
		p_cfg->active_set = 0;

	/* build key of active_set state_[01] */
	sprintf (tmpkey, "%s%d", KEY_STATE, p_cfg->active_set);
	pValue = get_value_of_key(p_cfg->buffer, tmpkey);

	/* check for state 'approved' */
	if (0 == strncmp(pValue, state_str[approved], strnlen(state_str[approved], MAX_KEY_LEN))) {
	        /* if state is approved then boot the system */
		return 0;
	}

	return 1; /* error do not boot */
}

int write_default_bootcfg(void)
{
	uint32_t		crc32_calc, length, error;
	uchar* 			buffer;

	/* get pointer to buffer in RAM */
	buffer = (uchar*)simple_strtoul(getenv("loadaddr"), NULL, 16);

	if (NULL == buffer) {
		printf  ("No buffer address defined (loadaddr)!\n");
		return -1;
	}

	length = strnlen(BOOTCFG_DEFAULT , BOOTCFG_EEPROM_SIZE);
	memcpy(buffer, BOOTCFG_DEFAULT, STR_LEN_TERMINATED(length));

	crc32_calc = crc32(0, buffer, STR_LEN_TERMINATED(length));
	memcpy(&buffer[STR_LEN_TERMINATED(length)], &crc32_calc, sizeof(uint32_t));

	error = i2c_write_eeprom(	buffer,
					BOOTCFG_EEPROM_OFFSET,
					(STR_LEN_TERMINATED(length) + sizeof(uint32_t)));

	return error;
}

int process_bootcfg_data(struct bootcfg* tmp_bootcfg)
{
	char* pValue = NULL;

	pValue = get_value_of_key(tmp_bootcfg->buffer, KEY_ACTIVE_SET);
	if (pValue == NULL)	{
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

int prepare_boot(struct bootcfg* p_cfg)
{
	switch (p_cfg->active_set)
	{
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


int display_bootcfg (void)
{
        uint32_t error = 0;
        struct bootcfg tmp_bootcfg;

        /* get pointer to buffer in RAM */
        tmp_bootcfg.buffer = (char*)simple_strtoul(getenv("loadaddr"), NULL, 16);

        if (NULL == tmp_bootcfg.buffer) {
                printf  ("No address defined (loadaddr)!\n");
                error = -1;
                goto exit;
        }

        error = read_bootcfg(&tmp_bootcfg);
        if (error)
                goto exit;

        printf("read data = %s\n", tmp_bootcfg.buffer);

exit:
        return error;
}

int load_bootcfg (void)
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

	error = read_bootcfg(&tmp_bootcfg);
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

int do_cfi2config(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	/* at least two arguments please */
	if (argc < 2)
	goto usage;

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
