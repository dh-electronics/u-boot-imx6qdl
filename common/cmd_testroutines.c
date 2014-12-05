/*
 * File: cmd_testroutines.c
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
 * brief:		Collection of different HW-tests
 * description:	This test-routines are only for development.
 * 				They can be excluded for customer versions/builds (#undef CONFIG_CMD_TESTROUTINES).
 *
 */

#include <common.h>
#include <command.h>

#include <linux/ctype.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch/gpio.h>

#include <i2c.h>
#include <watchdog.h>
#include <malloc.h>
#include <asm/byteorder.h>
#include <jffs2/jffs2.h>
#include <s_record.h>
#include <ata.h>
#include <part.h>
#include <net.h>
#include <fat.h>

		   // use i2c functionality
#include <RV_3029_rtc.h>  // defines to use the rtc mcp79411

#include <dh_settings.h>

DECLARE_GLOBAL_DATA_PTR;

#ifdef	CMD_MEM_DEBUG
#define	PRINTF(fmt,args...)	printf (fmt ,##args)
#else
#define PRINTF(fmt,args...)
#endif

#if defined(CONFIG_CMD_TESTROUTINES)
#define DEVELOPERTESTS

/*
 * Perform a memory test on external sram. The address-space is hardcoded!!
 * This routine is similar to the more complete alternative test which can be configure
 * with CONFIG_SYS_ALT_MEMTEST for the mtest command.
 */
int ext_sram_testdh(int iteration_limit)
{
	vu_long	*addr, *start, *end;
	ulong	val;
	ulong	readback;
	ulong	errs = 0;
	int iterations = 1;

	vu_long	len;
	vu_long	offset;
	vu_long	test_offset;
	vu_long	pattern;
	vu_long	temp;
	vu_long	anti_pattern;
	vu_long	num_words;
	vu_long *dummy ;
	int	j;

	static const ulong bitpattern[] = {
		0x00000001,	/* single bit */
		0x00000003,	/* two adjacent bits */
		0x00000007,	/* three adjacent bits */
		0x0000000F,	/* four adjacent bits */
		0x00000005,	/* two non-adjacent bits */
		0x00000015,	/* three non-adjacent bits */
		0x00000055,	/* four non-adjacent bits */
		0xaaaaaaaa,	/* alternating 1/0 */
	};

	// set start and stop address from board configuration -> A16 and A17 are not connected -> max 128 kByte
	start = (ulong *)(0x08000000);
	end = (ulong *)(0x0801FFFF);

	// choose test pattern
	pattern = 0;

	printf ("Testing %08x ... %08x:\n", (uint)start, (uint)end);
	PRINTF("%s:%d: start 0x%p end 0x%p\n",	__FUNCTION__, __LINE__, start, end);

        dummy = map_sysmem(CONFIG_SYS_MEMTEST_SCRATCH, sizeof(vu_long));

	for (;;) {

		// check if count of test-loops reached
		if (iteration_limit && iterations > iteration_limit)
		{
			printf("Tested %d iteration(s) with %lu errors.\n",	iterations-1, errs);
			return errs != 0;
		}

		printf("Iteration: %6d\r", iterations);
		PRINTF("\n");
		iterations++;

		/*
		 * Data line test:
		 * write a pattern to the first location, write the 1's complement to a 'parking'
		 * address (changes the state of the data bus so a floating bus doen't give a false OK), and then
		 * read the value back. Note that we read it back into a variable because the next time we read it,
		 * it might be right (been there, tough to explain to the quality guys why it prints a failure when the
		 * "is" and "should be" are obviously the same in the error message).
		 *
		 * Rather than exhaustively testing, we test some
		 * patterns by shifting '1' bits through a field of
		 * '0's and '0' bits through a field of '1's (i.e.
		 * pattern and ~pattern).
		 */
		addr = start;
		for (j = 0; j < sizeof(bitpattern)/sizeof(bitpattern[0]); j++)
		{
			val = bitpattern[j];
			for(; val != 0; val <<= 1)
			{
				*addr  = val;
				*dummy  = ~val; /* clear the test data off of the bus */
				readback = *addr;
				if(readback != val)
				{
					printf ("FAILURE (data line): expected %08lx, actual %08lx\n", val, readback);
					errs++;
				}
				*addr  = ~val;
				*dummy  = val;
				readback = *addr;
				if(readback != ~val)
				{
					printf ("FAILURE (data line): Is %08lx, should be %08lx\n",	readback, ~val);
					errs++;
				}
			}
		}

		/*
		 * Based on code whose Original Author and Copyright
		 * information follows: Copyright (c) 1998 by Michael
		 * Barr. This software is placed into the public
		 * domain and may be used for any purpose. However,
		 * this notice must not be changed or removed and no
		 * warranty is either expressed or implied by its
		 * publication or distribution.
		 */

		/*
		 * Address line test
		 *
		 * Description: Test the address bus wiring in a memory region by performing a walking
		 *              1's test on the relevant bits of the address and checking for aliasing.
		 *              This test will find single-bit address failures such as stuck -high,
		 *              stuck-low, and shorted pins. The base address and size of the region are
		 *              selected by the caller.
		 *
		 * Notes:	For best results, the selected base address should have enough LSB 0's to
		 *              guarantee single address bit changes. For example, to test a 64-Kbyte
		 *              region, select a base address on a 64-Kbyte boundary. Also, select the
		 *              region size as a power-of-two if at all possible.
		 */
		len = ((ulong)end - (ulong)start)/sizeof(vu_long);
		pattern = (vu_long) 0xaaaaaaaa;
		anti_pattern = (vu_long) 0x55555555;

		PRINTF("%s:%d: length = 0x%.8lx\n",	__FUNCTION__, __LINE__,	len);
		/*
		 * Write the default pattern at each of the
		 * power-of-two offsets.
		 */
		for (offset = 1; offset < len; offset <<= 1)
		{
			start[offset] = pattern;
		}

		/*
		 * Check for address bits stuck high.
		 */
		test_offset = 0;
		start[test_offset] = anti_pattern; // change state of lsb-address-line
										   // write anti-pattern if lsb-address is defect you would write to offset=1 and not test_offset=0

		for (offset = 1; offset < len; offset <<= 1)
		{
			temp = start[offset];
			if (temp != pattern)
			{
			  printf ("\nFAILURE: Address bit stuck high @ 0x%.8lx: expected 0x%.8lx, actual 0x%.8lx\n", (ulong)&start[offset], pattern, temp);
			  errs++;
			}
		}
		start[test_offset] = pattern;
		WATCHDOG_RESET();

		/*
		 * Check for addr bits stuck low or shorted.
		 */
		for (test_offset = 1; test_offset < len; test_offset <<= 1)
		{
			start[test_offset] = anti_pattern;

			for (offset = 1; offset < len; offset <<= 1)
			{
			  temp = start[offset];
			  if ((temp != pattern) && (offset != test_offset))
			  {
				printf ("\nFAILURE: Address bit stuck low or shorted @ 0x%.8lx: expected 0x%.8lx, actual 0x%.8lx\n", (ulong)&start[offset], pattern, temp);
				errs++;
			  }
			}
			start[test_offset] = pattern;
		}

		/*
		 * Description:
		 * Test the integrity of a physical memory device by performing an
		 * increment/decrement test over the entire region. In the process every
		 * storage bit in the device is tested as a zero and a one. The base address
		 * and the size of the region are selected by the caller.
		 */

		num_words = ((ulong)end - (ulong)start)/sizeof(vu_long) + 1;

		/*
		 * Fill memory with a known pattern.
		 */
		for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++)
		{
			WATCHDOG_RESET();
			start[offset] = pattern;
		}

		/*
		 * Check each location and invert it for the second pass.
		 */
		for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++)
		{
			WATCHDOG_RESET();
			temp = start[offset];
			if (temp != pattern)
			{
			  printf ("\nFAILURE (read/write) @ 0x%.8lx: expected 0x%.8lx, actual 0x%.8lx)\n", (ulong)&start[offset], pattern, temp);
			  errs++;
			}

			anti_pattern = ~pattern;
			start[offset] = anti_pattern;
		}

		/*
		 * Check each location for the inverted pattern and zero it.
		 */
		for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++)
		{
			WATCHDOG_RESET();
			anti_pattern = ~pattern;
			temp = start[offset];
			if (temp != anti_pattern)
			{
			  printf ("\nFAILURE (read/write): @ 0x%.8lx: expected 0x%.8lx, actual 0x%.8lx)\n",	(ulong)&start[offset], anti_pattern, temp);
			  errs++;
			}
			start[offset] = 0;
		}
	}
}

/*
 * Perform a memory test. This routine is similar to the more complete alternative test which can be configure
 * with CONFIG_SYS_ALT_MEMTEST for the mtest command. The complete test loops until
 * interrupted by a failure of one of the sub-tests or test is finished.
 */
int ddr3_testdh(int iteration_limit)
{
	vu_long	*addr, *start, *end;
	ulong	val;
	ulong	readback;
	ulong	errs = 0;
	int iterations = 1;

	vu_long	len;
	vu_long	offset;
	vu_long	test_offset;
	vu_long	pattern;
	vu_long	temp;
	vu_long	anti_pattern;
	vu_long	num_words;
	vu_long *dummy;
	int	j;

	static const ulong bitpattern[] = {
		0x00000001,	/* single bit */
		0x00000003,	/* two adjacent bits */
		0x00000007,	/* three adjacent bits */
		0x0000000F,	/* four adjacent bits */
		0x00000005,	/* two non-adjacent bits */
		0x00000015,	/* three non-adjacent bits */
		0x00000055,	/* four non-adjacent bits */
		0xaaaaaaaa,	/* alternating 1/0 */
	};

	// set start and stop address from board configuration
	start = (ulong *)CONFIG_SYS_MEMTEST_START;
	end = (ulong *)(CONFIG_SYS_MEMTEST_END);

	// choose test pattern
	pattern = 0;

	// configure hardcoded iteration limit
//	iteration_limit = 1;

	printf ("Testing %08x ... %08x:\n", (uint)start, (uint)end);
	PRINTF("%s:%d: start 0x%p end 0x%p\n",	__FUNCTION__, __LINE__, start, end);

	dummy = map_sysmem(CONFIG_SYS_MEMTEST_SCRATCH, sizeof(vu_long));

	for (;;) {

		// check if count of test-loops reached
		if (iteration_limit && iterations > iteration_limit)
		{
			printf("Tested %d iteration(s) with %lu errors.\n",	iterations-1, errs);
			return errs != 0;
		}

		printf("Iteration: %6d\r", iterations);
		PRINTF("\n");
		iterations++;

		/*
		 * Data line test:
		 * write a pattern to the first location, write the 1's complement to a 'parking'
		 * address (changes the state of the data bus so a floating bus doen't give a false OK), and then
		 * read the value back. Note that we read it back into a variable because the next time we read it,
		 * it might be right (been there, tough to explain to the quality guys why it prints a failure when the
		 * "is" and "should be" are obviously the same in the error message).
		 *
		 * Rather than exhaustively testing, we test some
		 * patterns by shifting '1' bits through a field of
		 * '0's and '0' bits through a field of '1's (i.e.
		 * pattern and ~pattern).
		 */
		addr = start;
		for (j = 0; j < sizeof(bitpattern)/sizeof(bitpattern[0]); j++)
		{
			val = bitpattern[j];
			for(; val != 0; val <<= 1)
			{
				*addr  = val;
				*dummy  = ~val; /* clear the test data off of the bus */
				readback = *addr;
				if(readback != val)
				{
					printf ("FAILURE (data line): expected %08lx, actual %08lx\n", val, readback);
					errs++;
				}
				*addr  = ~val;
				*dummy  = val;
				readback = *addr;
				if(readback != ~val)
				{
					printf ("FAILURE (data line): Is %08lx, should be %08lx\n",	readback, ~val);
					errs++;
				}

			}
		}

		/*
		 * Based on code whose Original Author and Copyright
		 * information follows: Copyright (c) 1998 by Michael
		 * Barr. This software is placed into the public
		 * domain and may be used for any purpose. However,
		 * this notice must not be changed or removed and no
		 * warranty is either expressed or implied by its
		 * publication or distribution.
		 */

		/*
		 * Address line test
		 *
		 * Description: Test the address bus wiring in a memory region by performing a walking
		 *              1's test on the relevant bits of the address and checking for aliasing.
		 *              This test will find single-bit address failures such as stuck -high,
		 *              stuck-low, and shorted pins. The base address and size of the region are
		 *              selected by the caller.
		 *
		 * Notes:	For best results, the selected base address should have enough LSB 0's to
		 *              guarantee single address bit changes. For example, to test a 64-Kbyte
		 *              region, select a base address on a 64-Kbyte boundary. Also, select the
		 *              region size as a power-of-two if at all possible.
		 */
		len = ((ulong)end - (ulong)start)/sizeof(vu_long);
		pattern = (vu_long) 0xaaaaaaaa;
		anti_pattern = (vu_long) 0x55555555;

		PRINTF("%s:%d: length = 0x%.8lx\n",	__FUNCTION__, __LINE__,	len);
		/*
		 * Write the default pattern at each of the
		 * power-of-two offsets.
		 */
		for (offset = 1; offset < len; offset <<= 1)
		{
			start[offset] = pattern;
		}

		/*
		 * Check for address bits stuck high.
		 */
		test_offset = 0;
		start[test_offset] = anti_pattern; // change state of lsb-address-line
										   // write anti-pattern if lsb-address is defect you would write to offset=1 and not test_offset=0

		for (offset = 1; offset < len; offset <<= 1)
		{
			temp = start[offset];
			if (temp != pattern)
			{
			  printf ("\nFAILURE: Address bit stuck high @ 0x%.8lx: expected 0x%.8lx, actual 0x%.8lx\n", (ulong)&start[offset], pattern, temp);
			  errs++;
			}
		}
		start[test_offset] = pattern;
		WATCHDOG_RESET();

		/*
		 * Check for addr bits stuck low or shorted.
		 */
		for (test_offset = 1; test_offset < len; test_offset <<= 1)
		{
			start[test_offset] = anti_pattern;

			for (offset = 1; offset < len; offset <<= 1)
			{
			  temp = start[offset];
			  if ((temp != pattern) && (offset != test_offset))
			  {
				printf ("\nFAILURE: Address bit stuck low or shorted @ 0x%.8lx: expected 0x%.8lx, actual 0x%.8lx\n", (ulong)&start[offset], pattern, temp);
				errs++;
			  }
			}
			start[test_offset] = pattern;
		}

		/*
		 * Description:
		 * Test the integrity of a physical memory device by performing an
		 * increment/decrement test over the entire region. In the process every
		 * storage bit in the device is tested as a zero and a one. The base address
		 * and the size of the region are selected by the caller.
		 */

		num_words = ((ulong)end - (ulong)start)/sizeof(vu_long) + 1;

		/*
		 * Fill memory with a known pattern.
		 */
		for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++)
		{
			WATCHDOG_RESET();
			start[offset] = pattern;
		}

		/*
		 * Check each location and invert it for the second pass.
		 */
		for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++)
		{
			WATCHDOG_RESET();
			temp = start[offset];
			if (temp != pattern)
			{
			  printf ("\nFAILURE (read/write) @ 0x%.8lx: expected 0x%.8lx, actual 0x%.8lx)\n", (ulong)&start[offset], pattern, temp);
			  errs++;
			}

			anti_pattern = ~pattern;
			start[offset] = anti_pattern;
		}

		/*
		 * Check each location for the inverted pattern and zero it.
		 */
		for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++)
		{
			WATCHDOG_RESET();
			anti_pattern = ~pattern;
			temp = start[offset];
			if (temp != anti_pattern)
			{
			  printf ("\nFAILURE (read/write): @ 0x%.8lx: expected 0x%.8lx, actual 0x%.8lx)\n",	(ulong)&start[offset], anti_pattern, temp);
			  errs++;
			}
			start[offset] = 0;
		}
	}

}


/*
 * Set rtc via i2c to required date.
 */
int i2c_set_rtc_testdh(int year, int month, int day, int hour, int minutes, int seconds)
{
	//DISABLE_PRINTF() // disable printf while i2c operation

	// set control register
	i2c_reg_write(DEVICE_ADDR_RTC, ADDR_CONTROL_1, 0x50);	// initialize CONTROL	register

	// set date
	i2c_reg_write(DEVICE_ADDR_RTC, ADDR_YEAR, (uchar)(0xFF & year));			// initialize YEAR	register
	i2c_reg_write(DEVICE_ADDR_RTC, ADDR_MNTH, (uchar)(0xFF & month));     		// initialize MONTH register
	i2c_reg_write(DEVICE_ADDR_RTC, ADDR_DAY, (uchar)(0xFF & day));     			// initialize DAY  register
	i2c_reg_write(DEVICE_ADDR_RTC, ADDR_HOUR, (uchar)(0xFF & hour));     		// initialize HOUR  register
	i2c_reg_write(DEVICE_ADDR_RTC, ADDR_MIN,  (uchar)(0xFF & minutes));     	// initialize MIN   register
	i2c_reg_write(DEVICE_ADDR_RTC, ADDR_SEC,  (uchar)(0xFF & seconds));  		// initialize SEC register
	
	// start watch
	i2c_reg_write(DEVICE_ADDR_RTC, ADDR_CONTROL_1, 0x51);

	//SESSION_DEPENDED_PRINTF_ENABLE() // enable printf after i2c operation

	return 0;
}

/*
 * Set rtc via i2c to required date.
 */
int i2c_read_rtc_testdh(void)
{
	uchar year, month, day, hour, minutes, seconds;

	//DISABLE_PRINTF() // disable printf while i2c operation

	// set date
	year 	= 0xFF & i2c_reg_read(DEVICE_ADDR_RTC, ADDR_YEAR);   // read YEAR	register
	month 	= 0x1F & i2c_reg_read(DEVICE_ADDR_RTC, ADDR_MNTH);   // read MONTH register
	day 	= 0x3F & i2c_reg_read(DEVICE_ADDR_RTC, ADDR_DAY);   // read DAY  register
	hour 	= 0x3F & i2c_reg_read(DEVICE_ADDR_RTC, ADDR_HOUR);   // read HOUR  register
	minutes = 0x7F & i2c_reg_read(DEVICE_ADDR_RTC, ADDR_MIN);    // read MIN   register
	seconds = 0xFF & i2c_reg_read(DEVICE_ADDR_RTC, ADDR_SEC);  	// read SEC register

	//SESSION_DEPENDED_PRINTF_ENABLE() // enable printf after i2c operation

	printf("\ndate read from rtc: 20%02x-%02x-%02x  time: %02x:%02x:%02x\n\n",year, month, day, hour, minutes, seconds);

	return 0;
}

#define EIM_CS0RCR1 0x21B8008
#define EIM_CS0WCR1 0x21B8010 
int do_dhtestroutines(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int iteration_limit = 1;
	int year, month, day, hour, minutes, seconds;
	char *cmd;

	/* at least two arguments please */
	if (argc < 2)
		goto usage;

	cmd = argv[1];

	/* run ddr3 test */
	if (strcmp(cmd, "ddr3") == 0)
	{
		printf("starting ddr3 test-routine ...\n\n");
		if (argc >= 3)
		{
			iteration_limit = (int)simple_strtoul(argv[2], NULL, 16);
			if(iteration_limit == 0)
			{
				iteration_limit = 1;
			}
		}
		return ddr3_testdh(iteration_limit);
	}

	/* run sram test */
	if (strcmp(cmd, "sram") == 0)
	{
		iteration_limit = 100;
		printf("starting %d sram test-routines ...\n\n", iteration_limit);
		
		if(argc > 2)
		{
			cmd = argv[2];
			if(strcmp(cmd, "highspeed") == 0)
			{
				writel(0xA022000, EIM_CS0RCR1);
				writel(0xA092480, EIM_CS0WCR1);	
				printf("Speed: ~12MByte/sec.\n");				
			}
		}
		else
		{
			printf("Speed: ~7MByte/sec.\n");
		}		
		ext_sram_testdh(iteration_limit);
		if(argc > 2)
		{		
			cmd = argv[2];
			if(strcmp(cmd, "highspeed") == 0)
			{
				writel(0x1C022000, EIM_CS0RCR1);
				writel(0x1C092480, EIM_CS0WCR1);				
			}
		}
		
		return 0;
	}	

	/* configure rtc and start rtc */
	if (strcmp(cmd, "setrtc") == 0)
	{
		if (argc != 8)
			goto usage;
		printf("set rtc to specified date and start ...\n\n");

		year = (int)simple_strtoul(argv[2], NULL, 16);
		month = (int)simple_strtoul(argv[3], NULL, 16);
		day = (int)simple_strtoul(argv[4], NULL, 16);
		hour = (int)simple_strtoul(argv[5], NULL, 16);
		minutes = (int)simple_strtoul(argv[6], NULL, 16);
		seconds = (int)simple_strtoul(argv[7], NULL, 16);

		return i2c_set_rtc_testdh(year, month, day, hour, minutes, seconds);
	}

	/* read date from rtc */
	if (strcmp(cmd, "readrtc") == 0)
	{
		if (argc != 2)
			goto usage;
		printf("read date from rtc ...\n\n");
		return i2c_read_rtc_testdh();
	}
	
	/* run endless test */
	if (strcmp(cmd, "endless") == 0)
	{
		while(true)
		{
			printf("\nstarting ddr3 test-routine ...\n");
			iteration_limit = 2;
			ddr3_testdh(iteration_limit);
			iteration_limit = 10;
			printf("\nstarting %d sram test-routines ...\n", iteration_limit);
			printf("Speed: ~7MByte/sec.\n");
			ext_sram_testdh(iteration_limit);
		}
	}
usage:
	cmd_usage(cmdtp);
	return 1;
}

/* ====================================================================== */

U_BOOT_CMD(
  testdh,     CONFIG_SYS_MAXARGS, 0, do_dhtestroutines,
  "DH Testroutines",
  "-> DH Test Subsystem\n"
  "---------------------------\n\n"
  "testdh ddr3 [interations] - run ddr3 test\n"
  "testdh setrtc yy mm dd hour min sec\n"
  "testdh readrtc - read date from rtc\n"
  "testdh sram [highspeed] - run external sram test\n"
  "testdh endless - run endless sram and ddr3 test\n");

#endif/* CONFIG_CMD_TESTROUTINES */
