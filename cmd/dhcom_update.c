/*
 * File: cmd_DHCOMupdate.c
 *
 * (C) Copyright 2011-2012
 * Andreas Geisreiter, DH electronics GmbH , ageisreiter@dh-electronics.de
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

/* -----------------------------------------------------------------------------
 *
 * Abstract: Command line function to update os images from storage device.
 *
 * Notes:
 *     Created by:  Andreas Geisreiter (ageisreiter@dh-electronics.de)
 *     Created:     January 03, 2012
 *
 * Description:
 *     The following command line calls are possible:
 *     1. "update auto":
 *        Is called after the start of the bootloader with the env variable 
 *	  "bootcmd". This function call is used for automatic update at the
 *        system start. The function tries to update the flash content via
 *        the DHupdate.ini file. The update mechanism is searching for the 
 *        update files on the specified storage devices. The update devices must
 *        be defined at the settings block. With the DHupdate.ini file you have
 *        the possibility to run more then one updates, e.g. OS image and
 *	  settings block.
 *
 *     2. "update":
 *        Is used form the command line. This function call tries to update the
 *        flash content via the DHupdate.ini file. In contrast to 1. the update
 *        mechanism is searching on every available storage device for the
 *        update files.
 *
 *     3. "update <type> [filename]":
 *        Is used form the command line. This function call doesn't use the
 *        DHupdate.ini file. It allows only one update and the update mechanism
 *        is searching on every available storage device for the update files.
 *        With the <type> parameter you have to specify the update type.
 *
 */

#include <common.h>
#include <command.h>
#include <config.h>
#include <environment.h>
#include <errno.h>
#include <i2c.h>
#include <malloc.h>
#include <linux/ctype.h>
#include <asm/gpio.h>
#include <asm/io.h>
#ifdef DH_IMX6_NAND_VERSION
#  include <nand.h>
#endif

#define WINCE_IMG_NB0	'1'
#define WINCE_IMG_GZ	'2'

typedef enum {
    LOAD_UPDATEKERNEL, UPDATE_UBOOT, REFRESH_SETTINGS,
    UPDATE_SETTINGS_EEPROM, EXEC_RESET, EXEC_SCRIPT, UPDATE_WINCE,
    UPDATE_EBOOT 
} TYPE;

typedef enum {
   SUCCESS = 0,
   ERR_DHUPDATE_INI = 1,
   ERR_FILE_NOT_FOUND = 2,
   ERR_FLASH = 3,
   ERR_IMAGE_TYPE = 4,
   ERR_INVALID_FILE = 5,
   ERR_IMAGE_SIZE = 6,
   ERR_LOAD_UPDATEKERNEL = 7
} ERR_STATE;

#define UPDATEINI_ID         "##DHCOMupdate##"
#define UPDATEINI_DISPLAY    "display"
#define UPDATEINI_LED        "led"
#define UPDATEINI_UPDATE     "update"
#define UPDATEINI_END        "end"
#define UPDATEINI_MAX_STEPS  10

typedef struct {
    char *p_cUpdateType;
    char *p_cFilename;
} step_t;

typedef struct {  
    // Display update bmp's
    bool display;
    char *display_progressBmp;
    char *display_doneBmp;
    char *display_errorBmp;
    
    // Update GPIO settings
    bool led;
    char*led_gpioName;
    bool led_activeState;  // false = low active, true = high active
    
    // Updates
    int iUpdateCounter;    
    step_t update_steps[UPDATEINI_MAX_STEPS];
} updateini_t;

typedef struct {
    bool have_ini;		// true if update uses DHupdate.ini
    TYPE type;			// type: uboot, settings, .. 
    char *filename;		// image filename
    char *src_dev;		// storage device
    char *src_part;		// storage partition
    ulong loadaddr;	// ram load address
#ifdef DH_IMX6_NAND_VERSION
    nand_info_t *nand;		// reference to current nand device
#endif
} context_t;

unsigned int led_gpio = UINT_MAX;

// Extern defined functions
#ifdef CONFIG_CMD_DHCOM_SETTINGS
extern void settings_gen_kernel_args(void);
extern int  settings_bin_to_struct(ulong addr, bool is_DA_eeprom);
extern bool settings_get_microSD(void);
extern bool settings_get_extSD(void);
extern bool settings_get_usbhost1(void);
extern bool settings_get_usbotg(void);
#endif

int led_init(updateini_t *DHupdateINI)
{
	int ret;
	enum { GPIO_A = 0, GPIO_B, GPIO_C, GPIO_D, GPIO_E, GPIO_F,
	       GPIO_G, GPIO_H, GPIO_I, GPIO_UNDEFINED } UpdateGPIO;

	unsigned int DHCOM_gpios[] = {
		DHCOM_GPIO_A,
		DHCOM_GPIO_B,
		DHCOM_GPIO_C,
		DHCOM_GPIO_D,
		DHCOM_GPIO_E,
		DHCOM_GPIO_F,
		DHCOM_GPIO_G,
		DHCOM_GPIO_H,
		DHCOM_GPIO_I,
	};

    	char matchLED[7] = {"GPIO_A\0"};
   
	for(UpdateGPIO = GPIO_A;
	    UpdateGPIO < GPIO_UNDEFINED;
	    UpdateGPIO++, matchLED[5]++) {

		ret=memcmp(DHupdateINI->led_gpioName,matchLED,sizeof(matchLED));
		if(ret == 0)
			break;
	}
	   
	if (UpdateGPIO >= GPIO_UNDEFINED)
		return -1;

	led_gpio = DHCOM_gpios[UpdateGPIO];
	return 0;
}

void led_set(bool active_high, int value)
{
	if (led_gpio == UINT_MAX)
		return;

        if (active_high)
                gpio_direction_output(led_gpio , value);
        else
                gpio_direction_output(led_gpio , !value);
}

void led_blink(bool active_high, int blinking)
{
        for( ; blinking > 0; blinking--) {
                udelay(250*1000);
                led_set(active_high, 1);
                udelay(250*1000);
                led_set(active_high, 0);
        }
}

//------------------------------------------------------------------------------
//
//  Function:  write_spiflash
//
//  Update Flash content on the specified address.
//
//  Commit values:  write_offset    = Flash Offset address
//                  loadaddr 	    = SDRAM Buffer address
//                  erase_blocks    = count of blocks that should be updated
//                  erase_size      = Flash block size
//
//  Return value:   0 = No error
//                  1 = Flash erase error (the error block number is specified from bit 3 to bit 31)
//                  2 = Flash write error (the error block number is specified from bit 3 to bit 31)
//
int write_spiflash(ulong write_offset, ulong loadaddr, ulong erase_blocks, ulong erase_size)
{
        int ret = 0;
	ulong erase_offset;
	ulong update_size;
	char cmd[128];
       
        printf ("--> Update: Erase/Write new image into flash ... \n");
        
        // set erase offset and size
       	erase_offset = (write_offset/erase_size) * erase_size; // meet erase block border !!!
        update_size  = erase_blocks * erase_size;
                                
        ret = run_command ("sf probe", 0);
        if(ret != 0)
                return ret;

	snprintf(cmd, sizeof(cmd), "sf erase %08lx %08lx", erase_offset, update_size);
        ret = run_command (cmd, 0);
        if(ret != 0)
                return ret;

	snprintf(cmd, sizeof(cmd), "sf write %08lx %08lx %08lx", loadaddr, write_offset, update_size);
        return run_command (cmd, 0);
}

//------------------------------------------------------------------------------
//
//  Function:  write_nandflash
//
//  Update Flash content on the specified address
//
//  Commit values:  - ulFlashAddress    = FLash Block address
//                  - ulSDRAMBufferAddress  = SDRAM Buffer address
//                  - ulBlocks              = Number of block that should be updated
//                  - ulFlashBlockSize      = Flash block size
//                  - maxsize = max. Partition size
//
//  Return value:   0 = No error
//                  1 = Flash erase error (the error block number is specified from bit 3 to bit 31)
//                  2 = Flash write error (the error block number is specified from bit 3 to bit 31)
//
#ifdef DH_IMX6_NAND_VERSION
int write_nandflash(char* itemtext, ulong ulFlashAddress, ulong ulSDRAMBufferAddress, ulong ulBlocks, ulong ulFlashBlockSize, loff_t maxsize)
{
    nand_info_t *nand;
    nand_erase_options_t opts;
    ulong ulBlockOffset = ulFlashAddress;
    size_t write_size = 0;
    ulong i,j;
    int ret = 0;

    printf ("\n--> Update: The new %s image File needs %lu Flash blocks\n", itemtext, ulBlocks);
    printf ("    e = Erase Block\n");
    printf ("    w = Write to Block\n");
    printf ("    d = Done\n");
    printf ("    Write File to Flash:[");

    // Get nand-info structure from current device
    nand = &nand_info[nand_curr_device];

    //Initialize erase settings
    memset(&opts, 0, sizeof(opts));
    opts.length = ulFlashBlockSize;
    opts.scrub = 0;
    opts.jffs2  = 1;
    opts.quiet  = 0;
    opts.spread = 0;

    for(j = 1, i = 0; i < ulBlocks; j++, i++) {
        if(j == 51) {
            printf ("\n                         ");
            j = 1;
        }

       // Check if current block is bad?
       while (nand_block_isbad(nand, ulBlockOffset)) {
            ulBlockOffset = ulBlockOffset + ulFlashBlockSize;
       }

       // Set block address for erase
       opts.offset = ulBlockOffset;

       // Erase flash block
       printf ("e");

       ret = nand_erase_opts(nand, &opts);
       if(ret != 0) { // Exit on Erase ERROR
           ret = (i << 2) | 1;
           return ret;
       }

       // Write flash block
       printf ("\bw");

       // Copy content of SDRAM to current block
       write_size = ulFlashBlockSize;
       ret = nand_write_skip_bad(nand, ulBlockOffset, &write_size, NULL,
				 maxsize,(u_char *)(ulSDRAMBufferAddress + i * ulFlashBlockSize), 0);
       if(ret != 0) { // Exit on Write ERROR
           ret = (i << 2) | 2;
           return ret;
       }

        printf ("\bd");

       // Increment the block offset
       ulBlockOffset = ulBlockOffset + ulFlashBlockSize;
    }
    printf ("]");
    return 0;
}
#endif /* DH_IMX6_NAND_VERSION  */

//------------------------------------------------------------------------------
//
//  Function:  write_eeprom
//
//  Update EEProm content on the specified address.
//
//  Commit values:  - ulEepromRegisterAddress = Eeprom register offset
//                  - ulSDRAMBufferAddress  = SDRAM Buffer address
//                  - ulBytes               = Number of bytes that should be updated
//
//  Return value:   0 = No error
//                  1 = EEPROM write error
//
#ifdef DISPLAY_ADAPTER_EEPROM_ADDR
int write_eeprom(ulong ulEepromRegisterAddress, ulong ulSDRAMBufferAddress, ulong ulBytes)
{
        unsigned int i,j;
        int ret;
        uchar ucBuffer;

        /* Set i2c driver to use i2c bus 0  */
        ret = run_command ("i2c dev 0", 0);
        if (ret != 0)
                return ret;

        for(j = 1, i = 0; i < ulBytes; j++, i++) {
                if(j == 51)  {
                        printf ("\n                         ");
                        j = 1;
                }

                // Write eeprom content
                printf ("w");
                ret = i2c_write(DISPLAY_ADAPTER_EEPROM_ADDR, i, 1, (uchar*)(ulSDRAMBufferAddress+i), 1);
                udelay(8*1000);
                if(ret != 0) { // Exit on Write ERROR
                        /* Set i2c driver back to use i2c bus 2 */
                        run_command ("i2c dev 2", 0);
                        return ret;
                }
                printf ("\bd");
        }

        // Verify EEPROM data
        for(i = 0; i < ulBytes; i++) {
                ret = i2c_read(DISPLAY_ADAPTER_EEPROM_ADDR, i, 1, &ucBuffer, 1);
                if((ret != 0) || (ucBuffer != *(uchar*)(ulSDRAMBufferAddress+i))) { // Exit on Write ERROR
                        /* Set i2c driver back to use i2c bus 2 */
                        run_command ("i2c dev 2", 0);
                        return ret;
                }
        }

        printf ("]");

        /* Set i2c driver back to use i2c bus 2 */
        ret = run_command ("i2c dev 2", 0);
        if (ret != 0) {
                return 1;
        }

        return 0;
}
#else
int write_eeprom(ulong ulEepromRegisterAddress, ulong ulSDRAMBufferAddress, ulong ulBytes)
{
	return 0;
}
#endif

int check_imagesize(context_t *context, ulong filesize, ulong partsize)
{
        if (filesize > partsize) {
                printf("\n==> Update ERROR:");
		printf(" %s is to large: max %li bytes\n", 
				context->filename, partsize);
                return -1; // error
        }
        return 0;
}

int load_file(context_t *context)
{
	int ret;
	char cmd[128];

	ret = snprintf(cmd, sizeof(cmd), "load %s %s %08lx %s ", 
				context->src_dev, context->src_part, 
				context->loadaddr, context->filename);
	if (ret < 0 || ret > sizeof(cmd)) {
		printf("ERROR: load command!\n");
		return -EFAULT;
	}

	ret = run_command(cmd, 0);
	if (ret != 0) {
                printf("\n--> No %s file found on %s %s\n", 
			context->filename, 
			context->src_dev, 
			context->src_part);
                return -ENOENT;
	}

	ret = (int)simple_strtoul(env_get("filesize"), NULL, 16);
        printf("--> Loaded %s to SDRAM (%d bytes)\n\n", context->filename, ret);

	return ret;
}

//------------------------------------------------------------------------------
//
//  Function:  parse_DHupdateINI
//
//  Read DHupdate.ini file content.
//
//  Commit values:  - *DHupdateINI = Pointer to DHupdateINI struct
//                  - loadaddr = DHupdate.ini file SDRAM address
//
//  Return value:   0 = No error
//                  1 = error
//
int parse_DHupdateINI(updateini_t *DHupdateINI, ulong loadaddr, int filesize)
{
        void *buf;        // Pointer to DHupdate.ini file in RAM
        int refIndex;     // Act. position in DHupdate.ini file
        int refIndex_old; // Old position in DHupdate.ini file
        int iDifference;  // differenc between act. and old position
        int iSpecialFilename = 0;

        // Set to invalid
        DHupdateINI->display = false;
        DHupdateINI->led = false;
        DHupdateINI->iUpdateCounter = 0;

        // Search in DHupdate.ini file for valid mask
        for(refIndex = 0; *(char*)(loadaddr + refIndex) != '\n'; refIndex++);
        *(char*)(loadaddr + refIndex) = '\0';

        // check valid mark
        if (memcmp((char*)loadaddr, UPDATEINI_ID, 15))
                return -EEXIST;
	
	/* allocate memory to store DHupdate.ini */
	buf = malloc(filesize);
	if (!buf)
		return -ENOMEM;
	
	/* copy DHupdate.ini into buffer*/
	memcpy(buf, (void*)loadaddr, filesize);

        refIndex++;
        // Search for '['']' content
        while(*(char*)(buf + refIndex) == '[') {
                refIndex++;
                refIndex_old = refIndex;
                while(*(char*)(buf + refIndex) != ']') {
                        refIndex++;
                }
                *(char*)(buf + refIndex) = '\0';
                iDifference = refIndex - refIndex_old;

                if(!(memcmp ( (char*)(buf+refIndex-iDifference), UPDATEINI_DISPLAY, iDifference))) {
                        // **********************************************************
                        // ***************** Update Block [display] *****************
                        // **********************************************************

                        DHupdateINI->display = true;
                        // Progress Bmp
                        refIndex+=2; // +2, because ']' and '\n'
                        refIndex_old = refIndex;
                        while(*(char*)(buf + refIndex) != '\n') {
                                refIndex++;
                        }
                        *(char*)(buf + refIndex) = '\0';
                        DHupdateINI->display_progressBmp = (char*)(buf+refIndex_old);
                        // Ok Bmp
                        refIndex++;
                        refIndex_old = refIndex;
                        if(*(char*)(buf + refIndex) == '[') {
                                // DHupdate.ini display file missing
                                return 1;
                        }
                        while(*(char*)(buf + refIndex) != '\n') {
                                refIndex++;
                        }
                        *(char*)(buf + refIndex) = '\0';
                        DHupdateINI->display_doneBmp = (char*)(buf+refIndex_old);
                        // Error Bmp
                        refIndex++;
                        refIndex_old = refIndex;
                        if(*(char*)(buf + refIndex) == '[') {
                                // DHupdate.ini display file missing
                                return 1;
                        }
                        while(*(char*)(buf + refIndex) != '\n') {
                                refIndex++;
                        }
                        *(char*)(buf + refIndex) = '\0';
                        DHupdateINI->display_errorBmp = (char*)(buf+refIndex_old);
                        refIndex++;
                }
                else if(!(memcmp ( (char*)(buf+refIndex-iDifference), UPDATEINI_LED, iDifference))) {
                        // **********************************************************
                        // ******************* Update Block [led] *******************
                        // **********************************************************
                        DHupdateINI->led = true;
                        // Update GPIO
                        refIndex+=2; // +2, because ']' and '\n'
                        refIndex_old = refIndex;
                        while((*(char*)(buf + refIndex) != '\n') && (*(char*)(buf + refIndex) != ' ')) {
                                refIndex++;
                        }

                        if(*(char*)(buf + refIndex) == '\n') {
                                // Active state is missing --> Don't set Update GPIO, because we don't know the active state of the connected LED
                                DHupdateINI->led = false;
                                refIndex++;
                        }
                        else {
                                // GPIO name
                                *(char*)(buf + refIndex) = '\0';
                                DHupdateINI->led_gpioName = (char*)(buf+refIndex_old);
                                refIndex++;

                                // GPIO Active state
                                refIndex_old = refIndex;
                                while(*(char*)(buf + refIndex) != '\n') {
                                        refIndex++;
                                }

                                *(char*)(buf + refIndex) = '\0';

                                if (strcmp((char*)(buf+refIndex_old), "high") == 0)
                                        DHupdateINI->led_activeState = true;
                                else if (strcmp((char*)(buf+refIndex_old), "low") == 0)
                                        DHupdateINI->led_activeState = false;
                                else // unknown argument
                                        DHupdateINI->led = false;

                                refIndex++;
                        }
                }
                else if(!(memcmp ( (char*)(buf+refIndex-iDifference), UPDATEINI_UPDATE, iDifference))) {
                        // **********************************************************
                        // ***************** Update Block [update] ******************
                        // **********************************************************
                        refIndex+=2; // +2, because ']' and '\n'

                        while(*(char*)(buf + refIndex) != '[') {
                                if(DHupdateINI->iUpdateCounter >= UPDATEINI_MAX_STEPS)
                                        return 1; // Too much update arguments in DHupdate.ini file

                                DHupdateINI->iUpdateCounter++;
                                // Update Type
                                refIndex_old = refIndex;

                                // read text until '\n' or ' '
                                while((*(char*)(buf + refIndex) != '\n') && (*(char*)(buf + refIndex) != ' ')) {
                                        refIndex++;
                                }

                                // Only if ' ' after update type a custom filename should be available
                                if(*(char*)(buf + refIndex) == ' ')
                                        iSpecialFilename = 1;
                                else
                                        iSpecialFilename = 0;

                                *(char*)(buf + refIndex) = '\0'; // Set '\0' to mark end of update-type string

                                // set pointer to update-type string in update_steps array
                                DHupdateINI->update_steps[DHupdateINI->iUpdateCounter - 1].p_cUpdateType = (char*)(buf+refIndex_old);
                                refIndex++;

                                if(iSpecialFilename) {
                                        // Update Type
                                        refIndex_old = refIndex;
                                        while(*(char*)(buf + refIndex) != '\n') {
                                                refIndex++;
                                        }
                                        *(char*)(buf + refIndex) = '\0'; // Set '\0' to mark end of filename string

                                        // set pointer to filename string in update_steps array
                                        DHupdateINI->update_steps[DHupdateINI->iUpdateCounter - 1].p_cFilename = (char*)(buf+refIndex_old);
                                        refIndex++;
                                }
                                else {
                                        DHupdateINI->update_steps[DHupdateINI->iUpdateCounter - 1].p_cFilename = NULL;
                                }
                        }
                }
                else if(!(memcmp ( (char*)(buf+refIndex-iDifference), UPDATEINI_END, iDifference)))
                        return 0;
                else
                        return 1; /* missing [end] tag ?*/
        }
        return 1;
}

int load_DHupdateini(updateini_t *stDHupdateINI, context_t context)
{
	int ret;

	context.filename = "DHupdate.ini";
	ret = load_file(&context);
	if(ret < 0)
		return -EAGAIN; // try again on other storage

        // parse DHupdate.ini file content
        if (parse_DHupdateINI(stDHupdateINI, context.loadaddr, ret)) {
                printf("\n--> Update ERROR: Wrong DHupdate.ini file content \n");
                return -EEXIST;
        }

        printf ("\n--> Update: found DHupdate.ini (%d bytes)\n", (int)simple_strtoul (env_get("filesize"), NULL, 16));
	return 0;
}

int display_bmp(char *filename, char *storage_dev, char *storage_part)
{
	int ret;
	char *buffer;
	char command[128];
	char const *panel;

	panel = env_get("panel");
	if ( panel != NULL && strcmp(panel, "no_panel") == 0) {
		return 0; // panel is disabled
	}

	/* get buffer in ddr3 */
	buffer = (char*)simple_strtoul(env_get("loadaddr"), NULL, 16);

	ret = snprintf(command, sizeof(command), "load %s %s %08x %s ", 
		     storage_dev, storage_part, (unsigned int)buffer, filename);
	if (ret < 0 || ret > sizeof(command)) {
		printf("ERROR: load command!\n");
		return 1;
	}
	
	ret = run_command(command, 0); // load bitmap to ddr3
	if (ret != 0) {
		printf("ERROR: loading bitmap %s\n", filename);
		return 1;
	}

	ret = snprintf(command, sizeof(command), "bmp display %08x 32767 32767",
						 (unsigned int)buffer);
	if (ret < 0 || ret > sizeof(command)) {
		printf("ERROR: display bitmap %s\n", filename);
		return 1;
	}

	run_command(command, 0); // display bitmap
        return 0;
}

void handle_error(context_t *context, updateini_t *DHupdateINI, ERR_STATE code)
{
	int blinking = 0;

        if (!context->have_ini)
		return; // return if in manual mode
        
        if (DHupdateINI->display) // if DHupdate.ini contains [display] section
                display_bmp(DHupdateINI->display_errorBmp,
				context->src_dev,
				context->src_part);

        if (DHupdateINI->led) {
		blinking = code; // ERR_STATE enum == blink intervall

		led_set(DHupdateINI->led_activeState, 0);
		while(1) { // forever
		        led_blink(DHupdateINI->led_activeState, blinking);
			udelay(1750*1000);
		}                            
	}
}

int update_bootloader(context_t *context, updateini_t *DHupdateINI)
{
	int ret;
	ulong uboot_offset = CONFIG_SYS_SPI_SPL_OFFS;
        ulong uboot_partsize = CONFIG_ENV_OFFSET;

	ulong blocksize = (64 * 1024); 
        ulong filesize;
        ulong ulBlocks;

        printf("\n==> Update bootloader\n");

	ret = load_file(context);
	if(ret < 0) {
		handle_error(context, DHupdateINI, ERR_FILE_NOT_FOUND);
		return ret;
	}

        // Calculate necessary sectors in flash for the u-boot image file.
        filesize = ret; // u-boot.bin filesize
        ulBlocks = (filesize+uboot_offset) / blocksize + 0x1;

	ret = check_imagesize(context, filesize+uboot_offset, uboot_partsize);
        if (ret) {
                handle_error(context, DHupdateINI, ERR_IMAGE_SIZE);
                return -ENOSPC;
        }
                
        ret = write_spiflash(uboot_offset, context->loadaddr, ulBlocks, blocksize);
        if (ret != 0) {
                printf("\n--> Update ERROR: Failed flashing ... \n");
                handle_error(context, DHupdateINI, ERR_FLASH);
                return -EFAULT;
        }

        printf("\n--> Update: U-Boot Update done\n");
	return 0;
}

// Update eeprom.bin file 
int update_eeprom_settings(context_t *context, updateini_t *DHupdateINI)
{
	int ret;
	ulong filesize;

	printf ("\n==> Update display adapter eeprom\n");

	ret = load_file(context);
	if(ret < 0) {
		handle_error(context, DHupdateINI, ERR_FILE_NOT_FOUND);
		return ret;
	}

        filesize = ret; // eeprom.bin filesize

#ifdef DHCOM_DISPLAY_SETTINGS_SIZE
        // Check if the file fits into to the eeprom-partition
        ret = check_imagesize(context, filesize, DHCOM_DISPLAY_SETTINGS_SIZE);
        if (ret) {
                handle_error(context, DHupdateINI, ERR_IMAGE_SIZE);
                return -ENOSPC;
        }
#else
	#warning Please define DHCOM_DISPLAY_SETTINGS_SIZE for image size check
#endif

        printf("\n--> Update: The new eeprom File needs %lu Bytes\n", filesize);
        printf("    w = Write to addr\n");
        printf("    d = Done\n");
        printf("    Write File to EEPROM:[");

        ret = write_eeprom(0x00, context->loadaddr, filesize);
        if (ret != 0) {
                printf("\n--> Update ERROR: EEPROM write error\n");
                handle_error(context, DHupdateINI, ERR_FLASH);
                return -EFAULT;
        }

        printf ("\n--> Update: Settings Update done\n");
	return 0;
}

int refresh_settings(context_t *context, updateini_t *DHupdateINI)
{
	int ret;

        printf("\n==> Update: refresh settings\n");

	ret = load_file(context);
	if(ret < 0) {
		handle_error(context, DHupdateINI, ERR_FILE_NOT_FOUND);
		return ret;
	}

#ifdef CONFIG_CMD_DHCOM_SETTINGS
	ret =  settings_bin_to_struct(context->loadaddr, false);
        if (ret == 0)
                settings_gen_kernel_args();
#else
	#warning refresh only available with CONFIG_CMD_DHCOM_SETTINGS
#endif


        printf("\n--> Update: refresh settings done\n");
	return 0;
}

int execute_script(context_t *context, updateini_t *DHupdateINI)
{
	int ret;
	char cmd[128];

        printf("\n==> Update: Execute Bootloader Script\n\n");

	ret = load_file(context);
	if(ret < 0) {
		handle_error(context, DHupdateINI, ERR_FILE_NOT_FOUND);
		return ret;
	}

	snprintf(cmd, sizeof(cmd), "source %08lx", context->loadaddr);

	ret = run_command(cmd, 0);
	if (ret != 0) {
		handle_error(context, DHupdateINI, ERR_IMAGE_TYPE);
		return -ENOENT;
	}

        printf("\n\n--> Update: Script Execution done\n");
	return 0;
}

int update_wince(context_t *context, updateini_t *DHupdateINI, char OSFileType)
{
        printf ("\n==> Update WEC Image\n");

#ifndef DH_IMX6_NAND_VERSION
        printf("\n==> Update ERROR: eMMC Flash not supported for WEC image!\n");
        handle_error(context, DHupdateINI, ERR_FLASH);
        return -ENOSYS;
#else 
	int ret;
        ulong filesize;
        ulong ulBlocks;

	// determine NAND block size
        ulong ulNANDFlashBlockSize = context->nand->erasesize; 

	char cImageFileSize[9] = {"12345677\0"};

	ret = load_file(context);
	if(ret < 0) {
		handle_error(context, DHupdateINI, ERR_FILE_NOT_FOUND);
		return ret;
	}

        filesize = ret; // nk.nb0 filesize
        ulBlocks = filesize / ulNANDFlashBlockSize + 0x1;

#ifdef WEC_PARTITION_SIZE
        // Check if the file fits into to the WEC-Image-partition
	ret = check_imagesize(context, filesize, WEC_PARTITION_SIZE);
        if (ret) {
                handle_error(context, DHupdateINI, ERR_IMAGE_SIZE);
                return -ENOSPC;
        }
#else
	#warning Please define WEC_PARTITION_SIZE for image size check
#endif
        env_set("redundant_wince_image", "");
        env_set("wec_image_size", "");
        env_set("wec_image_type_gz", "");
        env_set("wec_image_type_nb0", "");
        env_save();

        ret = write_nandflash("WinCE", WEC_IMAGE_FLASH_ADDRESS,
				context->loadaddr, ulBlocks,
				ulNANDFlashBlockSize, WEC_PARTITION_SIZE);
        if((ret & 0x3) == 1) {
                printf("\n--> Update ERROR: NAND erase error on block 0x%08x\n",
			(unsigned int)(WEC_IMAGE_FLASH_ADDRESS + (ret >> 2) * ulNANDFlashBlockSize));
                handle_error(context, DHupdateINI, ERR_FLASH);
                return -EFAULT;
        }
        else if((ret & 0x3) == 2) {
                printf("\n--> Update ERROR: NAND write error on block 0x%08x\n",
			(unsigned int)(WEC_IMAGE_FLASH_ADDRESS + (ret >> 2) * ulNANDFlashBlockSize));
                handle_error(context, DHupdateINI, ERR_FLASH);
                return -EFAULT;
        }

        sprintf (&cImageFileSize[0], "%08x", (unsigned int)filesize);
        env_set("wec_image_size", &cImageFileSize[0]);

        if (OSFileType == WINCE_IMG_GZ) 
                env_set("wec_image_type_gz", "1");
        else
                env_set("wec_image_type_nb0", "1");
        env_save();

#endif /* DH_IMX6_NAND_VERSION  */

        // Windows Embedded CE Flash Update done.
        printf ("\n--> Update: WinCE Update done");
	return 0;
}

int update_eboot(context_t *context, updateini_t *DHupdateINI)
{
	int ret;
        ulong eboot_offset;
	ulong blocksize = CONFIG_ENV_SECT_SIZE;
        ulong filesize;
        ulong ulBlocks;

	eboot_offset = simple_strtoul(env_get ("eboot_flash_offset"), NULL, 16);

        printf ("\n==> Update eboot\n");

#ifndef DH_IMX6_NAND_VERSION
        printf("\n==> Update ERROR: eMMC unsupported for WEC image!\n");
        handle_error(context, DHupdateINI, ERR_FLASH);
        return -ENOSYS;
#endif /* DH_IMX6_NAND_VERSION  */

	ret = load_file(context);
	if(ret < 0) {
		handle_error(context, DHupdateINI, ERR_FILE_NOT_FOUND);
		return ret;
	}

        // Calculate necessary sectors in flash for the eboot image file.
        filesize = ret;
        ulBlocks = (filesize / blocksize) + 0x1;
#ifdef EBOOT_PARTITION_SIZE
        // Check if the file fits into to the flash-partition
	ret = check_imagesize(context, filesize, EBOOT_PARTITION_SIZE);
        if (ret) {
                // ERROR: file is to large for the specified partition
                handle_error(context, DHupdateINI, ERR_IMAGE_SIZE);
                return -ENOSPC;
        }
#else
	#warning Please define EBOOT_PARTITION_SIZE for image size check
#endif

        env_set("eboot_image", "");
        env_save();

        ret = write_spiflash(eboot_offset,context->loadaddr,ulBlocks,blocksize);
        if (ret != 0) {
                printf("\n--> Update ERROR: Failed flashing ... \n");
                handle_error(context, DHupdateINI, ERR_FLASH);
                return -EFAULT;
        }

        env_set("eboot_image", "1");
        env_save();

        printf ("\n--> Update: eboot Update done\n");
	return 0;
}

int loading_updatekernel(context_t *context, updateini_t *DHupdateINI)
{
	char *cmd = env_get("load_update_kernel");
        if (cmd == NULL) {
                printf("\n--> Update ERROR: \"load_update_kernel\" not defined\n");
                handle_error(context, DHupdateINI, ERR_LOAD_UPDATEKERNEL);
                return -EINVAL;
        }

        printf ("\n==> Update: load Update Kernel\n");

        // set update device and partition for update kernel cmdline
        env_set("src_intf", context->src_dev);
        env_set("src_dev_part", context->src_part);

        run_command(cmd, 0); // does not return

        printf("\n--> Update ERROR: Command \"load_update_kernel\"\n");
        handle_error(context, DHupdateINI, ERR_LOAD_UPDATEKERNEL);
        return -EINVAL;
}

//------------------------------------------------------------------------------
//
//  Function:  DHCOMupdate
//
//  DHCOM U-Boot Update function
//
//  Commit values:  - *cmdtp = Command Type, used for cmd_usage()
//                  - argc = Number of command line arguments
//                  - *argv[] = Command line argument strings
//                  - *DHupdateINI = Pointer to DHupdateINI struct
//                  - storage_dev = Specify the act. storage device (e.g. "usb")
//                  - storage_part = Specify the act. partition number of the storage device
//
//  Return value:   0 = No error
//                  1 = error
//
int DHCOMupdate(cmd_tbl_t *cmdtp, int argc, char * const argv[], char *storage_dev, char *storage_part)
{	
	// default filenames of binaries
        char *file_uboot    = "u-boot.imx";
        char *file_eeprom   = "eeprom.bin";
        char *file_script   = "script.bin";
        char *file_settings = "settings.bin";
        char *file_winceNB0 = "nk.gz";
        char *file_ebootNB0 = "eboot.nb0";

	int ret;
        int iter_updates = 0;
        bool load_updatekernel = false;

	context_t context;
	context.have_ini = true;
	context.src_dev = storage_dev;
	context.src_part = storage_part;
	context.loadaddr = simple_strtoul(env_get("loadaddr"), NULL, 16);
#ifdef DH_IMX6_NAND_VERSION
	context.nand = &nand_info[nand_curr_device];
#endif
        
	updateini_t updateINI;
	memset(&updateINI,0,sizeof(updateINI));

        if(argc > 1) { // parse arguments
                char *cmd = argv[1];
                if (strcmp(cmd, "auto") == 0) {
                        // automatic update with DHupdate.init
                        context.have_ini = true;    
                } else {
                        // manual command line update
			context.have_ini = false;
                        if (strcmp(cmd, "bootloader") == 0) {
                                context.type = UPDATE_UBOOT;
				context.filename = file_uboot;
                        } else if (strcmp(cmd, "eeprom") == 0) {
                                context.type = UPDATE_SETTINGS_EEPROM;
                                context.filename = file_eeprom;
                        } else if (strcmp(cmd, "script") == 0) {
                                context.type = EXEC_SCRIPT;
                                context.filename = file_script;
                        } else if (strcmp(cmd, "wince") == 0) {
                                context.type = UPDATE_WINCE;
                                context.filename = file_winceNB0;
                        } else if (strcmp(cmd, "eboot") == 0) {
                                context.type = UPDATE_EBOOT;
                                context.filename = file_ebootNB0;
                        } else { // unknown argument
                                printf("\n--> ERROR: Unkown command: %s\n",cmd);
                                goto usage;
                        } 
			if(argc > 2)
                                context.filename = argv[2];
                }
        }
        
        if (context.have_ini) { // Handle DHupdate.ini file
		ret = load_DHupdateini(&updateINI, context);
		if (ret != 0)
			return ret;
        }

        if(updateINI.display) { // if DHupdate.ini contains [display] section
		int i;
	        bool have_refresh = false;

                for (i = 0; i < updateINI.iUpdateCounter; i++) {
                        if (strcmp(updateINI.update_steps[i].p_cUpdateType, "refresh") == 0) {
                                have_refresh = true;
                                break;
                        }
                }
                if(!have_refresh)
			// load progress bitmap
                        display_bmp(updateINI.display_progressBmp, context.src_dev, context.src_part);
        }

        if(updateINI.led) { // if DHupdate.ini has [led] section
                if (led_init(&updateINI) != 0) {
                        printf ("\n--> Update INFO: Can't initialize update LED Pin %s", updateINI.led_gpioName);
                        updateINI.led = false;
                } else
                        led_set(updateINI.led_activeState, 1); // switch on
        }

        do {
                // Identify the next specified update in DHupdate.ini file
                if (context.have_ini) {
			char *type_ptr = updateINI.update_steps[iter_updates].p_cUpdateType;
			char *file_ptr = updateINI.update_steps[iter_updates].p_cFilename;

                        if (strcmp(type_ptr, "eeprom") == 0) {
                                context.type = UPDATE_SETTINGS_EEPROM;
                                context.filename = file_eeprom;
                                if (file_ptr != NULL)
                                        context.filename = file_ptr;
                        }
                        else if (strcmp(type_ptr, "script") == 0) {
                                context.type = EXEC_SCRIPT;
				context.filename = file_script;
                                if (file_ptr != NULL)
                                        context.filename = file_ptr;
                        }
                        else if (strcmp(type_ptr, "refresh") == 0) {
                                context.type = REFRESH_SETTINGS;
				context.filename = file_settings;
                        }
                        else if (strcmp(type_ptr, "reset") == 0) {
                                context.type = EXEC_RESET;
                        }
                        else if (strcmp(type_ptr, "wince") == 0) {
                                context.type = UPDATE_WINCE;
				context.filename = file_winceNB0;
                                if (file_ptr != NULL)
					context.filename = file_ptr;
                        }
                        else if (strcmp(type_ptr, "eboot") == 0) {
                                context.type = UPDATE_EBOOT;
				context.filename = file_ebootNB0;
                                if (file_ptr != NULL)
                                        context.filename = file_ptr;
                        }
                        else {
                                load_updatekernel = true;
                                context.type = LOAD_UPDATEKERNEL;
                                if (strcmp(type_ptr, "settings") == 0) {
                                        // Set settings filename for refresh
                                        file_settings = file_ptr;
                                }
                        }
                }
                else { // Run only one command line update
                        updateINI.iUpdateCounter = 1;
		}

                switch (context.type) {
                case LOAD_UPDATEKERNEL:
                        break; // do nothing, load update kernel later...
                case UPDATE_UBOOT:
			ret = update_bootloader(&context, &updateINI);
			if(ret != 0)
				return ret;
                        break;
                case UPDATE_SETTINGS_EEPROM:
			ret = update_eeprom_settings(&context, &updateINI);
			if(ret != 0)
				return ret;
                        break;
                case REFRESH_SETTINGS:
			ret = refresh_settings(&context, &updateINI);
			if(ret != 0)
				return ret;
                        break;
                case EXEC_SCRIPT:
			ret = execute_script(&context, &updateINI);
			if(ret != 0)
				return ret;
                        break;
                case EXEC_RESET:
                        if(updateINI.iUpdateCounter != 1) {
                                printf("\n==> Update ERROR: Skipped reset!!!");
                                printf("\n==> Update ERROR: Reset has to be the last action in DHupdate.ini (Danger of infinite Update-Loop)!!!\n");
                                handle_error(&context, &updateINI, ERR_DHUPDATE_INI);
                                return -EINVAL;
                        }
                        if(!load_updatekernel)
                                run_command("reset",0);
                        break;
                case UPDATE_WINCE:
		{
			int i;
			char *file_type;

                        // check image type (*.gz or *.bin)
                        for (i = 0; context.filename[i] != '.'; i++);
                        file_type = &context.filename[i+1];

                        if (strcmp(file_type, "nb0") == 0) 
				ret = update_wince(&context, &updateINI, WINCE_IMG_NB0);
                        else if (strcmp(file_type, "gz") == 0)
				ret = update_wince(&context, &updateINI, WINCE_IMG_GZ);
                        else {
                                printf("\n--> Update ERROR: Wrong WinCE image type (Filename must end with .gz or .nb0)\n");
                                handle_error(&context, &updateINI, ERR_IMAGE_TYPE);
                                return -ENOEXEC;
                        }

			if(ret != 0)
				return ret;
                        break;
		}
                case UPDATE_EBOOT:
			ret = update_eboot(&context, &updateINI);
			if(ret != 0)
				return ret;
                        break;
                default:
                        if (context.have_ini) {
                                printf("\n--> Update ERROR: No Update Event defined for the Argument: %c\n", context.type);
                                handle_error(&context, &updateINI, ERR_DHUPDATE_INI);
                                return -ENOSYS;
                        }
                        else {
                                printf ("\n--> Update ERROR: No Update Event defined for this command line Argument\n");
                                goto usage;
                        }
                }

                iter_updates++;
                updateINI.iUpdateCounter--;

        } while (updateINI.iUpdateCounter > 0);


        if (load_updatekernel) {
		ret = loading_updatekernel(&context, &updateINI);
		if(ret != 0)
			return ret;
        }

        if (updateINI.display) // if DHupdate.ini has [display] section
                display_bmp(updateINI.display_doneBmp, context.src_dev, context.src_part);
        
        if (updateINI.led)     // if DHupdate.ini has [led] section
                led_set(updateINI.led_activeState, 0);

        printf ("\n");
        return 0;

usage:
        cmd_usage(cmdtp);
        return -EINVAL;
}

//------------------------------------------------------------------------------
//
//  Function:  do_dhcom_update
//
//  DHCOM U-Boot Update function
//  Description:        Scan for available storage devices
//
//  Return value:   0 == success
//                  0 >  error
//
static int do_dhcom_update(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
        int ret;
        bool update_auto = false;

        if(argc > 1) {
                // is this a automatically update
                if (strcmp(argv[1], "auto") == 0)
                        update_auto = true;
        }

        // MicroSD Card Slot
#ifdef CONFIG_CMD_DHCOM_SETTINGS
        if (settings_get_microSD() || !update_auto) {
#else
	if (true) {
#endif
        	// Call update function, if MicroSD init returns success
        	run_command("mmc dev 1", 0);
                ret = run_command("mmc rescan", 0);
                if (ret == 0) {
                	ret = DHCOMupdate(cmdtp, argc, argv, "mmc", "1:1");
			if (ret == -EAGAIN || (ret == -ENOENT && !update_auto)) {
				/**
				 * ret == -EAGAIN 
				 *     no DHupdate.ini file -> try next storage
				 * ret == -ENOENT && !update_auto
				 *     missing image file in console mode -> try next storage
				 */
				printf ("\n");
			} else
				return ret;
             	}
                else
                	printf ("\n--> Update INFO: No MicroSD - Card detected!\n");
      	}

        // SD/MMC Card Slot
#ifdef CONFIG_CMD_DHCOM_SETTINGS
        if (settings_get_extSD() || !update_auto) {
#else
	if (true) {
#endif
                // Call update function, if MMC/SD init returns success
                run_command("mmc dev 0", 0);
                ret = run_command("mmc rescan", 0);
                if(ret == 0) {
                        ret = DHCOMupdate(cmdtp, argc, argv, "mmc", "0:1");
			if (ret == -EAGAIN || (ret == -ENOENT && !update_auto)) {
				/**
				 * ret == -EAGAIN 
				 *     no DHupdate.ini file -> try next storage
				 * ret == -ENOENT && !update_auto
				 *     missing image file in console mode -> try next storage
				 */
				printf ("\n");
			} else
				return ret;
                }
                else
                        printf ("\n--> Update INFO: No MMC/SD - Card detected!\n");
        }

	// USB Host 1 Port
#ifdef CONFIG_CMD_DHCOM_SETTINGS
        if (settings_get_usbhost1() || !update_auto) {
#else
	if (true) {
#endif
                // Initialize USB Stick
                printf ("--> Update: Initialize USB Stick on Host Port");
                ret = run_command("usb start", 0);
                if(ret == 0) {
                        ret = DHCOMupdate(cmdtp, argc, argv, "usb", "0:1");
			if (ret == -EAGAIN || (ret == -ENOENT && !update_auto)) {
				/**
				 * ret == -EAGAIN 
				 *     no DHupdate.ini file -> try next storage
				 * ret == -ENOENT && !update_auto
				 *     missing image file in console mode -> try next storage
				 */
				printf ("\n");
			} else
				return ret;
                }
                else
                        printf ("\n--> Update INFO: No USB Stick detected!\n");
        }

	// USB OTG Port
#ifdef CONFIG_CMD_DHCOM_SETTINGS
        if (settings_get_usbotg() || !update_auto) {
#else
	if (true) {
#endif
                printf ("\n--> Update INFO: USB OTG update is not supported!\n");
	}

        return 0;
}

U_BOOT_CMD(
	update, 3, 	0, 	do_dhcom_update,
        "DHCOM Update: update OS images from storage device.",
        "- Starts update with DHupdate.ini file\n"
        "         Necessary files on Storage Device:\n"
        "           - DHupdate.ini file\n"
        "           - uboot.imx, ...\n"
        "update <type> [filename] - starts update without DHupdate.ini file\n"
        "         Types:\n"
        "           - bootloader = Bootloader update (default file name u-boot.imx)\n"
        "           - eeprom = Display adpater EEPROM update (default file name eeprom.bin)\n"
        "           - script = Run bootloader script (default file name script.bin)\n"
        "           - auto = Run DHupdate.ini update from command line\n"
#ifdef DH_IMX6_NAND_VERSION
        "           - wince = WinCE image update (default file name nk.nb0)\n"
        "           - eboot = eboot image update (default file name eboot.nb0)\n"
#endif
);
