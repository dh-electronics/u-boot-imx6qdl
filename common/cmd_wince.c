#include <common.h>
#include <command.h>
#include <linux/ctype.h>
#include <net.h>
#include <dh_settings.h>
#include <wince_args.h>
#ifdef DH_IMX6_NAND_VERSION
#include <nand.h>
#endif /* DH_IMX6_NAND_VERSION  */	

#define DEFAULT_MAC_ADDRESS	{0x1020,0x3040,0x5060}

BSP_ARGS *g_pBSPArgs;

//------------------------------------------------------------------------------
//
//  Function:  do_bootwince
//
//  Description:    Load and check WEC Image
//
//  Return value:   0 = No error
//                  1 = error
//
int do_bootwince(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	uchar enetaddr[6];
	u16 EthMacAddress[] = DEFAULT_MAC_ADDRESS;
	char *command;

#ifdef DH_IMX6_NAND_VERSION
	nand_info_t *nand;
	size_t read_size = 0;
	size_t write_size = 0;
	loff_t flash_off = 0;
	loff_t redundant_flash_off = 0;
	nand_erase_options_t opts;	
	int i = 0;
	char *p_WEC_NAND_flash_address          = getenv ("wec_nand_flash_address");
	char *p_WEC_partition_size              = getenv ("wec_nand_partition_size");
	char *p_eboot_RAM_address		= getenv ("eboot_image_addr");
	char *p_WEC_RAM_buffer			= getenv ("wec_ram_buffer");
	char *p_RAMBufferAddress          	= getenv ("loadaddr");	
	char *p_WEC_image_size          	= getenv ("wec_image_size");		
	char *p_WEC_image_type_nb0		= getenv ("wec_image_type_nb0"); 
	char *p_WEC_image_type_gz		= getenv ("wec_image_type_gz"); 
#endif /* DH_IMX6_NAND_VERSION  */		
	
	// ***************** NAND Flash Layout *************************
	// Address		Description		Partition Size	
	// -------------------------------------------------------------
	// 0x0000_0000		First WEC Image		100MByte (94MB Image Size --> 6MB reserved for Bad Blocks)
	// ...				
	// 0x063F_FFFF		
	// 0x0640_0000		Redundant WEC Image	100MByte (94MB Image Size --> 6MB reserved for Bad Blocks)
	// ...
	// 0x0C7F_FFFF
	// 0x0C80_0000		WEC Flash Partition	312MByte
	// ...
	// 0x1FFF_FFFF
	
	// ********************* RAM Layout ***********************************************************************************
	// Address		Description				
	// --------------------------------------------------------------------------------------------------------------------
	// 0x1000_1044		WEC boot arguments address
	// ...
	// 0x1004_1000		eboot RAM Address	
	// ...		
	// 0x1020_0000		ENV "wec_image_addr": WEC image SDRAM address
	// ... 
	// 0x1200_0000		ENV "loadaddr": Used for redundant image block check (only 128kB ( = NAND erase size) used)
	// ...
	// 0x1800_0000		ENV "wec_ram_buffer": RAM Buffer address for first copy of Image (size = max. 94MB)
	// ...
	// 0x1DDF_FFFF
	
#ifdef DH_IMX6_NAND_VERSION  
	unsigned long uleboot_RAM_address	= simple_strtoul (p_eboot_RAM_address, NULL, 16);
	unsigned long ulWEC_RAM_buffer		= simple_strtoul (p_WEC_RAM_buffer, NULL, 16);	
	unsigned long ulRAMBufferAddress	= simple_strtoul (p_RAMBufferAddress, NULL, 16);	
	unsigned long ulImageOffset = 0;
	unsigned long ulRedundantImageOffset = 0;
	unsigned long WinCEPartitionSize = 0;	
	unsigned long ulFilesize		= simple_strtoul (p_WEC_image_size, NULL, 16);
#endif /* DH_IMX6_NAND_VERSION  */
	
	// Initialize the BSP args structure.
	//
	g_pBSPArgs = (BSP_ARGS *) BOOTCE_ARGUMENTS_SDRAM_ADDRESS;
	
	memset((void *)g_pBSPArgs, 0, sizeof(BSP_ARGS));
	eth_getenv_enetaddr("ethaddr", enetaddr);
	EthMacAddress[2] = (enetaddr[5] << 8) | enetaddr[4];
	EthMacAddress[1] = (enetaddr[3] << 8) | enetaddr[2];
	EthMacAddress[0] = (enetaddr[1] << 8) | enetaddr[0];
	memcpy(g_pBSPArgs->mac,EthMacAddress,sizeof(g_pBSPArgs->mac)); 	
	g_pBSPArgs->RamNandLaunchAddress = simple_strtoul(getenv ("wec_image_addr"), NULL, 16);
	

#ifdef DH_IMX6_NAND_VERSION  
	/* run WEC image from nand */
	if ((argc == 2) && (strcmp (argv[1], "nand") == 0))  {	
		printf ("\n--> BootWinCE: WinEC 7 Image is in NAND Flash memory");  
		
		// Check if valid image is stored in NAND
		if ((p_WEC_image_type_gz == NULL) && (p_WEC_image_type_nb0 == NULL))
		{
			printf ("\n--> BootWinCE INFO: No valid WEC Image found on NAND Flash\n");  
			return 1;  
		}

		// Read first NAND Image
		nand = &nand_info[nand_curr_device];			// Get nand-info structure from current device

		// Calculate redundant OS flash offset 
		flash_off = simple_strtoul (p_WEC_NAND_flash_address, NULL, 16);	// WinEC image flash address			    
		WinCEPartitionSize = simple_strtoul (p_WEC_partition_size, NULL, 16);
		redundant_flash_off = flash_off + WinCEPartitionSize;				

		// Copy first image on second position if second image does not exist
		if (getenv("redundant_wince_image") == NULL)
		{
			read_size = ulFilesize;

			// Read first nk.gz copy
			if( nand_read_skip_bad(nand, flash_off, &read_size, NULL, nand->size, (u_char *)ulWEC_RAM_buffer) != 0)
			{
			    printf ("\n--> BootWinCE ERROR: Error during the reading of first image block.\n");
			    return 1;   
			}		

			printf ("\n--> BootWinCE: Create Redundant WinEC 7 Image in NAND Flash\n");  

			// Erase erase redundant image area
			memset(&opts, 0, sizeof(opts));
			opts.offset = redundant_flash_off;
			opts.length = (nand->size - redundant_flash_off - (2*nand->erasesize)); // Erase flash till the end - 2 block which are used for bad block table
			opts.scrub = 0; 
			opts.jffs2  = 1;
			opts.quiet  = 0;
			opts.spread = 0;  

			if(nand_erase_opts(nand, &opts))
			{
				printf ("\n--> BootWinCE ERROR: Can't erase redundant image area.\n");
				return 1;  	
			}

			// Create redundant image
			write_size = ((read_size / nand->erasesize) * nand->erasesize) + nand->erasesize;
			if( nand_write_skip_bad(nand, redundant_flash_off, &write_size, NULL, nand->size, (u_char *)ulWEC_RAM_buffer, 0) != 0)
			{
				printf ("\n--> BootWinCE ERROR: Can't create redundant image copy.\n");	
				return 1;   
			}

			setenv("redundant_wince_image", "1");
			saveenv();
		}

		// Otherwise Check if both OS flash copys are OK?
		else
		{
			printf ("\n    ...Running error check at first and redundant image\n");  
			ulImageOffset = flash_off; 
			ulRedundantImageOffset = redundant_flash_off; 
			
			// NOTE: dh_nand_read_skip_bad() also return error, if ECC was neccessary!!!
			// This funtion is only used for checking block quality
			
			for(i = 0; (i*nand->erasesize) < ulFilesize; i++)
			{
				// CHECK FIRST IMAGE
				// *****************
				// Check if current "first image" block is bad?
				while (nand_block_isbad(nand, ulImageOffset))
				{
					ulImageOffset = ulImageOffset + nand->erasesize;
				}	
				
				// Check if current "first image" block content is damaged?
				read_size = nand->erasesize;
				if( /*dh_*/nand_read_skip_bad(nand, ulImageOffset, &read_size, NULL, nand->size, (u_char *)(ulWEC_RAM_buffer+(i*nand->erasesize))) != 0)
				{
					printf ("\n    Warning: Refresh of first image block 0x%x is necessary\n", (unsigned int)ulImageOffset);  					
					// Read content of "redundant image" flash block
					while (nand_block_isbad(nand, ulRedundantImageOffset))
					{
						ulRedundantImageOffset = ulRedundantImageOffset + nand->erasesize;
					}
					read_size = nand->erasesize;
					if( nand_read_skip_bad(nand, ulRedundantImageOffset, &read_size, NULL, nand->size, (u_char *)(ulWEC_RAM_buffer+(i*nand->erasesize))) != 0)
					{
						printf ("\n--> BootWinCE ERROR: Can't restore image!!!\n");
						return 1;   
					}
					
					// Erase damaged flash block of "first image"
					memset(&opts, 0, sizeof(opts));
					opts.offset = (ulong)ulImageOffset;
					opts.length = (ulong)nand->erasesize; // Erase only one block
					opts.scrub = 0; 
					opts.jffs2  = 1;
					opts.quiet  = 0;
					opts.spread = 0; 
					
					if(nand_erase_opts(nand, &opts))
					{
						printf ("\n--> BootWinCE ERROR: Can't erase block 0x%x of first image area\n",(unsigned int)ulImageOffset);
						return 1;  	
					}
					
					// Copy content of "redundant image" block to damaged "first image" block
					write_size = nand->erasesize;
					if( nand_write_skip_bad(nand, ulImageOffset, &write_size, NULL, nand->size, (u_char *)(ulWEC_RAM_buffer+(i*nand->erasesize)), 0) != 0)
					{
						printf ("\n--> BootWinCE ERROR: Write to block 0x%x of first image wasn't successful\n",(unsigned int)ulImageOffset);
						return 1;   
					}
				}

				// CHECK REDUNDANT IMAGE
				// *********************				
				// Check if current "redundant image" block is bad?
				while (nand_block_isbad(nand, ulRedundantImageOffset))
				{
					ulRedundantImageOffset = ulRedundantImageOffset + nand->erasesize;
				}	
					
				// Check if current "redundant image" block content is damaged?
				read_size = nand->erasesize;
				if( /*dh_*/nand_read_skip_bad(nand, ulRedundantImageOffset, &read_size, NULL, nand->size, (u_char *)ulRAMBufferAddress) != 0)
				{
					printf ("\n    Warning: Refresh of redundant image block 0x%x is necessary\n", (unsigned int)ulRedundantImageOffset);  
					// Read content of "first image" flash block
					while (nand_block_isbad(nand, ulImageOffset))
					{
						ulImageOffset = ulImageOffset + nand->erasesize;
					}
					read_size = nand->erasesize;
					if( nand_read_skip_bad(nand, ulImageOffset, &read_size, NULL, nand->size, (u_char *)ulRAMBufferAddress) != 0)
					{
						printf ("\n--> BootWinCE ERROR: Can't restore image!!!\n");
						return 1;   
					}
						
					// Erase damaged flash block of "redundant image"
					memset(&opts, 0, sizeof(opts));
					opts.offset = (ulong)ulRedundantImageOffset;
					opts.length = (ulong)nand->erasesize; // Erase only one block
					opts.scrub = 0; 
					opts.jffs2  = 1;
					opts.quiet  = 0;
					opts.spread = 0; 
						
					if(nand_erase_opts(nand, &opts))
					{
						printf ("\n--> BootWinCE ERROR: Can't erase block 0x%x of redundant image area\n",(unsigned int)ulRedundantImageOffset);
						return 1;  	
					}

					// Copy content of "first image" block to damaged "redundant image" block
					write_size = nand->erasesize;
					if( nand_write_skip_bad(nand, ulRedundantImageOffset, &write_size, NULL, nand->size, (u_char *)ulRAMBufferAddress, 0) != 0)
					{
						printf ("\n--> BootWinCE ERROR: Write to block 0x%x of redundant image wasn't successful\n",(unsigned int)ulRedundantImageOffset);
						return 1;   
					}						
				}

				
				// Increment to next block
				ulImageOffset = ulImageOffset + nand->erasesize;
				ulRedundantImageOffset = ulRedundantImageOffset + nand->erasesize;
				
				
			}

		}
		
		// Copy (nk.nb0) or unzip (nk.gz) WEC Image file
		if ((command = getenv ("copy_wec_image")) == NULL) 
		{
			printf ("--> BootWinCE ERROR: \"copy_wec_image\" not defined\n");
			return 1;
		}

		if (run_command (command, 0) != 0) 
		{
			printf ("--> BootWinCE ERROR: Command \"copy_wec_image\" faild\n");
			return 1;
		}	
		
		// Copy eboot image
		if (getenv("eboot_image") != NULL)
		{	
			printf ("\n--> BootWinCE: ## Starting WinEC 7 at 0x%08lx ...\n\n", uleboot_RAM_address);
			
			if ((command = getenv ("copy_and_start_eboot_image")) == NULL) 
			{
				printf ("--> BootWinCE ERROR: \"copy_and_start_eboot_image\" not defined\n");
				return 1;
			}

			if (run_command (command, 0) != 0) 
			{
				printf ("--> BootWinCE ERROR: Command \"copy_and_start_eboot_image\" faild\n");
				return 1;
			}	
		}
		else
		{
			printf ("--> BootWinCE ERROR: No eboot image stored in SPI flash\n");
			return 1;
		}
  	
		// Jump to start address
		printf ("\n--> BootWinCE ERROR: U-boot should never come here.\n\n");	
	}
	
	// run WEC image via ENV variable "load_wec_image"
	else 
	{
#endif /* DH_IMX6_NAND_VERSION  */	
		// Run WEC Start Command
		if ((command = getenv ("load_wec_image")) == NULL) 
		{
			printf ("Error: \"load_wec_image\" not defined\n");
			return 1;
		}

		if (run_command (command, 0) != 0) 
		{
			printf ("Error: Command \"load_wec_image\" faild\n");
			return 1;
		}
#ifdef DH_IMX6_NAND_VERSION  		
	}
#endif /* DH_IMX6_NAND_VERSION  */	
	
	return 0;
}

U_BOOT_CMD(
  bootwince,      3,      0,      do_bootwince,
  "Load Windows Embedded Compact Image",
  "- Load WEC image with env variable \"load_wec_image\".\n"
  "bootwince nand - Load WEC image from NAND flash.\n"
);
