#include <common.h>
#include <command.h>
#include <linux/ctype.h>
#include <net.h>
#include <dh_settings.h>
#include <wince_args.h>
#include <nand.h>

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
	
	// Run WEC Start Command
	if ((command = getenv ("load_wec_image")) == NULL) {
		printf ("Error: \"load_wec_image\" not defined\n");
		return 1;
	}

	if (run_command (command, 0) != 0) {
		printf ("Error: Command \"load_wec_image\" faild\n");
		return 1;
	}	
	
	return 0;
}

U_BOOT_CMD(
  bootwince,      3,      0,      do_bootwince,
  "Load Windows Embedded Compact image",
  "[xxx] - default text.\n"
);