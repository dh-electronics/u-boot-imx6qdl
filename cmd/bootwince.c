#include <common.h>
#include <environment.h>

#define BOOTCE_ARGUMENTS_SDRAM_ADDRESS	0x10001044
#define DEFAULT_MAC_ADDRESS		{0x1020,0x3040,0x5060}

typedef struct {
    u8 mac[6];                          // 0x10001044
    u32 RamNandLaunchAddress;           // DH Electonics NAND Image in RAM Position, 0x1000104C
                                        // the Address will be written by uBoot
} BSP_ARGS;

BSP_ARGS *g_pBSPArgs;

int do_bootwince(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	uchar enetaddr[6];
	u16 EthMacAddress[] = DEFAULT_MAC_ADDRESS;
	char *command;

	/* Initialize the BSP args structure. */
	g_pBSPArgs = (BSP_ARGS *)BOOTCE_ARGUMENTS_SDRAM_ADDRESS;

	memset((void *)g_pBSPArgs, 0, sizeof(BSP_ARGS));
	eth_env_get_enetaddr("ethaddr", enetaddr);
	EthMacAddress[2] = (enetaddr[5] << 8) | enetaddr[4];
	EthMacAddress[1] = (enetaddr[3] << 8) | enetaddr[2];
	EthMacAddress[0] = (enetaddr[1] << 8) | enetaddr[0];
	memcpy(g_pBSPArgs->mac,EthMacAddress,sizeof(g_pBSPArgs->mac));
	g_pBSPArgs->RamNandLaunchAddress = simple_strtoul(env_get("wec_image_addr"), NULL, 16);

	/* Run WEC Start Command */
	if ((command = env_get("load_wec_image")) == NULL)
	{
		printf("Error: \"load_wec_image\" not defined\n");
		return 1;
	}

	if (run_command(command, 0) != 0)
	{
		printf("Error: Command \"load_wec_image\" faild\n");
		return 1;
	}

	return 0;
}

U_BOOT_CMD(
  bootwince,      3,      0,      do_bootwince,
  "Load Windows Embedded Compact Image",
  "- Load WEC image with env variable \"load_wec_image\".\n"
);
