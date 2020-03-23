// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2020 DH electronics GmbH
 * Christoph Niedermaier <cniedermaier@dh-electronics.com>
 *
 * Based on the fuse code:
 * (C) Copyright 2009-2013 ADVANSEE
 * Benoît Thébaudeau <benoit.thebaudeau@advansee.com>
 */

#include <common.h>
#include <asm/arch/sys_proto.h>
#include <command.h>
#include <console.h>
#include <fuse.h>
#include <linux/errno.h>

static int strtou32(const char *str, unsigned int base, u32 *result)
{
	char *ep;

	*result = simple_strtoul(str, &ep, base);
	if (ep == str || *ep != '\0')
		return -EINVAL;

	return 0;
}

static int confirm_prog(void)
{
	puts("Warning: Programming fuses is an irreversible operation!\n"
			"         Use this command only if you are sure of "
					"what you are doing!\n"
			"\nReally perform this fuse programming? <y/N>\n");

	if (confirm_yesno())
		return 1;

	puts("Fuse programming aborted\n");
	return 0;
}

static int do_imxfmac(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	const char *op = argc >= 2 ? argv[1] : NULL;
	int confirmed = argc >= 3 && !strcmp(argv[2], "-y");
	u32 fec_id;
	int ret;
	bool has_second_mac = is_mx7() || is_mx6sx() || is_mx6ul() || is_mx6ull();
	unsigned char mac[6];
	char macstr[18];

	argc -= 2 + confirmed;
	argv += 2 + confirmed;

	if (argc != 1 || strtou32(argv[0], 0, &fec_id))
		return CMD_RET_USAGE;

	if ((fec_id == 1 && !has_second_mac) || fec_id >= 2) {
		printf("fec_id=%d is not supported by this device!\n", fec_id);
		goto err;
	}

	if (!strcmp(op, "read")) {
		u32 mac_addr0, mac_addr1, mac_addr2;

		if (fec_id == 0) {
			fuse_read(4, 2, &mac_addr0);
			fuse_read(4, 3, &mac_addr1);

			mac[0] = mac_addr1 >> 8;
			mac[1] = mac_addr1;

			mac[2] = mac_addr0 >> 24;
			mac[3] = mac_addr0 >> 16;
			mac[4] = mac_addr0 >> 8;
			mac[5] = mac_addr0;
		}

		if (fec_id == 1) {
			fuse_read(4, 3, &mac_addr1);
			fuse_read(4, 4, &mac_addr2);

			mac[0] = mac_addr2 >> 24;
			mac[1] = mac_addr2 >> 16;
			mac[2] = mac_addr2 >> 8;
			mac[3] = mac_addr2;

			mac[4] = mac_addr1 >> 24;
			mac[5] = mac_addr1 >> 16;
		}

		sprintf(macstr, "%02x:%02x:%02x:%02x:%02x:%02x",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

		printf("Reading from fuse...\nEthernet MAC[%d]=%s\n",
		       fec_id, macstr);

	} else if (!strcmp(op, "sense")) {
		u32 mac_addr0, mac_addr1, mac_addr2;

		if (fec_id == 0) {
			fuse_sense(4, 2, &mac_addr0);
			fuse_sense(4, 3, &mac_addr1);

			mac[0] = mac_addr1 >> 8;
			mac[1] = mac_addr1;

			mac[2] = mac_addr0 >> 24;
			mac[3] = mac_addr0 >> 16;
			mac[4] = mac_addr0 >> 8;
			mac[5] = mac_addr0;
		}

		if (fec_id == 1) {
			fuse_sense(4, 3, &mac_addr1);
			fuse_sense(4, 4, &mac_addr2);

			mac[0] = mac_addr2 >> 24;
			mac[1] = mac_addr2 >> 16;
			mac[2] = mac_addr2 >> 8;
			mac[3] = mac_addr2;

			mac[4] = mac_addr1 >> 24;
			mac[5] = mac_addr1 >> 16;
		}

		sprintf(macstr, "%02x:%02x:%02x:%02x:%02x:%02x",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

		printf("Sensing from fuse...\nEthernet MAC[%d]=%s\n",
		       fec_id, macstr);

	} else if (!strcmp(op, "prog")) {
		u32 mac_addr0, mac_addr1, mac_addr2;

		ret = eth_env_get_enetaddr_by_index("eth", fec_id, mac);
		sprintf(macstr, "%02x:%02x:%02x:%02x:%02x:%02x",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		if (ret) {
			printf("Found MAC address in environment:\neth%saddr=%s\n",
			       fec_id ? simple_itoa(fec_id) : "", macstr);
		} else {
			printf("Couldn't read environment variable \"eth%saddr\"!\n",
			       fec_id ? simple_itoa(fec_id) : "");
			goto err;
		}

		if (!is_valid_ethaddr(mac)) {
			printf("It is an invalid MAC address!\n");
			goto err;
		}

		if (fec_id == 0) {
			mac_addr1 = mac[0] << 8  |
				    mac[1];

			mac_addr0 = mac[2] << 24 |
				    mac[3] << 16 |
				    mac[4] << 8  |
				    mac[5];

			printf("Programming ethernet MAC[%d]...\n", fec_id);
			if (!confirmed && !confirm_prog())
				return CMD_RET_FAILURE;
			ret = fuse_prog(4, 2, mac_addr0);
			if (ret)
				goto err;
			ret = fuse_prog(4, 3, mac_addr1);
			if (ret)
				goto err;
		}

		if (fec_id == 1) {
			mac_addr2 = mac[0] << 24 |
				    mac[1] << 16 |
				    mac[2] << 8  |
				    mac[3];

			mac_addr1 = mac[4] << 24 |
				    mac[5] << 16;

			printf("Programming ethernet MAC[%d]...\n", fec_id);
			if (!confirmed && !confirm_prog())
				return CMD_RET_FAILURE;
			ret = fuse_prog(4, 3, mac_addr1);
			if (ret)
				goto err;
			ret = fuse_prog(4, 4, mac_addr2);
			if (ret)
				goto err;
		}
	} else {
		return CMD_RET_USAGE;
	}

	return 0;

err:
	puts("ERROR\n");
	return CMD_RET_FAILURE;
}

U_BOOT_CMD(
	imxfmac, CONFIG_SYS_MAXARGS, 0, do_imxfmac,
	"fuse i.MX ethernet MAC address from evironment value",
	        "read <fec_id> - read ethernet MAC from fuse\n"
	"imxfmac sense <fec_id> - sense ethernet MAC from fuse\n"
	"imxfmac prog [-y] <fec_id> - program ethernet MAC fuse from evironment value\n"
	"       fec_id=0 from evironment value 'ethaddr'\n"
	"       fec_id=1 from evironment value 'eth1addr'"

);
