//------------------------------------------------------------------------------
//
//  Type:  BSP_ARGS
//
//  Shared argument structure used by the bootloader and the OS image.
//
typedef struct {
    u8 mac[6];                       	// 0x10001044
    u32 RamNandLaunchAddress;           // DH Electonics NAND Image in RAM Position, 0x1000104C
                                        // the Address will be written by uBoot
} BSP_ARGS;


/* End of File */