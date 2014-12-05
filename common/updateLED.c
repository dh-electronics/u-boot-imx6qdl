/*
 * 2012 (c) DH electronics GmbH
 * Author: Andreas Geisreiter <ageisreiter at dh-electronics.de>
 *
 */

#include <common.h>
#include <command.h>
#include <linux/ctype.h>
#include <asm/io.h>
#include <dh_settings.h>
#include <dh_update.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>

extern void udelay(unsigned long usec);

enum UpdateGPIOEnum eDHCOMUpdateGPIO = GPIO_NOT_DEFINED;
unsigned iUpdateLEDPin;

unsigned DHCOM_update_gpios[] = {
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

//------------------------------------------------------------------------------
//
//  Function:  DHCOMUpdateLED_Init
//
//  This function initialize the DHCOM update LED GPIO
//
//  Commit values:  - *p_stDHupdateINI = Pointer to DHupdateINI struct
//
//  Return value:   0 = No error
//                  1 = error
//
int DHCOMUpdateLED_Init(updateinfo_t *p_stDHupdateINI)
{
   char cCompareStringLEDGPIO[7] = {"GPIO_A\0"};
   
   for(eDHCOMUpdateGPIO = GPIO_A; eDHCOMUpdateGPIO != GPIO_NOT_DEFINED; eDHCOMUpdateGPIO++, cCompareStringLEDGPIO[5]++)
   {
        if(!(memcmp ( p_stDHupdateINI->p_cUpdateGpioName, (char*)&cCompareStringLEDGPIO[0], 7)))
        {
            break;
        }
   }
   
    switch (eDHCOMUpdateGPIO) {
        case GPIO_A:
			iUpdateLEDPin = DHCOM_update_gpios[0];
            break;
        case GPIO_B:
			iUpdateLEDPin = DHCOM_update_gpios[1];
            break;
        case GPIO_C:
			iUpdateLEDPin = DHCOM_update_gpios[2];
            break;
        case GPIO_D:
			iUpdateLEDPin = DHCOM_update_gpios[3];
            break;
        case GPIO_E:
			iUpdateLEDPin = DHCOM_update_gpios[4];
            break;
        case GPIO_F:
			iUpdateLEDPin = DHCOM_update_gpios[5];
            break;
        case GPIO_G:
			iUpdateLEDPin = DHCOM_update_gpios[6];
            break;
        case GPIO_H:
			iUpdateLEDPin = DHCOM_update_gpios[7];
            break;
        case GPIO_I:
			iUpdateLEDPin = DHCOM_update_gpios[8];
            break;
        default:
            return 1;
            break;
    }

    // Set to output

    
    return 0;
}

//------------------------------------------------------------------------------
//
//  Function:  DHCOMUpdateLED_SetHigh
//
//  This function sets the DHCOM update LED GPIO to high
//
//  Commit values:  - 
//
//  Return value:   -
//
void DHCOMUpdateLED_SetHigh(void)
{
    if(eDHCOMUpdateGPIO != GPIO_NOT_DEFINED)
    {
    	// Set to high
		gpio_direction_output(iUpdateLEDPin , 1);
    }
}

//------------------------------------------------------------------------------
//
//  Function:  DHCOMUpdateLED_SetLow
//
//  This function sets the DHCOM update LED GPIO to low
//
//  Commit values:  -
//
//  Return value:   -
//
void DHCOMUpdateLED_SetLow(void)
{
    if(eDHCOMUpdateGPIO != GPIO_NOT_DEFINED)
    {
		// Set to low
		gpio_direction_output(iUpdateLEDPin , 0);
    }
}

//------------------------------------------------------------------------------
//
//  Function:  get_DHCOMUpdateLED
//
//  This function is to read the update-led gpio number
//
//  Commit values:  -
//
//  Return value:   -
//
int get_DHCOMUpdateLED(void)
{
	return iUpdateLEDPin;
}

//------------------------------------------------------------------------------
//
//  Function:  DHCOMUpdateDelayMs
//
//  This function calls the CPU specified uDelay function
//
//  Commit values:  - msec = delay time in msec
//
//  Return value:   -
//
void DHCOMUpdateDelayMs(unsigned long msec)
{

    unsigned long usec;
    
    usec = msec * 1000;
    
    udelay(usec);
}
