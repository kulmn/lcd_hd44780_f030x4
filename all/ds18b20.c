/*
 * ds18b20.c
 *
 *  Created on: 30 ���� 2015 �.
 *      Author: kulish_y
 */

#include "ds18b20.h"

void DS18B20_Init(OWI *owi_int, uint8_t config)
{
//	OWI_Init(owi_int);

//    OWI_DetectPresence(owi_int);

	for (uint8_t i=0; i<4; i++)
	{
    OWI_DetectPresence(owi_int);

    OWI_SendByte(owi_int, OWI_ROM_SKIP);

    OWI_SendByte(owi_int, OWI_WRITE_SCRATCHPAD);
    OWI_SendByte(owi_int, 0);
    OWI_SendByte(owi_int, 0);
    OWI_SendByte(owi_int, config);
	}
//    OWI_DetectPresence(owi_int);
//    OWI_SendByte(owi_int, OWI_ROM_SKIP);

//    OWI_SendByte(owi_int, OWI_COPY_SCRATCHPAD);

}


/******************************************************************************/
uint16_t DS18B20_ReadTemperature(OWI *owi_int)
{
	uint8_t temp_hi,temp_lo;

/*

	OWI_DetectPresence(owi_int);
	    OWI_SendByte(owi_int, OWI_ROM_SKIP);

	    OWI_SendByte(owi_int, OWI_WRITE_SCRATCHPAD);
	    OWI_SendByte(owi_int, 0);
	    OWI_SendByte(owi_int, 0);
	    OWI_SendByte(owi_int, 0x7f);

*/
	FreeRtos_Delay(WAIT_TICKS_A);
    if (!OWI_DetectPresence(owi_int))				// 960 mks
    	return -127; // Error

    FreeRtos_Delay(WAIT_TICKS_A);
    OWI_SendByte(owi_int, OWI_ROM_SKIP);			// 560 mks
    FreeRtos_Delay(WAIT_TICKS_A);
    // Send start conversion command.
    OWI_SendByte(owi_int, OWI_START_CONVERSION);	// 560 mks

    FreeRtos_Delay(WAIT_TICKS_A);
    // Wait until conversion is finished.
    // Bus line is held low until conversion is finished.
    while (!OWI_ReadBit(owi_int)) 						// 70 mks
    	FreeRtos_Delay(WAIT_TICKS_B);

    FreeRtos_Delay(WAIT_TICKS_A);
    // Reset, presence.
    if(!OWI_DetectPresence(owi_int))				// 960 mks
    	return -127; // Error

    FreeRtos_Delay(WAIT_TICKS_A);
    OWI_SendByte(owi_int, OWI_ROM_SKIP);			// 560 mks
    FreeRtos_Delay(WAIT_TICKS_A);
    // Send READ SCRATCHPAD command.
    OWI_SendByte(owi_int, OWI_READ_SCRATCHPAD);		// 560 mks

    FreeRtos_Delay(WAIT_TICKS_A);
   // Read only two first bytes (temperature low, temperature high)
    temp_lo = OWI_ReceiveByte(owi_int);				// 560 mks
    FreeRtos_Delay(WAIT_TICKS_A);
    temp_hi = OWI_ReceiveByte(owi_int);				// 560 mks

	return ((temp_hi<<8) | temp_lo);
}




