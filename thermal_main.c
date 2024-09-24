/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC6 Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include <stdio.h>

#include "main.h"
#define ENABLE_I2C 1

uint16_t thermal_data[24][32] = {{0x0000}};
int xOffset;
int yOffset;

void find_max_pix(void){
	int i;
	int j;
	int maxX = 0;
	int maxY = 0;
    int curMaxVal = -32768;
	for(i = 0; i< 24;i++){
		for(j=0; j<32;j++){
			uint16_t cur_pix = thermal_data[i][j];
			int negative = (cur_pix & (1 << 15)) != 0;
			int decimal_read;
			if (negative)
			    decimal_read = cur_pix | ~((1 << 16) - 1);
            else
                decimal_read = cur_pix;
			if(decimal_read>curMaxVal){
				curMaxVal = decimal_read;
				maxX = j;
				maxY = i;
			}
		}
	}
	xOffset = maxX;
	yOffset = maxY;
}

int main(void)
{

    i2c_init();

	/* Enable global interrupts */
	__enable_irq();


    while(1)
    {


		#if ENABLE_I2C


    		//read data from thermal camera


    			uint8_t buffer[PACKET_SIZE];
    			buffer[0] = PACKET_BLANK;
    			buffer[1] = PACKET_BLANK;


    			uint16_t RAM_ADDR = 0x0400;

    			int k;
				int pixel_row = 0;
    			for(k=0;k<24;k++){



    			//Read 16 pixels from subpage 0
					int i;
					int pixel_col = 0;
					for(i=0;i<16;i++){

						if(CY_RSLT_SUCCESS != cyhal_i2c_master_mem_read(&i2c_master_obj, I2C_SLAVE_ADDR, RAM_ADDR, 2, buffer, PACKET_SIZE, 0)){
							while(1){};
						}
						uint16_t pixel_reading = (buffer[0] << 8) + buffer[1];
						thermal_data[pixel_row][pixel_col] = pixel_reading;
						RAM_ADDR += 0x0001;
						pixel_col += 2;
					}
					//Write to control register to switch to subpage 1
					uint16_t CONTROL_REG_ADDR = 0x800D;
					buffer[0] = 0x19;
					buffer[1] = 0x11;
					if(CY_RSLT_SUCCESS != cyhal_i2c_master_mem_write(&i2c_master_obj, I2C_SLAVE_ADDR, CONTROL_REG_ADDR, 2, buffer, PACKET_SIZE, 0)){
						while(1){};
					}
					//Read 16 pixels from subpage 1
					int j;
					pixel_col = 1;
					RAM_ADDR -= 0x0010;
					for(j=0;j<16;j++){
						if(CY_RSLT_SUCCESS != cyhal_i2c_master_mem_read(&i2c_master_obj, I2C_SLAVE_ADDR, RAM_ADDR, 2, buffer, PACKET_SIZE, 0)){
							while(1){};
						}
						uint16_t pixel_reading = (buffer[0] << 8) + buffer[1];
						thermal_data[pixel_row][pixel_col] = pixel_reading;
						RAM_ADDR += 0x0001;
						pixel_col += 2;
					}
					//Write to control register to switch back to subpage 0
					buffer[0] = 0x19;
					buffer[1] = 0x10;
					if(CY_RSLT_SUCCESS != cyhal_i2c_master_mem_write(&i2c_master_obj, I2C_SLAVE_ADDR, CONTROL_REG_ADDR, 2, buffer, PACKET_SIZE, 0)){
						while(1){};
					}

					RAM_ADDR += 0x0010;
					pixel_row++;

    			}

    			find_max_pix();


		#endif

    }
}



/* [] END OF FILE */

