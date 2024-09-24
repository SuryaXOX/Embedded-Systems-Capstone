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
#include "stdbool.h"

int main(void)
{



	cy_rslt_t init_p5_0;

    // Initialize pin P5_2 as an input -- DIR
	init_p5_0 = cyhal_gpio_init(P5_0, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULL_NONE, false);

    if(init_p5_0 == CY_RSLT_TYPE_ERROR || init_p5_0 == CY_RSLT_TYPE_FATAL){
        	while(1){

        	}
    }

	cy_rslt_t init_p5_2;

    // Initialize pin P5_2 as an input -- DIR\

	//P9_1 FOR BLUE PSOC

	init_p5_2 = cyhal_gpio_init(P9_1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULL_NONE, true);

    if(init_p5_2 == CY_RSLT_TYPE_ERROR || init_p5_2 == CY_RSLT_TYPE_FATAL){
        	while(1){

        	}
        }

    cy_rslt_t init_p5_3;

    // Initialize pin P0_0 as an input

    //P9_2 FOR BLUE PSOC

    init_p5_3 = cyhal_gpio_init(P9_2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULL_NONE, false);


    if(init_p5_3 == CY_RSLT_TYPE_ERROR || init_p5_3 == CY_RSLT_TYPE_FATAL){
    	while(1){

    	}
    }


while (1) {
	int i = 50;					   //determines how many steps we will be taking set to five for now
	for (int k = 0;k < i;k++)
		{

		//cyhal_gpio_write(P5_2,dir);
		cyhal_gpio_toggle(P9_2);
			cyhal_system_delay_ms(100);
			cyhal_gpio_toggle(P9_2);
			cyhal_system_delay_ms(100);
		}
	cyhal_gpio_toggle(P9_1);
	cyhal_system_delay_ms(100);
	for (int k = 0; k < i*2; k++)
		{
		//cyhal_gpio_write(P5_2,dir);
		cyhal_gpio_toggle(P9_2);
		cyhal_system_delay_ms(100);
		cyhal_gpio_toggle(P9_2);
		cyhal_system_delay_ms(100);
		//CYHAL_GPIO_DRIVE_PULL_NONE(P5_2,dir);

		}
	cyhal_gpio_toggle(P9_1);
	cyhal_system_delay_ms(100);
	for (int k = 0; k < i; k++)
		{
		//cyhal_gpio_write(P5_2,dir);
		cyhal_gpio_toggle(P9_2);
		cyhal_system_delay_ms(100);
		cyhal_gpio_toggle(P9_2);
		cyhal_system_delay_ms(100);
		//CYHAL_GPIO_DRIVE_PULL_NONE(P5_2,dir);

		//cyhal_system_delay_us(5);
		}

	}

}


/* [] END OF FILE */
