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



int main(void)
{

	//enable interrupts
	    __enable_irq();
	    cy_rslt_t   rslt;
	    cyhal_pwm_t pwm_obj;

	      //Initialize PWM on the supplied pin and assign a new clock
	    rslt = cyhal_pwm_init(&pwm_obj, P9_0, NULL);

	      // Set a duty cycle of 25% and frequency of 100Hz
        rslt = cyhal_pwm_set_duty_cycle(&pwm_obj, 25, 100);

	      // Start the PWM output
        rslt = cyhal_pwm_start(&pwm_obj);

	      while (true)
	         {
	             // Stop the PWM output
	             rslt = cyhal_pwm_stop(&pwm_obj);

	             // Delay for observing the output
	             cyhal_system_delay_ms(2500);

	             // (Re-)start the PWM output
	             rslt = cyhal_pwm_start(&pwm_obj);

	             // Delay for observing the output
	             cyhal_system_delay_ms(5000);
	         }
}



/* [] END OF FILE */
