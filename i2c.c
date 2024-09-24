/*
 * i2c.c
 *
 *  Created on: Jan 21, 2022
 *      Author: Joe Krachey
 */

#include "i2c.h"

#define SL_RD_BUFFER_SIZE (0x02u)
#define SL_WR_BUFFER_SIZE (0x02u)

#define STS_CMD_DONE            (0x00UL)
#define STS_CMD_FAIL            (0xFFUL)

cyhal_i2c_t i2c_master_obj;
cyhal_i2c_t i2c_slave_obj;


uint8_t i2c_read_buffer[SL_RD_BUFFER_SIZE] = {PACKET_SOP, STS_CMD_FAIL, PACKET_EOP};
uint8_t i2c_write_buffer[SL_WR_BUFFER_SIZE];


// Define the I2C monarch configuration structure
cyhal_i2c_cfg_t i2c_master_config =
{
    CYHAL_I2C_MODE_MASTER,
    0, // address is not used for master mode
    I2C_MASTER_FREQUENCY
};

cyhal_i2c_cfg_t i2c_slave_config =
{
		true,
		I2C_SLAVE_ADDR,
		I2C_SLAVE_FREQUENCY
};


/*
 * Writes to a register on the I2C thermal camera
 */
void Thermal_write_reg(uint8_t reg, uint8_t value)
{
	// Master Config
	uint8_t write_buffer[2];

	// Initialize write_buffer[] so that the register address and value are transmitted
	write_buffer[0] = reg;
	write_buffer[1] = value;

	// Use cyhal_i2c_master_write to write the required data to the device.
	cyhal_i2c_master_write(&i2c_master_obj, I2C_SLAVE_ADDR, write_buffer, 2, 0, true);


	//Slave Config

}

/**
 *
 * Reades a register on I2C thermal camera
 *
 */
uint8_t Thermal_read_reg(uint8_t reg)
{

// Master Config
	uint8_t write_buffer[1];
	uint8_t read_buffer[1];

	// Use cyhal_i2c_master_write to write the register address to the device.
	// Send the register address, do not generate a stop condition.  This will result in
	// a restart condition.
	write_buffer[0] = reg;
	cyhal_i2c_master_write(&i2c_master_obj, I2C_SLAVE_ADDR, write_buffer, 2, 0, false);

	// Use cyhal_i2c_master_read to read the required data from the device.
	// The register address has already been set in the write above, so read a single byte
	// of data.
	cyhal_i2c_master_read(&i2c_master_obj, I2C_SLAVE_ADDR, read_buffer, 2, 0, true);

	// Return the data
	return read_buffer[0];




}

void handle_slave_event(void *callback_arg, cyhal_i2c_event_t event)
{
    if (0UL == (CYHAL_I2C_SLAVE_ERR_EVENT & event))
    {
        if (0UL != (CYHAL_I2C_SLAVE_WR_CMPLT_EVENT & event))
        {
            /* Check start and end of packet markers. */
            if ((i2c_write_buffer[PACKET_SOP_POS] == PACKET_SOP) &&
                (i2c_write_buffer[PACKET_EOP_POS] == PACKET_EOP))
                {



                    /* Update status of received command. */
                    i2c_read_buffer[PACKET_STS_POS] = STS_CMD_DONE;
                }

            /* Configure read buffer for the next write */
            i2c_write_buffer[PACKET_SOP_POS] = 0;
            i2c_write_buffer[PACKET_EOP_POS] = 0;
            cyhal_i2c_slave_config_write_buffer(&i2c_master_obj, i2c_write_buffer, PACKET_SIZE);
        }

        if (0UL != (CYHAL_I2C_SLAVE_RD_CMPLT_EVENT & event))
        {
            /* Configure write buffer for the next read */
            i2c_read_buffer[PACKET_STS_POS] = STS_CMD_DONE;
            cyhal_i2c_slave_config_read_buffer(&i2c_master_obj, i2c_read_buffer, PACKET_SIZE);
        }
    }
}




/*******************************************************************************
* Function Name: i2c_init
********************************************************************************
* Summary:
* Initializes the I2C interface
*
* Parameters:
*  void
*
* Return:
*
*
*******************************************************************************/
void i2c_init(void)
{

	//Slave Config
    //Initialize thermal camera as slave device

		/*
		cy_rslt_t   rslt;

       // Initialize I2C slave, set the SDA and SCL pins and assign a new clock
       rslt = cyhal_i2c_init(&i2c_slave_obj, PIN_MCU_SDA, PIN_MCU_SCL, NULL);

       if (rslt != CY_RSLT_SUCCESS){
    	   printf("Error Initializing I2C/r/n/n");
    	   return;
       }

       if (rslt == CY_RSLT_SUCCESS)
           {
    	   // Configure the I2C resource to be slave
    	   rslt = cyhal_i2c_configure(&i2c_slave_obj, &i2c_slave_config);

           // Configure I2C slave write buffer for I2C master to write into
           rslt = cyhal_i2c_slave_config_write_buffer(&i2c_slave_obj, i2c_write_buffer, SL_WR_BUFFER_SIZE);

           // Configure I2C slave read buffer for I2C master to read from
           rslt = cyhal_i2c_slave_config_read_buffer(&i2c_slave_obj, i2c_read_buffer, SL_RD_BUFFER_SIZE);
           }

       	   // Handle error if I2C slave configuration failed
        if (rslt != CY_RSLT_SUCCESS){
        	printf("Error Initializing I2C (Slave) /r/n/n");
        	return;
        }


        // Register I2C slave event callback
        cyhal_i2c_register_callback(&i2c_slave_obj, (cyhal_i2c_event_callback_t)handle_i2c_events, NULL);

        // Enable I2C Events
        cyhal_i2c_enable_event(&i2c_slave_obj, (cyhal_i2c_event_t)   \
                                          (CYHAL_I2C_SLAVE_WR_CMPLT_EVENT \
                                           | CYHAL_I2C_SLAVE_RD_CMPLT_EVENT \
                                           | CYHAL_I2C_SLAVE_ERR_EVENT),    \
                                          3u, true);

        // Register I2C slave address callback
        cyhal_i2c_register_address_callback(&i2c_slave_obj, (cyhal_i2c_address_callback_t)handle_i2c_address_events, NULL);

        // Enable I2C Address Events
        cyhal_i2c_enable_address_event(&i2c_slave_obj, (cyhal_i2c_addr_event_t) (CYHAL_I2C_ADDR_MATCH_EVENT), 3u, true);

        */



        //Master Config
        // Initialize I2C monarch, set the SDA and SCL pins and assign a new clock
	cy_rslt_t   rslt;
	rslt = cyhal_i2c_init(&i2c_master_obj, PIN_MCU_SDA, PIN_MCU_SCL, NULL);

	if(rslt != CY_RSLT_SUCCESS)
	{
		while(1){};
	}

        // Configure the I2C resource to be monarch
	rslt =  cyhal_i2c_configure(&i2c_master_obj, &i2c_master_config);
	if(rslt & CYHAL_I2C_RSLT_ERR_INVALID_PIN){
		while(1){};
	}
        if(rslt != CY_RSLT_SUCCESS)
        	{
        		while(1){};
        	}
        //cyhal_i2c_slave_config_read_buffer( &i2c_master_obj, i2c_read_buffer, PACKET_SIZE);
        //cyhal_i2c_slave_config_write_buffer( &i2c_master_obj, i2c_write_buffer, PACKET_SIZE);
        //cyhal_i2c_register_callback( &i2c_master_obj, handle_slave_event, NULL);
        /*cyhal_i2c_enable_event( &i2c_master_obj,
            (cyhal_i2c_event_t)(CYHAL_I2C_SLAVE_WR_CMPLT_EVENT
                               | CYHAL_I2C_SLAVE_RD_CMPLT_EVENT
                               | CYHAL_I2C_SLAVE_ERR_EVENT),
                               I2C_SLAVE_IRQ_PRIORITY, true);
                               */



}
