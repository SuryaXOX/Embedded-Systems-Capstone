/*
 * i2c.h
 *
 *  Created on: Jan 21, 2022
 *      Author: Joe Krachey
 */

#ifndef I2C_H_
#define I2C_H_

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

/* Macros */
#define PIN_MCU_SCL			P6_4
#define PIN_MCU_SDA			P6_5
#define I2C_MASTER_FREQUENCY 100000u
#define I2C_SLAVE_FREQUENCY  100000u
#define I2C_SLAVE_ADDR    0x33
#define SL_RD_BUFFER_SIZE (0x02u)
#define SL_WR_BUFFER_SIZE (0x02u)

//#define 	CYHAL_I2C_MODE_MASTER

#define I2C_SLAVE_IRQ_PRIORITY  (7u)
#define PACKET_SIZE             (2UL)
#define PACKET_SOP_POS          (0UL)
#define PACKET_CMD_POS          (1UL)
#define PACKET_EOP_POS          (2UL)
#define PACKET_STS_POS          (1UL)
#define PACKET_SOP              (0x01UL)
#define PACKET_EOP              (0x17UL)
#define PACKET_BLANK            (0x00UL)

/* Public Global Variables */
extern cyhal_i2c_t i2c_master_obj;
extern cyhal_i2c_t i2c_slave_obj;

extern cyhal_i2c_t i2c_master_obj;
extern cyhal_i2c_t i2c_slave_obj;

extern uint8_t i2c_read_buffer[SL_RD_BUFFER_SIZE];
extern uint8_t i2c_write_buffer[SL_WR_BUFFER_SIZE];

/* Public API */

void Thermal_write_reg(uint8_t reg, uint8_t value);

uint8_t Thermal_read_reg(uint8_t reg);

void i2c_init(void);

#endif /* I2C_H_ */
