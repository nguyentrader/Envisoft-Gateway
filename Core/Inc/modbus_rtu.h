/*
 * modbus_rtu.h
 *
 *  Created on: 13 Nov 2026
 *      Author: LE TRAN NGUYEN
 */

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define MODBUS_RTU_DEFAULT_TIMEOUT       1000
#define MODBUS_RTU_MAX_REGISTERS         125
#define MODBUS_RTU_MAX_WRITE_REGISTERS   123

typedef enum
{
    MODBUS_RTU_OK = 0,
    MODBUS_RTU_ERROR,
    MODBUS_RTU_TIMEOUT,
    MODBUS_RTU_CRC_ERROR,
    MODBUS_RTU_EXCEPTION,
    MODBUS_RTU_INVALID_RESPONSE
} ModbusRTU_Status;

/* FC03: Read Holding Registers */
ModbusRTU_Status ModbusRTU_ReadHoldingRegisters(
    UART_HandleTypeDef *huart,
    uint8_t slave_id,
    uint16_t start_address,
    uint16_t quantity,
    uint16_t *registers,
    uint32_t timeout
);

/* FC04: Read Input Registers */
ModbusRTU_Status ModbusRTU_ReadInputRegisters(
    UART_HandleTypeDef *huart,
    uint8_t slave_id,
    uint16_t start_address,
    uint16_t quantity,
    uint16_t *registers,
    uint32_t timeout
);

/* FC06: Write Single Holding Register */
ModbusRTU_Status ModbusRTU_WriteSingleRegister(
    UART_HandleTypeDef *huart,
    uint8_t slave_id,
    uint16_t address,
    uint16_t value,
    uint32_t timeout
);

/* FC16: Write Multiple Holding Registers */
ModbusRTU_Status ModbusRTU_WriteMultipleRegisters(
    UART_HandleTypeDef *huart,
    uint8_t slave_id,
    uint16_t start_address,
    uint16_t quantity,
    const uint16_t *registers,
    uint32_t timeout
);

/* Modbus CRC16 */
uint16_t ModbusRTU_CalculateCRC(
    const uint8_t *data,
    uint16_t length
);

#endif /* MODBUS_RTU_H */

