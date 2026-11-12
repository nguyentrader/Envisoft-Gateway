/*
 * modbus_rtu.c
 *
 *  Created on: 13 Nov 2026
 *      Author: LE TRAN NGUYEN
 */

#include "modbus_rtu.h"
#include <string.h>

#define MODBUS_FC_READ_HOLDING       0x03
#define MODBUS_FC_READ_INPUT         0x04
#define MODBUS_FC_WRITE_SINGLE       0x06
#define MODBUS_FC_WRITE_MULTIPLE     0x10

#define MODBUS_EXCEPTION_MASK        0x80
#define MODBUS_MAX_FRAME_SIZE        256

/* =========================================================
 * CRC16
 * ========================================================= */

uint16_t ModbusRTU_CalculateCRC(
    const uint8_t *data,
    uint16_t length)
{
    uint16_t crc = 0xFFFF;

    if (data == NULL)
    {
        return 0;
    }

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/* =========================================================
 * Common UART / RTU functions
 * ========================================================= */

static ModbusRTU_Status RTU_TransmitRequest(
    UART_HandleTypeDef *huart,
    uint8_t *request,
    uint16_t length,
    uint32_t timeout)
{
    if (huart == NULL || request == NULL || length == 0)
    {
        return MODBUS_RTU_ERROR;
    }

    /* Clear old received data before sending a new request. */
    __HAL_UART_FLUSH_DRREGISTER(huart);

    if (HAL_UART_Transmit(
            huart,
            request,
            length,
            timeout) != HAL_OK)
    {
        return MODBUS_RTU_ERROR;
    }

    return MODBUS_RTU_OK;
}

static ModbusRTU_Status RTU_ReceiveBytes(
    UART_HandleTypeDef *huart,
    uint8_t *buffer,
    uint16_t length,
    uint32_t timeout)
{
    if (huart == NULL || buffer == NULL || length == 0)
    {
        return MODBUS_RTU_ERROR;
    }

    if (HAL_UART_Receive(
            huart,
            buffer,
            length,
            timeout) != HAL_OK)
    {
        return MODBUS_RTU_TIMEOUT;
    }

    return MODBUS_RTU_OK;
}

static ModbusRTU_Status RTU_CheckCRC(
    const uint8_t *frame,
    uint16_t length)
{
    if (frame == NULL || length < 4)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    uint16_t received_crc =
        (uint16_t)frame[length - 2] |
        ((uint16_t)frame[length - 1] << 8);

    uint16_t calculated_crc =
        ModbusRTU_CalculateCRC(frame, length - 2);

    if (received_crc != calculated_crc)
    {
        return MODBUS_RTU_CRC_ERROR;
    }

    return MODBUS_RTU_OK;
}

/*
 * Receive and validate a Modbus exception response.
 *
 * Exception frame:
 * Slave ID | Function | Exception Code | CRC Low | CRC High
 *
 * The first two bytes have already been received.
 */
static ModbusRTU_Status RTU_ReceiveException(
    UART_HandleTypeDef *huart,
    uint8_t slave_id,
    uint8_t function,
    uint8_t *first_two_bytes,
    uint32_t timeout)
{
    uint8_t response[5];

    response[0] = first_two_bytes[0];
    response[1] = first_two_bytes[1];

    if (response[0] != slave_id ||
        response[1] != (uint8_t)(function | MODBUS_EXCEPTION_MASK))
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    if (RTU_ReceiveBytes(
            huart,
            &response[2],
            3,
            timeout) != MODBUS_RTU_OK)
    {
        return MODBUS_RTU_TIMEOUT;
    }

    ModbusRTU_Status crc_status =
        RTU_CheckCRC(response, sizeof(response));

    if (crc_status != MODBUS_RTU_OK)
    {
        return crc_status;
    }

		volatile uint8_t modbus_exception_code = response[2];

    return MODBUS_RTU_EXCEPTION;
}

/* =========================================================
 * FC03 / FC04: Read Registers
 * ========================================================= */

static ModbusRTU_Status RTU_ReadRegisters(
    UART_HandleTypeDef *huart,
    uint8_t slave_id,
    uint8_t function,
    uint16_t start_address,
    uint16_t quantity,
    uint16_t *registers,
    uint32_t timeout)
{
    uint8_t request[8];
    uint8_t response[MODBUS_MAX_FRAME_SIZE];

    uint16_t crc;
    uint16_t response_length;

    if (huart == NULL ||
        registers == NULL ||
        slave_id == 0 ||
        slave_id > 247 ||
        quantity == 0 ||
        quantity > MODBUS_RTU_MAX_REGISTERS ||
        (uint32_t)start_address + quantity > 65536UL)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    /* Build request */
    request[0] = slave_id;
    request[1] = function;
    request[2] = (uint8_t)(start_address >> 8);
    request[3] = (uint8_t)(start_address & 0xFF);
    request[4] = (uint8_t)(quantity >> 8);
    request[5] = (uint8_t)(quantity & 0xFF);

    crc = ModbusRTU_CalculateCRC(request, 6);

    request[6] = (uint8_t)(crc & 0xFF);
    request[7] = (uint8_t)(crc >> 8);

    /* Transmit request */
    ModbusRTU_Status status =
        RTU_TransmitRequest(
            huart,
            request,
            sizeof(request),
            timeout
        );

    if (status != MODBUS_RTU_OK)
    {
        return status;
    }

    /* Receive Slave ID and Function Code */
    if (RTU_ReceiveBytes(
            huart,
            response,
            2,
            timeout) != MODBUS_RTU_OK)
    {
        return MODBUS_RTU_TIMEOUT;
    }

    if (response[0] != slave_id)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    /* Exception response */
    if (response[1] == (uint8_t)(function | MODBUS_EXCEPTION_MASK))
    {
        return RTU_ReceiveException(
            huart,
            slave_id,
            function,
            response,
            timeout
        );
    }

    if (response[1] != function)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    /* Receive byte count */
    if (RTU_ReceiveBytes(
            huart,
            &response[2],
            1,
            timeout) != MODBUS_RTU_OK)
    {
        return MODBUS_RTU_TIMEOUT;
    }

    if (response[2] != quantity * 2)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    response_length = 3 + response[2] + 2;

    /* Receive register data and CRC */
    if (RTU_ReceiveBytes(
            huart,
            &response[3],
            response_length - 3,
            timeout) != MODBUS_RTU_OK)
    {
        return MODBUS_RTU_TIMEOUT;
    }

    /* Validate CRC */
    status = RTU_CheckCRC(response, response_length);

    if (status != MODBUS_RTU_OK)
    {
        return status;
    }

    /* Decode registers */
    for (uint16_t i = 0; i < quantity; i++)
    {
        registers[i] =
            ((uint16_t)response[3 + i * 2] << 8) |
            response[4 + i * 2];
    }

    return MODBUS_RTU_OK;
}

/* FC03 */
ModbusRTU_Status ModbusRTU_ReadHoldingRegisters(
    UART_HandleTypeDef *huart,
    uint8_t slave_id,
    uint16_t start_address,
    uint16_t quantity,
    uint16_t *registers,
    uint32_t timeout)
{
    return RTU_ReadRegisters(
        huart,
        slave_id,
        MODBUS_FC_READ_HOLDING,
        start_address,
        quantity,
        registers,
        timeout
    );
}

/* FC04 */
ModbusRTU_Status ModbusRTU_ReadInputRegisters(
    UART_HandleTypeDef *huart,
    uint8_t slave_id,
    uint16_t start_address,
    uint16_t quantity,
    uint16_t *registers,
    uint32_t timeout)
{
    return RTU_ReadRegisters(
        huart,
        slave_id,
        MODBUS_FC_READ_INPUT,
        start_address,
        quantity,
        registers,
        timeout
    );
}

/* =========================================================
 * FC06 / FC16: Write Registers
 * ========================================================= */

/*
 * FC06 response: echo of the 8-byte request.
 * FC16 response: Slave | 0x10 | Start Address | Quantity | CRC.
 *
 * The caller provides the expected four bytes after the
 * Slave ID and Function Code.
 */
static ModbusRTU_Status RTU_WriteRequest(
    UART_HandleTypeDef *huart,
    uint8_t slave_id,
    uint8_t function,
    const uint8_t *request,
    uint16_t request_length,
    const uint8_t expected_data[4],
    uint32_t timeout)
{
    uint8_t response[8];

    ModbusRTU_Status status =
        RTU_TransmitRequest(
            huart,
            (uint8_t *)request,
            request_length,
            timeout
        );

    if (status != MODBUS_RTU_OK)
    {
        return status;
    }

    /* Receive Slave ID and Function Code */
    if (RTU_ReceiveBytes(
            huart,
            response,
            2,
            timeout) != MODBUS_RTU_OK)
    {
        return MODBUS_RTU_TIMEOUT;
    }

    if (response[0] != slave_id)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    /* Exception response */
    if (response[1] == (uint8_t)(function | MODBUS_EXCEPTION_MASK))
    {
        return RTU_ReceiveException(
            huart,
            slave_id,
            function,
            response,
            timeout
        );
    }

    if (response[1] != function)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    /* Receive remaining response bytes */
    if (RTU_ReceiveBytes(
            huart,
            &response[2],
            6,
            timeout) != MODBUS_RTU_OK)
    {
        return MODBUS_RTU_TIMEOUT;
    }

    /* Validate CRC */
    status = RTU_CheckCRC(response, sizeof(response));

    if (status != MODBUS_RTU_OK)
    {
        return status;
    }

    /* Validate echoed address and value/quantity */
    if (memcmp(
            &response[2],
            expected_data,
            4) != 0)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    return MODBUS_RTU_OK;
}

/* FC06 */
ModbusRTU_Status ModbusRTU_WriteSingleRegister(
    UART_HandleTypeDef *huart,
    uint8_t slave_id,
    uint16_t address,
    uint16_t value,
    uint32_t timeout)
{
    uint8_t request[8];
    uint8_t expected_data[4];

    if (huart == NULL ||
        slave_id == 0 ||
        slave_id > 247)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    request[0] = slave_id;
    request[1] = MODBUS_FC_WRITE_SINGLE;
    request[2] = (uint8_t)(address >> 8);
    request[3] = (uint8_t)(address & 0xFF);
    request[4] = (uint8_t)(value >> 8);
    request[5] = (uint8_t)(value & 0xFF);

    uint16_t crc = ModbusRTU_CalculateCRC(request, 6);

    request[6] = (uint8_t)(crc & 0xFF);
    request[7] = (uint8_t)(crc >> 8);

    memcpy(expected_data, &request[2], 4);

    return RTU_WriteRequest(
        huart,
        slave_id,
        MODBUS_FC_WRITE_SINGLE,
        request,
        sizeof(request),
        expected_data,
        timeout
    );
}

/* FC16 */
ModbusRTU_Status ModbusRTU_WriteMultipleRegisters(
    UART_HandleTypeDef *huart,
    uint8_t slave_id,
    uint16_t start_address,
    uint16_t quantity,
    const uint16_t *registers,
    uint32_t timeout)
{
    uint8_t request[MODBUS_MAX_FRAME_SIZE];
    uint8_t expected_data[4];

    if (huart == NULL ||
        registers == NULL ||
        slave_id == 0 ||
        slave_id > 247 ||
        quantity == 0 ||
        quantity > MODBUS_RTU_MAX_WRITE_REGISTERS ||
        (uint32_t)start_address + quantity > 65536UL)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    request[0] = slave_id;
    request[1] = MODBUS_FC_WRITE_MULTIPLE;
    request[2] = (uint8_t)(start_address >> 8);
    request[3] = (uint8_t)(start_address & 0xFF);
    request[4] = (uint8_t)(quantity >> 8);
    request[5] = (uint8_t)(quantity & 0xFF);
    request[6] = (uint8_t)(quantity * 2);

    for (uint16_t i = 0; i < quantity; i++)
    {
        request[7 + i * 2] =
            (uint8_t)(registers[i] >> 8);

        request[8 + i * 2] =
            (uint8_t)(registers[i] & 0xFF);
    }

    uint16_t request_length = 7 + quantity * 2;

    uint16_t crc = ModbusRTU_CalculateCRC(
        request,
        request_length
    );

    request[request_length] = (uint8_t)(crc & 0xFF);
    request[request_length + 1] = (uint8_t)(crc >> 8);

    expected_data[0] = request[2];
    expected_data[1] = request[3];
    expected_data[2] = request[4];
    expected_data[3] = request[5];

    return RTU_WriteRequest(
        huart,
        slave_id,
        MODBUS_FC_WRITE_MULTIPLE,
        request,
        request_length + 2,
        expected_data,
        timeout
    );
}

