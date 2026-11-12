/*
 * w5500.c
 *
 *  Created on: 12 Nov 2026
 *      Author: LE TRAN NGUYEN
 */

#include "w5500.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

#define W5500_SPI_TIMEOUT          100U

/* -------------------------------------------------------------------------- */
/* Chip select                                                                */
/* -------------------------------------------------------------------------- */
static void W5500_Select(void)
{
    HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_RESET);
}

static void W5500_Unselect(void)
{
    HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_SET);
}

/* -------------------------------------------------------------------------- */
/* Common register access (BSB = 0)                                           */
/* -------------------------------------------------------------------------- */
HAL_StatusTypeDef W5500_ReadRegister(uint16_t address,
                                     uint8_t *data,
                                     uint8_t *rx)
{
    uint8_t tx[4] = {0};
    HAL_StatusTypeDef status;

    if ((data == NULL) || (rx == NULL))
        return HAL_ERROR;

    tx[0] = (uint8_t)(address >> 8);
    tx[1] = (uint8_t)(address & 0xFFU);
    tx[2] = 0x00U; /* Common register block, read, variable length */
    tx[3] = 0x00U;

    memset(rx, 0, 4U);

    W5500_Select();
    status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4U,
                                     W5500_SPI_TIMEOUT);
    W5500_Unselect();

    if (status != HAL_OK)
        return status;

    *data = rx[3];
    return HAL_OK;
}

HAL_StatusTypeDef W5500_WriteRegister(uint16_t address,
                                      uint8_t data)
{
    uint8_t tx[4];
    HAL_StatusTypeDef status;

    tx[0] = (uint8_t)(address >> 8);
    tx[1] = (uint8_t)(address & 0xFFU);
    tx[2] = 0x04U; /* Common register block, write, variable length */
    tx[3] = data;

    W5500_Select();
    status = HAL_SPI_Transmit(&hspi1, tx, 4U, W5500_SPI_TIMEOUT);
    W5500_Unselect();

    return status;
}

HAL_StatusTypeDef W5500_ReadVersion(uint8_t *version, uint8_t *rx)
{
    return W5500_ReadRegister(W5500_VERSIONR, version, rx);
}

/* -------------------------------------------------------------------------- */
/* Network configuration                                                     */
/* -------------------------------------------------------------------------- */
static const uint8_t mac_address[6] = {
    W5500_MAC_0, W5500_MAC_1, W5500_MAC_2,
    W5500_MAC_3, W5500_MAC_4, W5500_MAC_5
};

static const uint8_t gateway_address[4] = {
    W5500_GATEWAY_0, W5500_GATEWAY_1,
    W5500_GATEWAY_2, W5500_GATEWAY_3
};

static const uint8_t subnet_mask[4] = {
    W5500_SUBNET_0, W5500_SUBNET_1,
    W5500_SUBNET_2, W5500_SUBNET_3
};

static const uint8_t ip_address[4] = {
    W5500_IP_0, W5500_IP_1, W5500_IP_2, W5500_IP_3
};

static HAL_StatusTypeDef W5500_WriteArray(uint16_t address,
                                          const uint8_t *data,
                                          uint8_t length,
                                          uint16_t *failed_address)
{
    uint8_t i;
    HAL_StatusTypeDef status;

    for (i = 0U; i < length; i++) {
        status = W5500_WriteRegister((uint16_t)(address + i), data[i]);
        if (status != HAL_OK) {
            if (failed_address != NULL)
                *failed_address = (uint16_t)(address + i);
            return status;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef W5500_VerifyArray(uint16_t address,
                                           const uint8_t *expected,
                                           uint8_t length,
                                           uint16_t *failed_address)
{
    uint8_t i;
    uint8_t data;
    uint8_t rx[4];
    HAL_StatusTypeDef status;

    for (i = 0U; i < length; i++) {
        status = W5500_ReadRegister((uint16_t)(address + i), &data, rx);
        if ((status != HAL_OK) || (data != expected[i])) {
            if (failed_address != NULL)
                *failed_address = (uint16_t)(address + i);
            return (status == HAL_OK) ? HAL_ERROR : status;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef W5500_ConfigureNetwork(uint16_t *failed_address)
{
    HAL_StatusTypeDef status;

    if (failed_address != NULL)
        *failed_address = 0xFFFFU;

#define W5500_CONFIG_STEP(expr)          \
    do {                                 \
        status = (expr);                 \
        if (status != HAL_OK)            \
            return status;               \
    } while (0)

    /* Write network parameters */
    W5500_CONFIG_STEP(W5500_WriteArray(W5500_GAR, gateway_address, 4U,
                                       failed_address));
    W5500_CONFIG_STEP(W5500_WriteArray(W5500_SUBR, subnet_mask, 4U,
                                       failed_address));
    W5500_CONFIG_STEP(W5500_WriteArray(W5500_SHAR, mac_address, 6U,
                                       failed_address));
    W5500_CONFIG_STEP(W5500_WriteArray(W5500_SIPR, ip_address, 4U,
                                       failed_address));

    /* Read back and verify network parameters */
    W5500_CONFIG_STEP(W5500_VerifyArray(W5500_GAR, gateway_address, 4U,
                                        failed_address));
    W5500_CONFIG_STEP(W5500_VerifyArray(W5500_SUBR, subnet_mask, 4U,
                                        failed_address));
    W5500_CONFIG_STEP(W5500_VerifyArray(W5500_SHAR, mac_address, 6U,
                                        failed_address));
    W5500_CONFIG_STEP(W5500_VerifyArray(W5500_SIPR, ip_address, 4U,
                                        failed_address));

#undef W5500_CONFIG_STEP
    return HAL_OK;
}

/* -------------------------------------------------------------------------- */
/* Socket 0 register definitions                                               */
/* -------------------------------------------------------------------------- */
#define S0_MR                       0x0000U
#define S0_CR                       0x0001U
#define S0_IR                       0x0002U
#define S0_SR                       0x0003U
#define S0_PORT                     0x0004U
#define S0_DIPR                     0x000CU
#define S0_DPORT                    0x0010U
#define S0_TX_FSR                   0x0020U
#define S0_TX_WR                    0x0024U
#define S0_RX_RSR                   0x0026U
#define S0_RX_RD                    0x0028U

#define SOCK_CLOSED                 0x00U
#define SOCK_INIT                   0x13U
#define SOCK_ESTABLISHED            0x17U
#define SOCK_CLOSE_WAIT             0x1CU

#define CMD_OPEN                    0x01U
#define CMD_CONNECT                 0x04U
#define CMD_CLOSE                   0x10U
#define CMD_SEND                    0x20U
#define CMD_RECV                    0x40U

#define IR_TIMEOUT                  0x08U
#define IR_SEND_OK                  0x10U

#define SOCKET_BUFFER_SIZE          2048U
#define SOCKET_BUFFER_MASK          0x07FFU
#define SOCKET_LOCAL_PORT           50000U

/* -------------------------------------------------------------------------- */
/* Socket SPI transfer                                                        */
/* BSB 1 = Socket 0 registers, 2 = TX buffer, 3 = RX buffer                   */
/* -------------------------------------------------------------------------- */
static HAL_StatusTypeDef W5500_Socket_Transfer(uint16_t address,
                                               uint8_t block,
                                               uint8_t write,
                                               uint8_t *data,
                                               uint16_t length)
{
    uint8_t header[3];
    HAL_StatusTypeDef status;

    if ((data == NULL) || (length == 0U))
        return HAL_ERROR;

    header[0] = (uint8_t)(address >> 8);
    header[1] = (uint8_t)(address & 0xFFU);
    header[2] = (uint8_t)((block << 3) | (write ? 0x04U : 0x00U));

    W5500_Select();
    status = HAL_SPI_Transmit(&hspi1, header, 3U, W5500_SPI_TIMEOUT);

    if (status == HAL_OK) {
        if (write != 0U)
            status = HAL_SPI_Transmit(&hspi1, data, length,
                                      W5500_SPI_TIMEOUT);
        else
            status = HAL_SPI_Receive(&hspi1, data, length,
                                     W5500_SPI_TIMEOUT);
    }

    W5500_Unselect();
    return status;
}

static HAL_StatusTypeDef W5500_Socket_Read(uint16_t address,
                                           uint8_t *data,
                                           uint16_t length)
{
    return W5500_Socket_Transfer(address, 1U, 0U, data, length);
}

static HAL_StatusTypeDef W5500_Socket_Write(uint16_t address,
                                            const uint8_t *data,
                                            uint16_t length)
{
    return W5500_Socket_Transfer(address, 1U, 1U,
                                 (uint8_t *)data, length);
}

static HAL_StatusTypeDef W5500_Socket_Read8(uint16_t address, uint8_t *value)
{
    if (value == NULL)
        return HAL_ERROR;
    return W5500_Socket_Read(address, value, 1U);
}

static HAL_StatusTypeDef W5500_Socket_Write8(uint16_t address, uint8_t value)
{
    return W5500_Socket_Write(address, &value, 1U);
}

static HAL_StatusTypeDef W5500_Socket_Read16(uint16_t address,
                                             uint16_t *value)
{
    uint8_t data[2];
    HAL_StatusTypeDef status;

    if (value == NULL)
        return HAL_ERROR;

    status = W5500_Socket_Read(address, data, 2U);
    if (status != HAL_OK)
        return status;

    *value = ((uint16_t)data[0] << 8) | data[1];
    return HAL_OK;
}

static HAL_StatusTypeDef W5500_Socket_Write16(uint16_t address,
                                              uint16_t value)
{
    uint8_t data[2];

    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
    return W5500_Socket_Write(address, data, 2U);
}

static HAL_StatusTypeDef W5500_Socket_Command(uint8_t command)
{
    uint8_t value;
    uint32_t start;
    HAL_StatusTypeDef status;

    status = W5500_Socket_Write8(S0_CR, command);
    if (status != HAL_OK)
        return status;

    start = HAL_GetTick();
    do {
        status = W5500_Socket_Read8(S0_CR, &value);
        if (status != HAL_OK)
            return status;
        if (value == 0U)
            return HAL_OK;
    } while ((HAL_GetTick() - start) < 100U);

    return HAL_TIMEOUT;
}

uint8_t W5500_Socket0_GetStatus(void)
{
    uint8_t status = 0U;
    (void)W5500_Socket_Read8(S0_SR, &status);
    return status;
}

void W5500_Socket0_Close(void)
{
    (void)W5500_Socket_Command(CMD_CLOSE);
    HAL_Delay(2U);
}

HAL_StatusTypeDef W5500_Socket0_Connect(const uint8_t server_ip[4],
                                        uint16_t server_port)
{
    uint8_t status = 0U;
    uint8_t interrupt;
    uint32_t start;
    HAL_StatusTypeDef result;

    if (server_ip == NULL)
        return HAL_ERROR;

    W5500_Socket0_Close();

    result = W5500_Socket_Write8(S0_IR, 0xFFU);
    if (result != HAL_OK) return result;

    /* TCP mode */
    result = W5500_Socket_Write8(S0_MR, 0x01U);
    if (result != HAL_OK) return result;

    result = W5500_Socket_Write16(S0_PORT, SOCKET_LOCAL_PORT);
    if (result != HAL_OK) return result;

    result = W5500_Socket_Command(CMD_OPEN);
    if (result != HAL_OK) return result;

    start = HAL_GetTick();
    do {
        result = W5500_Socket_Read8(S0_SR, &status);
        if (result != HAL_OK) return result;
        if (status == SOCK_INIT) break;
    } while ((HAL_GetTick() - start) < 1000U);

    if (status != SOCK_INIT)
        return HAL_TIMEOUT;

    result = W5500_Socket_Write(S0_DIPR, server_ip, 4U);
    if (result != HAL_OK) return result;

    result = W5500_Socket_Write16(S0_DPORT, server_port);
    if (result != HAL_OK) return result;

    result = W5500_Socket_Write8(S0_IR, 0xFFU);
    if (result != HAL_OK) return result;

    result = W5500_Socket_Command(CMD_CONNECT);
    if (result != HAL_OK) return result;

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 5000U) {
        result = W5500_Socket_Read8(S0_SR, &status);
        if (result != HAL_OK) return result;

        if (status == SOCK_ESTABLISHED)
            return HAL_OK;

        if ((status == SOCK_CLOSED) || (status == SOCK_CLOSE_WAIT))
            return HAL_ERROR;

        result = W5500_Socket_Read8(S0_IR, &interrupt);
        if (result != HAL_OK) return result;

        if ((interrupt & IR_TIMEOUT) != 0U) {
            (void)W5500_Socket_Write8(S0_IR, IR_TIMEOUT);
            W5500_Socket0_Close();
            return HAL_TIMEOUT;
        }
    }

    W5500_Socket0_Close();
    return HAL_TIMEOUT;
}

HAL_StatusTypeDef W5500_Socket0_Send(const uint8_t *data, uint16_t length)
{
    uint16_t free_size = 0U;
    uint16_t tx_write;
    uint16_t offset;
    uint16_t first_length;
    uint8_t status;
    uint8_t interrupt;
    uint32_t start;
    HAL_StatusTypeDef result;

    if ((data == NULL) || (length == 0U) ||
        (length > SOCKET_BUFFER_SIZE))
        return HAL_ERROR;

    result = W5500_Socket_Read8(S0_SR, &status);
    if (result != HAL_OK) return result;
    if (status != SOCK_ESTABLISHED) return HAL_ERROR;

    start = HAL_GetTick();
    do {
        result = W5500_Socket_Read16(S0_TX_FSR, &free_size);
        if (result != HAL_OK) return result;
        if (free_size >= length) break;
    } while ((HAL_GetTick() - start) < 2000U);

    if (free_size < length)
        return HAL_TIMEOUT;

    result = W5500_Socket_Read16(S0_TX_WR, &tx_write);
    if (result != HAL_OK) return result;

    offset = tx_write & SOCKET_BUFFER_MASK;
    first_length = (uint16_t)(SOCKET_BUFFER_SIZE - offset);
    if (first_length > length) first_length = length;

    result = W5500_Socket_Transfer(offset, 2U, 1U,
                                   (uint8_t *)data, first_length);
    if (result != HAL_OK) return result;

    if (length > first_length) {
        result = W5500_Socket_Transfer(0U, 2U, 1U,
                                       (uint8_t *)&data[first_length],
                                       (uint16_t)(length - first_length));
        if (result != HAL_OK) return result;
    }

    tx_write = (uint16_t)(tx_write + length);
    result = W5500_Socket_Write16(S0_TX_WR, tx_write);
    if (result != HAL_OK) return result;

    result = W5500_Socket_Write8(S0_IR, IR_SEND_OK | IR_TIMEOUT);
    if (result != HAL_OK) return result;

    result = W5500_Socket_Command(CMD_SEND);
    if (result != HAL_OK) return result;

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 3000U) {
        result = W5500_Socket_Read8(S0_IR, &interrupt);
        if (result != HAL_OK) return result;

        if ((interrupt & IR_SEND_OK) != 0U) {
            (void)W5500_Socket_Write8(S0_IR, IR_SEND_OK);
            return HAL_OK;
        }

        if ((interrupt & IR_TIMEOUT) != 0U) {
            (void)W5500_Socket_Write8(S0_IR, IR_TIMEOUT);
            return HAL_TIMEOUT;
        }
    }

    return HAL_TIMEOUT;
}

HAL_StatusTypeDef W5500_Socket0_Receive(uint8_t *data,
                                        uint16_t max_length,
                                        uint16_t *received_length,
                                        uint32_t timeout)
{
    uint16_t received_size = 0U;
    uint16_t rx_read;
    uint16_t read_length;
    uint16_t offset;
    uint16_t first_length;
    uint8_t status;
    uint32_t start;
    HAL_StatusTypeDef result;

    if ((data == NULL) || (received_length == NULL) ||
        (max_length == 0U))
        return HAL_ERROR;

    *received_length = 0U;
    start = HAL_GetTick();

    /* Wait for data. A zero timeout performs one immediate status check. */
    do {
        result = W5500_Socket_Read8(S0_SR, &status);
        if (result != HAL_OK) return result;

        if ((status != SOCK_ESTABLISHED) &&
            (status != SOCK_CLOSE_WAIT))
            return HAL_ERROR;

        result = W5500_Socket_Read16(S0_RX_RSR, &received_size);
        if (result != HAL_OK) return result;

        if (received_size > 0U) break;
    } while ((HAL_GetTick() - start) < timeout);

    if (received_size == 0U)
        return HAL_TIMEOUT;

    read_length = (received_size > max_length) ? max_length : received_size;

    result = W5500_Socket_Read16(S0_RX_RD, &rx_read);
    if (result != HAL_OK) return result;

    offset = rx_read & SOCKET_BUFFER_MASK;
    first_length = (uint16_t)(SOCKET_BUFFER_SIZE - offset);
    if (first_length > read_length) first_length = read_length;

    result = W5500_Socket_Transfer(offset, 3U, 0U, data, first_length);
    if (result != HAL_OK) return result;

    if (read_length > first_length) {
        result = W5500_Socket_Transfer(0U, 3U, 0U,
                                       &data[first_length],
                                       (uint16_t)(read_length - first_length));
        if (result != HAL_OK) return result;
    }

    rx_read = (uint16_t)(rx_read + read_length);
    result = W5500_Socket_Write16(S0_RX_RD, rx_read);
    if (result != HAL_OK) return result;

    result = W5500_Socket_Command(CMD_RECV);
    if (result != HAL_OK) return result;

    *received_length = read_length;
    return HAL_OK;
}
