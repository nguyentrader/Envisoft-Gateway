/*
 * w5500.h
 *
 *  Created on: 12 Nov 2026
 *      Author: LE TRAN NGUYEN
 */

#ifndef W5500_H
#define W5500_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* W5500 chip select: STM32F401RCT6 PB0 */
#define W5500_CS_PORT              GPIOB
#define W5500_CS_PIN               GPIO_PIN_0

/* Common register addresses */
#define W5500_VERSIONR             0x0039
#define W5500_PHYCFGR              0x002E
#define W5500_EXPECTED_VERSION     0x04

/* MAC address */
#define W5500_MAC_0                0x02
#define W5500_MAC_1                0x08
#define W5500_MAC_2                0xDC
#define W5500_MAC_3                0x01
#define W5500_MAC_4                0x02
#define W5500_MAC_5                0x03

/* Network configuration */
#define W5500_IP_0                 192
#define W5500_IP_1                 168
#define W5500_IP_2                 10
#define W5500_IP_3                 2

#define W5500_SUBNET_0             255
#define W5500_SUBNET_1             255
#define W5500_SUBNET_2             255
#define W5500_SUBNET_3             0

#define W5500_GATEWAY_0            0
#define W5500_GATEWAY_1            0
#define W5500_GATEWAY_2            0
#define W5500_GATEWAY_3            0

#define W5500_DNS_0                0
#define W5500_DNS_1                0
#define W5500_DNS_2                0
#define W5500_DNS_3                0

/* W5500 common register addresses */
#define W5500_GAR                  0x0001
#define W5500_SUBR                 0x0005
#define W5500_SHAR                 0x0009
#define W5500_SIPR                 0x000F

/* Socket 0 */
#define W5500_SOCKET0              0
#define W5500_SOCKET_OK            0
#define W5500_SOCKET_ERROR         1
#define W5500_SOCKET_TIMEOUT       2
#define W5500_SOCKET_CLOSED        3

/* Common register access */
HAL_StatusTypeDef W5500_ReadRegister(uint16_t address,
                                     uint8_t *data,
                                     uint8_t *rx);

HAL_StatusTypeDef W5500_WriteRegister(uint16_t address,
                                      uint8_t data);

HAL_StatusTypeDef W5500_ReadVersion(uint8_t *version,
                                    uint8_t *rx);

/* Network configuration */
HAL_StatusTypeDef W5500_ConfigureNetwork(uint16_t *failed_address);

/* Socket 0 TCP functions */
HAL_StatusTypeDef W5500_Socket0_Connect(const uint8_t server_ip[4],
                                        uint16_t server_port);

HAL_StatusTypeDef W5500_Socket0_Send(const uint8_t *data,
                                     uint16_t length);

HAL_StatusTypeDef W5500_Socket0_Receive(uint8_t *data,
                                        uint16_t max_length,
                                        uint16_t *received_length,
                                        uint32_t timeout);

void W5500_Socket0_Close(void);
uint8_t W5500_Socket0_GetStatus(void);

#endif /* W5500_H */

