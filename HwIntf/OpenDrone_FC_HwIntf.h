// MIT License

// Copyright (c) 2024 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __HW_INTF_H__
#define __HW_INTF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "err_code.h"
#include "bmp280.h"
#include "icm42688.h"
#include "qmc5883l.h"
#include "nrf24l01.h"
#include "esc_dshot.h"
#include "OpenDrone_FC_Define.h"

uint32_t hw_intf_get_time_us(void);
void hw_intf_delay_ms(uint32_t time_ms);

#ifdef USE_SERIAL_DEBUG
err_code_t hw_intf_uart_debug_send(uint8_t *log_buf, uint16_t len);
#endif

#ifdef USE_NRF24L01
nrf24l01_status_t hw_intf_nrf24l01_spi_send(uint8_t *buf_send, uint16_t len);
nrf24l01_status_t hw_intf_nrf24l01_spi_recv(uint8_t *buf_recv, uint16_t len);
nrf24l01_status_t hw_intf_nrf24l01_set_cs(uint8_t level);
nrf24l01_status_t hw_intf_nrf24l01_set_ce(uint8_t level);
nrf24l01_status_t hw_intf_nrf24l01_get_irq(uint8_t *level);
#endif

#ifdef USE_SX1278
err_code_t hw_intf_sx1278_spi_send(uint8_t *buf_send, uint16_t len);
err_code_t hw_intf_sx1278_spi_recv(uint8_t *buf_recv, uint16_t len);
err_code_t hw_intf_sx1278_set_cs(uint8_t level);
err_code_t hw_intf_sx1278_set_rst(uint8_t level);
err_code_t hw_intf_sx1278_get_irq(uint8_t *level);
#endif

#ifdef USE_MPU6050
err_code_t hw_intf_mpu6050_i2c_send(uint8_t reg_addr, uint8_t *buf, uint16_t len);
err_code_t hw_intf_mpu6050_i2c_recv(uint8_t reg_addr, uint8_t *buf, uint16_t len);
#endif

#ifdef USE_ICM42688
icm42688_status_t hw_intf_icm42688_spi_send(uint8_t *buf_send, uint16_t len);
icm42688_status_t hw_intf_icm42688_spi_recv(uint8_t *buf_recv, uint16_t len);
icm42688_status_t hw_intf_icm42688_set_cs(uint8_t level);
#endif

#ifdef USE_HMC5883L
err_code_t hw_intf_hmc5883l_i2c_send(uint8_t reg_addr, uint8_t *buf, uint16_t len);
err_code_t hw_intf_hmc5883l_i2c_recv(uint8_t reg_addr, uint8_t *buf, uint16_t len);
#endif

#ifdef USE_QMC5883L
qmc5883l_status_t hw_intf_qmc5883l_i2c_send(uint8_t reg_addr, uint8_t *buf, uint16_t len);
qmc5883l_status_t hw_intf_qmc5883l_i2c_recv(uint8_t reg_addr, uint8_t *buf, uint16_t len);
#endif

#ifdef USE_BMP280
bmp280_status_t hw_intf_bmp280_i2c_send(uint8_t reg_addr, uint8_t *buf, uint16_t len);
bmp280_status_t hw_intf_bmp280_i2c_recv(uint8_t reg_addr, uint8_t *buf, uint16_t len);
#endif

#ifdef USE_ESC_DSHOT
esc_dshot_status_t hw_intf_fl_esc_dshot_send_dma(uint32_t *packet_dma);
esc_dshot_status_t hw_intf_fr_esc_dshot_send_dma(uint32_t *packet_dma);
esc_dshot_status_t hw_intf_bl_esc_dshot_send_dma(uint32_t *packet_dma);
esc_dshot_status_t hw_intf_br_esc_dshot_send_dma(uint32_t *packet_dma);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __HW_INTF_H__ */
