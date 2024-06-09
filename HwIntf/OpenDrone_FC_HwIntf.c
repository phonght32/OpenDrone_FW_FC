#include "OpenDrone_FC_HwIntf.h"

__weak uint32_t hw_intf_get_time_us(void)
{
	return 0;
}

__weak void hw_intf_delay_ms(uint32_t time_ms)
{

}

#ifdef USE_SERIAL_DEBUG
__weak err_code_t hw_intf_uart_debug_send(uint8_t *log_buf, uint16_t len)
{
	return ERR_CODE_SUCCESS;
}
#endif

#ifdef USE_NRF24L01
__weak err_code_t hw_intf_nrf24l01_spi_send(uint8_t *buf_send, uint16_t len)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_nrf24l01_spi_recv(uint8_t *buf_recv, uint16_t len)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_nrf24l01_set_cs(uint8_t level)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_nrf24l01_set_ce(uint8_t level)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_nrf24l01_get_irq(uint8_t *level)
{
	return ERR_CODE_SUCCESS;
}
#endif

#ifdef USE_SX1278
__weak err_code_t hw_intf_sx1278_spi_send(uint8_t *buf_send, uint16_t len)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_sx1278_spi_recv(uint8_t *buf_recv, uint16_t len)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_sx1278_set_cs(uint8_t level)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_sx1278_set_rst(uint8_t level)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_sx1278_get_irq(uint8_t *level)
{
	return ERR_CODE_SUCCESS;
}
#endif

#ifdef USE_MPU6050
__weak err_code_t hw_intf_mpu6050_i2c_send(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_mpu6050_i2c_recv(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	return ERR_CODE_SUCCESS;
}
#endif

#ifdef USE_HMC5883L
__weak err_code_t hw_intf_hmc5883l_i2c_send(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_hmc5883l_i2c_recv(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	return ERR_CODE_SUCCESS;
}
#endif

#ifdef USE_ESC_DSHOT
__weak err_code_t hw_intf_fl_esc_dshot_set_auto_reload(uint32_t auto_reload)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_fr_esc_dshot_set_auto_reload(uint32_t auto_reload)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_bl_esc_dshot_set_auto_reload(uint32_t auto_reload)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_br_esc_dshot_set_auto_reload(uint32_t auto_reload)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_fl_esc_dshot_send_dma(uint32_t *packet_dma)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_fr_esc_dshot_send_dma(uint32_t *packet_dma)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_bl_esc_dshot_send_dma(uint32_t *packet_dma)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_br_esc_dshot_send_dma(uint32_t *packet_dma)
{
	return ERR_CODE_SUCCESS;
}

__weak err_code_t hw_intf_esc_dshot_start(void)
{
	return ERR_CODE_SUCCESS;
}
#endif
