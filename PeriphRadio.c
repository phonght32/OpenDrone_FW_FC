#include "string.h"
#include "Periph.h"
#include "OpenDrone_FC_Define.h"
#include "OpenDrone_FC_HwIntf.h"
#include "OpenDrone_TxProtocol.h"
#include "nrf24l01.h"
#include "sx1278.h"


#ifdef USE_NRF24L01
nrf24l01_handle_t nrf24l01_handle;
#endif

#ifdef USE_SX1278
sx1278_handle_t sx1278_handle;
#endif

err_code_t PeriphRadio_Init(void)
{
#ifdef USE_NRF24L01
	nrf24l01_handle = nrf24l01_init();
	nrf24l01_cfg_t nrf24l01_cfg = {
		.channel 			= CONFIG_NRF24L01_CHANNEL,
		.packet_len 		= CONFIG_NRF24L01_PAYLOAD_LEN,
		.crc_len 			= CONFIG_NRF24L01_CRC_LEN,
		.addr_width 		= CONFIG_NRF24L01_ADDR_WIDTH,
		.retrans_cnt 		= CONFIG_NRF24L01_RETRANS_CNT,
		.retrans_delay 		= CONFIG_NRF24L01_RETRANS_DELAY,
		.data_rate 			= CONFIG_NRF24L01_DATA_RATE,
		.output_pwr 		= CONFIG_NRF24L01_OUTPUT_PWR,
		.transceiver_mode 	= CONFIG_NRF24L01_TRANSCEIVER_MODE,
		.spi_send 			= hw_intf_nrf24l01_spi_send,
		.spi_recv 			= hw_intf_nrf24l01_spi_recv,
		.set_cs 			= hw_intf_nrf24l01_set_cs,
		.set_ce 			= hw_intf_nrf24l01_set_ce,
		.get_irq 			= hw_intf_nrf24l01_get_irq,
		.delay 				= HAL_Delay
	};
	nrf24l01_set_config(nrf24l01_handle, nrf24l01_cfg);
	nrf24l01_config(nrf24l01_handle);
#endif

#ifdef USE_SX1278
	sx1278_handle = sx1278_init();
	sx1278_cfg_t sx1278_cfg = {
		.freq 				= CONFIG_SX1278_FREQ,
		.packet_len 		= CONFIG_SX1278_PACKET_LEN,
		.output_pwr 		= CONFIG_SX1278_OUTPUT_PWR,
		.spread_factor 		= CONFIG_SX1278_SPREAD_FACTOR,
		.bandwidth 			= CONFIG_SX1278_BANDWIDTH,
		.coding_rate 		= CONFIG_SX1278_CODING_RATE,
		.crc_en 			= CONFIG_SX1278_CRC_EN,
		.transceiver_mode 	= CONFIG_SX1278_TRANSCEIVER_MODE,
		.spi_send 			= hw_intf_sx1278_spi_send,
		.spi_recv 			= hw_intf_sx1278_spi_recv,
		.set_cs 			= hw_intf_sx1278_set_cs,
		.set_rst 			= hw_intf_sx1278_set_rst,
		.get_irq 			= hw_intf_sx1278_get_irq,
		.delay 				= HAL_Delay
	};
	sx1278_set_config(sx1278_handle, sx1278_cfg);
	sx1278_config(sx1278_handle);
#endif

	return ERR_CODE_SUCCESS;
}

err_code_t PeriphRadio_Receive(uint8_t *data)
{
#ifdef USE_NRF24L01
	uint8_t irq_level;
	OpenDrone_TxProtocol_Msg_t OpenDrone_TxProtocol_Msg;

	hw_intf_nrf24l01_get_irq(&irq_level);

	if (irq_level == NRF24L01_IRQ_ACTIVE_LEVEL)
	{
		nrf24l01_receive(nrf24l01_handle, (uint8_t *)&OpenDrone_TxProtocol_Msg);

		if (OpenDrone_TxProtocol_Msg.MsgId != OPENDRONE_TXPROTOCOL_MSG_ID_INVALID)
		{
			memcpy(data, (uint8_t *)&OpenDrone_TxProtocol_Msg, CONFIG_NRF24L01_PAYLOAD_LEN);
			nrf24l01_clear_rx_dr(nrf24l01_handle);
		}
	}
#endif

#ifdef USE_SX1278
	sx1278_lora_receive(sx1278_handle, data);
#endif

	return ERR_CODE_SUCCESS;
}
