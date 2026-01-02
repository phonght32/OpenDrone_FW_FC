#include "PeriphESC.h"
#include "OpenDrone_FC_Config.h"
#include "OpenDrone_FC_HwIntf.h"
#include "esc_dshot.h"

#ifdef USE_ESC_DSHOT
esc_dshot_handle_t fl_esc_dshot_handle;
esc_dshot_handle_t fr_esc_dshot_handle;
esc_dshot_handle_t bl_esc_dshot_handle;
esc_dshot_handle_t br_esc_dshot_handle;

uint32_t fl_esc_dshot_dmabuffer[DSHOT_DMA_BUFFER];
uint32_t fr_esc_dshot_dmabuffer[DSHOT_DMA_BUFFER];
uint32_t bl_esc_dshot_dmabuffer[DSHOT_DMA_BUFFER];
uint32_t br_esc_dshot_dmabuffer[DSHOT_DMA_BUFFER];
#endif

void PeriphEsc_Init(void)
{
#ifdef USE_ESC_DSHOT
	fl_esc_dshot_handle = esc_dshot_init();
	fr_esc_dshot_handle = esc_dshot_init();
	bl_esc_dshot_handle = esc_dshot_init();
	br_esc_dshot_handle = esc_dshot_init();

	esc_dshot_cfg_t fl_esc_dshot_cfg = {
		.dshot_type = CONFIG_ESC_DSHOT_TYPE,
		.tick_bit   = 560,
		.send_dma 	= hw_intf_fl_esc_dshot_send_dma
	};

	esc_dshot_cfg_t fr_esc_dshot_cfg = {
		.dshot_type = CONFIG_ESC_DSHOT_TYPE,
		.tick_bit   = 560,
		.send_dma 	= hw_intf_fr_esc_dshot_send_dma
	};

	esc_dshot_cfg_t bl_esc_dshot_cfg = {
		.dshot_type = CONFIG_ESC_DSHOT_TYPE,
		.tick_bit   = 560,
		.send_dma 	= hw_intf_bl_esc_dshot_send_dma
	};

	esc_dshot_cfg_t br_esc_dshot_cfg = {
		.dshot_type = CONFIG_ESC_DSHOT_TYPE,
		.tick_bit   = 560,
		.send_dma 	= hw_intf_br_esc_dshot_send_dma
	};

	esc_dshot_set_config(fl_esc_dshot_handle, fl_esc_dshot_cfg);
	esc_dshot_set_config(fr_esc_dshot_handle, fr_esc_dshot_cfg);
	esc_dshot_set_config(bl_esc_dshot_handle, bl_esc_dshot_cfg);
	esc_dshot_set_config(br_esc_dshot_handle, br_esc_dshot_cfg);

	esc_dshot_config(fl_esc_dshot_handle);
	esc_dshot_config(fr_esc_dshot_handle);
	esc_dshot_config(bl_esc_dshot_handle);
	esc_dshot_config(br_esc_dshot_handle);

#endif
}

void PeriphEsc_Send(uint16_t fl_throttle, uint16_t fr_throttle, uint16_t bl_throttle, uint16_t br_throttle)
{
#ifdef USE_ESC_DSHOT
	esc_dshot_prepare_packet(fl_esc_dshot_handle, fl_throttle, fl_esc_dshot_dmabuffer);
	esc_dshot_prepare_packet(fr_esc_dshot_handle, fr_throttle, fr_esc_dshot_dmabuffer);
	esc_dshot_prepare_packet(bl_esc_dshot_handle, bl_throttle, bl_esc_dshot_dmabuffer);
	esc_dshot_prepare_packet(br_esc_dshot_handle, br_throttle, br_esc_dshot_dmabuffer);

	esc_dshot_send_packet(fl_esc_dshot_handle, fl_esc_dshot_dmabuffer);
	esc_dshot_send_packet(fr_esc_dshot_handle, fr_esc_dshot_dmabuffer);
	esc_dshot_send_packet(bl_esc_dshot_handle, bl_esc_dshot_dmabuffer);
	esc_dshot_send_packet(br_esc_dshot_handle, br_esc_dshot_dmabuffer);
#endif
}
