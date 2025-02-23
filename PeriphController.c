#include "Periph.h"
#include "OpenDrone_FC_Define.h"
#include "pid_controller.h"

#ifdef USE_PID_CONTROLLER
pid_controller_handle_t pid_controller_handle;
#endif

err_code_t PeriphController_Init(void)
{
#ifdef USE_PID_CONTROLLER
	pid_controller_handle = pid_controller_init();
	pid_controller_cfg_t pid_controller_cfg = {
		.kp 			= CONFIG_PID_CONTROLLER_KP,
		.ki 			= CONFIG_PID_CONTROLLER_KI,
		.kd 			= CONFIG_PID_CONTROLLER_KD,
		.tau 			= CONFIG_PID_CONTROLLER_TAU,
		.lim_min 		= CONFIG_PID_CONTROLLER_LIM_MIN,
		.lim_max 		= CONFIG_PID_CONTROLLER_LIM_MAX,
		.int_lim_min 	= CONFIG_PID_CONTROLLER_INT_LIM_MIN,
		.int_lim_max 	= CONFIG_PID_CONTROLLER_INT_LIM_MAX,
		.sample_time 	= CONFIG_PID_CONTROLLER_SAMPLE_TIME
	};
	pid_controller_set_config(pid_controller_handle, pid_controller_cfg);
	pid_controller_config(pid_controller_handle);
#endif

	return ERR_CODE_SUCCESS;
}

