#include "stdio.h"
#include "string.h"

#include "OpenDrone_FC.h"
#include "OpenDrone_FC_Define.h"
#include "OpenDrone_FC_HwIntf.h"
#include "OpenDrone_TxProtocol.h"
#include "Periph.h"


#define IDX_TASK_1MS             	0
#define IDX_TASK_10MS             	1
#define IDX_TASK_20MS              	2
#define IDX_TASK_200MS            	3
#define NUM_OF_TASK                 4


#define DURATION_1MS 				1000
#define DURATION_10MS 				10000
#define DURATION_20MS 				20000
#define DURATION_200MS 				200000


#ifdef USE_SERIAL_DEBUG
uint8_t log_buf[128];
uint32_t cyclic_task_ms[NUM_OF_TASK];
float debug_roll, debug_pitch, debug_yaw;
float debug_accel_x, debug_accel_y, debug_accel_z, debug_gyro_x, debug_gyro_y, debug_gyro_z, debug_mag_x, debug_mag_y, debug_mag_z;
#endif

uint32_t last_time_us[NUM_OF_TASK] = {0};
OpenDrone_TxProtocol_Msg_t OpenDrone_TxProtocol_Msg = {0};

err_code_t OpenDrone_FC_Init(void)
{
	PeriphIMU_Init();
	PeriphRadio_Init();
	PeriphEsc_Init();
	PeriphController_Init();

	return ERR_CODE_SUCCESS;
}

err_code_t OpenDrone_FC_Main(void)
{
	uint32_t current_time = hw_intf_get_time_us();

	/* Task 1ms */
	if ((current_time - last_time_us[IDX_TASK_1MS]) >= DURATION_1MS)
	{
		PeriphIMU_UpdateAccel();
		PeriphIMU_UpdateGyro();
		PeriphIMU_UpdateFilter();

		cyclic_task_ms[IDX_TASK_1MS] = current_time - last_time_us[IDX_TASK_1MS];

		last_time_us[IDX_TASK_1MS] = current_time;
	}

	/* Task 10ms */
	if ((current_time - last_time_us[IDX_TASK_10MS]) >= DURATION_10MS)
	{
		cyclic_task_ms[IDX_TASK_10MS] = current_time - last_time_us[IDX_TASK_10MS];
		
		last_time_us[IDX_TASK_10MS] = current_time;
	}

	/* Task 20ms */
	if ((current_time - last_time_us[IDX_TASK_20MS]) >= DURATION_20MS)
	{
		PeriphIMU_UpdateMag();
		PeriphRadio_Receive((uint8_t *)&OpenDrone_TxProtocol_Msg);

		uint16_t throttle = 500;
		PeriphEsc_PreparePacket(throttle, throttle, throttle, throttle);
		PeriphEsc_Send();

		cyclic_task_ms[IDX_TASK_20MS] = current_time - last_time_us[IDX_TASK_20MS];

		last_time_us[IDX_TASK_20MS] = current_time;
	}

	/* Task 200ms */
	if ((current_time - last_time_us[IDX_TASK_200MS]) >= DURATION_200MS)
	{
#ifdef USE_SERIAL_DEBUG

		/* Send debug 9-DoF */
		// PeriphIMU_GetAccel(&debug_accel_x, &debug_accel_y, &debug_accel_z);
		// PeriphIMU_GetGyro(&debug_gyro_x, &debug_gyro_y, &debug_gyro_z);
		// PeriphIMU_GetMag(&debug_mag_x, &debug_mag_y, &debug_mag_z);

		// sprintf((char *)log_buf, "\n%f,%f,%f,%f,%f,%f,%f,%f,%f",
		//         debug_accel_x, debug_accel_y, debug_accel_z,
		//         debug_gyro_x, debug_gyro_y, debug_gyro_z,
		//         debug_mag_x, debug_mag_y, debug_mag_z);
		// hw_intf_uart_debug_send(log_buf, strlen(log_buf));


		/* Send debug angle */
		PeriphIMU_GetAngel(&debug_roll, &debug_pitch, &debug_yaw);
		sprintf((char *)log_buf, "%f,%f,%f\n", debug_roll, debug_pitch, debug_yaw);
		hw_intf_uart_debug_send(log_buf, strlen((char*)log_buf));

		/* Send debug frequency */
		// sprintf((char *)log_buf, "Task cyclic: %d us, %d us, %d us, %d us\n",
		//         cyclic_task_ms[IDX_TASK_1MS],
		//         cyclic_task_ms[IDX_TASK_10MS],
		//         cyclic_task_ms[IDX_TASK_20MS],
		//         cyclic_task_ms[IDX_TASK_200MS]);
		// hw_intf_uart_debug_send(log_buf, strlen((char*)log_buf));


		/* Send debug command */
		// sprintf((char *)log_buf, "\nthrottle: %03d \troll: %03d \tpitch: %03d \tyaw: %03d",
		//         OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.throttle,
		//         OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.roll,
		//         OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.pitch,
		//         OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.yaw);
		// hw_intf_uart_debug_send(log_buf, (uint16_t)strlen((char*)log_buf));
#endif

		cyclic_task_ms[IDX_TASK_200MS] = current_time - last_time_us[IDX_TASK_200MS];

		last_time_us[IDX_TASK_200MS] = current_time;
	}

	return ERR_CODE_SUCCESS;
}
