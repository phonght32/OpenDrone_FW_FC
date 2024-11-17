#include "stdio.h"

#include "OpenDrone_FC.h"
#include "OpenDrone_FC_Define.h"
#include "OpenDrone_FC_HwIntf.h"
#include "OpenDrone_TxProtocol.h"
#include "Periph.h"


#define PRINT_ANGLE

#define FREQ_HZ_TO_TIME_US(x)       (1000000.0f/(x))
#define TIME_US_TO_FREQ_HZ(x)       (1000000.0f/(x))

#define IDX_TASK_250_HZ             0
#define IDX_TASK_100_HZ             1
#define IDX_TASK_50_HZ              2
#define IDX_TASK_5_HZ               3
#define NUM_OF_TASK                 4

#define FREQ_250_HZ_TIME_US         FREQ_HZ_TO_TIME_US(250)
#define FREQ_100_HZ_TIME_US         FREQ_HZ_TO_TIME_US(100)
#define FREQ_50_HZ_TIME_US          FREQ_HZ_TO_TIME_US(50)
#define FREQ_5_HZ_TIME_US           FREQ_HZ_TO_TIME_US(5)

#ifdef USE_SERIAL_DEBUG
uint8_t log_buf[50];
uint16_t task_freq[NUM_OF_TASK];
#endif

uint32_t last_time_us[NUM_OF_TASK] = {0};
OpenDrone_TxProtocol_Msg_t OpenDrone_TxProtocol_Msg = {0};

err_code_t OpenDrone_FC_Init(void)
{
	PeriphIMU_Init();
#ifdef USE_SERIAL_DEBUG
	sprintf((char *)log_buf, "\r\nInit peripheral IMU complete");
	hw_intf_uart_debug_send(log_buf, 30);
#endif

	// PeriphRadio_Init();
#ifdef USE_SERIAL_DEBUG
	sprintf((char *)log_buf, "\r\nInit peripheral RADIO complete");
	hw_intf_uart_debug_send(log_buf, 32);
#endif

	PeriphEsc_Init();
#ifdef USE_SERIAL_DEBUG
	sprintf((char *)log_buf, "\r\nInit peripheral ESC complete");
	hw_intf_uart_debug_send(log_buf, 30);
#endif

	PeriphController_Init();
#ifdef USE_SERIAL_DEBUG
	sprintf((char *)log_buf, "\r\nInit peripheral CONTROLLER complete");
	hw_intf_uart_debug_send(log_buf, 37);
#endif

	return ERR_CODE_SUCCESS;
}

err_code_t OpenDrone_FC_Main(void)
{
	uint32_t current_time = hw_intf_get_time_us();

	/* Task 250 Hz */
	if ((current_time - last_time_us[IDX_TASK_250_HZ]) >= FREQ_250_HZ_TIME_US)
	{
		PeriphIMU_UpdateAccel();
		PeriphIMU_UpdateGyro();
		PeriphIMU_UpdateFilter();

#ifdef USE_SERIAL_DEBUG
		task_freq[IDX_TASK_250_HZ] = TIME_US_TO_FREQ_HZ(current_time - last_time_us[IDX_TASK_250_HZ]);
#endif

		last_time_us[IDX_TASK_250_HZ] = current_time;
	}

	/* Task 100 Hz */
	if ((current_time - last_time_us[IDX_TASK_100_HZ]) >= FREQ_100_HZ_TIME_US)
	{
		last_time_us[IDX_TASK_100_HZ] = current_time;
	}

	/* Task 50 Hz */
	if ((current_time - last_time_us[IDX_TASK_50_HZ]) >= FREQ_50_HZ_TIME_US)
	{
		// PeriphRadio_Receive((uint8_t *)&OpenDrone_TxProtocol_Msg);

#ifdef USE_SERIAL_DEBUG
		task_freq[IDX_TASK_50_HZ] = TIME_US_TO_FREQ_HZ(current_time - last_time_us[IDX_TASK_50_HZ]);
#endif

		last_time_us[IDX_TASK_50_HZ] = current_time;
	}

	/* Task 5 Hz */
	if ((current_time - last_time_us[IDX_TASK_5_HZ]) >= FREQ_5_HZ_TIME_US)
	{
#ifdef USE_SERIAL_DEBUG

#ifdef PRINT_ANGLE
		float debug_roll, debug_pitch, debug_yaw;
		PeriphIMU_GetAngel(&debug_roll, &debug_pitch, &debug_yaw);

		task_freq[IDX_TASK_5_HZ] = TIME_US_TO_FREQ_HZ(current_time - last_time_us[IDX_TASK_5_HZ]);

		sprintf((char *)log_buf, "\r\nROLL: %7.4f\t\tPITCH: %7.4f\t\tYAW: %7.4f", debug_roll, debug_pitch, debug_yaw);
		hw_intf_uart_debug_send(log_buf, 50);

		// sprintf((char *)log_buf, "\r\nTask 200 Hz actual frequency: %d Hz", task_freq[IDX_TASK_250_HZ]);
		// hw_intf_uart_debug_send(log_buf, 45);

		// sprintf((char *)log_buf, "\r\nTask 50 Hz actual frequency: %d Hz", task_freq[IDX_TASK_50_HZ]);
		// hw_intf_uart_debug_send(log_buf, 45);

		// sprintf((char *)log_buf, "\r\nTask 5 Hz actual frequency: %d Hz", task_freq[IDX_TASK_5_HZ]);
		// hw_intf_uart_debug_send(log_buf, 45);
#endif

		// sprintf((char *)log_buf, "\r\nthrottle: %03d \troll: %03d \tpitch: %03d \tyaw: %03d",
		//         OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.throttle,
		// 		OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.roll,
		// 		OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.pitch,
		// 		OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.yaw);
		// hw_intf_uart_debug_send(log_buf, 50);
#endif
		last_time_us[IDX_TASK_5_HZ] = current_time;
	}

	return ERR_CODE_SUCCESS;
}
