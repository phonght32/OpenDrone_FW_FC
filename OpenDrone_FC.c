#include "stdio.h"
#include "string.h"

#include "OpenDrone_FC.h"
#include "OpenDrone_FC_Define.h"
#include "OpenDrone_FC_HwIntf.h"
#include "OpenDrone_TxProtocol.h"
#include "PeriphController.h"
#include "PeriphESC.h"
#include "PeriphIMU.h"
#include "PeriphRadio.h"

#define IDX_TASK_HIGH               0
#define IDX_TASK_MEDIUM             1
#define IDX_TASK_LOW                2
#define IDX_TASK_DEBUG              3
#define NUM_OF_TASK                 4

#define DURATION_TASK_HIGH          4000U    // 4 ms -> ~250 Hz control loop
#define DURATION_TASK_MEDIUM        20000U // 20 ms
#define DURATION_TASK_LOW           50000U    // 50 ms
#define DURATION_TASK_DEBUG         200000U // 200 ms

#define RADIO_TIMEOUT_US            500000U // 500 ms -> failsafe

#ifdef USE_SERIAL_DEBUG
uint8_t log_buf[128];
uint32_t cyclic_task_ms[NUM_OF_TASK];
#endif

static uint32_t last_rx_time_us = 0; // last time radio packet received
static uint8_t is_armed = 1; // simple arming flag, handle with care in your system
uint16_t output_dshot_motors[4];

uint32_t last_time_us[NUM_OF_TASK] = {0};
OpenDrone_TxProtocolMsg_t OpenDrone_TxProtocolMsg = {0};

static void OpenDrone_FC_PrintInfo(void);

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

    /* Task high */
    if ((current_time - last_time_us[IDX_TASK_HIGH]) >= DURATION_TASK_HIGH) {
        // uint16_t throttle = 48 +
        // OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.throttle;
        // PeriphEsc_Send(48, throttle, 48, 48);

        PeriphIMU_UpdateAccel();
        PeriphIMU_UpdateGyro();
        PeriphIMU_UpdateFilter();

        int rx_ret = PeriphRadio_Receive((uint8_t *)&OpenDrone_TxProtocolMsg);
        if (rx_ret > 0) {
            last_rx_time_us = hw_intf_get_time_us();
        }

        PeriphController_Update(
            OpenDrone_TxProtocolMsg.Payload.StabilizerCtrl.roll,
            OpenDrone_TxProtocolMsg.Payload.StabilizerCtrl.pitch,
            OpenDrone_TxProtocolMsg.Payload.StabilizerCtrl.yaw,
            OpenDrone_TxProtocolMsg.Payload.StabilizerCtrl.throttle);

        PeriphController_GetMotorControl(
            &output_dshot_motors[0], &output_dshot_motors[1],
            &output_dshot_motors[2], &output_dshot_motors[3]);

        /* Only send if armed */
        if (is_armed) {
            PeriphEsc_Send(output_dshot_motors[0], output_dshot_motors[1],
                           output_dshot_motors[2], output_dshot_motors[3]);

            // sprintf((char *)log_buf, "%d,%d,%d,%d\n", dshot_motors[0],
            // dshot_motors[1], dshot_motors[2], dshot_motors[3]);
            // hw_intf_uart_debug_send(log_buf, strlen((char*)log_buf));
        } else {
            /* Disarmed: send idle (48) to keep ESC alive but motors stopped */
            PeriphEsc_Send(48, 48, 48, 48);
        }

        cyclic_task_ms[IDX_TASK_HIGH] = current_time - last_time_us[IDX_TASK_HIGH];
        last_time_us[IDX_TASK_HIGH] = current_time;
    }

    /* Task medium */
    if ((current_time - last_time_us[IDX_TASK_MEDIUM]) >= DURATION_TASK_MEDIUM) {
        cyclic_task_ms[IDX_TASK_MEDIUM] =
            current_time - last_time_us[IDX_TASK_MEDIUM];

        last_time_us[IDX_TASK_MEDIUM] = current_time;
    }

    /* Task low */
    if ((current_time - last_time_us[IDX_TASK_LOW]) >= DURATION_TASK_LOW) {
        PeriphIMU_UpdateMag();
        PeriphIMU_UpdateBaro();
        PeriphIMU_UpdateFilterHeight();

        cyclic_task_ms[IDX_TASK_LOW] = current_time - last_time_us[IDX_TASK_LOW];

        last_time_us[IDX_TASK_LOW] = current_time;
    }

    /* Task debug */
    if ((current_time - last_time_us[IDX_TASK_DEBUG]) >= DURATION_TASK_DEBUG) {
#ifdef USE_SERIAL_DEBUG
        OpenDrone_FC_PrintInfo();
#endif
        cyclic_task_ms[IDX_TASK_DEBUG] =
            current_time - last_time_us[IDX_TASK_DEBUG];

        last_time_us[IDX_TASK_DEBUG] = current_time;
    }

    return ERR_CODE_SUCCESS;
}

static void OpenDrone_FC_PrintInfo(void) 
{
    /* Send debug 9-DoF */
    // float debug_accel_x, debug_accel_y, debug_accel_z, debug_gyro_x,
    // debug_gyro_y, debug_gyro_z, debug_mag_x, debug_mag_y, debug_mag_z;
    // PeriphIMU_GetAccel(&debug_accel_x, &debug_accel_y, &debug_accel_z);
    // PeriphIMU_GetGyro(&debug_gyro_x, &debug_gyro_y, &debug_gyro_z);
    // PeriphIMU_GetMag(&debug_mag_x, &debug_mag_y, &debug_mag_z);

    // sprintf((char *)log_buf, "\n%f,%f,%f,%f,%f,%f,%f,%f,%f",
    //         debug_accel_x, debug_accel_y, debug_accel_z,
    //         debug_gyro_x, debug_gyro_y, debug_gyro_z,
    //         debug_mag_x, debug_mag_y, debug_mag_z);
    // hw_intf_uart_debug_send(log_buf, strlen(log_buf));

    /* Send debug angle */
    // PeriphIMU_GetAngel(&roll_angle, &pitch_angle, &yaw_angle);
    // sprintf((char *)log_buf, "%f,%f,%f\n", roll_angle, pitch_angle, yaw_angle);
    // hw_intf_uart_debug_send(log_buf, strlen((char*)log_buf));

    /* Send debug altitude */
    // float debug_altitude, debug_baro;
    // PeriphIMU_GetBaro(&debug_baro);
    // PeriphIMU_GetAltitude(&debug_altitude);
    // sprintf((char *)log_buf, "%f,%f\n", debug_baro, debug_altitude * 100);
    // hw_intf_uart_debug_send(log_buf, strlen((char*)log_buf));

    /* Send debug frequency */
    // sprintf((char *)log_buf, "Task cyclic: %d us, %d us, %d us, %d us\n",
    //         cyclic_task_ms[IDX_TASK_HIGH],
    //         cyclic_task_ms[IDX_TASK_MEDIUM],
    //         cyclic_task_ms[IDX_TASK_LOW],
    //         cyclic_task_ms[IDX_TASK_DEBUG]);
    // hw_intf_uart_debug_send(log_buf, strlen((char*)log_buf));

    /* Send debug command */
    // sprintf((char *)log_buf, "\nthrottle: %03d \troll: %03d \tpitch: %03d
    // \tyaw: %03d",
    //         OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.throttle,
    //         OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.roll,
    //         OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.pitch,
    //         OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.yaw);
    // hw_intf_uart_debug_send(log_buf, (uint16_t)strlen((char*)log_buf));
}
