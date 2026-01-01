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

#define DURATION_TASK_HIGH          4000U
#define DURATION_TASK_MEDIUM        20000U
#define DURATION_TASK_LOW           50000U
#define DURATION_TASK_DEBUG         200000U

#define RADIO_TIMEOUT_US            1000000U

#ifdef USE_SERIAL_DEBUG
uint8_t log_buf[128];
uint32_t cyclic_task_ms[NUM_OF_TASK];
#endif

typedef enum
{
    OPEN_DRONE_FC_STATE_IDLE,
    OPEN_DRONE_FC_STATE_ARMING,
    OPEN_DRONE_FC_STATE_WATING_FOR_ARM,
    OPEN_DRONE_FC_STATE_RUNNING,
    OPEN_DRONE_FC_STATE_FAILSAFE,
    OPEN_DRONE_FC_STATE_STOPPING
} OpenDrone_FC_State_t;

typedef enum
{
    OPEN_DRONE_FC_EVENT_NONE,
    OPEN_DRONE_FC_EVENT_ARM,
    OPEN_DRONE_FC_EVENT_DISARM,
    OPEN_DRONE_FC_EVENT_RADIO_TIMEOUT
} OpenDrone_FC_Event_t;

OpenDrone_FC_Event_t new_event = OPEN_DRONE_FC_EVENT_NONE;
static OpenDrone_FC_State_t main_state = OPEN_DRONE_FC_STATE_IDLE;
static uint32_t arming_time_us = 5000000u;
static uint32_t arming_start_time = 0u;

static uint32_t current_time = 0;
static uint32_t last_time_us[NUM_OF_TASK] = {0};

static uint32_t last_rx_time_us = 0; // last time radio packet received
static uint8_t is_armed = 0; // simple arming flag, handle with care in your system

static OpenDrone_TxProtocolMsg_t OpenDrone_TxProtocolMsg = {0};
int16_t rc_angle_roll, rc_angle_pitch, rc_rate_yaw, rc_throttle;

static float measured_angle_roll, measured_angle_pitch, measured_angle_yaw;
static float measured_rate_roll, measured_rate_pitch, measured_rate_yaw;

static stPeriphController_Input_t controller_input;
static stPeriphController_Output_t controller_output;

static void OpenDrone_FC_PrintInfo(void);
static void OpenDrone_FC_ParseRadioCommand(void);
static void OpenDrone_FC_StateRunning(void);

void OpenDrone_FC_Init(void)
{
    PeriphIMU_Init();
    PeriphRadio_Init();
    PeriphEsc_Init();
    PeriphController_Init();
}

void OpenDrone_FC_Main(void)
{
    current_time = hw_intf_get_time_us();
    OpenDrone_FC_Status_t rx_ret;

    rx_ret = PeriphRadio_Receive((uint8_t *)&OpenDrone_TxProtocolMsg);
    if (rx_ret == OPENDRONE_FC_STATUS_SUCCESS)
    {
        last_rx_time_us = current_time;
        OpenDrone_FC_ParseRadioCommand();
    }

    if ((current_time - last_rx_time_us) >= RADIO_TIMEOUT_US)
    {
        new_event = OPEN_DRONE_FC_EVENT_RADIO_TIMEOUT;
    }

    switch (main_state)
    {
    case OPEN_DRONE_FC_STATE_IDLE:
        main_state = OPEN_DRONE_FC_STATE_WATING_FOR_ARM;
        break;

    case OPEN_DRONE_FC_STATE_WATING_FOR_ARM:
        if (new_event == OPEN_DRONE_FC_EVENT_ARM)
        {
            new_event = OPEN_DRONE_FC_EVENT_NONE;
            main_state = OPEN_DRONE_FC_STATE_ARMING;
            arming_start_time = current_time;
        }
        break;

    case OPEN_DRONE_FC_STATE_ARMING:
        PeriphEsc_Send(48, 48, 48, 48);
        hw_intf_delay_ms(4);

        if ((current_time - arming_start_time) >= arming_time_us)
        {
            main_state = OPEN_DRONE_FC_STATE_RUNNING;
            is_armed = 1;
        }

        break;

    case OPEN_DRONE_FC_STATE_RUNNING:
        if (new_event == OPEN_DRONE_FC_EVENT_DISARM)
        {
            new_event = OPEN_DRONE_FC_EVENT_NONE;
            main_state = OPEN_DRONE_FC_STATE_WATING_FOR_ARM;
            is_armed = 0;
        }
        else if (new_event == OPEN_DRONE_FC_EVENT_RADIO_TIMEOUT)
        {
            new_event = OPEN_DRONE_FC_EVENT_NONE;
            main_state = OPEN_DRONE_FC_STATE_FAILSAFE;
        }
        else
        {
            OpenDrone_FC_StateRunning();
        }

        break;

    case OPEN_DRONE_FC_STATE_FAILSAFE:
        PeriphEsc_Send(48, 48, 48, 48);
        hw_intf_delay_ms(4);
        break;
        
    default:
        break;
    }


}

static void OpenDrone_FC_StateRunning(void)
{
    /* Task high */
    if ((current_time - last_time_us[IDX_TASK_HIGH]) >= DURATION_TASK_HIGH)
    {
        PeriphIMU_UpdateAccel();
        PeriphIMU_UpdateGyro();
        PeriphIMU_UpdateFilter();

        /* Read angle in deg */
        PeriphIMU_GetAngel(&measured_angle_roll, &measured_angle_pitch, &measured_angle_yaw);

        /* Read gyro in deg/s */
        PeriphIMU_GetGyro(&measured_rate_roll, &measured_rate_pitch, &measured_rate_yaw);

        controller_input.rc_angle_roll          = rc_angle_roll;
        controller_input.rc_angle_pitch         = rc_angle_pitch;
        controller_input.rc_rate_yaw            = rc_rate_yaw;
        controller_input.rc_throttle            = rc_throttle;
        controller_input.measured_angle_roll    = measured_angle_roll;
        controller_input.measured_angle_pitch   = measured_angle_pitch;
        controller_input.measured_angle_yaw     = measured_angle_yaw;
        controller_input.measured_rate_roll     = measured_rate_roll;
        controller_input.measured_rate_pitch    = measured_rate_pitch;
        controller_input.measured_rate_yaw      = measured_rate_yaw;

        PeriphController_Update(&controller_input, &controller_output);

        PeriphEsc_Send(
            48 + controller_output.dshot_m1,
            48 + controller_output.dshot_m2,
            48 + controller_output.dshot_m3,
            48 + controller_output.dshot_m4);

        cyclic_task_ms[IDX_TASK_HIGH] = current_time - last_time_us[IDX_TASK_HIGH];
        last_time_us[IDX_TASK_HIGH] = current_time;
    }

    /* Task medium */
    if ((current_time - last_time_us[IDX_TASK_MEDIUM]) >= DURATION_TASK_MEDIUM)
    {
        cyclic_task_ms[IDX_TASK_MEDIUM] = current_time - last_time_us[IDX_TASK_MEDIUM];

        last_time_us[IDX_TASK_MEDIUM] = current_time;
    }

    /* Task low */
    if ((current_time - last_time_us[IDX_TASK_LOW]) >= DURATION_TASK_LOW)
    {
        PeriphIMU_UpdateMag();
        PeriphIMU_UpdateBaro();
        PeriphIMU_UpdateFilterHeight();

        cyclic_task_ms[IDX_TASK_LOW] = current_time - last_time_us[IDX_TASK_LOW];

        last_time_us[IDX_TASK_LOW] = current_time;
    }

    /* Task debug */
    if ((current_time - last_time_us[IDX_TASK_DEBUG]) >= DURATION_TASK_DEBUG)
    {
#ifdef USE_SERIAL_DEBUG
        OpenDrone_FC_PrintInfo();
#endif
        cyclic_task_ms[IDX_TASK_DEBUG] = current_time - last_time_us[IDX_TASK_DEBUG];

        last_time_us[IDX_TASK_DEBUG] = current_time;
    }
}

static void OpenDrone_FC_ParseRadioCommand(void)
{
    switch (OpenDrone_TxProtocolMsg.MsgId)
    {
    case OPENDRONE_TXPROTOCOLMSG_ID_ARM_DISARM:
        if (OpenDrone_TxProtocolMsg.Payload.ArmDisarm.arm == 1)
        {
            new_event = OPEN_DRONE_FC_EVENT_ARM;
        }
        else
        {
            new_event = OPEN_DRONE_FC_EVENT_DISARM;
        }
        break;
    case OPENDRONE_TXPROTOCOLMSG_ID_STABILIZER_CONTROL:
        rc_angle_roll   = OpenDrone_TxProtocolMsg.Payload.StabilizerCtrl.roll;
        rc_angle_pitch  = OpenDrone_TxProtocolMsg.Payload.StabilizerCtrl.pitch;
        rc_rate_yaw     = OpenDrone_TxProtocolMsg.Payload.StabilizerCtrl.yaw;
        rc_throttle     = OpenDrone_TxProtocolMsg.Payload.StabilizerCtrl.throttle;
        break;
    default:
        break;
    }
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
