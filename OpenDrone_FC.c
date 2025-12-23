#include "stdio.h"
#include "string.h"

#include "OpenDrone_FC.h"
#include "OpenDrone_FC_Define.h"
#include "OpenDrone_FC_HwIntf.h"
#include "OpenDrone_TxProtocol.h"
#include "Periph.h"
#include "OpenDrone_IMU.h"
#include "pid_controller.h"


#define IDX_TASK_HIGH             	0
#define IDX_TASK_MEDIUM             1
#define IDX_TASK_LOW              	2
#define IDX_TASK_DEBUG            	3
#define NUM_OF_TASK                 4

#define DURATION_TASK_HIGH          4000U    // 4 ms -> ~250 Hz control loop
#define DURATION_TASK_MEDIUM        20000U   // 20 ms
#define DURATION_TASK_LOW           50000U   // 50 ms
#define DURATION_TASK_DEBUG         200000U  // 200 ms

#define LOOP_DT                     0.004f

#define RADIO_TIMEOUT_US            500000U  // 500 ms -> failsafe

// RC -> desired mapping
// Adjust these scales depending on your RC value ranges
#define RC_ROLL_SCALE_DEG           10.0f   // stick full => +/- 10 degrees
#define RC_PITCH_SCALE_DEG          10.0f   // stick full => +/- 10 degrees
#define RC_YAW_SCALE_DPS            10.0f   // stick full => +/- 10 deg/s
#define RATE_TO_MOTOR_SCALE         0.60f   // tune: 0.2..0.6 typical

// Motor output range (example): 1000..2000 use floats 0..1000, adapt to your ESC interface
#define MOTOR_MIN                   0.0f
#define MOTOR_MAX                   1000.0f

#ifdef USE_SERIAL_DEBUG
uint8_t log_buf[128];
uint32_t cyclic_task_ms[NUM_OF_TASK];
#endif

#ifdef CONFIG_CONTROLLER_PID
pid_controller_handle_t pid_angle_roll;
pid_controller_handle_t pid_angle_pitch;
pid_controller_handle_t pid_rate_roll;
pid_controller_handle_t pid_rate_pitch;
pid_controller_handle_t pid_rate_yaw;
#endif


static uint32_t last_rx_time_us = 0;    // last time radio packet received
static uint8_t is_armed = 1;            // simple arming flag, handle with care in your system

float roll_angle, pitch_angle, yaw_angle;
float roll_rate, pitch_rate, yaw_rate;

float roll_angle_out, pitch_angle_out;
float roll_rate_out, pitch_rate_out, yaw_rate_out;

uint32_t last_time_us[NUM_OF_TASK] = {0};
OpenDrone_TxProtocolMsg_t OpenDrone_TxProtocolMsg = {0};


static void OpenDrone_FC_PrintInfo(void);

static float clampf(float v, float a, float b) {
    if (v < a) return a;
    if (v > b) return b;
    return v;
}

static uint16_t map_motor_to_dshot(float val)
{
    float scaled = 48.0f + (clampf(val, MOTOR_MIN, MOTOR_MAX) / MOTOR_MAX) * (2047.0f - 48.0f);
    return (uint16_t)scaled;
}

static void motor_mixer_quad_x(float throttle, float roll_cmd, float pitch_cmd, float yaw_cmd, float motor_out[4])
{
    // roll_cmd/pitch_cmd/yaw_cmd are contributions (positive means right/forward/yaw-right depending on your convention)
    // Basic mixer (additive)
    // M1: front-left   (+pitch, -roll, -yaw)
    // M2: front-right  (+pitch, +roll, +yaw)
    // M3: rear-right   (-pitch, +roll, -yaw)
    // M4: rear-left    (-pitch, -roll, +yaw)

    motor_out[0] = throttle + pitch_cmd - roll_cmd - yaw_cmd; // M1
    motor_out[1] = throttle + pitch_cmd + roll_cmd + yaw_cmd; // M2
    motor_out[2] = throttle - pitch_cmd + roll_cmd - yaw_cmd; // M3
    motor_out[3] = throttle - pitch_cmd - roll_cmd + yaw_cmd; // M4

    // clamp
    for (int i = 0; i < 4; i++) {
        motor_out[i] = clampf(motor_out[i], MOTOR_MIN, MOTOR_MAX);
    }
}

err_code_t OpenDrone_Controller_Init(void)
{
    // Outer angle PID config (produces desired rate in deg/s typically)
    pid_controller_cfg_t cfg_angle = {
        .kp = 12.0f,
        .ki = 0.02f,
        .kd = 5.0f,
        .tau = 0.02f,
        .lim_min = -200.0f,
        .lim_max = 200.0f,
        .int_lim_min = -100.0f,
        .int_lim_max = 100.0f,
        .sample_time = LOOP_DT
    };

    pid_angle_roll  = pid_controller_init();
    pid_angle_pitch = pid_controller_init();
    pid_controller_set_config(pid_angle_roll,  cfg_angle);
    pid_controller_set_config(pid_angle_pitch, cfg_angle);

    // Inner rate PID config (works on angular rates)
    pid_controller_cfg_t cfg_rate = {
        .kp = 0.8f,
        .ki = 0.01f,
        .kd = 0.001f,
        .tau = 0.02f,
        .lim_min = -1000.0f,
        .lim_max = 1000.0f,
        .int_lim_min = -500.0f,
        .int_lim_max = 500.0f,
        .sample_time = LOOP_DT
    };

    pid_rate_roll  = pid_controller_init();
    pid_rate_pitch = pid_controller_init();
    pid_rate_yaw   = pid_controller_init();
    pid_controller_set_config(pid_rate_roll,  cfg_rate);
    pid_controller_set_config(pid_rate_pitch, cfg_rate);
    pid_controller_set_config(pid_rate_yaw,   cfg_rate);

    return ERR_CODE_SUCCESS;
}

err_code_t OpenDrone_Controller_Update(void)
{
    // Read raw RC
    int16_t rc_roll_raw     = OpenDrone_TxProtocolMsg.Payload.StabilizerCtrl.roll;
    int16_t rc_pitch_raw    = OpenDrone_TxProtocolMsg.Payload.StabilizerCtrl.pitch;
    int16_t rc_yaw_raw      = OpenDrone_TxProtocolMsg.Payload.StabilizerCtrl.yaw;
    int16_t rc_throttle_raw = OpenDrone_TxProtocolMsg.Payload.StabilizerCtrl.throttle;

    // Normalize RC from 0..1000
    float rc_norm_roll  = ((float)rc_roll_raw  - 500.0f) / 500.0f;      // From 0..1000 to -1..+1
    float rc_norm_pitch = ((float)rc_pitch_raw - 500.0f) / 500.0f;      // From 0..1000 to -1..+1
    float rc_norm_yaw   = ((float)rc_yaw_raw   - 500.0f) / 500.0f;      // From 0..1000 to -1..+1
    float rc_norm_thr   = ((float)rc_throttle_raw) / 1000.0f;           // From 0..1000 to 0..+1

    rc_norm_roll  = clampf(rc_norm_roll  , -1.0f , 1.0f);
    rc_norm_pitch = clampf(rc_norm_pitch , -1.0f , 1.0f);
    rc_norm_yaw   = clampf(rc_norm_yaw   , -1.0f , 1.0f);
    rc_norm_thr   = clampf(rc_norm_thr   ,  0.0f , 1.0f);

    // Map to angle & rates (Stabilize)
    float desired_roll_deg  = rc_norm_roll  * RC_ROLL_SCALE_DEG;
    float desired_pitch_deg = rc_norm_pitch * RC_PITCH_SCALE_DEG;
    float desired_yaw_rate  = rc_norm_yaw   * RC_YAW_SCALE_DPS;
    float throttle_out      = rc_norm_thr   * MOTOR_MAX;

    /* Read angle in deg */
    OpenDrone_IMU_GetAngel(&roll_angle, &pitch_angle, &yaw_angle);

    /* Read gyro in deg/s */
    OpenDrone_IMU_GetGyro(&roll_rate, &pitch_rate, &yaw_rate);

    /* Outer loop: angle -> desired rate */
    pid_controller_update(pid_angle_roll,  desired_roll_deg,  roll_angle,  &roll_angle_out);
    pid_controller_update(pid_angle_pitch, desired_pitch_deg, pitch_angle, &pitch_angle_out);

    /* Inner loop: rate -> torque (these outputs are contributions to mixer) */
    pid_controller_update(pid_rate_roll,  roll_angle_out,  roll_rate,  &roll_rate_out);
    pid_controller_update(pid_rate_pitch, pitch_angle_out, pitch_rate, &pitch_rate_out);
    pid_controller_update(pid_rate_yaw,   desired_yaw_rate, yaw_rate,  &yaw_rate_out);

    /* Scale rate outputs to motor domain to avoid saturate */
    float roll_contrib  = roll_rate_out  * RATE_TO_MOTOR_SCALE;
    float pitch_contrib = pitch_rate_out * RATE_TO_MOTOR_SCALE;
    float yaw_contrib   = yaw_rate_out   * RATE_TO_MOTOR_SCALE;

    /* Mixer */
    float motors[4];
    motor_mixer_quad_x(throttle_out, roll_contrib, pitch_contrib, yaw_contrib, motors);

    /* Convert motor outputs (0..1000) to DShot range (48..2047) */
    uint16_t dshot_motors[4];
    for (int i = 0; i < 4; i++) {
        dshot_motors[i] = map_motor_to_dshot(motors[i]);
    }

    /* Only send if armed */
    if (is_armed) 
    {
        PeriphEsc_Send(dshot_motors[0], dshot_motors[1], dshot_motors[2], dshot_motors[3]);

        // sprintf((char *)log_buf, "%d,%d,%d,%d\n", dshot_motors[0], dshot_motors[1], dshot_motors[2], dshot_motors[3]);
        // hw_intf_uart_debug_send(log_buf, strlen((char*)log_buf));
    } 
    else 
    {
        /* Disarmed: send idle (48) to keep ESC alive but motors stopped */
        PeriphEsc_Send(48, 48, 48, 48);
    }

    return ERR_CODE_SUCCESS;
}

err_code_t OpenDrone_FC_Init(void)
{
	OpenDrone_IMU_Init();
	PeriphRadio_Init();
	PeriphEsc_Init();

	OpenDrone_Controller_Init();

	return ERR_CODE_SUCCESS;
}

err_code_t OpenDrone_FC_Main(void)
{
	uint32_t current_time = hw_intf_get_time_us();

	/* Task high */
	if ((current_time - last_time_us[IDX_TASK_HIGH]) >= DURATION_TASK_HIGH)
	{
        // uint16_t throttle = 48 + OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.throttle;
        // PeriphEsc_Send(48, throttle, 48, 48);

		OpenDrone_IMU_UpdateAccel();
		OpenDrone_IMU_UpdateGyro();
		OpenDrone_IMU_UpdateFilter();

        int rx_ret = PeriphRadio_Receive((uint8_t *)&OpenDrone_TxProtocolMsg);
        if (rx_ret > 0) 
        {
            last_rx_time_us = hw_intf_get_time_us();
        }

		OpenDrone_Controller_Update();

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
		OpenDrone_IMU_UpdateMag();
		OpenDrone_IMU_UpdateBaro();
		OpenDrone_IMU_UpdateFilterHeight();

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

	return ERR_CODE_SUCCESS;
}

static void OpenDrone_FC_PrintInfo(void)
{
	/* Send debug 9-DoF */
    // float debug_accel_x, debug_accel_y, debug_accel_z, debug_gyro_x, debug_gyro_y, debug_gyro_z, debug_mag_x, debug_mag_y, debug_mag_z;
	// OpenDrone_IMU_GetAccel(&debug_accel_x, &debug_accel_y, &debug_accel_z);
	// OpenDrone_IMU_GetGyro(&debug_gyro_x, &debug_gyro_y, &debug_gyro_z);
	// OpenDrone_IMU_GetMag(&debug_mag_x, &debug_mag_y, &debug_mag_z);

	// sprintf((char *)log_buf, "\n%f,%f,%f,%f,%f,%f,%f,%f,%f",
	//         debug_accel_x, debug_accel_y, debug_accel_z,
	//         debug_gyro_x, debug_gyro_y, debug_gyro_z,
	//         debug_mag_x, debug_mag_y, debug_mag_z);
	// hw_intf_uart_debug_send(log_buf, strlen(log_buf));

	/* Send debug angle */
    // OpenDrone_IMU_GetAngel(&roll_angle, &pitch_angle, &yaw_angle);
    // sprintf((char *)log_buf, "%f,%f,%f\n", roll_angle, pitch_angle, yaw_angle);
    // hw_intf_uart_debug_send(log_buf, strlen((char*)log_buf));

	/* Send debug altitude */
    // float debug_altitude, debug_baro;
	// OpenDrone_IMU_GetBaro(&debug_baro);
	// OpenDrone_IMU_GetAltitude(&debug_altitude);
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
	// sprintf((char *)log_buf, "\nthrottle: %03d \troll: %03d \tpitch: %03d \tyaw: %03d",
	//         OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.throttle,
	//         OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.roll,
	//         OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.pitch,
	//         OpenDrone_TxProtocol_Msg.Payload.StabilizerCtrl.yaw);
	// hw_intf_uart_debug_send(log_buf, (uint16_t)strlen((char*)log_buf));
}
