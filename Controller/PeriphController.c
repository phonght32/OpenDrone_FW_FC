#include "pid_controller.h"
#include "OpenDrone_FC_Define.h"
#include "PeriphController.h"

#define LOOP_DT                     0.004f


// RC -> desired mapping
// Adjust these scales depending on your RC value ranges
#define RC_ROLL_SCALE_DEG           10.0f   // stick full => +/- 10 degrees
#define RC_PITCH_SCALE_DEG          10.0f   // stick full => +/- 10 degrees
#define RC_YAW_SCALE_DPS            10.0f   // stick full => +/- 10 deg/s
#define RATE_TO_MOTOR_SCALE         0.60f   // tune: 0.2..0.6 typical

// Motor output range (example): 1000..2000 use floats 0..1000, adapt to your ESC interface
#define MOTOR_MIN                   0.0f
#define MOTOR_MAX                   1000.0f

#ifdef CONFIG_CONTROLLER_PID
pid_controller_handle_t pid_angle_roll;
pid_controller_handle_t pid_angle_pitch;
pid_controller_handle_t pid_rate_roll;
pid_controller_handle_t pid_rate_pitch;
pid_controller_handle_t pid_rate_yaw;
#endif

static float clampf(float v, float a, float b) {
	if (v < a)
		return a;
	if (v > b)
		return b;
	return v;
}

static uint16_t map_motor_to_dshot(float val) {
	float scaled = 48.0f + (clampf(val, MOTOR_MIN, MOTOR_MAX) / MOTOR_MAX) *
	               (2047.0f - 48.0f);
	return (uint16_t)scaled;
}

static void motor_mixer_quad_x(float throttle, float roll_cmd, float pitch_cmd,
                               float yaw_cmd, float motor_out[4]) {
	// roll_cmd/pitch_cmd/yaw_cmd are contributions (positive means
	// right/forward/yaw-right depending on your convention) Basic mixer
	// (additive) M1: front-left   (+pitch, -roll, -yaw) M2: front-right  (+pitch,
	// +roll, +yaw) M3: rear-right   (-pitch, +roll, -yaw) M4: rear-left (-pitch,
	// -roll, +yaw)

	motor_out[0] = throttle + pitch_cmd - roll_cmd - yaw_cmd; // M1
	motor_out[1] = throttle + pitch_cmd + roll_cmd + yaw_cmd; // M2
	motor_out[2] = throttle - pitch_cmd + roll_cmd - yaw_cmd; // M3
	motor_out[3] = throttle - pitch_cmd - roll_cmd + yaw_cmd; // M4

	// clamp
	for (int i = 0; i < 4; i++) {
		motor_out[i] = clampf(motor_out[i], MOTOR_MIN, MOTOR_MAX);
	}
}

void PeriphController_Init(void)
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
}

void PeriphController_Update(const stPeriphController_Input_t *aInput, stPeriphController_Output_t *aOutput)
{
	// Read raw RC
	int16_t rc_roll_raw     = aInput->rc_angle_roll;
	int16_t rc_pitch_raw    = aInput->rc_angle_pitch;
	int16_t rc_yaw_raw      = aInput->rc_rate_yaw;
	int16_t rc_throttle_raw = aInput->rc_throttle;

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

	/* Outer loop: angle -> desired rate */
	float roll_angle_out, pitch_angle_out;
	float roll_rate_out, pitch_rate_out, yaw_rate_out;
	pid_controller_update(pid_angle_roll,  desired_roll_deg, aInput->measured_angle_roll,  &roll_angle_out);
	pid_controller_update(pid_angle_pitch, desired_pitch_deg, aInput->measured_angle_pitch, &pitch_angle_out);

	/* Inner loop: rate -> torque (these outputs are contributions to mixer) */
	pid_controller_update(pid_rate_roll,  roll_angle_out, aInput->measured_rate_roll,  &roll_rate_out);
	pid_controller_update(pid_rate_pitch, pitch_angle_out, aInput->measured_rate_pitch, &pitch_rate_out);
	pid_controller_update(pid_rate_yaw,   desired_yaw_rate, aInput->measured_rate_yaw,  &yaw_rate_out);

	/* Scale rate outputs to motor domain to avoid saturate */
	float roll_contrib  = roll_rate_out  * RATE_TO_MOTOR_SCALE;
	float pitch_contrib = pitch_rate_out * RATE_TO_MOTOR_SCALE;
	float yaw_contrib   = yaw_rate_out   * RATE_TO_MOTOR_SCALE;

	/* Mixer */
	float motors[4];
	motor_mixer_quad_x(throttle_out, roll_contrib, pitch_contrib, yaw_contrib, motors);

	aOutput->dshot_m1 = map_motor_to_dshot(motors[0]);
	aOutput->dshot_m2 = map_motor_to_dshot(motors[1]);
	aOutput->dshot_m3 = map_motor_to_dshot(motors[2]);
	aOutput->dshot_m4 = map_motor_to_dshot(motors[3]);
}

