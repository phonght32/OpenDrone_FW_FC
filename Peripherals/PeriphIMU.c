#include "Periph.h"
#include "OpenDrone_FC_Define.h"
#include "OpenDrone_FC_HwIntf.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "imu_madgwick.h"

#define MAX_IMU_SAMPLES  	10

typedef struct {
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
} imu_data_t;

imu_data_t imu_data_sum = {0};
uint8_t num_accel_samples = 0;
uint8_t num_gyro_samples = 0;
uint8_t num_mag_samples = 0;

#ifdef USE_MPU6050
mpu6050_handle_t mpu6050_handle;
#endif

#ifdef USE_HMC5883L
hmc5883l_handle_t hmc5883l_handle;
#endif

#ifdef USE_IMU_MADGWICK_6DOF
imu_madgwick_handle_t imu_madgwick_handle = NULL;
#endif

err_code_t PeriphIMU_Init(void)
{
#ifdef USE_MPU6050
	mpu6050_handle = mpu6050_init();
	mpu6050_cfg_t mpu6050_cfg = {
		.clksel 		= CONFIG_MPU6050_CLKSEL,
		.dlpf_cfg 		= CONFIG_MPU6050_DLPF,
		.sleep_mode 	= CONFIG_MPU6050_SLEEP_MODE,
		.gfs_sel 		= CONFIG_MPU6050_GFS,
		.afs_sel 		= CONFIG_MPU6050_AFS,
		.accel_bias_x 	= 0,
		.accel_bias_y 	= 0,
		.accel_bias_z 	= 0,
		.gyro_bias_x 	= 0,
		.gyro_bias_y 	= 0,
		.gyro_bias_z 	= 0,
		.i2c_send 		= hw_intf_mpu6050_i2c_send,
		.i2c_recv 		= hw_intf_mpu6050_i2c_recv,
		.delay 			= HAL_Delay
	};
	mpu6050_set_config(mpu6050_handle, mpu6050_cfg);
	mpu6050_config(mpu6050_handle);
	mpu6050_auto_calib(mpu6050_handle);
#endif

#ifdef USE_HMC5883L
	hmc5883l_handle = hmc5883l_init();
	hmc5883l_cfg_t hmc5883l_cfg = {
		.range 		= CONFIG_HMC5883L_RANGE,
		.opr_mode 	= CONFIG_HMC5883L_OPR_MODE,
		.data_rate 	= CONFIG_HMC5883L_DATA_RATE,
		.samples 	= CONFIG_HMC5883L_SAMPLES,
		.mag_bias_x = 0,
		.mag_bias_y = 0,
		.mag_bias_z = 0,
		.i2c_send 	= hw_intf_hmc5883l_i2c_send,
		.i2c_recv 	= hw_intf_hmc5883l_i2c_recv,
		.delay 		= HAL_Delay
	};
	hmc5883l_set_config(hmc5883l_handle, hmc5883l_cfg);
	hmc5883l_config(hmc5883l_handle);
#endif

#ifdef USE_IMU_MADGWICK_6DOF
	imu_madgwick_handle = imu_madgwick_init();
	imu_madgwick_cfg_t imu_madgwick_cfg = {
		.beta 		 = CONFIG_IMU_MADGWICK_BETA,
		.sample_freq = CONFIG_IMU_MADGWICK_SAMPLE_FREQ
	};
	imu_madgwick_set_config(imu_madgwick_handle, imu_madgwick_cfg);
	imu_madgwick_config(imu_madgwick_handle);
#endif

	return ERR_CODE_SUCCESS;
}

err_code_t PeriphIMU_UpdateAccel(void)
{
	err_code_t err_ret;
	float accel_x, accel_y, accel_z;

#ifdef USE_MPU6050
	err_ret = mpu6050_get_accel_scale(mpu6050_handle, &accel_x, &accel_y, &accel_z);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}
#endif

	imu_data_sum.accel_x += accel_x;
	imu_data_sum.accel_y += accel_y;
	imu_data_sum.accel_z += accel_z;

	num_accel_samples += 1;

	return ERR_CODE_SUCCESS;
}

err_code_t PeriphIMU_UpdateGyro(void)
{
	err_code_t err_ret;
	float gyro_x, gyro_y, gyro_z;

#ifdef USE_MPU6050
	err_ret = mpu6050_get_gyro_scale(mpu6050_handle, &gyro_x, &gyro_y, &gyro_z);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}
#endif

	imu_data_sum.gyro_x += gyro_x;
	imu_data_sum.gyro_y += gyro_y;
	imu_data_sum.gyro_z += gyro_z;

	num_gyro_samples += 1;

	return ERR_CODE_SUCCESS;
}

err_code_t PeriphIMU_UpdateMag(void)
{
	return ERR_CODE_SUCCESS;
}

err_code_t PeriphIMU_UpdateFilter(void)
{
	err_code_t err_ret;
	float accel_x = 0, accel_y = 0, accel_z = 0, gyro_x = 0, gyro_y = 0, gyro_z = 0;

	if (num_accel_samples != 0)
	{
		accel_x = imu_data_sum.accel_x / num_accel_samples;
		accel_y = imu_data_sum.accel_y / num_accel_samples;
		accel_z = imu_data_sum.accel_z / num_accel_samples;

		num_accel_samples = 0;
		imu_data_sum.accel_x = 0;
		imu_data_sum.accel_y = 0;
		imu_data_sum.accel_z = 0;
	}

	if (num_gyro_samples != 0)
	{
		gyro_x = imu_data_sum.gyro_x / num_gyro_samples;
		gyro_y = imu_data_sum.gyro_y / num_gyro_samples;
		gyro_z = imu_data_sum.gyro_z / num_gyro_samples;

		num_gyro_samples = 0;
		imu_data_sum.gyro_x = 0;
		imu_data_sum.gyro_y = 0;
		imu_data_sum.gyro_z = 0;
	}

#ifdef USE_IMU_MADGWICK_6DOF
	err_ret = imu_madgwick_update_6dof(imu_madgwick_handle, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}
#endif

	return ERR_CODE_SUCCESS;
}

err_code_t PeriphIMU_GetAngel(float *roll, float *pitch, float *yaw)
{
#ifdef USE_IMU_MADGWICK_6DOF
	err_code_t err_ret;
	float q0, q1, q2, q3;

	err_ret = imu_madgwick_get_quaternion(imu_madgwick_handle, &q0, &q1, &q2, &q3);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	*roll = 180.0 / 3.14 * atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
	*pitch = 180.0 / 3.14 * asin(2 * (q0 * q2 - q3 * q1));
	*yaw = 180.0 / 3.14 * atan2f(q0 * q3 + q1 * q2, 0.5f - q2 * q2 - q3 * q3);
#endif

	return ERR_CODE_SUCCESS;
}
