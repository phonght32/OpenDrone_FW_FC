#include "string.h"

#include "Periph.h"
#include "OpenDrone_FC_Define.h"
#include "OpenDrone_FC_HwIntf.h"
#include "mpu6050.h"
#include "icm42688.h"
#include "hmc5883l.h"
#include "qmc5883l.h"
#include "bmp280.h"
#include "imu_madgwick.h"
#include "kalman_height_estimation.h"

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
	float pressure;
} imu_data_t;

imu_data_t imu_data = {0};

#ifdef USE_MPU6050
mpu6050_handle_t mpu6050_handle;
#endif

#ifdef USE_ICM42688
icm42688_handle_t icm42688_handle;
#endif

#ifdef USE_HMC5883L
hmc5883l_handle_t hmc5883l_handle;
#endif

#ifdef USE_QMC5883L
qmc5883l_handle_t qmc5883l_handle;
#endif

#ifdef USE_BMP280
bmp280_handle_t bmp280_handle;
#endif


#ifdef USE_IMU_MADGWICK_6DOF
imu_madgwick_handle_t imu_madgwick_handle = NULL;
#endif

#ifdef USE_IMU_MADGWICK_9DOF
imu_madgwick_handle_t imu_madgwick_handle = NULL;
#endif

#ifdef USE_KALMAN_HEIGHT_ESTIMATION
kalman_height_estimation_handle_t kalman_height_estimation_handle = NULL;
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

#ifdef USE_ICM42688
	icm42688_handle = icm42688_init();
	icm42688_cfg_t icm42688_cfg = {
		.gyro_mode  	= CONFIG_ICM42688_GYRO_MODE,
		.gyro_fs_sel 	= CONFIG_ICM42688_GYRO_FS_SEL,
		.gyro_odr 		= CONFIG_ICM42688_GYRO_ODR,
		.accel_mode  	= CONFIG_ICM42688_ACCEL_MODE,
		.accel_fs_sel 	= CONFIG_ICM42688_ACCEL_FS_SEL,
		.accel_odr 		= CONFIG_ICM42688_ACCEL_ODR,
		.accel_bias_x 	= 19,
		.accel_bias_y 	= 21,
		.accel_bias_z 	= 26,
		.gyro_bias_x 	= 12,
		.gyro_bias_y 	= 0,
		.gyro_bias_z 	= 17,
		.comm_mode 		= CONFIG_ICM42688_COMM_MODE,
		.spi_send 		= hw_intf_icm42688_spi_send,
		.spi_recv 		= hw_intf_icm42688_spi_recv,
		.set_cs  		= hw_intf_icm42688_set_cs,
		.delay 			= HAL_Delay
	};
	icm42688_set_config(icm42688_handle, icm42688_cfg);
	icm42688_config(icm42688_handle);
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
	hmc5883l_auto_calib(hmc5883l_handle);
#endif

#ifdef USE_QMC5883L
	qmc5883l_handle = qmc5883l_init();
	qmc5883l_cfg_t qmc5883l_cfg = {
		.range 			= CONFIG_QMC5883L_RANGE,
		.opr_mode 		= CONFIG_QMC5883L_OPR_MODE,
		.data_rate 		= CONFIG_QMC5883L_DATA_RATE,
		.sample_rate 	= CONFIG_QMC5883L_SAMPLES,
		.intr_en 		= CONFIG_QMC5883L_INTR_ENABLE,
		.hard_bias_x     = 154.7851961238576,
		.hard_bias_y     = -12.923460697083387,
		.hard_bias_z     = -16.002331463439372,
		.soft_bias_c11   = 0.6232115045089357,
		.soft_bias_c12   = 0.004997471269166363,
		.soft_bias_c13   = 0.022147854060797133,
		.soft_bias_c21   = 0.004997471269166429,
		.soft_bias_c22   = 0.6324010352739647,
		.soft_bias_c23   = -0.00080240879779977,
		.soft_bias_c31   = 0.02214785406079713,
		.soft_bias_c32   = -0.000802408797799873,
		.soft_bias_c33   = 0.7179876576597246,
		.i2c_send 		= hw_intf_qmc5883l_i2c_send,
		.i2c_recv 		= hw_intf_qmc5883l_i2c_recv,
		.delay 			= HAL_Delay
	};
	qmc5883l_set_config(qmc5883l_handle, qmc5883l_cfg);
	qmc5883l_config(qmc5883l_handle);
#endif

#ifdef USE_BMP280
	bmp280_cfg_t bmp280_cfg = {
		.opr_mode                   = CONFIG_BMP280_OPR_MODE,
		.filter                     = CONFIG_BMP280_FILTER,
		.over_sampling_pressure     = CONFIG_BMP280_OVER_SAMPLING_PRES,
		.over_sampling_temperature  = CONFIG_BMP280_OVER_SAMPLING_TEMP,
		.over_sampling_humidity     = CONFIG_BMP280_OVER_SAMPLING_HUMD,
		.standby_time               = CONFIG_BMP280_STANDBY_TIME,
		.comm_mode                  = CONFIG_BMP280_COMM_MODE,
		.i2c_send                   = hw_intf_bmp280_i2c_send,
		.i2c_recv                   = hw_intf_bmp280_i2c_recv,
		.delay                      = HAL_Delay,
	};
	bmp280_handle = bmp280_init();
	bmp280_set_config(bmp280_handle, bmp280_cfg);
	bmp280_config(bmp280_handle);
#endif

	imu_madgwick_handle = imu_madgwick_init();
	imu_madgwick_cfg_t imu_madgwick_cfg = {
		.beta 		 = CONFIG_IMU_MADGWICK_BETA,
		.sample_freq = CONFIG_IMU_MADGWICK_SAMPLE_FREQ
	};
	imu_madgwick_set_config(imu_madgwick_handle, imu_madgwick_cfg);
	imu_madgwick_config(imu_madgwick_handle);

#ifdef USE_KALMAN_HEIGHT_ESTIMATION
	kalman_height_estimation_handle = kalman_height_estimation_init();
	kalman_height_estimation_cfg_t kalman_height_estimation_cfg = {
		.dt = 0.02f,
	};
	kalman_height_estimation_set_config(kalman_height_estimation_handle, kalman_height_estimation_cfg);
	kalman_height_estimation_config(kalman_height_estimation_handle);
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

#ifdef USE_ICM42688
	err_ret = icm42688_get_accel_scale(icm42688_handle, &accel_x, &accel_y, &accel_z);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}
#endif

	imu_data.accel_x = accel_x;
	imu_data.accel_y = accel_y;
	imu_data.accel_z = accel_z;

#ifdef USE_SERIAL_DEBUG

#endif

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

#ifdef USE_ICM42688
	err_ret = icm42688_get_gyro_scale(icm42688_handle, &gyro_x, &gyro_y, &gyro_z);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}
#endif

	imu_data.gyro_x = gyro_x;
	imu_data.gyro_y = gyro_y;
	imu_data.gyro_z = gyro_z;

	return ERR_CODE_SUCCESS;
}

err_code_t PeriphIMU_UpdateMag(void)
{
	err_code_t err_ret;
	float mag_x = 0, mag_y = 0, mag_z = 0;

#ifdef USE_QMC5883L
	err_ret = qmc5883l_get_mag_calib(qmc5883l_handle, &mag_x, &mag_y, &mag_z);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}
#endif

	imu_data.mag_x = mag_x;
	imu_data.mag_y = mag_y;
	imu_data.mag_z = mag_z;

	return ERR_CODE_SUCCESS;
}

#ifdef USE_BMP280
err_code_t PeriphIMU_UpdateBaro(void)
{
	err_code_t err_ret;
	float pressure = 0;

#ifdef USE_BMP280
	err_ret = bmp280_get_pressure(bmp280_handle, &pressure);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}
#endif

	imu_data.pressure = pressure;

	return ERR_CODE_SUCCESS;
}
#endif

err_code_t PeriphIMU_UpdateFilter(void)
{
#ifdef USE_IMU_MADGWICK_6DOF
	imu_madgwick_update_6dof(imu_madgwick_handle,
	                         imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
	                         imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
#endif

#ifdef USE_IMU_MADGWICK_9DOF
	imu_madgwick_update_9dof(imu_madgwick_handle,
	                         imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
	                         imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
	                         imu_data.mag_x, imu_data.mag_y, imu_data.mag_z);
#endif

	return ERR_CODE_SUCCESS;
}

err_code_t PeriphIMU_UpdateFilterHeight(void)
{
#ifdef USE_KALMAN_HEIGHT_ESTIMATION
	float altitude = 0.0;
	bmp280_convert_pressure_to_altitude(bmp280_handle, imu_data.pressure, &altitude);

	kalman_height_estimation_update(kalman_height_estimation_handle, imu_data.accel_z*0.99 - 9.99f, altitude);
#endif

	return ERR_CODE_SUCCESS;
}

err_code_t PeriphIMU_GetAccel(float *accel_x, float *accel_y, float *accel_z)
{
	*accel_x = imu_data.accel_x;
	*accel_y = imu_data.accel_y;
	*accel_z = imu_data.accel_z;

	return ERR_CODE_SUCCESS;
}

err_code_t PeriphIMU_GetGyro(float *gyro_x, float *gyro_y, float *gyro_z)
{
	*gyro_x = imu_data.gyro_x;
	*gyro_y = imu_data.gyro_y;
	*gyro_z = imu_data.gyro_z;

	return ERR_CODE_SUCCESS;
}

err_code_t PeriphIMU_GetMag(float *mag_x, float *mag_y, float *mag_z)
{
	*mag_x = imu_data.mag_x;
	*mag_y = imu_data.mag_y;
	*mag_z = imu_data.mag_z;

	return ERR_CODE_SUCCESS;
}

err_code_t PeriphIMU_GetBaro(float *baro)
{
	*baro = imu_data.pressure;

	return ERR_CODE_SUCCESS;
}

err_code_t PeriphIMU_GetAngel(float *roll, float *pitch, float *yaw)
{
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

	return ERR_CODE_SUCCESS;
}

err_code_t PeriphIMU_GetAltitude(float *altitude)
{
#ifdef USE_KALMAN_HEIGHT_ESTIMATION
	kalman_height_estimation_get_height(kalman_height_estimation_handle, altitude);
#endif

	return ERR_CODE_SUCCESS;
}
