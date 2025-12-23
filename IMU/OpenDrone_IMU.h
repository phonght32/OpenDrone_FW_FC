// MIT License

// Copyright (c) 2024 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __OPENDRONE_IMU_H__
#define __OPENDRONE_IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "err_code.h"

err_code_t OpenDrone_IMU_Init(void);
err_code_t OpenDrone_IMU_UpdateAccel(void);
err_code_t OpenDrone_IMU_UpdateGyro(void);
err_code_t OpenDrone_IMU_UpdateMag(void);
err_code_t OpenDrone_IMU_UpdateBaro(void);
err_code_t OpenDrone_IMU_UpdateFilter(void);
err_code_t OpenDrone_IMU_UpdateFilterHeight(void);
err_code_t OpenDrone_IMU_GetAccel(float *accel_x, float *accel_y, float *accel_z);
err_code_t OpenDrone_IMU_GetGyro(float *gyro_x, float *gyro_y, float *gyro_z);
err_code_t OpenDrone_IMU_GetMag(float *mag_x, float *mag_y, float *mag_z);
err_code_t OpenDrone_IMU_GetBaro(float *baro);
err_code_t OpenDrone_IMU_GetAngel(float *roll, float *pitch, float *yaw);
err_code_t OpenDrone_IMU_GetAltitude(float *altitude);

#ifdef __cplusplus
}
#endif

#endif /* __OPENDRONE_IMU_H__ */