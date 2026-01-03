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

#ifndef __PERIPH_IMU_H__
#define __PERIPH_IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

void PeriphIMU_Init(void);
void PeriphIMU_Calibrate(void);
void PeriphIMU_UpdateAccel(void);
void PeriphIMU_UpdateGyro(void);
void PeriphIMU_UpdateMag(void);
void PeriphIMU_UpdateBaro(void);
void PeriphIMU_UpdateFilter(void);
void PeriphIMU_UpdateFilterHeight(void);
void PeriphIMU_GetAccel(float *accel_x, float *accel_y, float *accel_z);
void PeriphIMU_GetGyro(float *gyro_x, float *gyro_y, float *gyro_z);
void PeriphIMU_GetMag(float *mag_x, float *mag_y, float *mag_z);
void PeriphIMU_GetBaro(float *baro);
void PeriphIMU_GetAngel(float *roll, float *pitch, float *yaw);
void PeriphIMU_GetAltitude(float *altitude);

#ifdef __cplusplus
}
#endif

#endif /* __PERIPH_IMU_H__ */
