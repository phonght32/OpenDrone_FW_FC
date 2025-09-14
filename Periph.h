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

#ifndef __PERIPH_H__
#define __PERIPH_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "err_code.h"

err_code_t PeriphIMU_Init(void);
err_code_t PeriphIMU_UpdateAccel(void);
err_code_t PeriphIMU_UpdateGyro(void);
err_code_t PeriphIMU_UpdateMag(void);
err_code_t PeriphIMU_UpdateBaro(void);
err_code_t PeriphIMU_UpdateFilter(void);
err_code_t PeriphIMU_UpdateFilterHeight(void);
err_code_t PeriphIMU_GetAccel(float *accel_x, float *accel_y, float *accel_z);
err_code_t PeriphIMU_GetGyro(float *gyro_x, float *gyro_y, float *gyro_z);
err_code_t PeriphIMU_GetMag(float *mag_x, float *mag_y, float *mag_z);
err_code_t PeriphIMU_GetBaro(float *baro);
err_code_t PeriphIMU_GetAngel(float *roll, float *pitch, float *yaw);
err_code_t PeriphIMU_GetAltitude(float *altitude);

err_code_t PeriphController_Init(void);

err_code_t PeriphEsc_Init(void);
err_code_t PeriphEsc_PreparePacket(uint16_t fl_throttle, uint16_t fr_throttle, uint16_t bl_throttle, uint16_t br_throttle);
err_code_t PeriphEsc_Send(void);

err_code_t PeriphRadio_Init(void);
err_code_t PeriphRadio_Receive(uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __PERIPH_H__ */
