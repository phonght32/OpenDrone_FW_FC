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

#ifndef __PERIPH_CONTROLLER_H__
#define __PERIPH_CONTROLLER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "err_code.h"

typedef struct
{
	float rc_angle_roll;
	float rc_angle_pitch;
	float rc_rate_yaw;
	float rc_throttle;
	float measured_angle_roll;
	float measured_angle_pitch;
	float measured_angle_yaw;
	float measured_rate_roll;
	float measured_rate_pitch;
	float measured_rate_yaw;
} stPeriphController_Input_t;

typedef struct
{
	uint16_t dshot_m1;
	uint16_t dshot_m2;
	uint16_t dshot_m3;
	uint16_t dshot_m4;
} stPeriphController_Output_t;

err_code_t PeriphController_Init(void);
err_code_t PeriphController_Update(const stPeriphController_Input_t *aInput, stPeriphController_Output_t *aOutput);

#ifdef __cplusplus
}
#endif

#endif /* __PERIPH_CONTROLLER_H__ */
