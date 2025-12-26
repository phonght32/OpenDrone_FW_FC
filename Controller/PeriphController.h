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

err_code_t PeriphController_Init(void);
err_code_t PeriphController_Update(int16_t roll, int16_t pitch, int16_t yaw, int16_t throttle);
err_code_t PeriphController_GetMotorControl(uint16_t *m1, uint16_t *m2, uint16_t *m3,  uint16_t *m4);

#ifdef __cplusplus
}
#endif

#endif /* __PERIPH_CONTROLLER_H__ */
