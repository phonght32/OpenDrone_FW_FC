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

#ifndef __OPENDRONE_FC_H__
#define __OPENDRONE_FC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "Algorithm/pid_controller/pid_controller.h"
#include "EscProtocol/esc_dshot/esc_dshot.h"
#include "Filter/imu_madgwick/imu_madgwick.h"
#include "OSD/max7456/max7456.h"
#include "Radio/nrf24l01/nrf24l01.h"
#include "Radio/sx1278/sx1278.h"
#include "Sensor/bmp180/bmp180.h"
#include "Sensor/bmp280/bmp280.h"
#include "Sensor/hmc5883l/hmc5883l.h"
#include "Sensor/mpu6050/mpu6050.h"
#include "TxProtocol/OpenDrone_TxProto/OpenDrone_TxProto.h"

#ifdef __cplusplus
}
#endif

#endif /* __OPENDRONE_FC_H__ */