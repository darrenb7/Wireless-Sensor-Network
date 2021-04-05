/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/Power.h>

#include "CC2650_LAUNCHXL.h"

/* These #defines allow us to reuse TI-RTOS across other device families */
#define     Board_LED0              Board_RLED
#define     Board_LED1              Board_GLED
#define     Board_LED2              Board_LED0

#define     Board_BUTTON0           Board_BTN1
#define     Board_BUTTON1           Board_BTN2

#define     Board_I2C0              Board_I2C//added
#define     Board_I2C_TMP           Board_I2C0//added
#define     Board_UART0             Board_UART
#define     Board_AES0              Board_AES
#define     Board_WATCHDOG0         CC2650_LAUNCHXL_WATCHDOG0

#define     Board_ADC0              CC2650_LAUNCHXL_ADCVSS
#define     Board_ADC1              CC2650_LAUNCHXL_ADCVDDS

#define     Board_ADCBuf0           CC2650_LAUNCHXL_ADCBuf0
#define     Board_ADCBufChannel0    (0)
#define     Board_ADCBufChannel1    (1)

#define     Board_initGeneral() { \
    Power_init(); \
    if (PIN_init(BoardGpioInitTable) != PIN_SUCCESS) \
        {System_abort("Error with PIN_init\n"); \
    } \
}

#define     Board_initGPIO()
#define     Board_initPWM()        PWM_init()
#define     Board_initI2C()         I2C_init()//added
#define     Board_initSPI()         SPI_init()
#define     Board_initUART()        UART_init()
#define     Board_initWatchdog()    Watchdog_init()
#define     Board_initADCBuf()      ADCBuf_init()
#define     Board_initADC()         ADC_init()
#define     GPIO_toggle(n)
#define     GPIO_write(n,m)

/* MPU 6050 */
#define Board_MPU6050_ADDR  0x68
#define MPU6050_SELF_TEST_X 0x0D ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Y 0x0E ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Z 0x0F ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_A 0x10 ///< Self test factory calibrated values register
#define MPU6050_SMPLRT_DIV  0x19  ///< sample rate divisor register
#define MPU6050_CONFIG      0x1A      ///< General configuration register
#define MPU6050_GYRO_CONFIG 0x1B ///< Gyro specfic configuration register
#define MPU6050_ACCEL_CONFIG 0x1C ///< Accelerometer specific configration register
#define MPU6050_INT_PIN_CONFIG 0x37    ///< Interrupt pin configuration register
#define MPU6050_WHO_AM_I    0x75          ///< Device ID register
#define MPU6050_SIGNAL_PATH_RESET 0x68 ///< Signal path reset register
#define MPU6050_USER_CTRL   0x6A         ///< FIFO and I2C Master control register
#define MPU6050_PWR_MGMT_1  0x6B        ///< Primary power/sleep control register
#define MPU6050_PWR_MGMT_2  0x6C ///< Secondary power/sleep control register
#define MPU6050_TEMP_H      0x41     ///< Temperature data high byte register
#define MPU6050_TEMP_L      0x42


#define ACCEL_XH_OUT   0x3B
#define ACCEL_XL_OUT   0x3C
#define ACCEL_YH_OUT   0x3D
#define ACCEL_YL_OUT   0x3E
#define ACCEL_ZH_OUT   0x3F
#define ACCEL_ZL_OUT   0x40


#define PWR_MGMT_1_WAKE  0x00
#define PWR_MGMT_1_RESET 0x40

#define GYRO_RANGE_250  0x00
#define GYRO_RANGE_500  0x08
#define GYRO_RANGE_1000 0x10
#define GYRO_RANGE_2000 0x18

#define ACCEL_RANGE_2  0x00
#define ACCEL_RANGE_4  0x08
#define ACCEL_RANGE_8  0x10
#define ACCEL_RANGE_16 0x18


#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
