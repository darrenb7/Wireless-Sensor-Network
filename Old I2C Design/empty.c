/*
 * Copyright (c) 2016, Texas Instruments Incorporated
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

/*
 *    ======== i2ctmp007.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
/* Example/Board Header files */
#include "Board.h"

#define TASKSTACKSIZE       1024

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];
bool transferxDone = false;
bool transferyDone = false;
bool transferzDone = false;
unsigned int c=0;



static void transferCallback(I2C_Handle handle, I2C_Transaction *transac, bool result){
     // Set length bytes
      if (result) {
          if(c==0){
           transferxDone = true;
          c++;}
          if(c==1){
                     transferyDone = true;
                     c++;}
          if(c==2){
                     transferzDone = true;
                     c++;}

       } else {
           // Transaction failed, act accordingly...

       }
   }

uint16_t *getAccelData()
{
        uint16_t        Acceldata[3];
        uint8_t         txBufferXH[1],txBufferXL[1],txBufferYH[1],txBufferYL[1],txBufferZH[1],txBufferZL[1];
        uint8_t         rxBufferXH[1],rxBufferXL[1],rxBufferYH[1],rxBufferYL[1],rxBufferZH[1],rxBufferZL[1];
        I2C_Handle      i2cAccel;
        I2C_Params      AccelParams;
        I2C_Transaction XH,XL,YH,YL,ZH,ZL;

        /* Create I2C for usage */
        I2C_Params_init(&AccelParams);
        AccelParams.bitRate = I2C_400kHz;
        AccelParams.transferMode = I2C_MODE_CALLBACK;
        AccelParams.transferCallbackFxn = transferCallback;
        i2cAccel = I2C_open(Board_I2C_TMP, &AccelParams);
        if (i2cAccel == NULL) {
            System_abort("Error Initializing I2C\n");
        }
        else {
            System_printf("I2C Initialized!\n");
        }

    txBufferXH[0] = ACCEL_XH_OUT;
    XH.slaveAddress = Board_MPU6050_ADDR;
    XH.writeBuf = txBufferXH;
    XH.writeCount = 1;
    XH.readBuf = rxBufferXH;
    XH.readCount = 1;

    txBufferXL[0] = ACCEL_XL_OUT;
    XL.slaveAddress = Board_MPU6050_ADDR;
    XL.writeBuf = txBufferXL;
    XL.writeCount = 1;
    XL.readBuf = rxBufferXL;
    XL.readCount = 1;

    txBufferYH[0] = ACCEL_YH_OUT;
    YH.slaveAddress = Board_MPU6050_ADDR;
    YH.writeBuf = txBufferYH;
    YH.writeCount = 1;
    YH.readBuf = rxBufferYH;
    YH.readCount = 1;

    txBufferYL[0] = ACCEL_YL_OUT;
    YL.slaveAddress = Board_MPU6050_ADDR;
    YL.writeBuf = txBufferYL;
    YL.writeCount = 1;
    YL.readBuf = rxBufferYL;
    YL.readCount = 1;

    txBufferZH[0] = ACCEL_ZH_OUT;
    ZH.slaveAddress = Board_MPU6050_ADDR;
    ZH.writeBuf = txBufferZH;
    ZH.writeCount = 1;
    ZH.readBuf = rxBufferZH;
    ZH.readCount = 1;

    txBufferZL[0] = ACCEL_ZL_OUT;
    ZL.slaveAddress = Board_MPU6050_ADDR;
    ZL.writeBuf = txBufferZL;
    ZL.writeCount = 1;
    ZL.readBuf = rxBufferZL;
    ZL.readCount = 1;

    unsigned int i;
    for( i=0; i<50; i++){

    I2C_transfer(i2cAccel, &XH);
    I2C_transfer(i2cAccel, &XL);

    I2C_transfer(i2cAccel, &YH);
    I2C_transfer(i2cAccel, &YL);

    I2C_transfer(i2cAccel, &ZH);
    I2C_transfer(i2cAccel, &ZL);


    if (I2C_transfer(i2cAccel, &ZL))
    {
                            uint16_t x = (rxBufferXH[0]<< 8 | rxBufferXL[0]);
                            uint16_t y = (rxBufferYH[0]<< 8 | rxBufferYL[0]);
                            uint16_t z = (rxBufferZH[0]<< 8 | rxBufferZL[0]);
                            Acceldata[0] = x;
                            Acceldata[1] = y;
                            Acceldata[2] = z;


                            System_printf("Accel X = %d Y = %d Z = %d \n",x,y,z);
                            System_flush();
                        }
                        else
                        {
                            System_printf("Accel read Fail!\n");
                            System_flush();
                        }
            System_flush();
           Task_sleep(1000000 / Clock_tickPeriod);

        }

      I2C_close(i2cAccel);
      System_printf("I2C closed!\n");

      System_flush();

      return Acceldata;
  }

void setPowerMGMT(uint8_t op)
{
        uint8_t         txBuffer1[2];
        uint8_t         rxBuffer1[1];
        I2C_Handle      i2c1;
        I2C_Params      i2cParams1;
        I2C_Transaction i2cTransaction1;

        /* Create I2C for usage */
        I2C_Params_init(&i2cParams1);
        i2cParams1.bitRate = I2C_400kHz;
        i2c1 = I2C_open(Board_I2C_TMP, &i2cParams1);
        if (i2c1 == NULL) {
            System_abort("Error Initializing I2C\n");
        }
        else {
            System_printf("I2C Initialized!\n");
        }

    txBuffer1[0] = MPU6050_PWR_MGMT_1;
    txBuffer1[1] = op;
    i2cTransaction1.slaveAddress = Board_MPU6050_ADDR;
    i2cTransaction1.writeBuf = txBuffer1;
    i2cTransaction1.writeCount = 2;
    i2cTransaction1.readBuf = rxBuffer1;
    i2cTransaction1.readCount = 1;

    if (I2C_transfer(i2c1, &i2cTransaction1))
    {

                            System_printf("Power Reg set P1: %d  \n",rxBuffer1[0]);
                            System_flush();
                        }
                        else
                        {
                            System_printf("Power Reg set Fail!\n");
                            System_flush();
                        }

      I2C_close(i2c1);
      System_printf("I2C closed!\n");

      System_flush();
  }

void setAccelConfig(uint8_t op)
{
        uint8_t         txBufferA[2];
        uint8_t         rxBufferA[1];
        I2C_Handle      i2cA;
        I2C_Params      i2cParamsA;
        I2C_Transaction i2cTransactionA;

        /* Create I2C for usage */
        I2C_Params_init(&i2cParamsA);
        i2cParamsA.bitRate = I2C_400kHz;
        i2cA = I2C_open(Board_I2C_TMP, &i2cParamsA);
        if (i2cA == NULL) {
            System_abort("Error Initializing I2C\n");
        }
        else {
            System_printf("I2C Initialized!\n");
        }

    txBufferA[0] = MPU6050_GYRO_CONFIG;//MPU6050_ACCEL_CONFIG;
    txBufferA[1] = op;
    i2cTransactionA.slaveAddress = Board_MPU6050_ADDR;
    i2cTransactionA.writeBuf = txBufferA;
    i2cTransactionA.writeCount = 2;
    i2cTransactionA.readBuf = rxBufferA;
    i2cTransactionA.readCount = 1;

    if (I2C_transfer(i2cA, &i2cTransactionA))
    {

                            System_printf("Accel Reg set P1: %d  \n",rxBufferA[0]);
                            System_flush();
                        }
                        else
                        {
                            System_printf("Accel Reg set Fail!\n");
                            System_flush();
                        }

      I2C_close(i2cA);
      System_printf("I2C closed!\n");

      System_flush();
  }

void setGyroConfig(uint8_t op)
{
        uint8_t         txBufferG[2];
        uint8_t         rxBufferG[1];
        I2C_Handle      i2cG;
        I2C_Params      i2cParamsG;
        I2C_Transaction i2cTransactionG;

        /* Create I2C for usage */
        I2C_Params_init(&i2cParamsG);
        i2cParamsG.bitRate = I2C_400kHz;
        i2cG = I2C_open(Board_I2C_TMP, &i2cParamsG);
        if (i2cG == NULL) {
            System_abort("Error Initializing I2C\n");
        }
        else {
            System_printf("I2C Initialized!\n");
        }

    txBufferG[0] = MPU6050_GYRO_CONFIG;//MPU6050_ACCEL_CONFIG;
    txBufferG[1] = op;
    i2cTransactionG.slaveAddress = Board_MPU6050_ADDR;
    i2cTransactionG.writeBuf = txBufferG;
    i2cTransactionG.writeCount = 2;
    i2cTransactionG.readBuf = rxBufferG;
    i2cTransactionG.readCount = 1;

    if (I2C_transfer(i2cG, &i2cTransactionG))
    {

                            System_printf("Gyro Reg set G1: %d  \n",rxBufferG[0]);
                            System_flush();
                        }
                        else
                        {
                            System_printf("Gyro set Fail!\n");
                            System_flush();
                        }

      I2C_close(i2cG);
      System_printf("I2C closed!\n");

      System_flush();
  }


Void taskFxn(UArg arg0, UArg arg1)
{

    //setPowerMGMT(PWR_MGMT_1_RESET);
    setPowerMGMT(PWR_MGMT_1_WAKE);
    setGyroConfig(GYRO_RANGE_500);
    setAccelConfig(ACCEL_RANGE_8);

    getAccelData();


     System_flush();
     Task_sleep(1000000 / Clock_tickPeriod);


}

/*
 *  ======== main ========
 */
int main(void)
{

    Task_Params taskParams;

    /* Call board init functions */
    Board_initGeneral();
    Board_initI2C();

    /* Construct tmp007 Task thread */
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)taskFxn, &taskParams, NULL);


    System_printf("Starting the I2C example\nSystem provider is set to SysMin."
                  " Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
