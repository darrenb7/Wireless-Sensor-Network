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

/***** Includes *****/

#include <Sce.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"

#include "NodeTask.h"
#include "NodeRadioTask.h"


/***** Defines *****/
#define NODE_TASK_STACK_SIZE 1024
#define NODE_TASK_PRIORITY   3

#define NODE_EVENT_ALL                  0xFFFFFFFF
#define NODE_EVENT_NEW_MPU6050_VALUE    (uint32_t)(1 << 0)


/***** Variable declarations *****/
static Task_Params nodeTaskParams;
Task_Struct nodeTask;

static uint8_t nodeTaskStack[NODE_TASK_STACK_SIZE];
Event_Struct nodeEvent;

static Event_Handle nodeEventHandle;
static int16_t data[6];
static int packet;
static int i;


/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;


PIN_Config pinTable[] = {
    NODE_ACTIVITY_LED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/***** Prototypes *****/
static void nodeTaskFunction(UArg arg0, UArg arg1);
void getMPU6050Data();// Collects Data from Sensor Controller
void getData();// Creates consecutive string of numbers for testing.



/***** Function definitions *****/
void NodeTask_init(void)
{

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&nodeEvent, &eventParam);
    nodeEventHandle = Event_handle(&nodeEvent);

    /* Create the node task */
    Task_Params_init(&nodeTaskParams);
    nodeTaskParams.stackSize = NODE_TASK_STACK_SIZE;
    nodeTaskParams.priority = NODE_TASK_PRIORITY;
    nodeTaskParams.stack = &nodeTaskStack;
    Task_construct(&nodeTask, nodeTaskFunction, &nodeTaskParams, NULL);
}

static void nodeTaskFunction(UArg arg0, UArg arg1)
{
    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (!ledPinHandle)
    {
        System_abort("Error initializing board 3.3V domain pins\n");
    }

    /* Starts the SCE with a 1s sample period  */
    Sce_init(0x00010000);
    Sce_start();

    while(1) {

        printf("\nWaiting for event \n");
        //getData();
        getMPU6050Data();
        uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);


        printf("\nSensor Controller Data Recieved \n\n");
        //When new sensor controller data arriver send this data
        if (events & NODE_EVENT_NEW_MPU6050_VALUE) {
            /* Toggle activity LED */
            PIN_setOutputValue(ledPinHandle, NODE_ACTIVITY_LED,!PIN_getOutputValue(NODE_ACTIVITY_LED));

            //Sends sensor controller data to the radio task.
            NodeRadioTask_sendMPU6050Data(data);
        }
    }

}

void getMPU6050Data()
{

    data[0] = scifTaskData.mpu6050Sample.output.x;//Reads data from the x output buffer of the sensor controller.
    data[1] = scifTaskData.mpu6050Sample.output.y;
    data[2] = scifTaskData.mpu6050Sample.output.z;
    data[3] = scifTaskData.mpu6050Sample.output.gx;
    data[4] = scifTaskData.mpu6050Sample.output.gy;
    data[5] = scifTaskData.mpu6050Sample.output.gz;

    printf("xRaw= %d ",data[0]);
    printf("yRaw= %d ",data[1]);
    printf("zRaw= %d \n",data[2]);
    printf("GxRaw= %d ",data[3]);
    printf("GyRaw= %d ",data[4]);
    printf("GzRaw= %d \n",data[5]);

    Event_post(nodeEventHandle, NODE_EVENT_NEW_MPU6050_VALUE);
}

void getData(){//fills data array with consecutive numbers to send to the radio task for testing purposes

    for(i=0;i<6;i++){
        data[i]=packet;
        packet++;
    }
        printf("xTest= %d ",data[0]);
        printf("yTest= %d ",data[1]);
        printf("zTest= %d \n",data[2]);
        printf("GxTest= %d ",data[3]);
        printf("GyTest= %d ",data[4]);
        printf("GzTest= %d \n",data[5]);
    Event_post(nodeEventHandle, NODE_EVENT_NEW_MPU6050_VALUE);


}



