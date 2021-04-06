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

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <stdio.h>

#include <ti/sysbios/BIOS.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>

/* Drivers */
#include <ti/drivers/PIN.h>
/* Board Header files */
#include "Board.h"

#include "ConcentratorRadioTask.h"
#include "ConcentratorTask.h"
#include "RadioProtocol.h"


/***** Defines *****/
#define CONCENTRATOR_TASK_STACK_SIZE 1024
#define CONCENTRATOR_TASK_PRIORITY   3

#define CONCENTRATOR_EVENT_ALL                         0xFFFFFFFF
#define CONCENTRATOR_EVENT_NEW_MPU6050_SENSOR_VALUE    (uint32_t)(1 << 0)

#define CONCENTRATOR_MAX_NODES 7

#define CONCENTRATOR_DISPLAY_LINES 8

/***** Type declarations *****/
struct MPU6050SensorNode {
    uint8_t address;
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    uint32_t time100MiliSec;
    int8_t latestRssi;
};


/***** Variable declarations *****/
static Task_Params concentratorTaskParams;
Task_Struct concentratorTask;    /* not static so you can see in ROV */
static uint8_t concentratorTaskStack[CONCENTRATOR_TASK_STACK_SIZE];

Event_Struct concentratorEvent;  /* not static so you can see in ROV */
static Event_Handle concentratorEventHandle;

static struct MPU6050SensorNode latestActiveMPU6050SensorNode;
struct MPU6050SensorNode knownSensorNodes[CONCENTRATOR_MAX_NODES];
static struct MPU6050SensorNode* lastAddedSensorNode = knownSensorNodes;

//static Display_Handle hDisplayLcd;
//static Display_Handle hDisplaySerial;


/***** Prototypes *****/
static void concentratorTaskFunction(UArg arg0, UArg arg1);
static void packetReceivedCallback(union ConcentratorPacket* packet, int8_t rssi);
static void PrintValues(void);
static void PrintData(void);
static void addNewNode(struct MPU6050SensorNode* node);
static void updateNode(struct MPU6050SensorNode* node);
static uint8_t isKnownNodeAddress(uint8_t address);


/***** Function definitions *****/
void ConcentratorTask_init(void) {

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&concentratorEvent, &eventParam);
    concentratorEventHandle = Event_handle(&concentratorEvent);

    /* Create the concentrator radio protocol task */
    Task_Params_init(&concentratorTaskParams);
    concentratorTaskParams.stackSize = CONCENTRATOR_TASK_STACK_SIZE;
    concentratorTaskParams.priority = CONCENTRATOR_TASK_PRIORITY;
    concentratorTaskParams.stack = &concentratorTaskStack;
    Task_construct(&concentratorTask, concentratorTaskFunction, &concentratorTaskParams, NULL);
}

static void concentratorTaskFunction(UArg arg0, UArg arg1)
{

    /* Register a packet received callback with the radio task */
    ConcentratorRadioTask_registerPacketReceivedCallback(packetReceivedCallback);

    /* Enter main task loop */
    while(1) {
        /* Wait for event */
       printf("\nWaiting for Packet \n");
        uint32_t events = Event_pend(concentratorEventHandle, 0, CONCENTRATOR_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If we got a new sensor value */
        if(events & CONCENTRATOR_EVENT_NEW_MPU6050_SENSOR_VALUE) {
            printf("New Sensor Packet recieved\n");

            /* If we knew this node from before, update the value */
            if(isKnownNodeAddress(latestActiveMPU6050SensorNode.address)) {
                updateNode(&latestActiveMPU6050SensorNode);
                printf("Sensor Node Update\n");

            }
            else {
                /* Else add it */
                addNewNode(&latestActiveMPU6050SensorNode);
          printf("New Sensor Node: %x Added\n",latestActiveMPU6050SensorNode.address);

            }

            PrintValues();
            //PrintData(); //Test function for use with getData(); function on Node

        }
    }
}

static void packetReceivedCallback(union ConcentratorPacket* packet, int8_t rssi)
{
    if (packet->header.packetType == RADIO_PACKET_TYPE_MPU6050_PACKET)
    {
        /* Save the values */
        latestActiveMPU6050SensorNode.address = packet->header.sourceAddress;
        latestActiveMPU6050SensorNode.x = packet->mpu6050SensorPacket.x;
        latestActiveMPU6050SensorNode.y = packet->mpu6050SensorPacket.y;
        latestActiveMPU6050SensorNode.z = packet->mpu6050SensorPacket.z;
        latestActiveMPU6050SensorNode.gx = packet->mpu6050SensorPacket.gx;
        latestActiveMPU6050SensorNode.gy = packet->mpu6050SensorPacket.gy;
        latestActiveMPU6050SensorNode.gz = packet->mpu6050SensorPacket.gz;

        latestActiveMPU6050SensorNode.time100MiliSec = 0; //no timestamp value for packet
        latestActiveMPU6050SensorNode.latestRssi = rssi;

        Event_post(concentratorEventHandle, CONCENTRATOR_EVENT_NEW_MPU6050_SENSOR_VALUE);
    }
    /* If we recived an DualMode packet*/
    else if(packet->header.packetType == RADIO_PACKET_TYPE_DM_SENSOR_PACKET)
    {

        /* Save the values */
        latestActiveMPU6050SensorNode.address = packet->header.sourceAddress;
        latestActiveMPU6050SensorNode.x = packet->dmSensorPacket.x;
        latestActiveMPU6050SensorNode.y = packet->dmSensorPacket.y;
        latestActiveMPU6050SensorNode.z = packet->dmSensorPacket.z;
        latestActiveMPU6050SensorNode.gx = packet->dmSensorPacket.gx;
        latestActiveMPU6050SensorNode.gy = packet->dmSensorPacket.gy;
        latestActiveMPU6050SensorNode.gz = packet->dmSensorPacket.gz;
        latestActiveMPU6050SensorNode.time100MiliSec = packet->dmSensorPacket.time100MiliSec;
        latestActiveMPU6050SensorNode.latestRssi = rssi;

        Event_post(concentratorEventHandle, CONCENTRATOR_EVENT_NEW_MPU6050_SENSOR_VALUE);
    }
}

static uint8_t isKnownNodeAddress(uint8_t address) {
    uint8_t found = 0;
    uint8_t i;
    for (i = 0; i < CONCENTRATOR_MAX_NODES; i++)
    {
        if (knownSensorNodes[i].address == address)
        {
            found = 1;
            break;
        }
    }
    return found;
}

static void updateNode(struct MPU6050SensorNode* node) {
    uint8_t i;
    for (i = 0; i < CONCENTRATOR_MAX_NODES; i++) {
        if (knownSensorNodes[i].address == node->address)
        {
            knownSensorNodes[i].x = node->x;
            knownSensorNodes[i].y = node->y;
            knownSensorNodes[i].z = node->z;
            knownSensorNodes[i].gx = node->gx;
            knownSensorNodes[i].gy = node->gy;
            knownSensorNodes[i].gz = node->gz;
            knownSensorNodes[i].latestRssi = node->latestRssi;
            knownSensorNodes[i].time100MiliSec = node->time100MiliSec;
            break;
        }
    }
}

static void addNewNode(struct MPU6050SensorNode* node) {
    *lastAddedSensorNode = *node;

    /* Increment and wrap */
    lastAddedSensorNode++;
    if (lastAddedSensorNode > &knownSensorNodes[CONCENTRATOR_MAX_NODES-1])
    {
        lastAddedSensorNode = knownSensorNodes;
    }
}

static void PrintValues(void) {
    struct MPU6050SensorNode* nodePointer = knownSensorNodes;

    while ((nodePointer < &knownSensorNodes[CONCENTRATOR_MAX_NODES]) && (nodePointer->address != 0))
    {

        printf("\nNode Address= %x  \nX = %f Y= %f  Z= %f \nGX = %f GY= %f  GZ= %f \nRSSI= %d \nTime= %d\n",
                nodePointer->address,
                (float)(nodePointer->x)/(4096/9.8),(float)(nodePointer->y)/(4096/9.8),(float)(nodePointer->z)/(4096/9.8),
                (float)(nodePointer->gx)/(1000*65.5),(float)(nodePointer->gy)/(1000*65.5),(float)(nodePointer->gz)/(1000*65.5),
                nodePointer->latestRssi,nodePointer->time100MiliSec);

        nodePointer++;
    }
}

static void PrintData(void) {
    struct MPU6050SensorNode* nodePointer = knownSensorNodes;

    while ((nodePointer < &knownSensorNodes[CONCENTRATOR_MAX_NODES]) && (nodePointer->address != 0))
    {

        printf("\nNode Address= %x  \nX = %d Y= %d  Z= %d \nGX = %d GY= %d  GZ= %d \nRSSI= %d \nTime= %d\n",
                nodePointer->address,
                (nodePointer->x),(nodePointer->y),(nodePointer->z),
                (nodePointer->gx),(nodePointer->gy),(nodePointer->gz),
                nodePointer->latestRssi,nodePointer->time100MiliSec);

        nodePointer++;
    }
}
