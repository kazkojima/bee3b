/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "b3packet.h"

#include "vl53l1_api.h"

VL53L1_Dev_t dev;
VL53L1_DEV Dev = &dev;

static int i2c_handle = I2C_NUM_0;

extern int sockfd;
extern SemaphoreHandle_t send_sem;

void
rn_task(void *arg)
{
    while (sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelay(200 / portTICK_PERIOD_MS);

    Dev->I2cHandle = &i2c_handle;
    Dev->I2cDevAddr = 0x52;

    uint8_t byteData;
    uint16_t wordData;

    VL53L1_RdByte(Dev, 0x010F, &byteData);
    printf("VL53L1X Model_ID: %02X\n\r", byteData);
    VL53L1_RdByte(Dev, 0x0110, &byteData);
    printf("VL53L1X Module_Type: %02X\n\r", byteData);
    VL53L1_RdWord(Dev, 0x010F, &wordData);
    printf("VL53L1X: %02X\n\r", wordData);

    static VL53L1_RangingMeasurementData_t RangingData;
    int status = VL53L1_WaitDeviceBooted(Dev);
    status = VL53L1_DataInit(Dev);
    status = VL53L1_StaticInit(Dev);
    status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 43000);
    status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 50);
    status = VL53L1_StartMeasurement(Dev);

    if(status) {
        printf("VL53L1_StartMeasurement failed\n");
        vTaskDelete(NULL);
    }	

    union { float f; uint8_t bytes[sizeof(float)];} nof;
    nof.f = -2.0f;
    struct B3packet pkt;
    for (int i = 0; i < B3SIZE/sizeof(float); i++) {
        memcpy(&pkt.data[0], nof.bytes, sizeof(nof));
    }

    /* Autonomous ranging loop*/
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        TickType_t lap = xTaskGetTickCount() - xLastWakeTime;
        if (50/portTICK_PERIOD_MS > lap) {
            vTaskDelay(50/portTICK_PERIOD_MS - lap);
        }
        xLastWakeTime = xTaskGetTickCount();

        status = VL53L1_WaitMeasurementDataReady(Dev);
        if(!status) {
            status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);

            union { float f; uint8_t bytes[sizeof(float)];} df;
            if(status==0) {
                df.f = ((float)RangingData.RangeMilliMeter) * 0.1f;
                //printf("%3.2f\n", df.f);
            } else {
                df.f = -1.0f;
                printf("VL53L1_GetRangingMeasurementData failed\n");
            }
            memcpy(&pkt.data[0], df.bytes, sizeof(df));

            status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
            if(status) {
                printf("VL53L1_ClearInterruptAndStartMeasurement failed\n");
            }

            pkt.head = B3HEADER;
            pkt.tos = TOS_RANGE;
            xSemaphoreTake(send_sem, portMAX_DELAY);
            int n = send(sockfd, &pkt, sizeof(pkt), 0);
            if (n < 0) {
            }
            xSemaphoreGive(send_sem);
        }
    }
}
