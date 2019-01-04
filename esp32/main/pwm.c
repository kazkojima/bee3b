/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/xtensa_api.h"
#include "freertos/queue.h"
#include "esp_attr.h"   
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "b3packet.h"
#include "pwm.h"

static uint16_t scale(uint16_t width)
{
    // Map [1100, 1900] to [0, 4096]
    int32_t length = width - 1100;
    if (length < 0) {
        length = 0;
    } else {
        // 4096/800 times
        length = (length * 41) >> 3;
    }
    if (length > 4096) {
        length = 4096;
    } else if (length < 100) {
        length = 0;
    }
    return length;
}

#define ube16_val(v, idx) (((uint16_t)v[2*idx] << 8) | v[2*idx+1])
#define le16_val(v, idx) ((int16_t)(((uint16_t)(v)[2*idx+1] << 8) | (v)[2*idx]))

extern int sockfd;
extern SemaphoreHandle_t pwm_sem;

// pwm global
bool in_failsafe = false;
bool in_config = false;
bool in_arm = false;

uint32_t pwm_count = 0;
float last_width[NUM_CHANNELS];
bool pwm_stopped = false;

// Stop motors.
void pwm_shutdown(void)
{
    uint16_t low = 0;
    //printf("pwm_shutdown\n");
    xSemaphoreTake(pwm_sem, portMAX_DELAY);
    pwmbuf[0] = low;
    pwmbuf[1] = low;
    pwmbuf[2] = low;
    pwmbuf[3] = low;
    pwmbuf[5] = 0;
    pwmbuf[7] = 0;
    pwmbuf[15] = 0xbe3b;
    xSemaphoreGive(pwm_sem);
    pwm_stopped = true;
}

void pwm_output(uint16_t *wd)
{
    //printf("pwm_output %d %d %d %d\n", wd[0], wd[1], wd[2], wd[3]);
    // Swap motor for arducopter default
    uint16_t pwm1 = scale (wd[0]);
    uint16_t pwm2 = scale (wd[1]);
    uint16_t pwm3 = scale (wd[2]);
    uint16_t pwm0 = scale (wd[3]);
    xSemaphoreTake(pwm_sem, portMAX_DELAY);
    pwmbuf[0] = pwm0;
    pwmbuf[1] = pwm1;
    pwmbuf[2] = pwm2;
    pwmbuf[3] = pwm3;
    pwmbuf[5] = wd[5];
    pwmbuf[7] = wd[7];
    pwmbuf[15] = 0xbe3b;
    xSemaphoreGive(pwm_sem);
    pwm_stopped = (wd[0] <= LO_WIDTH && wd[1] <= LO_WIDTH
                   && wd[2] <= LO_WIDTH && wd[3] <= LO_WIDTH);
}

static bool cmp_mid(uint16_t wd[], int n)
{
    for (int i = 0; i < n; i++) {
        if (wd[i] > MID_WIDTH) {
            return true;
        }
    }
    return false;
}

#define PWM_UPDATE_LIMIT 0

void pwm_task(void *arg)
{
    in_arm = false;
    pwm_count = 0;
    pwm_stopped = false;
    for (int i = 0; i < NUM_CHANNELS; i++) {
        last_width[i] = LO_WIDTH;
    }
    pwm_shutdown();

    while (sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    struct B3packet pkt;
    TickType_t last_time = xTaskGetTickCount();
    while (1) {
        // Wait udp packet
        int n = recv(sockfd, &pkt, sizeof(pkt), 0);
        //printf("recv %d %x\n", n, pkt.head);
        if (n != sizeof(pkt) || pkt.head != B3HEADER)
            continue;
#if CONFIG_LOW_BATTERY_CHECK
        if (low_battery) {
            printf("low_battery: stop pwm_task\n");
            vTaskDelete(NULL);
        }
#endif
        if (pkt.tos != TOS_PWM || in_config) {
            continue;
        }

        if (!in_arm) {
            // Set in_arm when we get the first pwm packet.
            if (pwm_count == 0) {
                in_arm = true;
            } else {
                pwm_shutdown();
            }
        }
        pwm_count++;

#if PWM_UPDATE_LIMIT > 0
        TickType_t current_time = xTaskGetTickCount();
        // skip output so as not to update pwm too frequently
        if ((uint32_t)(current_time - last_time) <= PWM_UPDATE_LIMIT) {
            last_time = current_time;
            continue;
        } else {
            last_time = current_time;
        }
#endif

        if (in_failsafe || !in_arm) {
            continue;
        }

        uint16_t wd[NUM_CHANNELS];
        for (int i = 0; i < NUM_CHANNELS; i++) {
            uint16_t width = ube16_val(pkt.data, i);
            wd[i] = width;
            last_width[i] = (float)(width);
        }

        // Refuse arming if some pwms are over MID_WIDTH at startup
        if (pwm_count < PWM_STARTUP_COUNT && cmp_mid(wd, NUM_MOTORS)) {
            in_arm = false;
            // printf("Unsafe starting with high pwm\n");
            continue;
        }

        pwm_output(wd);
    }
}
