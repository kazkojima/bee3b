/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "soc/uart_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "b3packet.h"

/* Cheerson CX-OF PMW3901 optical flow device with uart interface

   CXOF serial packet description
   byte0: header (0xFE)
   byte1: reserved
   byte2: x-motion high byte;
   byte3: x-motion low byte;
   byte4: y-motion high byte;
   byte5: y-motion low byte;
   byte6: t-motion
   byte7: surface quality
   byte8: footer (0xAA)

   sensor sends packets at 25hz
 */

#define CXOF_HEADER (uint8_t)0xFE
#define CXOF_FOOTER (uint8_t)0xAA
#define CXOF_FRAME_LENGTH 9
#define CXOF_PIXEL_SCALING (1.0f/4000)
#define TIMEOUT_MS 255

#define UART_CXOF UART_NUM_2
#define CXOF_TXD 21
#define CXOF_RXD 16
#define BUF_SIZE 256

extern int sockfd;
extern SemaphoreHandle_t send_sem;

int16_t opt_flow_x;
int16_t opt_flow_y;
int16_t opt_quality;

static void uart_init()
{
    uart_config_t uart_config = {
       .baud_rate = 19200,
       .data_bits = UART_DATA_8_BITS,
       .parity = UART_PARITY_DISABLE,
       .stop_bits = UART_STOP_BITS_1,
       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
       .rx_flow_ctrl_thresh = 122,
    };
    uart_param_config(UART_CXOF, &uart_config);
    uart_set_pin(UART_CXOF, CXOF_TXD, CXOF_RXD,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_CXOF, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
}

#define be16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))

inline static
int sat(int x, int lo, int hi)
{
    if (x < lo)
        return lo;
    else if (x > hi)
        return hi;
    return x;
}

void of_task(void *pvParameters)
{
    while (sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    printf("CX-OF: optical flow sensor\n");

    uart_init();

    struct B3packet pkt;
    uint8_t frame[CXOF_FRAME_LENGTH];
    int index;
    int dt;
    TickType_t last_mark = xTaskGetTickCount();
    while (1) {
        uint8_t c;
        int nc = uart_read_bytes(UART_CXOF, &c, 1, TIMEOUT_MS);
        if (nc == 0) {
            printf("CX-OF: read timeout\n");
            continue;
        }
        if (index == 0) {
            if (c == CXOF_HEADER) {
                frame[index++] = c;
            }
        } else {
            frame[index++] = c;
            if (index >= CXOF_FRAME_LENGTH) {
                if (frame[index-1] != CXOF_FOOTER) {
                    // reset state
                    index = 0;
                    continue;
                }

                memset (&pkt.data[0], 0, B3SIZE);

                TickType_t mark =  xTaskGetTickCount();
                dt = mark - last_mark;
                last_mark = mark;
                //printf("dt %d\n", dt);

                if (dt > 0 && (dt < TIMEOUT_MS)) {
                    // decode frame
                    int16_t x_raw = be16_val(frame, 1);
                    int16_t y_raw = be16_val(frame, 2);
                    int16_t squal = (sat(frame[7], 64, 78) - 64)*255/14;

                    opt_flow_x = -x_raw;
                    opt_flow_y = -y_raw;
                    opt_quality = squal;

                    union { float f; uint8_t bytes[sizeof(float)]; } x, y;
                    // convert rad/s by multiplying 25/100000
                    // and adjust sensor alignment (-180deg)
                    x.f = -x_raw * CXOF_PIXEL_SCALING;
                    y.f = -y_raw * CXOF_PIXEL_SCALING;
                    memcpy (&pkt.data[0], x.bytes, sizeof(x));
                    memcpy (&pkt.data[4], y.bytes, sizeof(y));
                    pkt.data[8] = squal;
                    pkt.data[9] = frame[6];
                    pkt.data[10] = dt;
                } 

                // send packet
                pkt.head = B3HEADER;
                pkt.tos = TOS_OF;

                xSemaphoreTake(send_sem, portMAX_DELAY);
                int n = send(sockfd, &pkt, sizeof(pkt), 0);
                if (n < 0) {
                }
                xSemaphoreGive(send_sem);
                
                // reset state
                index = 0;
                continue;
            }
        }
    }
}

