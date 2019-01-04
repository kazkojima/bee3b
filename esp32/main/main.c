/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "b3packet.h"
#include "pwm.h"

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    system_event_sta_disconnected_t *disconn;
    wifi_mode_t mode;

    switch(event->event_id) {
    case SYSTEM_EVENT_STA_DISCONNECTED:
        disconn = &event->event_info.disconnected;
        switch (disconn->reason) {
        case WIFI_REASON_AUTH_FAIL:
            printf("WiFi: desconntcted after auth fail\r\n");
            break;
        default:
            // try to reconnect
            if (esp_wifi_get_mode(&mode) == ESP_OK) {
                if (mode & WIFI_MODE_STA) {
                    printf("WiFi: try to reconnect...\r\n");
                    esp_wifi_connect();
                }
            }
            break;
        }
    default:
        break;
    }
    return ESP_OK;
}

#define UDP_SERVER CONFIG_UDP_SERVER_ADDRESS
#define UDP_PORT CONFIG_UDP_PORT

int sockfd = -1;

SemaphoreHandle_t send_sem;
SemaphoreHandle_t i2c_sem;
SemaphoreHandle_t nvs_sem;
SemaphoreHandle_t pwm_sem;

#define PKT_QSIZE 32
static xQueueHandle pkt_queue;

volatile uint16_t pwmbuf[PACKET_SIZE / sizeof(uint16_t)];

extern uint16_t bat_voltage;
extern uint16_t range_milimeter;
extern int16_t opt_flow_x;
extern int16_t opt_flow_y;
extern int16_t opt_quality;

static void
spi_task(void *arg)
{
    char recvbuf[PACKET_SIZE];
    char sendbuf[PACKET_SIZE];

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = PACKET_SIZE*8;
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    uint32_t last_pwm_count = 0;
    float rate = 0;

    while(1) {
        xSemaphoreTake(pwm_sem, portMAX_DELAY);
        memcpy (sendbuf, (void *)pwmbuf, PACKET_SIZE);
        xSemaphoreGive(pwm_sem);

        esp_err_t ret = spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
        //printf("spi_slave_transmit -> %x\n", ret);
        if (ret == ESP_OK) {
            if (xQueueSend(pkt_queue, &recvbuf[0], 0) != pdTRUE) {
                printf("fail to queue packet\n");
            }
        }

        // compute pwm rate here. expect 100-150
        rate = (1.0f-0.01f)*rate + 0.01f*500.0f*(pwm_count - last_pwm_count);
        last_pwm_count = pwm_count;

        xSemaphoreTake(pwm_sem, portMAX_DELAY);
        pwmbuf[RC_RATE] = (uint16_t) rate;
        pwmbuf[BAT_IDX] = bat_voltage;
        pwmbuf[POSZ_IDX] = range_milimeter;
        pwmbuf[OFQ_IDX] = opt_quality;
        pwmbuf[OFX_IDX] = opt_flow_x;
        pwmbuf[OFY_IDX] = opt_flow_y;
        xSemaphoreGive(pwm_sem);
        //printf ("rate %3.2f\n", rate);
    }
}

static void udp_task(void *arg)
{
    struct sockaddr_in saddr;
    struct sockaddr_in caddr;
    int rtn;

    printf("UDP client task starting...\r\n");

    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if(s < 0) {
        printf("... Failed to allocate socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return;
    }

    printf("... allocated socket\r\n");

    int option = 1;
    setsockopt (sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

    memset((char *) &caddr, 0, sizeof(caddr));
    caddr.sin_family = AF_INET;
    caddr.sin_addr.s_addr = htonl(INADDR_ANY);
    caddr.sin_port = htons(UDP_PORT);

    memset((char *) &saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = inet_addr (UDP_SERVER);
    saddr.sin_port = htons(UDP_PORT);

    rtn = bind (s, (struct sockaddr *)&caddr, sizeof(caddr));
    if(rtn < 0) {
        printf("... Failed to bind socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        return;
    }
    rtn = connect(s, (struct sockaddr *) &saddr, sizeof(saddr));
    if (rtn < 0) {
        printf("... Failed to connect socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        return;
    }

    sockfd = s;

    xTaskCreate(spi_task, "spi_task", 2048, NULL, 10, NULL);

    char recvbuf[PACKET_SIZE];
    while(1) {
        if (xQueueReceive(pkt_queue, &recvbuf[0], portMAX_DELAY) == pdTRUE) {
            struct B3packet pkt;
            pkt.head = B3HEADER;
            pkt.tos = TOS_IMU;
            memcpy (&pkt.data[0], recvbuf, B3SIZE);
            //printf("send a IMU packet\r\n");
            xSemaphoreTake(send_sem, portMAX_DELAY);
            int n = send(s, &pkt, sizeof(pkt), 0);
            if (n < 0) {
            }
            xSemaphoreGive(send_sem);
        }
    }
}

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS    5
#define PIN_NUM_HS   22

static void post_setup_cb(spi_slave_transaction_t *trans)
{
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1<<PIN_NUM_HS));
}

static void post_trans_cb(spi_slave_transaction_t *trans) {
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1<<PIN_NUM_HS));
}

static void spi_init(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=0
    };
    spi_slave_interface_config_t slvcfg={
        .mode=0,
        .spics_io_num=PIN_NUM_CS,
        .queue_size=3,
        .flags=0,
        .post_setup_cb=post_setup_cb,
        .post_trans_cb=post_trans_cb
     };

    //Initialize the SPI bus
    ret = spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, 0);
    assert(ret == ESP_OK);
}

#define I2C_MASTER_SCL_IO               26
#define I2C_MASTER_SDA_IO               25
#define I2C_MASTER_NUM                  I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE       0
#define I2C_MASTER_RX_BUF_DISABLE       0
#define I2C_MASTER_FREQ_HZ              400000

#define I2C1_MASTER_SCL_IO              33
#define I2C1_MASTER_SDA_IO              32
#define I2C1_MASTER_NUM                 I2C_NUM_1
#define I2C1_MASTER_FREQ_HZ             400000

static void i2c_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE,
                       0);

    i2c_master_port = I2C1_MASTER_NUM;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C1_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C1_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C1_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE,
                       0);
}

static void nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        // NVS partition was truncated and needs to be erased
        const esp_partition_t *pa;
        pa = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                      ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
        assert(pa && "partition table must have an NVS partition");
        ESP_ERROR_CHECK(esp_partition_erase_range(pa, 0, pa->size));
        // Retry nvs_flash_init
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

#define GPIO_INS_INT	GPIO_NUM_35
#define GPIO_MARKER_LED GPIO_NUM_17
#define GPIO_POWER_HOLD GPIO_NUM_27
#define GPIO_PSW_SENS   GPIO_NUM_14

static void xgpio_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1<<GPIO_MARKER_LED)|(1<<GPIO_POWER_HOLD));
    io_conf.pull_down_en = 0;
    // This ensures that all leds are off in deep sleep mode.
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = ((1<<GPIO_PSW_SENS));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // Interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1LL<<GPIO_INS_INT);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(GPIO_POWER_HOLD, 1);
    gpio_set_level(GPIO_MARKER_LED, 0);

    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CS, GPIO_PULLUP_ONLY);
    gpio_set_level(PIN_NUM_HS, 0);
    gpio_set_direction(PIN_NUM_HS, GPIO_MODE_OUTPUT);
}

extern void pwm_task(void *arg);
extern void baro_task(void *arg);
extern void bat_task(void *arg);
extern void rn_task(void *arg);
extern void of_task(void *arg);

void app_main(void)
{
    nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = CONFIG_SSID,
            .password = CONFIG_SSID_PASSWORD,
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );

    esp_log_level_set("wifi", ESP_LOG_WARN);

    spi_init();
    i2c_init();
    // nvs_init();
    xgpio_init();

    vSemaphoreCreateBinary(send_sem);
    vSemaphoreCreateBinary(i2c_sem);
    vSemaphoreCreateBinary(nvs_sem);
    vSemaphoreCreateBinary(pwm_sem);

    // packet queue
    pkt_queue = xQueueCreate(PKT_QSIZE, PACKET_SIZE);

    xTaskCreate(udp_task, "udp_task", 2048, NULL, 9, NULL);
    xTaskCreate(pwm_task, "pwm_task", 2048, NULL, 8, NULL);
    xTaskCreate(baro_task, "baro_task", 2048, NULL, 7, NULL);
    xTaskCreate(rn_task, "rn_task", 4096, NULL, 6, NULL);
    xTaskCreate(of_task, "of_task", 2048, NULL, 5, NULL);
    xTaskCreate(bat_task, "bat_task", 2048, NULL, 4, NULL);

    gpio_set_level(GPIO_MARKER_LED, 1);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    int pushed = 0;
    bool green = false;
    bool keep = false;
    while (true) {
        if (keep || gpio_get_level(GPIO_PSW_SENS) == 0) {
            ++pushed;
            if (pushed > 10) {
                // Keep power on
                green = true;
                keep = true;
            } else {
                green = !green;
            }
            gpio_set_level(GPIO_MARKER_LED, (green ? 1 : 0));
        } else {
            pushed = 0;
            green = false;
            gpio_set_level(GPIO_MARKER_LED, 1);
            break;
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    vTaskDelay(4000 / portTICK_PERIOD_MS);

    while (true) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (gpio_get_level(GPIO_PSW_SENS) == 0) {
            ++pushed;
            if (pushed > 10) {
                // Power off
                pwm_shutdown();
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                gpio_set_level(GPIO_POWER_HOLD, 0);
                esp_deep_sleep_start();
            }
            green = !green;
            gpio_set_level(GPIO_MARKER_LED, (green ? 1 : 0));
        } else {
            pushed = 0;
            gpio_set_level(GPIO_MARKER_LED, 1);
        }
    }
}

