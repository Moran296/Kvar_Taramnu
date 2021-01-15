#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/semphr.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "HD44780.h"
#include "adxl.h"
#include "AP.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#define PORT 3333
#define LCD_ADDR 0x27
#define SDA_PIN  19
#define SCL_PIN  18
#define LCD_COLS 20
#define LCD_ROWS 4
#define INT_GPIO 4
#define GPIO_INPUT_PIN_SEL  1ULL << INT_GPIO
#define ESP_INTR_FLAG_DEFAULT 0
#define SOCKET_TAG "socket"

void LCD_Task(void* param);
void tcp_server_task(void *pvParameters);
static void Recieve(const int sock);

static char tag[] = "Kvar_Taramnu";
void adxl_task(void* param);
static TaskHandle_t adxlHandle;
static TaskHandle_t lcdHandle;
static TimerHandle_t knockTimer;
SemaphoreHandle_t lcdSem;
static bool isTiming = false;
//static char lcdBuf[80] = "I am not here!      Kvar Taramnu!";
static char lcdBuf[80] = "I am not here soplease go away";
static char rx_buffer[80];

static void  gpio_isr_handler(void* arg)
{
     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
     vTaskNotifyGiveFromISR(adxlHandle, &xHigherPriorityTaskWoken );
     portYIELD_FROM_ISR();
}

static void knockTimerFunc(TimerHandle_t timer) {
    isTiming = false;
}

void app_main(void)
{

    ADXL_init();
    knockTimer = xTimerCreate ( "knock time", pdMS_TO_TICKS(2500), pdFALSE, NULL, knockTimerFunc);
    xTaskCreate(&adxl_task, "adxl Task", 2048, NULL, 5, &adxlHandle);
    gpio_set_intr_type(INT_GPIO, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(INT_GPIO, gpio_isr_handler, (void*) INT_GPIO);
    lcdSem = xSemaphoreCreateBinary();
    xSemaphoreGive(lcdSem);


    ESP_LOGI(tag, "Starting up application");
    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
    APinit();

    xTaskCreate(&LCD_Task, "LCD Task", 2048, NULL, 5, &lcdHandle);
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
}

void adxl_task(void* param) {

    (void)param;
    uint8_t data;
    uint8_t reg = 0x30;

    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI("ADXL", "notified!");
        adxl_read(reg, &data);
        if(isTiming) {
            data = 0;
        }
        isTiming = true;
        xTimerStart(knockTimer, portMAX_DELAY);
        data &= 0x60;
        if(data) {
            printf("knock\n");
            xTaskNotifyGive(lcdHandle);
        }
    }
}

void LCD_Task(void* param)
{
    char space = ' ';
    char* spaceIndex = NULL;
    int wordlen;


    for (;;)
     {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI("LCD", "notified!");
        int row = 0, col = 0;
        LCD_home();
        LCD_clearScreen();
        xSemaphoreTake(lcdSem, portMAX_DELAY);
        for (int i = 0; i < 80; i++) {
            LCD_setCursor(col, row);
            if(!lcdBuf[i]) {
                break;
            }

            spaceIndex = strchr(lcdBuf + i, space);
            if(spaceIndex) {
                wordlen = spaceIndex - (lcdBuf + i);
                if(wordlen > (19 - col)) {
                    row++;
                    col = 0;
                    LCD_setCursor(col, row);
                }
            }

            LCD_writeChar(lcdBuf[i]);

            if (col++ >= 19) {
                col = 0;
                row++;
            }
            vTaskDelay(pdMS_TO_TICKS(25));
        }
            xSemaphoreGive(lcdSem);
            vTaskDelay(pdMS_TO_TICKS(2500));
            LCD_clearScreen();
            LCD_off();
    }
}

void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);


    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(SOCKET_TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(SOCKET_TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(SOCKET_TAG, "Socket unable to bind: errno %d", errno);
    }
    ESP_LOGI(SOCKET_TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(SOCKET_TAG, "Error occurred during listen: errno %d", errno);
    }

    while (1) {

        ESP_LOGI(SOCKET_TAG, "Socket listening");

        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        uint addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(SOCKET_TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Convert ip address to string
        if (source_addr.sin6_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        } else if (source_addr.sin6_family == PF_INET6) {
            inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(SOCKET_TAG, "Socket accepted ip address: %s", addr_str);

        Recieve(sock);
        if(strlen(rx_buffer) > 5){

            xSemaphoreTake(lcdSem, portMAX_DELAY);
            memcpy(lcdBuf, rx_buffer, strlen(rx_buffer));
            lcdBuf[strlen(rx_buffer)] = 0;
            xSemaphoreGive(lcdSem);
            ESP_LOGI(SOCKET_TAG, "memcopied");
        }

        shutdown(sock, 0);
        close(sock);
    }
}

static void Recieve(const int sock)
{
    int len;

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(SOCKET_TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(SOCKET_TAG, "Connection closed");
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(SOCKET_TAG, "Received %d bytes: %s", len, rx_buffer);

    /*
            int to_write = len;
            while (to_write > 0) {
                int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                }
                to_write -= written;
            }
            */
        }
    } while (len > 0);
    ESP_LOGI(SOCKET_TAG, "finished recieveing");
}



