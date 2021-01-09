#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "HD44780.h"
#include "adxl.h"

#define LCD_ADDR 0x27
#define SDA_PIN  19
#define SCL_PIN  18
#define LCD_COLS 20
#define LCD_ROWS 4
#define INT_GPIO 4
#define GPIO_INPUT_PIN_SEL  1ULL << INT_GPIO
#define ESP_INTR_FLAG_DEFAULT 0


static char tag[] = "Kvar_Taramnu";
void LCD_Task(void* param);
void adxl_task(void* param);
static TaskHandle_t adxlHandle;
static TaskHandle_t lcdHandle;
static TimerHandle_t knockTimer;
static bool isTiming = false;
static char lcdBuf[80] = "I am not here!        Go Away!!!";

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


    ESP_LOGI(tag, "Starting up application");
    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
    xTaskCreate(&LCD_Task, "LCD Task", 2048, NULL, 5, &lcdHandle);
}

void adxl_task(void* param) {

    (void)param;
    uint8_t data;
    uint8_t reg = 0x30;

    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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
    for (;;)
     {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int row = 0, col = 0;
        LCD_home();
        LCD_clearScreen();
        for (int i = 0; i < 80; i++) {
            LCD_setCursor(col, row);
            if(!lcdBuf[i]) {
                break;
            }
            LCD_writeChar(lcdBuf[i]);
            if (i >= 19) {
                row = (i + 1) / 20;
            }
            if (col++ >= 19) {
                col = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(25));
        }
            vTaskDelay(pdMS_TO_TICKS(2500));
            LCD_clearScreen();
            LCD_off();
    }
}