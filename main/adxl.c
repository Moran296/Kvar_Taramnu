#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "sdkconfig.h"
//#include "rom/ets_sys.h"
#include <esp_log.h>
#include "adxl.h"


#define I2C_PORT I2C_NUM_0
#define ADXL_ADDRESS 0x53
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */
#define SDA_PIN  19
#define SCL_PIN  18

static esp_err_t I2C_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 10000
    };
	i2c_param_config(I2C_NUM_0, &conf);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    return ESP_OK;
}


void ADXL_init()
{
    vTaskDelay(pdMS_TO_TICKS(1000));                                 // Initial 40 mSec delay
    I2C_init();
    vTaskDelay(100 / portTICK_RATE_MS);                                 // Initial 40 mSec delay

  adxl_write (0x2d, 0x00);  // reset all bits

  adxl_write (0x2d, 0x08);  // measure and wake up 8hz

  adxl_write (0x2A, 0x01);  // enable tap detection on Z axis

  adxl_write (0x1D, 40);  // set tap threshold 2.5g/.0625

  adxl_write (0x21, 32);  // set tap duration .02 sec/.000625

  adxl_write (0x22, 80);  // double tap latency .1sec/.00125

  adxl_write (0x23, 240);  // double tap window .3sec/.00125

  adxl_write (0x2F, 0x00);  // int map reg 0 means INT1

  adxl_write (0x2E, 0x60);  // int enable reg

}

static esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    printf(" error - %d\n", ret);
    return ret;
}


static esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


esp_err_t adxl_write(uint8_t reg, uint8_t data)
{
    vTaskDelay(pdMS_TO_TICKS(40));                                 
    return i2c_master_write_slave_reg(I2C_PORT, ADXL_ADDRESS, reg, data);
}

esp_err_t adxl_read(uint8_t reg, uint8_t *data)
{
    return i2c_master_read_slave_reg(I2C_PORT, ADXL_ADDRESS, reg, data, 1);
}







