/*
 * PCF8574.c
 *
 *  Created on: 28 сент. 2022 г.
 *      Author: ivanov
 */
#include "config_pj.h"
#if exp_port == 1

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "../main/output/PCF8574.h"
#define I2C_MASTER_NUM I2C_NUM_0
#define TAG "I2C"
#define I2C_TRANS_BUF_MINIMUM_SIZE    128

uint8_t data_out_exp[out_port_n / 8];

esp_err_t i2c_master_write_to_device1(i2c_port_t i2c_num, uint8_t device_address,const uint8_t* write_buffer, size_t write_size, TickType_t ticks_to_wait)
{
    esp_err_t err = ESP_OK;
    uint8_t buffer[I2C_TRANS_BUF_MINIMUM_SIZE] = { 0 };

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
//    assert (handle != NULL);

    err = i2c_master_start(handle);
    if (err != ESP_OK) {
        goto end;
    }

    err = i2c_master_write_byte(handle, device_address << 1 | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        goto end;
    }

    err = i2c_master_write(handle, write_buffer, write_size, true);
    if (err != ESP_OK) {
        goto end;
    }

    i2c_master_stop(handle);
    err = i2c_master_cmd_begin(i2c_num, handle, ticks_to_wait);

end:
i2c_cmd_link_delete(handle);
    return err;
}

void pcf8574_task(void *pvParameters) {

	while (1)
	{
	for(uint16_t ct=0;ct<100;ct++)
	{
		send_cmd(data_out_exp[0],addr_exp1);
		vTaskDelay(500 / portTICK_PERIOD_MS);

#if addr_exp2 != 0
		send_cmd(data_out_exp[1],addr_exp2);
		vTaskDelay(500 / portTICK_PERIOD_MS);
#endif

	}
	//refresh
	}
}

esp_err_t i2c_master_init(void) {
	i2c_config_t conf = { .mode = I2C_MODE_MASTER, .sda_io_num = GPIO_NUM_10,
			.scl_io_num = GPIO_NUM_9, .sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE, .master.clk_speed = 100000, };

	i2c_param_config(I2C_NUM_0, &conf);
	xTaskCreate(&pcf8574_task, "pcf8574_task", 2024, NULL, 10, NULL);

	return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

void send_cmd(char cmd, uint8_t addr) {
	esp_err_t err;
	char data_u, data_l;
	uint8_t data_t[4];
//	data_u = (cmd&0xf0);
//	data_l = ((cmd<<4)&0xf0);
	data_t[0] = cmd;  //en=1, rs=0
	data_t[1] = cmd;  //en=0, rs=0
//	data_t[2] = 0xf0;  //en=1, rs=0
//	data_t[3] = 0x0f;  //en=0, rs=0

	err = i2c_master_write_to_device1(I2C_MASTER_NUM, addr, data_t, 2, 1000);
	if (err != 0)
		ESP_LOGI(TAG, "Error no. %d in command", err);
}

void gpio_set_level_ext(uint8_t pin, uint8_t level) {

	if (pin < 8) {
		if (level == 0) {
			data_out_exp[0]&=~(1<<pin);
		} else {
			data_out_exp[0]|=1<<pin;
		}

		printf("data_out_exp[0]=%x\n\r",data_out_exp[0]);
	}
	if ((pin < 16) && (pin > 7)) {
		if (level == 0) {
			data_out_exp[1]&=~(1<<(pin-8));
		} else {
			data_out_exp[1]|=1<<(pin-8);
		}
		printf("data_out_exp[1]=%x\n\r",data_out_exp[1]);
	}



}

#endif /* MAIN_OUTPUT_PCF8574_H_ */

