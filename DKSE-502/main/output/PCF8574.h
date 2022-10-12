/*
 * PCF8574.h
 *
 *  Created on: 28 сент. 2022 г.
 *      Author: ivanov
 */
#include "config_pj.h"

#if exp_port == 1
#include "driver/i2c.h"
//esp_err_t i2c_master_write_to_device(i2c_port_t i2c_num, uint8_t device_address,
//                                     const uint8_t* write_buffer, size_t write_size,
//                                     TickType_t ticks_to_wait);
esp_err_t i2c_master_init(void);
void send_cmd (char cmd,uint8_t addr);
void gpio_set_level_ext(uint8_t pin , uint8_t level);
#endif /* MAIN_OUTPUT_PCF8574_H_ */
