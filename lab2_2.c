#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO       	8	/*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO       	10	/*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ      	100000 	/*!< I2C master clock frequency */
#define SHTC3_ADDR              	0x70  	/*!< SHTC3 device address */

void i2c_master_init(void)
{
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ,
	};
	i2c_param_config(I2C_MASTER_NUM, &conf);
	i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void read_humidity(void)
{
	uint8_t data[6];
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SHTC3_ADDR << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, 0x78, true);
	i2c_master_write_byte(cmd, 0x66, true);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
	i2c_cmd_link_delete(cmd);

	vTaskDelay(pdMS_TO_TICKS(25));

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SHTC3_ADDR << 1) | I2C_MASTER_READ, true);
	i2c_master_read_byte(cmd, &data[0], 0);
	i2c_master_read_byte(cmd, &data[1], 0);
	i2c_master_read_byte(cmd, &data[2], 0);
	i2c_master_read_byte(cmd, &data[3], 0);
	i2c_master_read_byte(cmd, &data[4], 0);
	i2c_master_read_byte(cmd, &data[5], 1);

	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
	i2c_cmd_link_delete(cmd);

	float humidity = (float)((uint16_t)data[3] << 8 | (uint16_t)data[4]) / 65535.0 * 100.0;
	float temperature = (float)((uint16_t)data[0] << 8 | (uint16_t)data[1]) / 65535.0 * 175.0 - 45.0;
	//printf("Humidity: %.2f%%\n", humidity);
	printf("Temperature is %.1fC (%.1fF) with a %.0f%% humidity\n", temperature, temperature * 1.8 + 32, humidity);
	//return humidity;
}

void app_main(void)
{
    	i2c_master_init();
    	while(1) {
    	read_humidity();
    	//printf("Humidity: %.2f%%\n", humidity);
	vTaskDelay(pdMS_TO_TICKS(2000));
    	}
}

