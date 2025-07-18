#include <stdio.h>
#include <cstdint>
#include "DFRobot_LCD.h"
#include <iostream>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"





#define I2C_MASTER_SCL_IO           	8   	/*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO           	10  	/*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          	100000  /*!< I2C master clock frequency */
#define SHTC3_ADDR                  	0x70	/*!< SHTC3 device address */



  const int colorR = 255;
  const int colorG = 0;
  const int colorB = 0;

  DFRobot_LCD lcd(16,2);  //16 characters and 2 lines of show





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
	
    

    	i2c_master_read_byte(cmd, &data[0],  I2C_MASTER_ACK);
    	i2c_master_read_byte(cmd, &data[1],  I2C_MASTER_ACK);
    	i2c_master_read_byte(cmd, &data[2],  I2C_MASTER_ACK);
    	i2c_master_read_byte(cmd, &data[3],  I2C_MASTER_ACK);
    	i2c_master_read_byte(cmd, &data[4],  I2C_MASTER_ACK);
    	i2c_master_read_byte(cmd, &data[5], I2C_MASTER_NACK);



    	i2c_master_stop(cmd);
    	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    	i2c_cmd_link_delete(cmd);
   
    	float humidity = (float)((uint16_t)data[3] << 8 | (uint16_t)data[4]) / 65535.0 * 100.0;
    	float temperature = (float)((uint16_t)data[0] << 8 | (uint16_t)data[1]) / 65535.0 * 175.0 - 45.0;
    	int h = static_cast<int>(humidity);
    	int t = static_cast<int>(temperature);
    	std::string tem  = "Temp: " + std::to_string(t) + "C";
    	std::string hum = "Hum  : " + std::to_string(h) + "%";
    	lcd.printstr(tem.c_str());
    	lcd.setCursor(0,1);
    	lcd.printstr(hum.c_str());
    	printf("Temperature is %.1fC (%.1fF) with a %.0f%% humidity\n", temperature, temperature * 1.8 + 32, humidity);
    	lcd.setCursor(0,0);


}
extern "C" void app_main(void)
{
    	lcd.init();
    	lcd.setRGB(0, 255, 0);
    	while(true){
            	read_humidity();
            	vTaskDelay(pdMS_TO_TICKS(2000));

    	}

}

