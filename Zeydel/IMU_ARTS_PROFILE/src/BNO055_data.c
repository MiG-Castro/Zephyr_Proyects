#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stdlib.h> 
#include "BNO055_data.h"

LOG_MODULE_REGISTER(BNO055_DATA_C, LOG_LEVEL_INF); //Enable the logging module in the application

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led2)

//THE DEVICE TREE NODE IDENTIFIER FOR THE BNO055 
//#define I2C1_NODE DT_NODELABEL(BNO055)
#define I2C_BNO055 DT_NODELABEL(bno055)

//static const struct gpio_dt_spec ledB = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_BNO055);

uint8_t id;
/**
 * @brief Initialize the BNO055 sensor.
 * @brief Get the device ID of the BNO055 sensor.
 */
void bno055_init(void){	
	uint8_t reg = CHIP_ID_ADDR;
	int ret = i2c_write_read_dt(&dev_i2c, &reg, sizeof(reg), &id, sizeof(id));
	if(ret == 0){
		LOG_INF("ID: %x\n\r",id);
		if(id != BNO055_ID){
			k_msleep(1000);
			if(id != BNO055_ID){
				LOG_INF("BNO055 not found!\n\r");
				return;
			}
		}
		LOG_INF("BNO055 found!\n\r");
	}else{
		LOG_INF("Error reading BNO055 ID!\n\r");
		return;
	}
}

/**
 * @brief Select the operation mode of the BNO055 sensor.
 * @param mode uint8_t The operation mode to be selected.
 */
void select_mode(uint8_t mode){
	int ret;
	uint8_t chance_mode[] = {OPR_MODE_ADDR, mode};
	ret = i2c_write_dt(&dev_i2c, chance_mode, sizeof(chance_mode));
	if(ret == 0){
		LOG_INF("BNO055 mode selected!\n\r");
	}else{
		LOG_INF("Error selecting BNO055 mode!\n\r");
	}
	k_msleep(30); //Wait for the mode to be selected
}

/**
 * @brief Read the quaternion data from the BNO055 sensor.
 */
void get_quaternion(uint8_t* buffer, size_t len){
	
	memset(buffer, 0, 8);

	// int16_t x, y, z, w;
  	// x = y = z = w = 0;

	uint8_t start_reg = BNO055_QUATERNION_DATA_W_LSB_ADDR;
	int ret = i2c_write_read_dt(&dev_i2c, &start_reg, sizeof(start_reg), buffer, len);
 	if (ret == 0) {
/*		w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
		x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
		y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
		z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

		float scale = 1.0f / 16384.0f;
		float fw = w * scale;
		float fx = x * scale;
		float fy = y * scale;
		float fz = z * scale;
		LOG_INF("Quaternion (float): w=%.3f, x=%.3f, y=%.3f, z=%.3f\n\r", (double)fw, (double)fx, (double)fy, (double)fz);
        LOG_INF("Quaternion raw data: ");
        for (int i = 0; i < 8; i++) {
            LOG_INF("0x%02X ", buffer[i]);
        }
        LOG_INF("\n"); */
	}else{
		LOG_INF("Error reading BNO055 quaternion data!\n\r");
		return;
	}
	
}

/**
 * @brief Get de euler data from the BNO055 sensor.
 * @param buffer uint8_t* buffer to store the data from the sensor
 */
void get_euler(uint8_t* buffer){
    
    // if (buffer == NULL) {
    //     return -EINVAL;  // Error code for invalid argument
    // }
	memset(buffer, 0, 6);

	int16_t heading, roll, pitch;
  	heading = roll = pitch = 0;

	uint8_t start_reg = BNO055_EULER_H_LSB_ADDR;
	int ret = i2c_write_read_dt(&dev_i2c, &start_reg, sizeof(start_reg), buffer, sizeof(buffer));
	
	if (ret == 0) {
		heading = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
		roll = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
		pitch = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);
		LOG_INF("ROLL=%.2f PITCH=%.2f YAW=%.2f\n", (double)roll / 16.0, (double)pitch / 16.0, (double)heading / 16.0);
        LOG_INF("Euler raw data: ");
        for (int i = 0; i < 6; i++) {
            LOG_INF("0x%02X ", buffer[i]);
        }
        LOG_INF("\n");
	}else{
		LOG_INF("Error reading BNO055 euler data!\n\r");
		return;
	}
}