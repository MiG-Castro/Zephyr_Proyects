#ifndef BT_IMU_BNO055_DATA_H_  //If the .h is not defined, then it is defined
#define BT_IMU_BNO055_DATA_H_

#ifdef __cplusplus // C++ related
    extern "C" {
#endif

#include <stdint.h>  //Best practice when using uint8_t, uint16_t, uint32_t


/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS 16

/*Sensor registers*/
#define default_addr 0x29 //Default address
#define alt_addr 0x28	// Alternative address (por defecto en la breakout board de adafruit)

#define OPR_MODE_ADDR  0X3D//Address of the operation mode register
#define CHIP_ID_ADDR 0X00 //Address of the ID register

/*Command data*/
#define BNO055_ID 0xA0 //Device ID 
#define OPERATION_MODE_IMU 0X08 //Operation mode: IMU
#define OPERATION_MODE_NDOF 0X0C //Operation mode: NDOF


/*Dta registers: read only*/
	/* Euler data registers */
	#define BNO055_EULER_H_LSB_ADDR 0X1A
	#define BNO055_EULER_H_MSB_ADDR 0X1B
	#define BNO055_EULER_R_LSB_ADDR 0X1C
	#define BNO055_EULER_R_MSB_ADDR 0X1D
	#define BNO055_EULER_P_LSB_ADDR 0X1E
	#define BNO055_EULER_P_MSB_ADDR 0X1F

	/* Quaternion data registers */
	#define BNO055_QUATERNION_DATA_W_LSB_ADDR  0X20
	#define BNO055_QUATERNION_DATA_W_MSB_ADDR  0X21
	#define BNO055_QUATERNION_DATA_X_LSB_ADDR  0X22
	#define BNO055_QUATERNION_DATA_X_MSB_ADDR  0X23
	#define BNO055_QUATERNION_DATA_Y_LSB_ADDR  0X24
	#define BNO055_QUATERNION_DATA_Y_MSB_ADDR  0X25
	#define BNO055_QUATERNION_DATA_Z_LSB_ADDR  0X26
	#define BNO055_QUATERNION_DATA_Z_MSB_ADDR  0X27

/**
 * @brief Initialize the BNO055 sensor.
 * @brief Get the device ID of the BNO055 sensor.
 */
void bno055_init(void);
/**
 * @brief Select the operation mode of the BNO055 sensor.
 * @param mode uint8_t The operation mode to be selected.
 */
void select_mode(uint8_t mode);
/**
 * @brief Read the quaternion data from the BNO055 sensor.
 * @param buffer uint8_t* buffer to store the data from the sensor
 */
void get_quaternion(uint8_t* buffer, size_t len);
/**
 * @brief Get de euler data from the BNO055 sensor.
 * @param buffer uint8_t* buffer to store the data from the sensor
 */
void get_euler(uint8_t* buffer);


#ifdef __cplusplus  // C++ related
}
#endif

#endif /* BT_IMU_BNO055_DATA_H_ */  //Closed: If the .h is not defined, then it is defined