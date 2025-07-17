//Archivo original del perfil LBS
//C:\ncs\v2.5.2\nrf\include\bluetooth\services\lbs.h


#ifndef BT_IMU_DATA_H_  //If the .h is not defined, then it is defined
#define BT_IMU_DATA_H_

#ifdef __cplusplus // C++ related
extern "C" {
#endif

#include <zephyr/types.h>

/** @brief IMU Data Service UUID. */
#define BT_UUID_IMU_DATA_SERVICE BT_UUID_128_ENCODE(0x1829d620,0xf043,0x4fa4,0x9cd5,0x660fe0834250)  //BT_UUID_128_ENCODE: Convert the array to the BLE necesary format 
/** @brief IMU Data Service Sensor characteristic UUID. */
#define BT_UUID_IMU_DATA_SENSOR_VAL BT_UUID_128_ENCODE(0x1829d621, 0xf043,0x4fa4,0x9cd5,0x660fe0834250)


#define BT_UUID_IMU_DATA BT_UUID_DECLARE_128(BT_UUID_IMU_DATA_SERVICE)               //BT_UUID_DECLARE_128: Convert the array to a generic UUID
#define BT_UUID_IMU_DATA_SENSOR BT_UUID_DECLARE_128(BT_UUID_IMU_DATA_SENSOR_VAL)     

/** @brief Send the sensor value as notification.
 * @param[in] sensor_value The value of the sensor in uint8_t buffer format.
 * @param[in] len The length of the buffer.
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int send_IMUsensor_notify(uint8_t *sensor_value, size_t len);

/**
 * @brief Get the notification enable status.
 */
bool get_IMUsensor_notify_enabled(void);

#ifdef __cplusplus  // C++ related
}
#endif

#endif /* BT_IMU_DATA_H_ */  //Closed: If the .h is not defined, then it is defined