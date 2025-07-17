//Archivo original del perfil LBS
//C:\ncs\v2.5.2\nrf\subsys\bluetooth\services

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "imu_data.h"

LOG_MODULE_REGISTER(IMU_DATA_C, LOG_LEVEL_INF); //Enable the logging module in the application

static bool IMUsensor_notify_enabled;

/**
 * @brief Callback for Notify characteristic configuration change.
 */
static void IMUsensor_Notify_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value){
    
    IMUsensor_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

/* IMU Data Service Declaration */
BT_GATT_SERVICE_DEFINE(
	imu_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_IMU_DATA), //Name and UUID of the service

	/* Definition of the characteristic imu_data (declaration of the characteristic) */
	BT_GATT_CHARACTERISTIC(BT_UUID_IMU_DATA_SENSOR, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, //Property: Notify. Permission: None (the server sends the info to the central)
			       NULL, NULL),                                                                   //No Read callback, No write callback (This functions excecutes it when there is a read or write operation), No User Data because the client doesn't read or write de value.
    //Definition of the Client Characteristic Configuration descriptor
	BT_GATT_CCC(IMUsensor_Notify_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),          //Configuration changed callback, Permission: Read and Write to enable/disable notifications notifications.

);
/**
 * @brief Send the sensor value as notification.  
 * @param[in] sensor_value The value of the sensor in uint32_t format.
 */
int send_IMUsensor_notify2(uint32_t sensor_value)
{
	if (!IMUsensor_notify_enabled) {
		return -EACCES;
	}
    //After the declaration of the service, a table attribute is created to store the characteristics (all the atributes of the service in order).
    //
	return bt_gatt_notify(NULL, &imu_svc.attrs[2], &sensor_value, sizeof(sensor_value)); 
}

/**
 * @brief Struct of the callback to be called when the notification is sent
 */
static struct bt_gatt_notify_params notify_params = {
    .uuid = NULL,              // Solo necesario si se notifica por UUID
    .attr = NULL,              // Se asigna luego
    .data = NULL,              // Se asigna en tiempo de envío
    .len = 0,                  // Se asigna en tiempo de envío
    .func = NULL,              // Se asigna si se quiere un callback
    .user_data = NULL
};

// static uint64_t last_notify_time = 0;
//static uint64_t last_time = 0;

// void notify_complete_cb(struct bt_conn *conn, void *user_data)
// {
//     // uint64_t now = k_cycle_get_64();
// 	uint64_t now1 = k_uptime_get(); // ms

//     /* if (last_notify_time != 0) {
//         uint64_t elapsed_us = k_cyc_to_us_floor64(now - last_notify_time);
//         LOG_INF("Tiempo entre notificaciones: %llu us\n", elapsed_us);
//     } else {
//         LOG_INF("Primera notificación enviada\n");
//     }
//     last_notify_time = now; */

// 	if (last_time != 0) {
//         uint64_t elapsed_ms = now1 - last_time;
//         LOG_INF("Tiempo1  %llu ms\n", elapsed_ms);
//     } else {
//         LOG_INF("Primera\n");
//     }

//     last_time = now1;
// }

/**
 * @brief Send the sensor value as notification. uint8_t buffer sensor_value
 * @param[in] sensor_value The value of the sensor in uint8_t* buffer format.
 * @param[in] len The length of the buffer.
 */
int send_IMUsensor_notify(uint8_t* sensor_value, size_t len)
{
	if (!IMUsensor_notify_enabled) {
		return -EACCES;
	}
    //After the declaration of the service, a table attribute is created to store the characteristics (all the atributes of the service in order).
	//return bt_gatt_notify(NULL, &imu_svc.attrs[2], sensor_value, len); 

	notify_params.attr = &imu_svc.attrs[2];  // Tu característica
    notify_params.data = sensor_value;
    notify_params.len = len;
    //notify_params.func = notify_complete_cb; // Opcional: tu callback
    notify_params.func = NULL; // Opcional: tu callback
    notify_params.user_data = NULL;
	return bt_gatt_notify_cb(NULL, &notify_params);
}

/**
 * @brief Get the notification status in the main function.
 */
bool get_IMUsensor_notify_enabled(void) {
    return IMUsensor_notify_enabled;
}
