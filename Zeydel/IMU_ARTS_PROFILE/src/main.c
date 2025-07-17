//Original file from C:\ncs\v2.5.2\nrf\samples\bluetooth\peripheral_lbs
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h> //Header for BLE GENERIC ACCESS PROFILE (GAP)
#include <zephyr/bluetooth/uuid.h> //Header for UUID
#include <zephyr/bluetooth/conn.h> //Header for CONNECTION MANAGEMENT (CONN)
#include <zephyr/bluetooth/gatt.h> //Header for GATT

#include "imu_data.h" // double quotes are necessary to include the header file that is include in the same folder. BLE IMU SERVICE
#include "BNO055_data.h" // Access to the BNO055 sensor


LOG_MODULE_REGISTER(ARTS_IMU_PROFILE_MAIN, LOG_LEVEL_INF); //Enable the logging module in the application
struct bt_conn *my_conn = NULL; //Store de connection
static struct bt_gatt_exchange_params exchange_params; //FOR MTU UPDATE

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME    // device name from prj.conf
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec ledR = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec ledG = GPIO_DT_SPEC_GET(LED1_NODE, gpios);


//Advertising process parameters (Advertising intervals, secondary_max_skip,peer, options like BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY, etc)  
static const struct bt_le_adv_param *advert_param = BT_LE_ADV_PARAM(
	(BT_LE_ADV_OPT_CONN |
	 BT_LE_ADV_OPT_USE_IDENTITY), /* Advertise using the identity address as the advertiser address.*/
	800, /* Min Advertising Interval 500ms (800*0.625ms) */
	801, /* Max Advertising Interval 500.625ms (801*0.625ms) */
	NULL); 

// Struct of an advertising pkt, exist various options to fill the advertising pkt
static const struct bt_data advert_data[] = {
    //BT_DATA AND BT_DATA_BYTES are macros to write flags for configurations, included in bluetooth.h
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)), // flgs for general discoverable mode and BR/EDR not supported
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),         // device name

};
// Struct of a scan response pkt, exist various options to fill it
static const struct bt_data scan_rsp_data[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_IMU_DATA_SERVICE), 
};

/**
 * @brief Update the connection data length parameters.
 * @param[in] conn The connection object.
 */
static void update_data_length(struct bt_conn *conn)
{
	int err;
	struct bt_conn_le_data_len_param my_data_len = {
		.tx_max_len = BT_GAP_DATA_LEN_MAX, //251 bytes
		.tx_max_time = BT_GAP_DATA_TIME_MAX, //17040µs  //Tiempo para transmitir los paquetes con una capa fisica determinada
	};
	err = bt_conn_le_data_len_update(my_conn, &my_data_len);
	if (err) {
		LOG_ERR("data_len_update failed (err %d)", err);
	}
}

/**
 * @brief Callback function for MTU exchange.
 */
static void exchange_func(struct bt_conn *conn, uint8_t att_err,
			  struct bt_gatt_exchange_params *params)
{
	LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");
    if (!att_err) {
        uint16_t payload_mtu = bt_gatt_get_mtu(conn) - 3;   // 3 bytes used for Attribute headers.
        LOG_INF("New MTU: %d bytes", payload_mtu);
    }
}
/**
 * @brief Update the connection MTU parameters.
 * @param[in] conn The connection object.
 */
static void update_mtu(struct bt_conn *conn)
{
	int err;
	exchange_params.func = exchange_func;  

	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
	}
}

/**
 * @brief Update the connection PHY parameters.
 * @param[in] conn The connection object.
 */
static void update_phy(struct bt_conn *conn)
{
	int err;
	const struct bt_conn_le_phy_param preferred_phy = {
		.options = BT_CONN_LE_PHY_OPT_NONE,
		.pref_rx_phy = BT_GAP_LE_PHY_2M,
		.pref_tx_phy = BT_GAP_LE_PHY_2M,
	};
	err = bt_conn_le_phy_update(conn, &preferred_phy);
	if (err) {
		LOG_ERR("bt_conn_le_phy_update() returned %d", err);
	}
}

//Callbacks for the connection
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_INF("Connection failed (err %u)\n", err);
		return;
	}
	LOG_INF("Connected\n");
    gpio_pin_set_dt(&ledG,1);
	my_conn = bt_conn_ref(conn);
	bt_le_adv_stop();//Stop de advertising

	// Get connection parameters INFORMATION
	struct bt_conn_info info;
	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_ERR("bt_conn_get_info() returned %d", err);
		return;
	}
	double connection_interval = info.le.interval*1.25; // in ms
	uint16_t supervision_timeout = info.le.timeout*10; // in ms
	LOG_INF("Connection parameters: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval, info.le.latency, supervision_timeout);

	//MODIFY CONNECTIONS PARAMETERS 
	update_phy(my_conn);
	k_sleep(K_MSEC(1000));  // Delay added to avoid link layer collisions.
	update_data_length(my_conn);
	update_mtu(my_conn);

}
//Callbacks for the disconnection
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);
    gpio_pin_set_dt(&ledG, 0);
	
	// Start advertising
	my_conn =NULL;
}

//Write a callback function to inform about in the connection parameters
void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	double connection_interval = interval*1.25;         // in ms
	uint16_t supervision_timeout = timeout*10;          // in ms
	LOG_INF("Connection parameters updated: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval, latency, supervision_timeout);
}
//Write a callback function to inform about updates in the PHY */
void on_le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
	// PHY Updated
	if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_1M) {
		LOG_INF("PHY updated. New PHY: 1M");
	}
	else if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_2M) {
		LOG_INF("PHY updated. New PHY: 2M");
	}
	else if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_CODED_S8) {
		LOG_INF("PHY updated. New PHY: Long Range");
	}
}
//Write a callback function to inform about updates in data length */
void on_le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
	uint16_t tx_len     = info->tx_max_len; 
	uint16_t tx_time    = info->tx_max_time;
	uint16_t rx_len     = info->rx_max_len;
	uint16_t rx_time    = info->rx_max_time;
	LOG_INF("Data length updated. Length %d/%d bytes, time %d/%d us", tx_len, rx_len, tx_time, rx_time);
}


//Tracking the state of the connection
struct bt_conn_cb connection_callbacks = {
	.connected = connected,  // callback function for when a connection is established
	.disconnected = disconnected, //callback function for when a connection is closed
	//.recycled     = on_recycled, //Esta en el curso, Cuando una conexión deja de usarse y se devuelve al grupo de conexiones, el sistema invoca on_recycled() para notificarte que puedes volver a aceptar nuevas conexiones.
	//Es útil, por ejemplo, en dispositivos que sólo aceptan una conexión a la vez: cuando se libera una, puedes volver a estar disponible para otros.
	.le_param_updated   = on_le_param_updated,
	.le_phy_updated     = on_le_phy_updated,
	.le_data_len_updated    = on_le_data_len_updated,
};

int seq_pkt = 0;
uint8_t buffer[6]; //Buffer to store the data from the sensor
uint8_t buffer_QUAT[8]; //Buffer to store the data from the sensor in quaternion format
uint8_t bufferTotal[10];

//Handler of k_work
void quat_work_handler(struct k_work *work) {
    if(get_IMUsensor_notify_enabled()){
		//get_euler(buffer);
		get_quaternion(buffer_QUAT, sizeof(buffer_QUAT));

		bufferTotal[0] = (uint8_t)(seq_pkt & 0xFF);        // LSB
		bufferTotal[1] = (uint8_t)((seq_pkt >> 8) & 0xFF); // MSB
		// Copiar quaternion
		memcpy(&bufferTotal[2], buffer_QUAT, sizeof(buffer_QUAT));
		//send_IMUsensor_notify(buffer, sizeof(buffer));
		send_IMUsensor_notify(bufferTotal, sizeof(bufferTotal));
		seq_pkt++;
	}
}
//k_work dfinition
K_WORK_DELAYABLE_DEFINE(quat_work, quat_work_handler);

//Handler of timer
void quat_timer_handler(struct k_timer *timer_id){	
	k_work_schedule(&quat_work, K_NO_WAIT);
}
// Timer definition
K_TIMER_DEFINE(quat_timer, quat_timer_handler, NULL);	

int main(void)
{
    int err;  // error variable

    // Initialize the GPIO device
    if (!gpio_is_ready_dt(&ledR)) {
		return 0;
	}
    err = gpio_pin_configure_dt(&ledR, GPIO_OUTPUT_ACTIVE); //Config GPIO pin as output
	if (err < 0) {
	        return 0;
	}
    err = gpio_pin_configure_dt(&ledG, GPIO_OUTPUT_ACTIVE); //Config GPIO pin as output
	if (err < 0) {
	        return 0;
	}
    gpio_pin_set_dt(&ledG, 0);

    //Activate Bluetooth LE stack in the application.
    err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return -1;
	}
    LOG_INF("Bluetooth initialized\n");
        
    //Register the callbacks for the connection
    bt_conn_cb_register(&connection_callbacks);
        
    // Start advertising
	if(!my_conn){
    	err = bt_le_adv_start(advert_param, advert_data, ARRAY_SIZE(advert_data), scan_rsp_data, ARRAY_SIZE(scan_rsp_data));
	}
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)\n", err);
		return -1;
	}
	LOG_INF("Advertising successfully started\n");


	//Initialize the BNO055 sensor
	bno055_init();
	//Select the IMU mode
	uint8_t mode = OPERATION_MODE_NDOF;
	select_mode(mode);

	// Inicia el temporizador con un intervalo de 16.67 ms (~60 Hz)
    k_timer_start(&quat_timer, K_USEC(96000), K_USEC(96000));

    while (1) {
		err = gpio_pin_toggle_dt(&ledR);
		if (err < 0) {
			return 0;
		}
		//LOG_INF("CICLO PRINCIPAL*****************************************************************");
		//Send the notification and increment the value only if the notification is enabled
		// if(get_IMUsensor_notify_enabled()){
		// 	//get_euler(buffer);
		// 	get_quaternion(buffer_QUAT, sizeof(buffer_QUAT));

		// 	//send_IMUsensor_notify(buffer, sizeof(buffer));
		// 	send_IMUsensor_notify(buffer_QUAT, sizeof(buffer_QUAT));
        //     seq_pkt++;
		// }

		k_msleep(1000);
		//k_msleep(BNO055_SAMPLERATE_DELAY_MS);
		//k_sleep(K_USEC(16667));
		//k_sleep(K_USEC(100000));
		//K_USEC(16667); //MICROSEGUNDOS para temporizadores
		//K_NO_WAIT
	}
	return 0;

}
