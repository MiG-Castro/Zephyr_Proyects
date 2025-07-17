#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/gpio.h>
// Include the custom IMU BLE service
#include "my_imu_ble_service.h"

LOG_MODULE_REGISTER(IMU_server, LOG_LEVEL_INF);

// LED configuration
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
static const struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec grn_led = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec blu_led = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

/******************************************************************************************
					Structure and definitions for the Bluetooth LE
******************************************************************************************/
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME	// Device Bluetooth name defined in prj.conf
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
struct bt_conn *my_conn = NULL;

/***** Advertising parameters strcture *****/
// Small advertising interval for fast connection
static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	(BT_LE_ADV_OPT_CONN |			// Connectable advertising
	BT_LE_ADV_OPT_USE_IDENTITY), 	// Use identity address (Static random address)
	BT_GAP_ADV_FAST_INT_MIN_1,		/* Min Advertising Interval = Value x 0.625ms = 30ms */
	BT_GAP_ADV_FAST_INT_MAX_1,		/* Max Advertising Interval = Value x 0.625ms = 60ms */
	NULL); /* Address of peer, Set to NULL for undirected advertising */

// Definition of advertising data
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL |	// General advertising
		BT_LE_AD_NO_BREDR)),							// No compatible with BR/EDR (Bluetooth Classic)
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN), // Use complete name
};

// Scan Response -> UUID of LBS Service
static const struct bt_data sd[] = {BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_IMUS_VAL),};

// Structure and definition for change parameters of the connection
static struct bt_gatt_exchange_params exchange_params;	// variable that holds callback for MTU negotiation

// Work-queue parameters for advertising
#define STACKSIZE 1024				// Stack size for the thread SendData
#define PRIORITY 7					// Thread priority for the thread SendData
static struct k_work adv_work;		// Work item for asynchronous advertising

// Flags to know the status "update" of: PHY, Data Length and Connection Parameters
static bool tryUp_PHY = false, tryUp_DataLen = false, tryUp_ConnP = false, connect = false;
int8_t try_connP = 0, triesUp = 4; //, try_PHY = 0, 

/******************************************************************************************
					Functions
******************************************************************************************/

// Thread for sending data periodically
void send_data_thread(void)
{
	// Delay to wait advertisign-connection-suscription-update processes
	k_sleep(K_MSEC(4000));										
	uint8_t app_sensor_value[6] = {0, 1, 2, 3, 4, 5};	// Variable to hold the sensor value
	uint16_t send_bytes = 4;							// Number of bytes to send
	while (1) {
		if (tryUp_ConnP && tryUp_PHY && tryUp_DataLen && connect) {
			if (get_imus_sensordata_notify_enabled()) {
				my_imus_sensordata_notify(app_sensor_value, send_bytes);
				app_sensor_value[0]++;
				k_sleep(K_MSEC(500));
				// LOG_INF("NOTIFY");
			}
			if (get_imus_exercisedetection_indicate_enabled()) {
				my_imus_exercisedetection_indicate(app_sensor_value, send_bytes);
				app_sensor_value[0]++;
				k_sleep(K_MSEC(500));
				// LOG_INF("INDICATE");
			}
		}
		k_sleep(K_MSEC(500));
	}
}

// Start the advertisign in the work-queue
static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {LOG_ERR("Advertising failed to start (err %d)\n", err);return;}
	LOG_INF("Advertising successfully started");
}

// Function to start advertising sending the work item to the work-queue
static void advertising_start(void){k_work_submit(&adv_work);}

// Update the connection parameters
static void update_conn_params(struct bt_conn *conn)
{
    int err;
	struct bt_le_conn_param *conn_param = BT_LE_CONN_PARAM(
		CONFIG_DESIRED_MIN_CONN_INTERVAL,	// Min Connection Interval = Value x 1.25ms
		CONFIG_DESIRED_MAX_CONN_INTERVAL,	// Min Connection Interval = Value x 1.25ms
		CONFIG_BT_PERIPHERAL_PREF_LATENCY,	// Latency (Number of connection intervals the peripheral can skip)
		CONFIG_BT_PERIPHERAL_PREF_TIMEOUT	// Time that the central will wait before disconnecting
	);
    err = bt_conn_le_param_update(conn, conn_param);
    if (err) {LOG_ERR("Failed to request connection parameters update (err %d)", err);}
	tryUp_ConnP = true; // Set the flag to true to indicate the try to update the Connection Parameters
}

// Update the connection PHY
static void update_phy(struct bt_conn *conn)
{
	int err;
	const struct bt_conn_le_phy_param preferred_phy = {
		.options = BT_CONN_LE_PHY_OPT_NONE,
		.pref_rx_phy = BT_GAP_LE_PHY_2M,
		.pref_tx_phy = BT_GAP_LE_PHY_2M,
	};
	err = bt_conn_le_phy_update(conn, &preferred_phy);
	if (err) {LOG_ERR("bt_conn_le_phy_update() returned %d", err);}
	tryUp_PHY = true; // Set the flag to true to indicate the try to update the PHY
}

// Update the connection data length
static void update_data_length(struct bt_conn *conn)
{
	int err;
	struct bt_conn_le_data_len_param my_data_len = {
		.tx_max_len = BT_GAP_DATA_LEN_MAX,
		.tx_max_time = BT_GAP_DATA_TIME_MAX,
	};
	err = bt_conn_le_data_len_update(my_conn, &my_data_len);
	if (err) {LOG_ERR("data_len_update failed (err %d)", err);}
	tryUp_DataLen = true; // Set the flag to true to indicate the try to update the data length
}

// Inform the MTU exchange
static void exchange_func(struct bt_conn *conn, uint8_t att_err, struct bt_gatt_exchange_params *params)
{
	LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");
	if (!att_err) {
		uint16_t payload_mtu = bt_gatt_get_mtu(conn) - 3;   // 3 bytes used for Attribute headers.
		LOG_INF("New MTU: %d bytes", payload_mtu);
	}
}

// Beginning the update of the connection's MTU
static void update_mtu(struct bt_conn *conn)
{
	int err;
	exchange_params.func = exchange_func;
	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
	}
}

/************************ Connection events callback functions ************************/
// Callback function for announcing the connection or error in the connection
static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {LOG_ERR("Connection failed (err %u)\n", err);return;}
	connect = true; // Set the connected flag to true
	LOG_INF("Connected!!!");
	LOG_INF("Reading intitial parameters...");

	my_conn = bt_conn_ref(conn);
	struct bt_conn_info info;
	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_ERR("bt_conn_get_info() returned %d", err);
		return;
	}

	// Connection parameters to the log
	double connection_interval = info.le.interval*1.25; // in ms
	uint16_t supervision_timeout = info.le.timeout*10; // in ms
	LOG_INF("- Connection parameters: interval %.2f ms, latency %d intervals, timeout %d ms", 
		connection_interval, info.le.latency, supervision_timeout);
	
	// Data Length parameters to the log
	LOG_INF("- Data length: TX %d bytes / %d us, RX %d bytes / %d us",
            info.le.data_len->tx_max_len, info.le.data_len->tx_max_time,
            info.le.data_len->rx_max_len, info.le.data_len->rx_max_time);

	// UPDATE the connection parameters
	LOG_INF("Beginning request for parameters update...");
	if (info.le.interval < CONFIG_DESIRED_MIN_CONN_INTERVAL &&
		info.le.interval > CONFIG_DESIRED_MAX_CONN_INTERVAL &&
		info.le.latency == CONFIG_BT_PERIPHERAL_PREF_LATENCY) {
			LOG_INF("Negociando conn params");
			update_conn_params(my_conn);
			k_sleep(K_MSEC(500));	// Small delay to not overload the central with requests.
		} else tryUp_ConnP = true;
	
	update_phy(my_conn);
	k_sleep(K_MSEC(1000));	// Delay to avoid link layer collisions.
	update_data_length(my_conn);
	update_mtu(my_conn);
}

//Callback function for announcing the disconnection
static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)\n", reason);
	bt_conn_unref(my_conn);
	// Reset the update flags
	tryUp_PHY = false;
	tryUp_DataLen = false;
	tryUp_ConnP = false;
	connect = false; // Reset the connected flag
	try_connP = 0;
	// try_PHY = 0;
}

// Callback function for when a connection object is recycled (restart advertising)
static void recycled_cb(void){LOG_INF("Disconnect complete. Restart advertising\n");advertising_start();}

// Callback to inform the connection parameter update
void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	double connection_interval = interval*1.25;         // in ms
	uint16_t supervision_timeout = timeout*10;          // in ms
	LOG_INF("Connection parameters updated: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval, latency, supervision_timeout);
	
	if (interval > CONFIG_DESIRED_MAX_CONN_INTERVAL && try_connP < triesUp) {
		try_connP++;
		update_conn_params(my_conn);
		LOG_INF("RETRYING connection parameters update (%d/%d)", try_connP, triesUp);
	}
}

// Callback to inform the update of PHY
void on_le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
	// PHY Updated
	if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_1M) {LOG_INF("PHY updated. New PHY: 1M");}
	else if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_2M) {LOG_INF("PHY updated. New PHY: 2M");}
	else if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_CODED_S8) {LOG_INF("PHY updated. New PHY: Long Range");}

	// RETRY TO UPDATE PHY (DON'T WORK)
	// if (param->tx_phy != BT_CONN_LE_TX_POWER_PHY_2M && try_PHY < 3) {
	// 	try_PHY++;
	// 	LOG_INF("RETRYING PHY update (%d/5)", try_PHY);
	// 	update_phy(my_conn);
	// }
}

// Callback to inform the update of data length
void on_le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
	uint16_t tx_len  = info->tx_max_len; 
	uint16_t tx_time = info->tx_max_time;
	uint16_t rx_len  = info->rx_max_len;
	uint16_t rx_time = info->rx_max_time;
	LOG_INF("Data length updated. Length %d/%d bytes, time %d/%d us", tx_len, rx_len, tx_time, rx_time);
}

// Callback structure for connections events
struct bt_conn_cb connection_callbacks = {
	.connected				= on_connected,
	.disconnected			= on_disconnected,
	.recycled				= recycled_cb,
	.le_param_updated		= on_le_param_updated,
	.le_phy_updated			= on_le_phy_updated,
	.le_data_len_updated    = on_le_data_len_updated,
};

// Callback function for receiving data from the Exercise Detection characteristic
static void app_exercise_detec_cb(const uint8_t *buf, uint16_t len)
{
    if (len == 0) {
        LOG_INF("Datos recibidos: (vacío)");
        return;
    }

    printk("Datos recibidos (len=%u): 0x", len);
    for (int i = 0; i < len; i++) {
        // %02X imprime un número hexadecimal con 2 dígitos, rellenando con 0 a la izquierda si es necesario.
        printk("%02X ", buf[i]);
    }
	printk("\n");
}

static struct my_imus_cb app_callbacks = {
	.exdetection_write_cb = app_exercise_detec_cb, 	// Exercise detection callback
};


// MAIN FUNCTION ********************************************************************
int main(void)
{
	printk("Starting Bluetooth IMU Service\n");
	int err;
	
	// Initialize LED
	if (!gpio_is_ready_dt(&red_led) && !gpio_is_ready_dt(&grn_led) && !gpio_is_ready_dt(&blu_led)) {
		LOG_ERR("Error led gpio\n");
		return -1;
	}
	gpio_pin_configure_dt(&red_led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&grn_led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&blu_led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set_dt(&red_led, 0);
	gpio_pin_set_dt(&grn_led, 0);
	gpio_pin_set_dt(&blu_led, 0);

	// Eneable the bluetooth stack
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return -1;
	}

	// Initialize the IMU service with the application callbacks
	err = my_imus_init(&app_callbacks);
	if (err) {
		LOG_ERR("Fail to init IMU Service (err %d)\n", err);
		return -1;
	}

	// Get the Bluetooth device address
	bt_addr_le_t addr;
	size_t count = 1;
	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_id_get(&addr, &count);
	bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
	LOG_INF("BLE Name=%s, Address=%s", CONFIG_BT_DEVICE_NAME, addr_str);

	// Register the connections events callbacks
	bt_conn_cb_register(&connection_callbacks);

	// Initialization of the work item for advertising
	k_work_init(&adv_work, adv_work_handler);
	// Start advertising using the the work-queue
	advertising_start();

	// Led blinking
	for (;;) {
		k_sleep(K_MSEC(2000)); 
		if (connect) gpio_pin_toggle_dt(&blu_led);
		else gpio_pin_set_dt(&blu_led, 0);
	}
}

// Define and initialize a thread to send data periodically
K_THREAD_DEFINE(send_data_thread_id, STACKSIZE, send_data_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
