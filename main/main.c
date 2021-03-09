/*
 * Brain Ring App
 * in progress
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "nvs_flash.h"
#include "mdns.h"
#include "lwip/apps/netbiosns.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "esp_vfs.h"
#include "driver/rmt.h"
#include "led_strip.h"

#define MDNS_INSTANCE "esp home web server"
#define SCRATCH_BUFSIZE (1024)
#define RMT_TX_GPIO (18)
#define N_PIXELS (5)
#define REF_TASK_PRIORITY (3)
#define CHASE_SPEED_MS (2000)
static const char *TAG = "mesh_main";
static const char *TEST_TAG = "mesh_numbers_producer";
static const char *REST_TAG = "esp-rest";
static bool should_generate = true;
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};
static bool is_mesh_connected = false;
static int mesh_layer = -1;
static mesh_addr_t mesh_parent_addr;
static esp_netif_t *netif_sta = NULL;

static SemaphoreHandle_t sync_led_task;

static QueueHandle_t numbers;

httpd_handle_t start_webserver(void);



static void numbers_producer(){
	ESP_LOGI(TEST_TAG, "task started");
	uint8_t counter = 0;
	while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        if(!should_generate) continue;
		if (counter == 100) counter = 0;
		ESP_LOGI(TEST_TAG, "value %d sent", counter);
		xQueueSend(numbers, &counter, portMAX_DELAY);
		counter++;
	}
}

static void initialise_mdns(void)
{
    mdns_init();
    mdns_hostname_set("esp-home");
    mdns_instance_name_set(MDNS_INSTANCE);

    mdns_txt_item_t serviceTxtData[] = {
        {"board", "esp32"},
        {"path", "/"}
    };

    ESP_ERROR_CHECK(mdns_service_add("ESP32-WebServer", "_http", "_tcp", 80, serviceTxtData,
                                     sizeof(serviceTxtData) / sizeof(serviceTxtData[0])));
}

static void esp_mesh_p2p_tx_main(void *arg) {
	uint8_t val = 0;
    esp_err_t err;
    mesh_addr_t route_table[10];
    mesh_data_t data;
    data.size = sizeof(uint8_t);
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;

    int size = 0;
	while (1) {
		xQueueReceive(numbers, &val, portMAX_DELAY);
		ESP_LOGI(TAG,"value to send to peers: %d", val);
		data.data = &val;
		esp_mesh_get_routing_table(route_table, 60, &size);
        for (int i = 0; i < size; i++) {
        	err = esp_mesh_send(&route_table[i], &data, MESH_DATA_P2P, NULL, 0);
        	if(err != ESP_OK) ESP_LOGI(TAG, "esp_mesh_send returned with error code %d", err);
        }
	}
}

void esp_mesh_comm_p2p_start(){
    xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 3072, NULL, 5, NULL);
}

void ip_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		start_webserver();
	}
}

void mesh_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    mesh_addr_t id = {0,};
    static uint16_t last_layer = 0;

    switch (event_id) {
    case MESH_EVENT_STARTED: {
        esp_mesh_get_id(&id);
        ESP_LOGI(TAG, "<MESH_EVENT_MESH_STARTED>ID:"MACSTR"", MAC2STR(id.addr));
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_STOPPED: {
        ESP_LOGI(TAG, "<MESH_EVENT_STOPPED>");
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *child_connected = (mesh_event_child_connected_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_CHILD_CONNECTED>aid:%d, "MACSTR"", child_connected->aid, MAC2STR(child_connected->mac));
    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child_disconnected = (mesh_event_child_disconnected_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, "MACSTR"",
                 child_disconnected->aid,
                 MAC2STR(child_disconnected->mac));
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_ADD: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(TAG, "<MESH_EVENT_ROUTING_TABLE_ADD>add %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(TAG, "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_NO_PARENT_FOUND: {
        mesh_event_no_parent_found_t *no_parent = (mesh_event_no_parent_found_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d",
                 no_parent->scan_times);
    }
    /* TODO handler for the failure */
    break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *connected = (mesh_event_connected_t *)event_data;
        esp_mesh_get_id(&id);
        mesh_layer = connected->self_layer;
        memcpy(&mesh_parent_addr.addr, connected->connected.bssid, 6);
        ESP_LOGI(TAG,
                 "<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, parent:"MACSTR"%s, ID:"MACSTR", duty:%d",
                 last_layer, mesh_layer, MAC2STR(mesh_parent_addr.addr),
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "", MAC2STR(id.addr), connected->duty);
        last_layer = mesh_layer;
        is_mesh_connected = true;
        esp_netif_dhcpc_start(netif_sta);
        esp_mesh_comm_p2p_start();
    }
    break;
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)event_data;
        ESP_LOGI(TAG,
                 "<MESH_EVENT_PARENT_DISCONNECTED>reason:%d",
                 disconnected->reason);
        is_mesh_connected = false;
        //mesh_disconnected_indicator();
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *layer_change = (mesh_event_layer_change_t *)event_data;
        mesh_layer = layer_change->new_layer;
        ESP_LOGI(TAG, "<MESH_EVENT_LAYER_CHANGE>layer:%d-->%d%s",
                 last_layer, mesh_layer,
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "");
        last_layer = mesh_layer;
        //mesh_connected_indicator(mesh_layer);
    }
    break;
    case MESH_EVENT_ROOT_ADDRESS: {
        mesh_event_root_address_t *root_addr = (mesh_event_root_address_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_ROOT_ADDRESS>root address:"MACSTR"",
                 MAC2STR(root_addr->addr));
    }
    break;
    case MESH_EVENT_VOTE_STARTED: {
        mesh_event_vote_started_t *vote_started = (mesh_event_vote_started_t *)event_data;
        ESP_LOGI(TAG,
                 "<MESH_EVENT_VOTE_STARTED>attempts:%d, reason:%d, rc_addr:"MACSTR"",
                 vote_started->attempts,
                 vote_started->reason,
                 MAC2STR(vote_started->rc_addr.addr));
    }
    break;
    case MESH_EVENT_VOTE_STOPPED: {
        ESP_LOGI(TAG, "<MESH_EVENT_VOTE_STOPPED>");
        break;
    }
    case MESH_EVENT_ROOT_SWITCH_REQ: {
        mesh_event_root_switch_req_t *switch_req = (mesh_event_root_switch_req_t *)event_data;
        ESP_LOGI(TAG,
                 "<MESH_EVENT_ROOT_SWITCH_REQ>reason:%d, rc_addr:"MACSTR"",
                 switch_req->reason,
                 MAC2STR( switch_req->rc_addr.addr));
    }
    break;
    case MESH_EVENT_ROOT_SWITCH_ACK: {
        /* new root */
        mesh_layer = esp_mesh_get_layer();
        esp_mesh_get_parent_bssid(&mesh_parent_addr);
        ESP_LOGI(TAG, "<MESH_EVENT_ROOT_SWITCH_ACK>layer:%d, parent:"MACSTR"", mesh_layer, MAC2STR(mesh_parent_addr.addr));
    }
    break;
    case MESH_EVENT_TODS_STATE: {
        mesh_event_toDS_state_t *toDs_state = (mesh_event_toDS_state_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_TODS_REACHABLE>state:%d", *toDs_state);
    }
    break;
    case MESH_EVENT_ROOT_FIXED: {
        mesh_event_root_fixed_t *root_fixed = (mesh_event_root_fixed_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_ROOT_FIXED>%s",
                 root_fixed->is_fixed ? "fixed" : "not fixed");
    }
    break;
    case MESH_EVENT_ROOT_ASKED_YIELD: {
        mesh_event_root_conflict_t *root_conflict = (mesh_event_root_conflict_t *)event_data;
        ESP_LOGI(TAG,
                 "<MESH_EVENT_ROOT_ASKED_YIELD>"MACSTR", rssi:%d, capacity:%d",
                 MAC2STR(root_conflict->addr),
                 root_conflict->rssi,
                 root_conflict->capacity);
    }
    break;
    case MESH_EVENT_CHANNEL_SWITCH: {
        mesh_event_channel_switch_t *channel_switch = (mesh_event_channel_switch_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_CHANNEL_SWITCH>new channel:%d", channel_switch->channel);
    }
    break;
    case MESH_EVENT_SCAN_DONE: {
        mesh_event_scan_done_t *scan_done = (mesh_event_scan_done_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_SCAN_DONE>number:%d",
                 scan_done->number);
    }
    break;
    case MESH_EVENT_NETWORK_STATE: {
        mesh_event_network_state_t *network_state = (mesh_event_network_state_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_NETWORK_STATE>is_rootless:%d",
                 network_state->is_rootless);
    }
    break;
    case MESH_EVENT_STOP_RECONNECTION: {
        ESP_LOGI(TAG, "<MESH_EVENT_STOP_RECONNECTION>");
    }
    break;
    case MESH_EVENT_FIND_NETWORK: {
        mesh_event_find_network_t *find_network = (mesh_event_find_network_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_FIND_NETWORK>new channel:%d, router BSSID:"MACSTR"",
                 find_network->channel, MAC2STR(find_network->router_bssid));
    }
    break;
    case MESH_EVENT_ROUTER_SWITCH: {
        mesh_event_router_switch_t *router_switch = (mesh_event_router_switch_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_ROUTER_SWITCH>new router:%s, channel:%d, "MACSTR"",
                 router_switch->ssid, router_switch->channel, MAC2STR(router_switch->bssid));
    }
    break;
    case MESH_EVENT_PS_PARENT_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_PS_PARENT_DUTY>duty:%d", ps_duty->duty);
    }
    break;
    case MESH_EVENT_PS_CHILD_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_PS_CHILD_DUTY>cidx:%d, "MACSTR", duty:%d", ps_duty->child_connected.aid-1,
                MAC2STR(ps_duty->child_connected.mac), ps_duty->duty);
    }
    break;
    default:
        ESP_LOGI(TAG, "Unknown id:%d", event_id);
        break;
    }
}

void mesh_init(){

}

static esp_err_t get_handler(httpd_req_t *req) {
    ESP_LOGI(REST_TAG, "req_length = %d, uri = %s", req->content_len, req->uri);

	httpd_resp_sendstr(req, "Sweet as bro");
	return ESP_OK;
}

/* Simple handler for light brightness control */
static esp_err_t post_handler(httpd_req_t *req)
{
    char content[100];
    size_t recv_size = req->content_len;
    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {  /* 0 return value indicates connection closed */
		/* Check if timeout occurred */
		if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
			/* In case of timeout one can choose to retry calling
			 * httpd_req_recv(), but to keep it simple, here we
			 * respond with an HTTP 408 (Request Timeout) error */
			httpd_resp_send_408(req);
		}
		return ESP_FAIL;
	}
    content[ret] = '\0';

    cJSON *root = cJSON_Parse(content);
    int red = cJSON_GetObjectItem(root, "red")->valueint;
    int green = cJSON_GetObjectItem(root, "green")->valueint;
    int blue = cJSON_GetObjectItem(root, "blue")->valueint;
    ESP_LOGI(REST_TAG, "Light control: red = %d, green = %d, blue = %d", red, green, blue);
    cJSON_Delete(root);
    httpd_resp_sendstr(req, "Post control value successfully");
    return ESP_OK;
}

//Function for starting the webserver
httpd_handle_t start_webserver(void)
{
    ESP_LOGI(REST_TAG, "Starting HTTP Server");
    //Generate default configuration
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    //Empty handle to esp_http_server
    httpd_handle_t server = NULL;


    /* Start the httpd server */
    if (httpd_start(&server, &config) == ESP_OK) {

    	httpd_uri_t uri_get = {
    		.uri = "/data",
    		.method = HTTP_GET,
    		.handler = get_handler,
    		.user_ctx = NULL
    	};

    	httpd_uri_t uri_post = {
    		.uri = "/hue",
    		.method = HTTP_POST,
    		.handler = post_handler,
    		.user_ctx = NULL
    	};
        /* Register URI handlers */
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_post);
    }
    /* If server failed to start, handle will be NULL */
    return server;
}

void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

/**
 * @brief   Task to drive led strip.
*/

static void led_task(void *arg)
{
	xSemaphoreTake(sync_led_task, portMAX_DELAY);
	uint32_t red = 0;
	uint32_t green = 0;
	uint32_t blue = 0;
	uint16_t hue = 0;
	uint16_t start_rgb = 0;

	//  Install RMT driver
	ESP_LOGI(TAG, "RMT installation..");
	rmt_config_t rmt_cfg = RMT_DEFAULT_CONFIG_TX(RMT_TX_GPIO, RMT_CHANNEL_0);
	// Decrease counter clock twice
	rmt_cfg.clk_div = 2;
	ESP_ERROR_CHECK( rmt_config( &rmt_cfg ) );
	ESP_ERROR_CHECK( rmt_driver_install( rmt_cfg.channel, 0, 0 ) );
	ESP_LOGI(TAG, "RMT driver installed");


	// Install led strip driver
	ESP_LOGI(TAG, "LED strip driver installation..");
	led_strip_config_t strip_cfg = LED_STRIP_DEFAULT_CONFIG( N_PIXELS, (led_strip_dev_t)rmt_cfg.channel);
	// Instantiate strip
	led_strip_t *strip = led_strip_new_rmt_sk6812(&strip_cfg);
	if (!strip) {
        ESP_LOGE(TAG, "install sk6812 driver failed");
        vTaskDelete(NULL);
	}

	// Clear LED strip (turn off all LEDs)
	ESP_ERROR_CHECK(strip->clear(strip, 100));
	// Show simple rainbow chasing pattern
	ESP_LOGI(TAG, "LED Rainbow Chase Start");
	while (true) {
		for (int i = 0; i < 3; i++) {
			for (int j = i; j < N_PIXELS; j += 3) {
				// Build RGB values
				hue = j * 360 / N_PIXELS + start_rgb;
				led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
				// Write RGB values to strip driver
				ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue, 0));
			}
			// Flush RGB values to LEDs
			ESP_ERROR_CHECK(strip->refresh(strip, 100));
			vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS));
			strip->clear(strip, 50);
			vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS));
		}
		start_rgb += 60;
	}
	ESP_ERROR_CHECK( rmt_driver_uninstall( rmt_cfg.channel));
	ESP_LOGI(TAG, "RMT driver un-installed");
}

void app_main(void)
{
	//Allow the second core to finish initialization
	vTaskDelay(pdMS_TO_TICKS(100));
	//NVS initialization
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	  ESP_ERROR_CHECK(nvs_flash_erase());
	  ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	//Queue memory allocation for number producer task
	numbers = xQueueCreate(10, sizeof(uint8_t));
	xTaskCreate(numbers_producer, "PRODUCER1", 3072, NULL, 5, NULL);

    //TCP/IP initialization
	ESP_ERROR_CHECK(esp_netif_init());
	//Event Loop initialization
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	//mDNS initialization
	initialise_mdns();
	netbiosns_init();
	netbiosns_set_name("esp-home");

	//Create network interfaces for mesh (only station instance saved for further manipulation, soft AP instance ignored */
	ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));
	//WiFi initialization
	wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
	wifi_config_t wifi_config = {
		.sta = {
			.ssid = CONFIG_MESH_ROUTER_SSID,
			.password = CONFIG_MESH_ROUTER_PASSWD,
			.channel = 0
		},
	};
	ESP_ERROR_CHECK(esp_wifi_init(&config));
	//IP Events handler registration
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
														&ip_event_handler, NULL, NULL));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
	ESP_ERROR_CHECK(esp_wifi_start());

	//Mesh initialization
	ESP_ERROR_CHECK(esp_mesh_init());
	//Mesh Events handler registration
    ESP_ERROR_CHECK(esp_event_handler_instance_register(MESH_EVENT, ESP_EVENT_ANY_ID,
    											&mesh_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_mesh_set_topology(CONFIG_MESH_TOPOLOGY));
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));
    ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));
    //ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1.0));
    /* Disable mesh PS function */
    ESP_ERROR_CHECK(esp_mesh_disable_ps());
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));
    mesh_cfg_t mesh_cfg = MESH_INIT_CONFIG_DEFAULT();
    /* mesh ID */
	memcpy((uint8_t *) &mesh_cfg.mesh_id, MESH_ID, 6);
	/* router */
	mesh_cfg.channel = CONFIG_MESH_CHANNEL;

	uint8_t router_ssid_len = strlen(CONFIG_MESH_ROUTER_SSID);
	mesh_cfg.router.ssid_len = router_ssid_len;
	memcpy((uint8_t *) &mesh_cfg.router.ssid, CONFIG_MESH_ROUTER_SSID, router_ssid_len);
	memcpy((uint8_t *) &mesh_cfg.router.password, CONFIG_MESH_ROUTER_PASSWD, strlen(CONFIG_MESH_ROUTER_PASSWD));

	/* mesh softAP */
	ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
	mesh_cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
	memcpy((uint8_t *) &mesh_cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD, strlen(CONFIG_MESH_AP_PASSWD));
	ESP_ERROR_CHECK(esp_mesh_set_config(&mesh_cfg));
	/* mesh start */
	ESP_ERROR_CHECK(esp_mesh_set_self_organized(false, false));
	//ESP_ERROR_CHECK(esp_mesh_fix_root(true));
	ESP_ERROR_CHECK(esp_mesh_set_parent(&wifi_config, NULL, MESH_ROOT, MESH_ROOT_LAYER));
	ESP_ERROR_CHECK(esp_mesh_start());

    ESP_LOGI(TAG, "mesh starts successfully, heap:%d, %s<%d>%s, ps:%d\n",  esp_get_minimum_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed",
             esp_mesh_get_topology(), esp_mesh_get_topology() ? "(chain)":"(tree)", esp_mesh_is_ps_enabled());

	//Create semaphores to synchronize
	sync_led_task = xSemaphoreCreateBinary();
	//Create and start led driver task
	xTaskCreatePinnedToCore(led_task, "led_task", 4096, NULL, 3, NULL, 1);
	xSemaphoreGive(sync_led_task);
}
