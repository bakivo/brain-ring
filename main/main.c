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
#include "strips.h"
#include "utils.h"
#include "network.h"
#define MDNS_INSTANCE "esp neopixel http server: "
#define SCRATCH_BUFSIZE (1024)
#define RMT_TX_GPIO (16)
#define RMT_CHANNEL RMT_CHANNEL_0
#define INIT_PIXELS_NUM (10)
#define REF_TASK_PRIORITY (3)
#define CHASE_SPEED_MS (100)
#define CORE_1 (0)
#define CORE_2 (1)
#define MODE_TASK_PRIO (6)
#define MESH_TASK_PRIO (5)
#define CONTROL_TASK_PRIO (4)
#define DRIVER_TASK_PRIO (3)
#define HUE_TASK_PRIO (2)
static const int STRIP_TYPE = SK6812_RGBW;
static const char *TAG = "mesh_main";
static const char *REST_TAG = "esp-rest";
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};
static uint8_t mesh_own_mac[6];
static bool is_mesh_connected = false;
static int mesh_layer = -1;
static mesh_addr_t mesh_parent_addr;
static esp_netif_t *netif_sta = NULL;
static pixel_t pixel = {0,0,0,0};
static uint8_t nodes_at_level[CONFIG_MESH_MAX_LAYER];  

static SemaphoreHandle_t sync_control_task;
static SemaphoreHandle_t sync_mode_control_task;

static SemaphoreHandle_t sync_mesh_tx_task;
static SemaphoreHandle_t sync_mesh_rx_task;

static QueueHandle_t hsv_values_handle;
static QueueHandle_t rgb_values_handle;
static QueueHandle_t strip_modes_handle;
static QueueHandle_t led_task_input_handle;
static QueueHandle_t mesh_tx_input_handle;

static TaskHandle_t test_task1_hdl;
static SemaphoreHandle_t test1_task_sync;
static TaskHandle_t test_task2_hdl;
static SemaphoreHandle_t test2_task_sync;
static TaskHandle_t *test_tasks[] = {&test_task1_hdl, &test_task2_hdl};

static SemaphoreHandle_t sync_hue_task;
static SemaphoreHandle_t sync_rainbow_task;
static SemaphoreHandle_t sync_switch_color_task;
static SemaphoreHandle_t sync_color_task;

static TaskHandle_t hue_sim_task;
static TaskHandle_t rainbow_sim_task;
static TaskHandle_t color_switch_sim_task;
static TaskHandle_t one_color_sim_task;
static TaskHandle_t *control_tasks[] = {
    &hue_sim_task,
    &rainbow_sim_task,
    &color_switch_sim_task,
    &one_color_sim_task
    };

typedef struct {
	uint16_t hue;
	uint8_t saturation;
	uint8_t value;
} hsv_t;

typedef struct {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} color_t;

typedef enum {
	MODE_ROTATING_HUE,
	MODE_RAINBOW,
    MODE_SWITCH_COLOR,
	MODE_COLOR
} led_mode_t;
static int8_t led_mode;
#define DEFAULT_LED_MODE (MODE_RAINBOW)

httpd_handle_t start_webserver(void);

static void initialise_mdns(void)
{
    mdns_init();
    mdns_hostname_set("esp-home");
    mdns_instance_name_set(MDNS_INSTANCE);

    mdns_txt_item_t serviceTxtData[] = {
        {"board", "esp32"},
        {"path", "/"}
    };

    ESP_ERROR_CHECK(mdns_service_add("ESP32-Mesh-root", "_http", "_tcp", 80, serviceTxtData,
                                     sizeof(serviceTxtData) / sizeof(serviceTxtData[0])));
}

static void esp_mesh_p2p_tx_main(void *arg) {
	xSemaphoreTake(sync_mesh_tx_task, portMAX_DELAY);
    uint8_t mac[6];
    esp_err_t err;
    color_t color;
    int size;

    mesh_addr_t route_table[10];
    mesh_data_t data;
    data.size = sizeof(color_t);
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;
    
	while (1) {
		xQueueReceive(mesh_tx_input_handle, &color, portMAX_DELAY);
		data.data = (uint8_t*)&color;
		esp_mesh_get_routing_table(route_table, 60, &size);
        for (int i = 0; i < size; i++) {
            memcpy(mac, &route_table[i].addr, 6);
            ESP_LOGI(TAG, "<MESH_MAC>: %d element in routing table:, "MACSTR"", i, MAC2STR(mac));

        	err = esp_mesh_send(&route_table[i], &data, MESH_DATA_P2P, NULL, 0);
        	if(err != ESP_OK) ESP_LOGI(TAG, "esp_mesh_send returned with error code %d", err);
        }
	}
}

static void esp_mesh_p2p_rx_main(void *args){
	xSemaphoreTake(sync_mesh_rx_task, portMAX_DELAY);
	esp_err_t err = 0;
	int flag = -1;
    mesh_data_t data;
    mesh_addr_t from;
    packet_t packet;
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;
    data.size = sizeof(packet_t);
	data.data = (packet_t*)&packet;

	while (1) {
		err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
		if (err != ESP_OK) {
			ESP_LOGI(TAG, "esp_mesh_recv returned with error code %d ; data size = %d", err, data.size);
		} else {
			ESP_LOGI(TAG, "RX: message type %d, received from "MACSTR", size:%d", packet.type, MAC2STR(from.addr), data.size);
            switch (packet.type)
            {
            case MESH_MESSAGE:
                ESP_LOGI(TAG,"%s", packet.message);
                break;
            
            case MESH_TREE_CHILD_ADDED:
			ESP_LOGI(TAG, ""MACSTR":"MACSTR":%d", MAC2STR(packet.child_info.mac), MAC2STR(packet.child_info.parent_mac),packet.child_info.level);
                break;
            
            case MESH_USER_DATA:
                ESP_LOGI(TAG,"USER PARAM: %d", packet.param1);
                break;
            
            default:
                ESP_LOGI(TAG,"unknown type of message received from Mesh");
                break;
            }
            // ************ code ************
		}
	}
}

void esp_mesh_comm_p2p_start(){
	ESP_LOGI(TAG, "MESH P2P communication started");
	sync_mesh_tx_task = xSemaphoreCreateBinary();
	sync_mesh_rx_task = xSemaphoreCreateBinary();

	xTaskCreatePinnedToCore(esp_mesh_p2p_tx_main, "MPTX", 8192, NULL, MESH_TASK_PRIO, NULL, CORE_1);
	xSemaphoreGive(sync_mesh_tx_task);
	xTaskCreatePinnedToCore(esp_mesh_p2p_rx_main, "MPRX", 8192, NULL, MESH_TASK_PRIO, NULL, CORE_1);
	xSemaphoreGive(sync_mesh_rx_task);


}

void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	switch (event_id) {
	case WIFI_EVENT_AP_PROBEREQRECVED: {
		//mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
		ESP_LOGI(TAG, "<WIFI_EVENT_AP_PROBEREQRECVED>");
	    }
	break;

	default:
		ESP_LOGI(TAG, "<WIFI_EVENT>: Unknown id:%d", event_id);
		break;
	}

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
        // to pass info about new child to graph API function, the self mac and layer to be added in addition to child'd mac
        // invoke esp_mesh_get_layer();
        if (esp_mesh_is_root())
        {
            /* code */
        }
        



    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child_disconnected = (mesh_event_child_disconnected_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, "MACSTR"", child_disconnected->aid, MAC2STR(child_disconnected->mac));
        // to pass info about lost child to graph API function, the self mac and layer to be added in addition to child'd mac

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

/* Simple handler for POST request containing JSON with Hue, Saturation and Value values */
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
    hsv_t hsv_values;
    hsv_values.hue = cJSON_GetObjectItem(root, "hue")->valueint;
    hsv_values.saturation = cJSON_GetObjectItem(root, "saturation")->valueint;
    hsv_values.value = cJSON_GetObjectItem(root, "value")->valueint;
    ESP_LOGI(REST_TAG, "Light control: hue = %d, saturation = %d, value = %d", hsv_values.hue, hsv_values.saturation, hsv_values.value);
    cJSON_Delete(root);
    httpd_resp_sendstr(req, "Post control value successfully");
	xQueueSend(hsv_values_handle, &hsv_values, portMAX_DELAY);
    return ESP_OK;
}

/* Simple handler for POST request containing Red, Green and Blue components of color */
static esp_err_t rgb_post_handler(httpd_req_t *req)
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
    color_t color;
    color.red = cJSON_GetObjectItem(root, "red")->valueint;
    color.green = cJSON_GetObjectItem(root, "green")->valueint;
    color.blue = cJSON_GetObjectItem(root, "blue")->valueint;
    ESP_LOGI(REST_TAG, "obtained from http POST: red = %d, green = %d, blue = %d", color.red, color.green, color.blue);
    cJSON_Delete(root);
    httpd_resp_sendstr(req, "Post control value successfully");
	xQueueSend(rgb_values_handle, &color, portMAX_DELAY);
    return ESP_OK;
}
// strip's mode http post method handler
static esp_err_t set_mode_post_handler(httpd_req_t *req)
{
    ESP_LOGI(REST_TAG, "req_length = %d, uri = %s", req->content_len, req->uri);
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

    uint8_t mode = cJSON_GetObjectItem(root, "mode")->valueint;
    ESP_LOGI(REST_TAG, "obtained from set mode http POST: %d", mode);
    cJSON_Delete(root);
    httpd_resp_sendstr(req, "Post control value successfully");
	xQueueSend(strip_modes_handle, &mode, portMAX_DELAY);
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

    	httpd_uri_t uri_post1 = {
    		.uri = "/hue",
    		.method = HTTP_POST,
    		.handler = post_handler,
    		.user_ctx = NULL
    	};

    	httpd_uri_t uri_post2 = {
    	    		.uri = "/rgb",
    	    		.method = HTTP_POST,
    	    		.handler = rgb_post_handler,
    	    		.user_ctx = NULL
    	};

    	httpd_uri_t uri_post3 = {
    	    		.uri = "/mode",
    	    		.method = HTTP_POST,
    	    		.handler = set_mode_post_handler,
    	    		.user_ctx = NULL
    	};

        /* Register URI handlers */
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_post1);
        httpd_register_uri_handler(server, &uri_post2);
        httpd_register_uri_handler(server, &uri_post3);

    }
    /* If server failed to start, handle will be NULL */
    return server;
}

/**
 * @brief   Task to drive led strip.
*/

static void led_task(void *arg)
{
	xSemaphoreTake(sync_color_task, portMAX_DELAY);
	ESP_LOGI(TAG, "LED One color task started");
    // this code populates all strip long with a color.
	color_t color;
	while(1) {
		xQueueReceive(led_task_input_handle, &color, portMAX_DELAY);
		//clear_strip();
		pixel.r = color.red;
		pixel.g = color.green;
		pixel.b = color.blue;
		ESP_LOGI(TAG,"led colors to flush: %d %d %d", pixel.r, pixel.g, pixel.b);
		for (int i = 0; i < INIT_PIXELS_NUM; i++){
			ESP_ERROR_CHECK( set_pixel(i, &pixel) );
		}
		refresh_strip();
	}
}
// TODO brightness control and speed
static void rotating_hue(void *arg) {
	xSemaphoreTake( sync_hue_task, portMAX_DELAY);
    ESP_LOGI(TAG, "LED Rotating Colors Start");
	pixel_t pixel = { .white = 0 };
	//uint16_t hue = (uint16_t)arg;
	uint16_t hue = 180;
	int8_t delta = 1;
	while ( true ) {
		hue = hue + delta;
		if( hue == 360 ) delta = -1;
		if( hue == 180 ) delta = 1;
		hsv2rgb(hue, 100, 10, &pixel.r, &pixel.g, &pixel.b);
        for (int i = 0; i < INIT_PIXELS_NUM; i++){
			ESP_ERROR_CHECK( set_pixel(i, &pixel) );
		}
		refresh_strip();
		//xQueueSend(rgb_values_handle, &color, portMAX_DELAY);
		vTaskDelay( pdMS_TO_TICKS(1000) );
	}
}

static void switching_blue_red(void *arg) {
	xSemaphoreTake( sync_switch_color_task, portMAX_DELAY);
    ESP_LOGI(TAG, "LED switching color Start");

	color_t color;
	uint16_t hue;
	bool alter = false;
	while(1) {
		if (alter) {
			hue = 210;
		}
		else {
			hue = 360;
		}
		alter = !alter;
		hsv2rgb(hue, 100, 10, &color.red, &color.green, &color.blue);
		xQueueSend(rgb_values_handle, &color, portMAX_DELAY);
		vTaskDelay( pdMS_TO_TICKS(4000) );
	}
}

static void rainbow_task(void *args) {
    xSemaphoreTake(sync_rainbow_task, portMAX_DELAY);
    ESP_LOGI(TAG, "LED Rainbow Chase Start");

    uint8_t shift = 0;
    pixel_t pixel = { .white = 0 };
    uint16_t hue = 0;
    while (true)
    {
        for (int i = 0; i < 3; i++) {
            for (int j = i; j < INIT_PIXELS_NUM; j += 3) {
                // Build RGB values
                hue = j * 360 / INIT_PIXELS_NUM + shift;
                hsv2rgb(hue, 100, 30, &pixel.r, &pixel.g, &pixel.b);
                // Write RGB values to strip driver
			    ESP_ERROR_CHECK( set_pixel(j, &pixel) );
            }
            // Flush RGB values to LEDs
		    refresh_strip();
            vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS));
		    clear_strip();
            vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS));
        }
        shift += 60;
    }
    
}

static void test_task1(void *arg) {
    ESP_LOGI(TAG, "test task1 started");
    while (true)
    {
        ESP_LOGI(TAG, "test task1 iter");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }   
}
static void test_task2(void *arg) {
    ESP_LOGI(TAG, "test task2 started");
    while (true)
    {
        ESP_LOGI(TAG, "test task2 iter");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }   
}

esp_err_t switch_led_mode(led_mode_t mode) {
    TaskHandle_t *task_ptr;
    char name[20];
    // delete current strip mode task
    for (int i = 0; i < sizeof(control_tasks)/sizeof(control_tasks[0]); i++) {
        task_ptr = control_tasks[i];
        if (*task_ptr == NULL) continue;
        memcpy(name, pcTaskGetName(*task_ptr), sizeof(name));
        ESP_LOGI(TAG, "task to close %s", name);
        vTaskDelete( *task_ptr );
        *task_ptr = NULL;
    }
    // activate new mode
    switch (mode)
    {
    case MODE_ROTATING_HUE:
        xTaskCreatePinnedToCore( rotating_hue, "rotating hue task", 4096, NULL, 2, &hue_sim_task, CORE_1);
        xSemaphoreGive( sync_hue_task );
        break;
    case MODE_RAINBOW:
        xTaskCreatePinnedToCore( rainbow_task, "rainbow_task", 4096, NULL, 2, &rainbow_sim_task, CORE_1);
        xSemaphoreGive( sync_rainbow_task );
        break;
    case MODE_SWITCH_COLOR:
        xTaskCreatePinnedToCore( switching_blue_red, "switching blue red task", 4096, NULL, 2, &color_switch_sim_task, CORE_1);
        xSemaphoreGive( sync_switch_color_task );
        break;
    case MODE_COLOR:
        xTaskCreatePinnedToCore( led_task, "one color task", 4096, NULL, 2, &one_color_sim_task, CORE_1);
        xSemaphoreGive( sync_color_task );
        break;

    default:
        ESP_LOGI(TAG, "unknown strip mode: %d", mode);
        break;
    }
    return 0;
}

static void mode_control_task(void *arg) {
	xSemaphoreTake(sync_mode_control_task, portMAX_DELAY);
    ESP_LOGI(TAG, "mode task started");
    switch_led_mode(DEFAULT_LED_MODE);
	uint8_t mode = 0;
	while (true) {
		xQueueReceive(strip_modes_handle, &mode, portMAX_DELAY);
        switch_led_mode(mode);
	}
}

static void control_task(void *arg)
{
	xSemaphoreTake(sync_control_task, portMAX_DELAY);
    //  Install RMT driver
	rmt_config_t config = RMT_DEFAULT_CONFIG_TX(RMT_TX_GPIO, RMT_CHANNEL);
	// set counter clock to 40MHz
	config.clk_div = 2;
	ESP_ERROR_CHECK(rmt_config(&config));
	ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
	// Install led strip driver
	ESP_ERROR_CHECK(set_new_strip(RMT_CHANNEL, STRIP_TYPE, INIT_PIXELS_NUM));

	sync_mode_control_task = xSemaphoreCreateBinary();
    sync_color_task = xSemaphoreCreateBinary();
    sync_switch_color_task = xSemaphoreCreateBinary();
	sync_hue_task = xSemaphoreCreateBinary();
    sync_rainbow_task = xSemaphoreCreateBinary();

	xTaskCreatePinnedToCore(mode_control_task, "mode_control_task", 4096, NULL, MODE_TASK_PRIO, NULL, CORE_1);
	xSemaphoreGive(sync_mode_control_task);

	color_t color;
	// loop in which RGB values is received and relayed to the own driving task and to the Mesh
	while(1) {
		xQueueReceive(rgb_values_handle, &color, portMAX_DELAY);
		xQueueSend(led_task_input_handle, &color, pdMS_TO_TICKS(3000));
		xQueueSend(mesh_tx_input_handle, &color, pdMS_TO_TICKS(3000));
	}
}


void app_main(void)
{
// Allow the second core to finish initialization
	vTaskDelay(pdMS_TO_TICKS(100));
// NVS initialization
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	  ESP_ERROR_CHECK(nvs_flash_erase());
	  ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

// Queue memory allocation for different tasks
	//hsv_values_handle = xQueueCreate(5, sizeof(hsv_t));
	rgb_values_handle = xQueueCreate(5, sizeof(color_t));
	strip_modes_handle = xQueueCreate(1, sizeof(uint8_t));
	mesh_tx_input_handle = xQueueCreate(5, sizeof(color_t));
	led_task_input_handle = xQueueCreate(5, sizeof(color_t));

// TCP/IP initialization
	ESP_ERROR_CHECK(esp_netif_init());
	//Event Loop initialization
	ESP_ERROR_CHECK(esp_event_loop_create_default());

// mDNS initialization
	initialise_mdns();
	netbiosns_init();
	netbiosns_set_name("esp-home");

// Create network interfaces for mesh (only station instance saved for further manipulation, soft AP instance ignored */
	ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));

// ******************** WiFi initialization **********************************************
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
	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

// ******************** Mesh initialization **********************************************
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
	ESP_ERROR_CHECK(esp_mesh_set_self_organized(true, false));
	//ESP_ERROR_CHECK(esp_mesh_fix_root(true));
	ESP_ERROR_CHECK(esp_mesh_set_parent(&wifi_config, NULL, MESH_ROOT, MESH_ROOT_LAYER));
	ESP_ERROR_CHECK(esp_mesh_start());

    ESP_LOGI(TAG, "mesh starts successfully, heap:%d, %s<%d>%s, ps:%d\n",  esp_get_minimum_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed",
             esp_mesh_get_topology(), esp_mesh_get_topology() ? "(chain)":"(tree)", esp_mesh_is_ps_enabled());

    ESP_ERROR_CHECK(esp_read_mac(&mesh_own_mac, ESP_MAC_WIFI_STA));
    ESP_LOGI(TAG, "<MESH_OWN_MAC, "MACSTR"", MAC2STR(mesh_own_mac));

    sync_control_task = xSemaphoreCreateBinary();
	xTaskCreatePinnedToCore(control_task, "control_task", 4096, NULL, CONTROL_TASK_PRIO, NULL, CORE_1);
	xSemaphoreGive(sync_control_task);
}
