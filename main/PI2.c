#include <stdio.h>
#include <string.h> 
#include <stdlib.h>
#include "freertos/FreeRTOS.h" 
#include "esp_system.h" 
#include "esp_wifi.h" 
#include "esp_log.h" 
#include "esp_event.h" 
#include "nvs_flash.h" 
#include "lwip/err.h" 
#include "lwip/sys.h" 
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "mqtt_client.h"
#include "DHT.h"
#include "cJSON.h"


// Define MQTT Broker settings
#define MQTT_BROKER_URI "mqtt://test.mosquitto.org" // Replace with your MQTT broker address
#define DEBUG_TOPIC "DEBUG_FGA"
#define PUBLISH_STATUS "STATUS_ALEXA_RESP"
#define PUBLISH_ACTIVATION "STATUS_ACTIVATION_RESP"
#define SUBSCRIBE_TOPIC "ALEXA_COMMANDS_MQTT"

// Define GPIO pins for trigger and echo
#define TRIGGER_PIN GPIO_NUM_23
#define ECHO_PIN GPIO_NUM_22
#define DHT_PIN GPIO_NUM_21

// Define constants for speed of sound and maximum measurable distance
#define SPEED_OF_SOUND 0.47 // Speed of sound in cm/µs
#define MAX_DISTANCE 400       // Maximum measurable distance in cm

char *ssid = "Pankeka";
char *pass = "*Imperatriz";


// Monitoration Variables
char hum_g[50];
char temp_g[50];
char event_m[150];
char distance_str[50];


SemaphoreHandle_t wifi_connected;
SemaphoreHandle_t task_handle;
int retry_num=0;


static esp_mqtt_client_handle_t mqtt_client = NULL;
static void read_distance_task(void *pvParameters);
void DHT_task(void *pvParameter);

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
    strcpy(event_m, "");
    mqtt_client = event->client;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI("MQTT", "EVENT_CONNECTED\n");
            esp_mqtt_client_subscribe(mqtt_client, SUBSCRIBE_TOPIC, 0);
            esp_mqtt_client_publish(mqtt_client, DEBUG_TOPIC, "Connected", 0, 1, 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGE("MQTT", "EVENT_DISCONNECTED\n");
            break;
        case MQTT_EVENT_DATA:
        // Receive Data
            ESP_LOGI("MQTT", "EVENT_DATA\n\n\n");
        
        // Define Received Message
            sprintf(event_m, "%s", event->data);
            event_m[event->data_len] = '\0';

        // Message Check for Tasks
            xSemaphoreGive(task_handle);
            if(strcmp(event_m, "get_status") == 0){
                ESP_LOGI("HEALTH CHECK", "=== START HC ===");
                //Run sensor Reads and return via MQTT
                //Read Ultrasound sensor
                if(xSemaphoreTake(task_handle, portMAX_DELAY)){
                    xTaskCreate(read_distance_task, "read_distance_task", 2048, NULL, 10, NULL);

                    if(xSemaphoreTake(task_handle, portMAX_DELAY)){
                        //Read Temperature and Humidity
                        xTaskCreate(DHT_task, "DHT_task", 2048, NULL, 10, NULL);
                        if(xSemaphoreTake(task_handle, portMAX_DELAY)){
                            //Organize message and publish
                            cJSON *json_root = cJSON_CreateObject();
                            cJSON_AddStringToObject(json_root, "temperature", temp_g);
                            cJSON_AddStringToObject(json_root, "humiity", hum_g);
                            cJSON_AddStringToObject(json_root, "reservatory_level", distance_str);


                            // Convert cJSON object to a string
                            char *json_str = cJSON_Print(json_root);

                            // Publish the JSON string
                            esp_mqtt_client_publish(mqtt_client, PUBLISH_STATUS, json_str, 0, 1, 0);

                            // Free cJSON and JSON string
                            ESP_LOGW("DEBUG", "%s", json_str);

                            cJSON_Delete(json_root);
                            free(json_str);
                            xSemaphoreGive(task_handle);

                            ESP_LOGI("HEALTH CHECK", "=== Finished HC ===\n\n\n");
                        }
                    
                    }
                }
                
            }else if(strcmp(event_m, "Start Run") == 0){
                //Run Motor
                esp_mqtt_client_publish(mqtt_client, PUBLISH_ACTIVATION, "active", 0, 1, 0);
            }else{
                // Publish error Code
            }
            break;
        default:
            break;
    }

    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){
    mqtt_event_handler_cb(event_data);
}

void mqtt_app_start() {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
    esp_mqtt_client_start(mqtt_client);
}

static void trigger_sensor() {
    gpio_set_level(TRIGGER_PIN, 1);
    esp_rom_delay_us(1);
    gpio_set_level(TRIGGER_PIN, 0);
}


static void read_distance_task(void *pvParameters) {
    int i = 0, total = 0;
    strcpy(distance_str, "");
    ESP_LOGI("HEALTH CHECK", "Start HR-SC04 Task");
    while (i < 10) {
        trigger_sensor();

        int duration = 0;
        while (gpio_get_level(ECHO_PIN) == 0) {
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        while (gpio_get_level(ECHO_PIN) == 1) {
            duration++;
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        int distance = (duration * SPEED_OF_SOUND) / 2;

        i++;
        total+=distance;
        vTaskDelay(500 / portTICK_PERIOD_MS); // Delay 1 second
    }

    total = total/10;
    ESP_LOGI("HEALTH CHECK", "%d cm\n", total);
    sprintf(distance_str, "%d", total);

    ESP_LOGI("HEALTH CHECK", "End HR-SC04 Task");
    
    xSemaphoreGive(task_handle);
    vTaskDelete(NULL);
}


void DHT_task(void *pvParameter)
{
    float hum, temp;

    setDHTgpio(DHT_PIN);
    ESP_LOGI("HEALTH CHECK", "Starting DHT22 Task\n\n");

    vTaskDelay(2000  /portTICK_PERIOD_MS);

    int ret = readDHT();

    errorHandler(ret);
    hum = getHumidity();
    temp = getTemperature();

    ESP_LOGI("HEALTH CHECK", "DHT\nHum: %.1f Tmp: %.1f\n", hum, temp);
    
    sprintf(temp_g, "%.1f", temp);
    sprintf(hum_g, "%.1f", hum);

    
    ESP_LOGI("HEALTH CHECK", "Finished DHT22 Task\n\n");

    xSemaphoreGive(task_handle);
    
    vTaskDelete(NULL);
}

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,void *event_data){
    if(event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI("WIFI", "WIFI CONNECTING....\n");
    }
    else if (event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI("WIFI", "WiFi CONNECTED\n");
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGE("WIFI", "WiFi lost connection\n");
        if(retry_num<5){
            esp_wifi_connect();retry_num++;
            ESP_LOGW("WIFI", "Retrying to Connect...\n");
        }
    }
    else if (event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGI("WIFI", "Wifi got IP...\n\n");
        xSemaphoreGive(wifi_connected);
    }
}

void wifi_connection()
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation); //     
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = "",
            .password = "",
            
           }
    
        };
    strcpy((char*)wifi_configuration.sta.ssid, ssid);
    strcpy((char*)wifi_configuration.sta.password, pass);    

    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    wifi_connected = xSemaphoreCreateBinary();

    esp_wifi_start();
    esp_wifi_set_mode(WIFI_MODE_STA);

    esp_wifi_connect();
    
}

void main_task(){
    if(xSemaphoreTake(wifi_connected, portMAX_DELAY)) {
        mqtt_app_start();
        sleep(1);
    }
}

void app_main(void)
{
    nvs_flash_init();
    wifi_connection();  

    vTaskDelay(1000 /portTICK_PERIOD_MS);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIGGER_PIN) | (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << ECHO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);  

    task_handle = xSemaphoreCreateBinary();
    while(1){
        main_task();
    }
}