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


// Define MQTT Broker settings
#define MQTT_BROKER_URI "mqtt://test.mosquitto.org" // Replace with your MQTT broker address
#define MQTT_TOPIC "HC-SR04_FGA"

// Define GPIO pins for trigger and echo
#define TRIGGER_PIN GPIO_NUM_23
#define ECHO_PIN GPIO_NUM_22

// Define constants for speed of sound and maximum measurable distance
#define SPEED_OF_SOUND 0.45 // Speed of sound in cm/µs
#define MAX_DISTANCE 410       // Maximum measurable distance in cm

char *ssid = "SSID";
char *pass = "PASS";
SemaphoreHandle_t wifi_connected;
int retry_num=0;


static esp_mqtt_client_handle_t mqtt_client = NULL;

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            printf("MQTT_EVENT_CONNECTED\n");
            break;
        case MQTT_EVENT_DISCONNECTED:
            printf("MQTT_EVENT_DISCONNECTED\n");
            break;
        default:
            break;
    }
    return ESP_OK;
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
    char distance_str[30];
    while (1) {
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

        if (distance > MAX_DISTANCE) {
            printf("Distance: Out of range\n");
            sprintf(distance_str, "%s", "Distance: Out of range\n");
        } else {
            printf("Distance: %d cm\n", distance);
            sprintf(distance_str, "%d", distance);

        }

        if(mqtt_client != NULL){
            esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, distance_str, 0, 0, 0);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay 1 second
    }
}

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,void *event_data){
    if(event_id == WIFI_EVENT_STA_START)
    {
        printf("WIFI CONNECTING....\n");
    }
    else if (event_id == WIFI_EVENT_STA_CONNECTED)
    {
        printf("WiFi CONNECTED\n");
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        printf("WiFi lost connection\n");
        if(retry_num<5){
            esp_wifi_connect();retry_num++;printf("Retrying to Connect...\n");
        }
    }
    else if (event_id == IP_EVENT_STA_GOT_IP)
    {
        printf("Wifi got IP...\n\n");
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
    printf( "wifi_init_softap finished. SSID:%s  password:%s",ssid,pass);
    
}


void app_main(void)
{
    nvs_flash_init();
    wifi_connection();  

    vTaskDelay(10000 /portTICK_PERIOD_MS);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIGGER_PIN) | (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << ECHO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);  

    if(xSemaphoreTake(wifi_connected, portMAX_DELAY)) {
        mqtt_app_start();
        sleep(10);
        xTaskCreate(read_distance_task, "read_distance_task", 4096, NULL, 10, NULL);
    }
}