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

// Configurações de Broker MQTT
#define MQTT_BROKER_URI "mqtt://test.mosquitto.org"
#define DEBUG_TOPIC "DEBUG_FGA"
#define PUBLISH_STATUS "STATUS_ALEXA_RESP"
#define PUBLISH_ACTIVATION "STATUS_ACTIVATION_RESP"
#define SUBSCRIBE_TOPIC "ALEXA_COMMANDS_MQTT"

// Pinos GPIO Sensores
#define TRIGGER_PIN GPIO_NUM_13
#define ECHO_PIN GPIO_NUM_12
#define dht_PIN GPIO_NUM_26

// Definição dos pinos do motor de passo, do relé e do driver
#define PIN_STEP GPIO_NUM_33
#define PIN_DIR GPIO_NUM_25
#define PIN_RELAY GPIO_NUM_23
#define PIN_EN GPIO_NUM_32
#define LED GPIO_NUM_2

// Definição de pinos de fim de curso
#define PIN_SENSOR_DIREITO GPIO_NUM_27
#define PIN_SENSOR_ESQUERDO GPIO_NUM_14

// Definição das variáveis de controle do motor
int velocidade = 1000; // Velocidade em microssegundos entre cada pulso (mais baixo = mais rápido)
int rpm = 7200;

// Constantes de medida de distancea
#define SPEED_OF_SOUND 0.0042 // Aproximação de velocidade do som
#define MAX_DISTANCE 40      // Distancia maxima em cm
#define HC_WAIT 2000
#define MIN_DISTANCE 2        // Distancia minima em cm
#define LED_BLINK_DELAY_MS 500

char *ssid = "OnePlus Nord N10 5G";
char *pass = "8085df2d08c4";

// Variaveis de leituras
char hum_g[50];
char temp_g[50];
char event_m[150];
char distance_str[50];

SemaphoreHandle_t wifi_connected;

int retry_num = 0;
int sentido = 0;
int level = 100;

static esp_mqtt_client_handle_t mqtt_client = NULL;

// Sensores
static void trigger_sensor();
int hc_read();
static void dht_task();
static void configure_pins();

// Motor
static void motor_check(void);
static void control_motor(int direction);
static void led_controller(void *pvParameters);


// MQTT
void mqtt_app_start();
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void handle_mqtt_connected_event();
void handle_mqtt_disconnected_event();
void handle_mqtt_data_event(esp_mqtt_event_handle_t event);
void handle_get_status_event();
void handle_turn_on_event();
void publish_status_json(cJSON *json_root);
cJSON *create_status_json(void);



// Wi-Fi
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void wifi_connection(void);
static void main_task(void);

///////////////////////////////////////////// Funções Principais
void main_task()
{
    if (xSemaphoreTake(wifi_connected, portMAX_DELAY))
    {
        mqtt_app_start();
        sleep(1);
    }
}

void app_main(void)
{
    configure_pins();

    nvs_flash_init();
    wifi_connection();


    // Conferir se o carrinho esta posicionado correto e ajustar
    if(gpio_get_level(PIN_SENSOR_ESQUERDO) == 0){
        gpio_set_level(PIN_EN, 0);
        ESP_LOGW("MOTOR", "Sentido Anti-Horario Start");
        while (gpio_get_level(PIN_SENSOR_ESQUERDO) == 0)
        {
            gpio_set_level(PIN_STEP, 1);
            esp_rom_delay_us(velocidade);
            gpio_set_level(PIN_STEP, 0);
            esp_rom_delay_us(velocidade);
        }
        ESP_LOGW("MOTOR", "Sentido Anti-Horario End");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    gpio_set_level(PIN_EN, 1);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIGGER_PIN) | (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << ECHO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    while (1)
    {
        main_task();
    }
}


// Configuração dos pinos
void configure_pins()
{
    gpio_config_t io_conf;

    // Configuração do pino de passo
    io_conf.pin_bit_mask = (1ULL << PIN_STEP);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Configuração do pino de LED
    io_conf.pin_bit_mask = (1ULL << LED);
    gpio_config(&io_conf);

    // Configuração do pino de direção
    io_conf.pin_bit_mask = (1ULL << PIN_DIR);
    gpio_config(&io_conf);

    // Configuração do pino do relé
    io_conf.pin_bit_mask = (1ULL << PIN_RELAY);
    gpio_config(&io_conf);

    // Configuração do pino de habilitação do driver
    io_conf.pin_bit_mask = (1ULL << PIN_EN);
    gpio_config(&io_conf);

    // Configuração dos pinos para os sensores de fim de curso
    io_conf.pin_bit_mask = (1ULL << PIN_SENSOR_ESQUERDO) | (1ULL << PIN_SENSOR_DIREITO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
}

/////////////////////////////////////////////////////////////////// Motor
// Verificar se o motor estiver respondendo
void motor_check()
{
    sentido = 1;
    gpio_set_level(PIN_DIR, sentido);

    // Habilitando o driver do motor (0 para habilitar)
    gpio_set_level(PIN_EN, 0);

    // Uma step
    for(int i = 0; i < 400; i++){
        gpio_set_level(PIN_STEP, 1);
        esp_rom_delay_us(velocidade);
        gpio_set_level(PIN_STEP, 0);
        esp_rom_delay_us(velocidade);
    }

    if (gpio_get_level(PIN_SENSOR_ESQUERDO) == 0)
    {
        error = 0;
    }
    else
    {
        error = 3;
    }

    // Habilitando o driver do motor (0 para habilitar)
    gpio_set_level(PIN_EN, 1);
}

// Função para girar o motor de passo e controlar a bomba e o driver
void control_motor(int sentido)
{
    // Definindo o sentido do motor
    gpio_set_level(PIN_DIR, sentido);

    // Habilitando o driver do motor (0 para habilitar)
    gpio_set_level(PIN_EN, 0);

    // Acionando o relé enquanto o carrinho estiver em movimento
    gpio_set_level(PIN_RELAY, 1);

    int i = 0;

    // Girando o motor por um segundo
    if (sentido == 0)
    {
        ESP_LOGW("MOTOR", "Sentido Anti-Horario Start");
        while (1)
        {
            i++;
            // Check de nivel do reservatorio
            if(i%1000 == 0){
                level = hc_read();
                ESP_LOGW("RESEVOIR - 1", "%d", level);

                if(level <= 10){
                    esp_mqtt_client_publish(mqtt_client, PUBLISH_ACTIVATION, "failed", 0, 1, 0);
                    gpio_set_level(PIN_RELAY, 0);
                }
            }
            gpio_set_level(PIN_STEP, 1);
            esp_rom_delay_us(velocidade);
            gpio_set_level(PIN_STEP, 0);
            esp_rom_delay_us(velocidade);

            if(gpio_get_level(PIN_SENSOR_ESQUERDO) == 1){
                break;
            }
        }
        ESP_LOGW("MOTOR", "Sentido Anti-Horario End");
    }
    else
    {
        ESP_LOGW("MOTOR", "Sentido Horario Start");
        while (1)
        {
            i++;
            // Check de nivel do reservatorio
            if(i%1000 == 0){
                level = hc_read();
                ESP_LOGW("RESEVOIR - 2", "%d", level);
                if(level <= 10){
                    esp_mqtt_client_publish(mqtt_client, PUBLISH_ACTIVATION, "failed", 0, 1, 0);
                    gpio_set_level(PIN_RELAY, 0);
                    break;
                }
            }
            gpio_set_level(PIN_STEP, 1);
            esp_rom_delay_us(velocidade);
            gpio_set_level(PIN_STEP, 0);
            esp_rom_delay_us(velocidade);

            if(gpio_get_level(PIN_SENSOR_DIREITO) == 1){
                break;
            }
        }
        ESP_LOGW("MOTOR", "Sentido Horario End");
    }
    // Desligando o relé quando o carrinho parar
    gpio_set_level(PIN_RELAY, 0);

    // Desabilitando o driver do motor (1 para desabilitar)
    gpio_set_level(PIN_EN, 1);
}


///////////////////////////////////////////////////////////////////// MQTT
// Inicializa MQTT
void mqtt_app_start()
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
    esp_mqtt_client_start(mqtt_client);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    mqtt_event_handler_cb(event_data);
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    strcpy(event_m, "");
    mqtt_client = event->client;

    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        handle_mqtt_connected_event();
        break;
    case MQTT_EVENT_DISCONNECTED:
        handle_mqtt_disconnected_event();
        break;
    case MQTT_EVENT_DATA:
        handle_mqtt_data_event(event);
        break;
    default:
        break;
    }

    return ESP_OK;
}

void handle_mqtt_connected_event()
{
    ESP_LOGI("MQTT", "EVENT_CONNECTED\n");
    esp_mqtt_client_subscribe(mqtt_client, SUBSCRIBE_TOPIC, 0);
    esp_mqtt_client_publish(mqtt_client, DEBUG_TOPIC, "Connected", 0, 1, 0);
}

void handle_mqtt_disconnected_event()
{
    ESP_LOGE("MQTT", "EVENT_DISCONNECTED\n");
}

void handle_mqtt_data_event(esp_mqtt_event_handle_t event)
{
    ESP_LOGI("MQTT", "EVENT_DATA\n\n\n");

    sprintf(event_m, "%s", event->data);
    event_m[event->data_len] = '\0';

    if (strcmp(event_m, "get_status") == 0)
    {
        handle_get_status_event();
    }
    else if (strcmp(event_m, "turn_on") == 0)
    {
        handle_turn_on_event();
    }
}

void handle_get_status_event()
{
    ESP_LOGI("HEALTH CHECK", "=== START HC ===");
    vTaskDelay(2200 / portTICK_PERIOD_MS);

    dht_task();
    level = hc_read();
    sprintf(distance_str, "%d", level);

    cJSON *json_root = create_status_json();
    publish_status_json(json_root);

    ESP_LOGI("HEALTH CHECK", "=== Finished HC ===\n\n\n");
}

cJSON *create_status_json(void)
{
    cJSON *json_root = cJSON_CreateObject();
    cJSON_AddStringToObject(json_root, "temperature", temp_g);
    cJSON_AddStringToObject(json_root, "humidity", hum_g);
    cJSON_AddStringToObject(json_root, "reservatoryLevel", distance_str);

    if (error == 1)
    {
        cJSON_AddStringToObject(json_root, "error_code", "1");
    }
    else if (error == 2)
    {
        cJSON_AddStringToObject(json_root, "error_code", "2");
    }
    else
    {
        cJSON_AddStringToObject(json_root, "error_code", "0");
    }

    return json_root;
}

void handle_turn_on_event()
{
    ESP_LOGW("MOTOR ACTIVATION", "STARTING RUN");

    motor_check();

    if (error == 0)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);

        level = hc_read();
        ESP_LOGW("RESEVOIR - 0", "%d", level);

        if (level > 20)
        {
            esp_mqtt_client_publish(mqtt_client, PUBLISH_ACTIVATION, "active", 0, 1, 0);
            control_motor(sentido);
            sentido = !sentido;
            control_motor(sentido);
        }
        else
        {
            esp_mqtt_client_publish(mqtt_client, PUBLISH_ACTIVATION, "failed", 0, 1, 0);
        }
    }
    else
    {
        esp_mqtt_client_publish(mqtt_client, PUBLISH_ACTIVATION, "failed", 0, 1, 0);
    }
}

void publish_status_json(cJSON *json_root)
{
    char *json_str = cJSON_Print(json_root);
    esp_mqtt_client_publish(mqtt_client, PUBLISH_STATUS, json_str, 0, 1, 0);
    ESP_LOGW("DEBUG", "%s", json_str);
    cJSON_Delete(json_root);
    free(json_str);
}

/////////////////////////////////////////////////////////////////////// Sensor Ultrassom
// Ativa sensor
static void trigger_sensor()
{
    gpio_set_level(TRIGGER_PIN, 1);
    esp_rom_delay_us(1);
    gpio_set_level(TRIGGER_PIN, 0);
}

// Leitura da distancia do sensor
int hc_read()
{
    int i = 0;
    float total = 0, timeout = 0;
    ESP_LOGI("HEALTH CHECK", "Start HR-SC04 Task");

    trigger_sensor();

    int duration = 0;
    while (gpio_get_level(ECHO_PIN) == 0 && timeout < HC_WAIT)
    {
        timeout += 1;
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    if(timeout < HC_WAIT) {
        while (gpio_get_level(ECHO_PIN) == 1)
        {
            duration += 100;
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        float distance = (duration * SPEED_OF_SOUND) / 2;

        if(distance >= HC_WAIT){
            distance = -1;
        }

        i++;
        ESP_LOGE("DISTANCE", "%.2f cm\n", distance);
        total += distance;
    }else{
        total += -1;
    }


    if (total > MAX_DISTANCE || total < MIN_DISTANCE)
    {
        ESP_LOGW("DISTANCE", "SENSOR ERROR");
        error = 2;
    }
    else if (error != 1)
    {
        error = 0;
    }

    int level = 100 -((total * 100)/28);

    if(level < 0){
        level = 0;
    }

    ESP_LOGI("HEALTH CHECK", "%d\n", level);

    ESP_LOGI("HEALTH CHECK", "End HR-SC04 Task");

    return level;
}

//////////////////////////////////////////////////////////////////////// DHT
void dht_task()
{
    float hum, temp;

    setDHTgpio(dht_PIN);
    ESP_LOGI("HEALTH CHECK", "Starting DHT22 Task\n\n");

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    int ret = readDHT();

    errorHandler(ret);
    hum = getHumidity();
    temp = getTemperature();

    ESP_LOGI("HEALTH CHECK", "DHT\nHum: %.1f Tmp: %.1f\n", hum, temp);

    sprintf(temp_g, "%.1f", temp);
    sprintf(hum_g, "%.1f", hum);

    ESP_LOGI("HEALTH CHECK", "Finished DHT22 Task\n\n");
}


/////////////////////////////////////////////////////////////////////// Wi-Fi
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_START)
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
        if (retry_num < 505)
        {
            xTaskCreate(led_controller, "led_controller", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
            esp_wifi_connect();
            retry_num++;
            ESP_LOGW("WIFI", "Retrying to Connect...\n");
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
    else if (event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGI("WIFI", "Wifi got IP...\n\n");
        gpio_set_level(LED, 1);
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
    strcpy((char *)wifi_configuration.sta.ssid, ssid);
    strcpy((char *)wifi_configuration.sta.password, pass);

    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    wifi_connected = xSemaphoreCreateBinary();

    esp_wifi_start();
    esp_wifi_set_mode(WIFI_MODE_STA);

    esp_wifi_connect();
}

////////////////////////////////////////////////////////////// LED
void led_controller(void *pvParameters)
{
    gpio_set_level(LED, 1);              // Turn on the LED
    vTaskDelay(LED_BLINK_DELAY_MS / 2 / portTICK_PERIOD_MS); // Wait for half of the blink duration
    gpio_set_level(LED, 0);              // Turn off the LED
    vTaskDelay(LED_BLINK_DELAY_MS / 2 / portTICK_PERIOD_MS); // Wait for the remaining half of the blink duration
    vTaskDelete(NULL);                   // Delete the task
}