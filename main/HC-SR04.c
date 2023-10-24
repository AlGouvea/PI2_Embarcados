#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "mqtt_client.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"

// Define GPIO pins for trigger and echo
#define TRIGGER_PIN GPIO_NUM_23
#define ECHO_PIN GPIO_NUM_22

// Define constants for speed of sound and maximum measurable distance
#define SPEED_OF_SOUND 0.5 // Speed of sound in cm/µs
#define MAX_DISTANCE 410       // Maximum measurable distance in cm

static void trigger_sensor() {
    gpio_set_level(TRIGGER_PIN, 1);
    esp_rom_delay_us(1);
    gpio_set_level(TRIGGER_PIN, 0);
}

static void read_distance_task(void *pvParameters) {
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
        } else {
            printf("Distance: %d cm\n", distance);

        }

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay 1 second
    }
}

void app_main()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIGGER_PIN) | (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << ECHO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);


    xTaskCreate(read_distance_task, "read_distance_task", 4096, NULL, 10, NULL);
}