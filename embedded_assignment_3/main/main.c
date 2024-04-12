#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include <math.h>


#define TASK2_FREQ_MIN 333
#define TASK2_FREQ_MAX 1000
#define TASK3_FREQ_MIN 500
#define TASK3_FREQ_MAX 1000
#define PERIOD_MS 200
#define BAUD_RATE 9600


#define EXAMPLE_PCNT_HIGH_LIMIT 3000
#define EXAMPLE_PCNT_LOW_LIMIT -3000
#define EXAMPLE_CHAN_GPIO_NUM 32

#define GPIO_TASK_2 35
#define GPIO_TASK_3 5


static const char *TAG = "ETX_PUSH_BUTTON";

#define BLINK_GPIO_1 2
#define BLINK_GPIO_2 25

#define BUTTON_PRESSED 0
#define BUTTON_RELEASED 1

// Event group to indicate button events
EventGroupHandle_t button_event_group;

// Event bits
#define BUTTON_EVENT_BIT (1 << 0)

// Function prototypes
void task_monitor_button(void *pvParameters);
void task_control_led(void *pvParameters);
void digitalSignalTask(void *parameters);
void task_sample_analog_input(void *pvParameters);
void periodicTask(void *pvParameter);

// Define the GPIO pin connected to your output
#define SIGNAL_PIN 14 // Replace if needed

// Define constants
#define LED_GPIO_PIN 21                  // GPIO pin connected to the LED
#define POTENTIOMETER_GPIO_PIN 32        // GPIO pin connected to the potentiometer
#define NUM_READINGS 10                  // Number of readings to keep track of for running average
#define MAX_VOLTAGE 3.3                  // Maximum voltage of the potentiometer (in volts)

// Global variables
float readings[NUM_READINGS];
int current_reading_index = 0;

float freq_task_2 = 0;
float freq_task_3 = 0;

void CPU_work(int time)
{
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(time));
    }
}

void periodicTask(void *pvParameter)
{
    CPU_work(2);
}

void freq_logging_task(void *pvParameter)
{
    int task2_freq, task3_freq;
    int task2_scaled, task3_scaled;

    while (1)
    {
        // Get the frequency values from Task 2 and Task 3
        task2_freq = freq_task_2;
        task3_freq = freq_task_3;

        task2_scaled = (int)fmax(0, fmin(99, floor((task2_freq - TASK2_FREQ_MIN) * 99.0 / (TASK2_FREQ_MAX - TASK2_FREQ_MIN))));
        task3_scaled = (int)fmax(0, fmin(99, floor((task3_freq - TASK3_FREQ_MIN) * 99.0 / (TASK3_FREQ_MAX - TASK3_FREQ_MIN))));

        // Log the information in comma-delimited format
        printf("%d,%d\n", task2_scaled, task3_scaled);

        vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
    }
}

void configure_uart()
{
    // Configure UART for logging
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
}

#ifndef FREQ_LOGGING_H
#define FREQ_LOGGING_H

void freq_logging_task(void *pvParameter);
void configure_uart();

#endif

pcnt_unit_handle_t configurePcnt(int pinNum)
{
    // Configure PCNT unit
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    // Configure PCNT channel
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = pinNum,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    // Set edge and level actions
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    // Enable PCNT unit
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));

    // Start counting
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    return pcnt_unit;
}

typedef struct
{
    pcnt_unit_handle_t pcnt_unit;
    int interval;
    int pinNum;
    float *val;
} timer_callback_args_t;

void freqCallBack(void *arg)
{
    int pulse_count = 0;
    timer_callback_args_t *callback_args = (timer_callback_args_t *)arg;
    pcnt_unit_handle_t pcnt_unit = callback_args->pcnt_unit;
    float *val = callback_args->val;
    int interval = callback_args->interval;

    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
    *val = ((double)pulse_count * 1000) / interval;
    pcnt_unit_clear_count(pcnt_unit);
}

void measureFreq(int interval, float *val, int pinNum)
{
    pcnt_unit_handle_t pcnt_unit = configurePcnt(pinNum);
    esp_timer_handle_t timer_handle;
    timer_callback_args_t callback_args = {
        .pcnt_unit = pcnt_unit,
        .val = val,
        .pinNum = pinNum > 10 ? 2 : 1, 
        .interval = interval};
    esp_timer_create_args_t timer_args = {
        .callback = &freqCallBack,
        .arg = &callback_args,
        .name = "one-shot-timer"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, interval * 1000));

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}

void measureFreqTask2(void *pvParameter)
{
    measureFreq(20, &freq_task_2, GPIO_TASK_2);
}

void measureFreqTask3(void *pvParameter)
{
    measureFreq(8, &freq_task_3, GPIO_TASK_3);
}



void digitalSignalTask(void *parameters) {
    for (;;) {  // Keep the infinite loop
        // High for 180us
        gpio_set_level(SIGNAL_PIN, 1);
        esp_rom_delay_us(180); 

        // Low for 40us
        gpio_set_level(SIGNAL_PIN, 0);
        esp_rom_delay_us(40);

        // High for 530us
        gpio_set_level(SIGNAL_PIN, 1);
        esp_rom_delay_us(530); 

        // Low for 3.25ms
        gpio_set_level(SIGNAL_PIN, 0);
        esp_rom_delay_us(3250); 
    } // End of the infinite loop
}

void task_sample_analog_input(void *pvParameters) {
    while(1) {
        // Read analog input
        uint32_t adc_reading = adc1_get_raw(ADC1_CHANNEL_4);
        float analog_voltage = adc_reading * (MAX_VOLTAGE / 4095.0); // Convert ADC reading to voltage
        
        // Update running average
        readings[current_reading_index] = analog_voltage;
        current_reading_index = (current_reading_index + 1) % NUM_READINGS;
        float average_analog_in = 0;
        for (int i = 0; i < NUM_READINGS; i++) {
            average_analog_in += readings[i];
        }
        average_analog_in /= NUM_READINGS;
        
        // Visualize error using LED
        if (average_analog_in > (MAX_VOLTAGE / 2)) {
            gpio_set_level(LED_GPIO_PIN, 1); // Turn on LED
        } else {
            gpio_set_level(LED_GPIO_PIN, 0); // Turn off LED
        }
        
        vTaskDelay(pdMS_TO_TICKS(20)); // Delay for 20ms (50Hz)
    }
    
}

void task_monitor_button(void *pvParameters)
{
    while (1)
    {
        // Check if button is pressed
        if (gpio_get_level(BLINK_GPIO_2) == BUTTON_PRESSED)
        {
            // Send event to control LED task
            xEventGroupSetBits(button_event_group, BUTTON_EVENT_BIT);
            ESP_LOGI(TAG, "Button pressed");
            // Wait until button is released
            while (gpio_get_level(BLINK_GPIO_2) == BUTTON_PRESSED) {
                vTaskDelay(pdMS_TO_TICKS(50)); // Debounce delay
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Task delay for debounce
    }
}

void task_control_led(void *pvParameters)
{
    bool led_state = false;

    while (1)
    {
        // Wait for button event
        EventBits_t bits = xEventGroupWaitBits(button_event_group, BUTTON_EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & BUTTON_EVENT_BIT)
        {
            // Toggle LED
            led_state = !led_state;
            gpio_set_level(BLINK_GPIO_1, led_state);
            ESP_LOGI(TAG, "Toggling LED");
        }
    }
}

void app_main() {

    //TASK1
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SIGNAL_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    xTaskCreate(digitalSignalTask, "Signal Task", 1024, NULL, 1, NULL);

    //TASK 6
    // Reset the pins
    gpio_reset_pin(BLINK_GPIO_1);
    gpio_reset_pin(BLINK_GPIO_2);

    // Set the GPIOs to Output/Input mode
    gpio_set_direction(BLINK_GPIO_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(BLINK_GPIO_2, GPIO_MODE_INPUT);

    // Enable Pullup for Input Pin
    gpio_pullup_en(BLINK_GPIO_2);

    // Create event group
    button_event_group = xEventGroupCreate();

    // Create task to monitor button
    xTaskCreate(task_monitor_button, "monitor_button_task", 2048, NULL, 5, NULL);

    // Create task to control LED
    xTaskCreate(task_control_led, "control_led_task", 2048, NULL, 5, NULL);

    //TASK4
    // Initialize ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_0); // Configure ADC to read from pin 32 (ADC1_CHANNEL_4)

    // Configure LED GPIO
    esp_rom_gpio_pad_select_gpio(LED_GPIO_PIN);
    gpio_set_direction(LED_GPIO_PIN, GPIO_MODE_OUTPUT);
    
    // Create task to sample analog input
    xTaskCreate(task_sample_analog_input, "sample_analog_input_task", 2048, NULL, 5, NULL);
    xTaskCreate(&measureFreqTask2, "measure_freq_task_2", 10000, NULL, 5, NULL);
    xTaskCreate(&measureFreqTask3, "measure_freq_task_3", 10000, NULL, 5, NULL);
    //configure_uart();
    xTaskCreate(&freq_logging_task, "FreqLoggingTask", 2048, NULL, 5, NULL);
    xTaskCreate(&periodicTask, "period_task", 10000, NULL, 5, NULL);
}
