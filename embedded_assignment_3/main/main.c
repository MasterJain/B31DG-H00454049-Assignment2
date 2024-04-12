
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp32/rom/ets_sys.h" // For ets_delay_us()
#include "esp_timer.h"
#include <time.h>
// #include "driver/time"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "driver/adc.h"
#include <math.h>

#define EXAMPLE_PCNT_HIGH_LIMIT 3000
#define EXAMPLE_PCNT_LOW_LIMIT -3000
#define EXAMPLE_CHAN_GPIO_NUM 13
#define SIGNAL_TASK_2 13
#define SIGNAL_TASK_3 14
#define SIGNAL_GPIO GPIO_NUM_2

#define LED_GPIO GPIO_NUM_23
#define BUTTON_GPIO GPIO_NUM_19
#define ADC_MAX_VALUE 4095  // Maximum value for 12-bit ADC
#define ADC_REF_VOLTAGE 3.3 // Reference voltage for ADC (in volts)

#define ADC_CHANNEL ADC2_CHANNEL_0
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_12
#define DEBOUNCE_DELAY 50
#define DEBOUNCE_DELAY 50
#define MULT 10

#define ADC_SAMPLES 10 // Number of samples for running average
// #define LED_GPIO_BUTTON GPIO_NUM_23

// TaskHandle_t Task2;
// # TODO Make this into a struct
// float freq_task_2 = 0;
// float freq_task_3 = 0;

typedef struct
{
    float freqTask2;
    float freqTask3;
} FrequencyData;

FrequencyData freqData;

void outputSignalTask(void *parameter)
{

    while (true)
    {
        // Set HIGH for 180μs
        gpio_set_level(SIGNAL_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(180 * MULT / 1000));

        // vTaskDelay(180 / portTICK_PERIOD_MS / 1000);

        // Set LOW for 40μs
        gpio_set_level(SIGNAL_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(40 * MULT / 1000));
        // vTaskDelay(40 / portTICK_PERIOD_MS / 1000);

        // Set HIGH for 530μs
        gpio_set_level(SIGNAL_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(530 * MULT / 1000));
        // vTaskDelay(530 / portTICK_PERIOD_MS / 1000);
        // Set LOW for 3.25ms
        gpio_set_level(SIGNAL_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(3250 * MULT / 1000));

        // vTaskDelay(3250 / portTICK_PERIOD_MS / 1000);

        // Wait for the remaining period to complete 4ms cycle
        // Note: Adjust the delay to account for the time spent in high/low states and function call overhead
        // ets_delay_us(1000); // Adjust this delay to fine-tune the 4ms period
        vTaskDelay(pdMS_TO_TICKS(4));
        // vTaskDelay(1000 / portTICK_PERIOD_MS / 1000);
    }
}

pcnt_unit_handle_t configurePcnt()
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
        .edge_gpio_num = EXAMPLE_CHAN_GPIO_NUM,
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
    float *val;
    // Add other members as needed
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
    // printf("Frequency: %f\n", *val);
    pcnt_unit_clear_count(pcnt_unit);
    // esp_task_wdt_reset();
    // pcnt_unit_start(pcnt_unit);
}

void measureFreq(int interval, float *val)
{
    // Initialize the task watchdog with a timeout of 10 seconds

    pcnt_unit_handle_t pcnt_unit = configurePcnt();
    // int64_t start_time, end_time, time_diff_us;
    // float time_diff_ms;
    esp_timer_handle_t timer_handle;
    timer_callback_args_t callback_args = {
        .pcnt_unit = pcnt_unit,
        .val = val,
        .interval = interval};
    esp_timer_create_args_t timer_args = {
        .callback = &freqCallBack,
        .arg = &callback_args,
        .name = "one-shot-timer"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, interval * 1000));

    while (1)
    {
        // Reset the task watchdog timer

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }

    // while (1)
    // {
    //     int pulse_count = 0;
    //     start_time = esp_timer_get_time();
    //     int timeDiff = interval;

    //     ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
    //     printf("Frequency: %f\n", ((double)pulse_count * 1000) / timeDiff);
    //     pcnt_unit_clear_count(pcnt_unit);
    //     vTaskDelay(pdMS_TO_TICKS(timeDiff));
    //     end_time = esp_timer_get_time();
    //     time_diff_ms = (end_time - start_time) / 1000;
    //     printf("Time diff: %f\n", time_diff_ms);
    // }
}

void measureFreqTask2(void *pvParameter)
{
    measureFreq(20, &freqData.freqTask2);
}

void measureFreqTask3(void *pvParameter)
{
    measureFreq(8, &freqData.freqTask3);
}

void configure_adc()
{
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
}

float read_voltage_mv()
{
    int adc_raw = 0;
    esp_err_t ret = adc2_get_raw(ADC_CHANNEL, ADC_WIDTH, &adc_raw); // Pass the address of adc_raw
    if (ret != ESP_OK)
    {
        printf("Failed to read ADC value: %d\n", ret);
        return 0;
    }
    float voltage_mv = (adc_raw * ADC_REF_VOLTAGE / ADC_MAX_VALUE);
    // printf("READ VALUE %f", voltage_mv);
    return voltage_mv;
}

float average(float array[], int size, int index)
{
    int maxVal = size;
    float sum = 0.0;
    if (index < size)
    {
        maxVal = index + 1;
    }
    for (int i = 0; i < maxVal; i++)
    {
        sum += array[i];
    }
    return sum / maxVal;
}

void measureVoltageTask4(void *pvParameter)
{
    configure_adc();
    float adc_readings[ADC_SAMPLES] = {};
    int counter = 0;
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    while (1)
    {
        float reading = read_voltage_mv();
        adc_readings[counter % ADC_SAMPLES] = reading;
        float average_val = average(adc_readings, ADC_SAMPLES, counter);
        if (reading > ADC_REF_VOLTAGE / 2)
        {
            //
            printf("LED ON \n");
            gpio_set_level(LED_GPIO, 1);
        }
        else
        {
            gpio_set_level(LED_GPIO, 0);
        }

        printf("Voltage: %f %f \n", reading, average_val);
        counter += 1;
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}

#include "driver/uart.h"

#define TASK2_FREQ_MIN 333
#define TASK2_FREQ_MAX 1000
#define TASK3_FREQ_MIN 500
#define TASK3_FREQ_MAX 1000

#define PERIOD_MS 200
#define BAUD_RATE 9600

void freq_logging_task(void *pvParameter)
{
    int task2_freq, task3_freq;
    int task2_scaled, task3_scaled;

    while (1)
    {
        // Get the frequency values from Task 2 and Task 3
        task2_freq = freqData.freqTask2;
        task3_freq = freqData.freqTask3;

        // // Scale the frequency values to the range of 0 to 99
        // task2_scaled = (int)fmin(99, floor((task2_freq - TASK2_FREQ_MIN) * 99.0 / (TASK2_FREQ_MAX - TASK2_FREQ_MIN)));
        // task3_scaled = (int)fmin(99, floor((task3_freq - TASK3_FREQ_MIN) * 99.0 / (TASK3_FREQ_MAX - TASK3_FREQ_MIN)));

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

    // Create the frequency logging task
}

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

void buttonTask(void *pvParameter)
{
    // Configure button GPIO as input with internal pull-up resistor
    gpio_reset_pin(BUTTON_GPIO);
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);

    // Configure LED GPIO as output
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    bool led_state = false;             // Initial state of the LED
    bool last_button_state = true;      // Last read state of the button
    bool debounced_button_state = true; // Debounced state of the button
    uint32_t last_debounce_time = 0;    // Last debounce time

    while (1)
    {
        bool current_button_state = gpio_get_level(BUTTON_GPIO);

        // Check if the button state has changed
        if (current_button_state != last_button_state)
        {
            last_debounce_time = xTaskGetTickCount(); // Reset debounce timer
        }

        // Stable state after debounce delay
        if ((xTaskGetTickCount() - last_debounce_time) > pdMS_TO_TICKS(DEBOUNCE_DELAY))
        {
            // Update debounced state only if it has changed and stabilized
            if (current_button_state != debounced_button_state)
            {
                debounced_button_state = current_button_state;

                // Toggle the LED if the button is released (goes to HIGH)
                if (debounced_button_state == true)
                {
                    led_state = !led_state;
                    gpio_set_level(LED_GPIO, led_state);
                }
            }
        }

        last_button_state = current_button_state; // Update last button state
        vTaskDelay(pdMS_TO_TICKS(10));            // Small delay to avoid excessive CPU usage
    }
}
void app_main(void)
{
    xTaskCreate(&outputSignalTask, "output_signal", 10000, NULL, 3, NULL);
    xTaskCreate(&measureFreqTask2, "measure_freq_task_2", 10000, NULL, 5, NULL);
    xTaskCreate(&measureFreqTask3, "measure_freq_task_3", 10000, NULL, 5, NULL);
    // configure_uart();
    xTaskCreate(&freq_logging_task, "FreqLoggingTask", 2048, NULL, 5, NULL);
    xTaskCreate(&measureVoltageTask4, "measure_voltage_task", 10000, NULL, 5, NULL);
    xTaskCreate(&periodicTask, "period_task", 10000, NULL, 5, NULL);
    xTaskCreate(&buttonTask, "button_task", 10048, NULL, 5, NULL);
    // measureFreqTask();
}