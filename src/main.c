#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_err.h"
#include "esp_log.h"

//DEFINES:
#define BAUD_RATE            115200               // Baud rate = 115200, quick and safe
#define RX_BUFFER_SIZE       256                  // Max data read 64, 256 for safety
#define TX_BUFFER_SIZE       0                    // No data transfer
#define PWM_FREQ             5000                 // FREQ = 5000 for PWM
#define LOW                  0                    // LOW = 0
#define LED_PIN              4                    // GPIO 4
#define HPOINT               0                    // HPOINT will be 0 for led use
#define TAG                  "APP"                // LOG TAG
#define READ_TIME            100                  // Timer period
#define DATA_SIZE            64                   // UART read data
#define PWM_RANGE            255                  // 8 bit PWM
#define POT_RANGE            4095                 // 12 bit ADC
#define CLI_STACK_DEPTH      2650                 // Used uxTaskGetStackHighWaterMark function
#define POT_READ_STACK_DEPTH 2300                 // Used uxTaskGetStackHighWaterMark function
#define PWM_STACK_DEPTH      2300                 // Used uxTaskGetStackHighWaterMark function



// HANDLERS:
QueueHandle_t pot_q = NULL;                       // Queue for potentiometer value
SemaphoreHandle_t status_semaphore = NULL;        // Semaphore to indicate system status (ON/OFF)
SemaphoreHandle_t read_semaphore = NULL;          // Semaphore to indicate reading status (read/don't read)
TimerHandle_t read_timer = NULL;                  // Timer of potentiometer readings


// FUNCTIONS:
// Timer function:
void timer_call_back(TimerHandle_t xTimer) {
    xSemaphoreGive(read_semaphore);               // Enable Readings of the potentiometer
}

// Tasks declaration:
void cli_task(void* pvParameter);
void pot_read_task(void* pvParameter);
void pwm_task(void* pvParameter);

// Helpful functions declaration:
void check_word(char* word);
bool is_word_on(char* word);
bool is_word_off(char* word);


// APP MAIN:
void app_main(void) {
    // UART SET:
    uart_config_t uart_cfg = {                                                          // UART config set
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .parity = UART_PARITY_DISABLE,
        .source_clk = UART_SCLK_APB,
        .stop_bits = UART_STOP_BITS_1
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_cfg));                          // UART configuration
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, RX_BUFFER_SIZE, TX_BUFFER_SIZE,
         0, NULL, 0));                                                                  // UART driver installation
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));                                       // UART set pin 
    
    // PWM SET:
    ledc_timer_config_t timer_cfg = {                                                   // PWM's timer config set
        .clk_cfg = LEDC_AUTO_CLK,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = PWM_FREQ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));                                     // PWM's timer configuration

    ledc_channel_config_t channel_cfg = {                                               // PWM's channel config set
        .channel = LEDC_CHANNEL_0,
        .duty = LOW,
        .gpio_num = LED_PIN,
        .hpoint = HPOINT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));                                 // PWM's channel configuration

    // ADC SET:
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11));        // ADC channel attenuation set
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));                               // ADC bit width set

    // QUEUE SET:
    pot_q = xQueueCreate(1, sizeof(int));
    if (pot_q == NULL) {                                                                // Check queue creation
        ESP_LOGE(TAG, "Queue creation failed");
        return;
    }

    // SEMAPHORE SET:
    // Status semaphore:
    status_semaphore = xSemaphoreCreateBinary();
    if (status_semaphore == NULL) {                                                     // Check semaphore creation
        ESP_LOGE(TAG, "Status semaphore creation failed");
        return;
    }

    // Read semaphore:
    read_semaphore = xSemaphoreCreateBinary();
    if (read_semaphore == NULL) {                                                       // Check semaphore creation
        ESP_LOGE(TAG, "Read semaphore creation failed");
        return;
    }

    // TIMER SET:
    read_timer = xTimerCreate("Read Timer", pdMS_TO_TICKS(READ_TIME), pdTRUE, 
    (void*) 0, timer_call_back);
    if (read_timer == NULL) {                                                           // Check timer creation
        ESP_LOGE(TAG, "Timer creation failed");
        return;
    }

    // TASKS SET:
    // CLI task:
    if (xTaskCreate(cli_task, "CLI Task", CLI_STACK_DEPTH, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "CLI task create failed");
        return;
    }

    // Pot read task:
    if (xTaskCreate(pot_read_task, "Pot Read Task", POT_READ_STACK_DEPTH, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Pot task create failed");
        return;
    }

    // PWM task:
    if (xTaskCreate(pwm_task, "PWM Read Task", PWM_STACK_DEPTH, NULL, 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "PWM task create failed");
        return;
    }
}


// TASKS FUNCTIONS:
//CLI task:
/*
Read & write CLI from user. Check if the data is ON/OFF or else.
In case data is not ON/OFF help the user.
Max letters to read 63. After the 63 letter, reading stops. 
*/
void cli_task(void* pvParameter) {
    vTaskDelay(pdMS_TO_TICKS(20));                                                // Small delay to let the app finish

    // READING VARIABLES:
    uint8_t ch;                                                                   // Data will be stored in this variable
    char line[DATA_SIZE];                                                         // Word will be stored in this string
    bool cr_flag = false;                                                         // Flag to indicate if last data was '/r' or not
    size_t len = 0;                                                               // String length

    uart_write_bytes(UART_NUM_0, "\r\n> ", 4);

    while (1) {
        // UART READING:
        if (uart_read_bytes(UART_NUM_0, &ch, 1, portMAX_DELAY) > 0) {  
            // CHECK DEL/BS:
            if ((ch == 0x08 || ch == 0x7F) && len > 0) {
                len--;
                uart_write_bytes(UART_NUM_0, "\b \b", 3);                         // Delete last char
                cr_flag = false;                                                  // Last char wasn't CR
            }

            // CHECK CR/LF ('\r', '\n'):
            else if ((char)ch == '\r' || (char)ch == '\n') {
                if (cr_flag) {                                                    // If last char was CR, no need to set a new line again
                    cr_flag = false;
                    continue;
                }
                line[len] = '\0';
                len = 0;
                cr_flag = true;                                                   // Last char was CR
                uart_write_bytes(UART_NUM_0, "\r\n> ", 4);                        // New line
                check_word(line);                                                 // Check word  
            }   

            // CHECK FOR NEW CHAR:
            else if ((char)ch >= 0x20 && (char)ch <= 0x7E && len < (DATA_SIZE - 1)) {
                line[len++] = (char)ch;                                           // Add char to line
                uart_write_bytes(UART_NUM_0, &ch, 1);                             // Write char to terminal
                cr_flag = false;                                                  // Last char wasn't CR
            }
        }   
    }
}

//Pot read task:
/*
Check if read_semaphore is available, if so get potentiometer value,
map it to PWM's range (0-255), and add it to the queue.
Timer function gives read_semaphore every period.
When system turns ON read semaphore will be available for an immediate read.
If read_semaphore is not available, wait.
*/
void pot_read_task(void* pvParameter) {
    vTaskDelay(pdMS_TO_TICKS(20));                                                // Small delay to let the app finish

    int pot_val = 0;

    while (1) {
        if (xSemaphoreTake(read_semaphore, portMAX_DELAY) == pdTRUE) {
            pot_val = adc1_get_raw(ADC1_CHANNEL_0) * PWM_RANGE / POT_RANGE;       // Read the potentiometer
            xQueueOverwrite(pot_q, &pot_val);                                     // Add pot_val to the queue
        }
    }
}

//PWM task:
/*
Check if system has been turned ON/OFF.
If system is turned off, turn off the led.
If system is turned on, turn on the led with 
a new potentiometer read.
If the system is ON, when a new read is available
set and update duty.
*/
void pwm_task(void* pvParameter) {
    vTaskDelay(pdMS_TO_TICKS(20));                                                // Small delay to let the app finish

    ledc_fade_func_install(0);                                                    // Install fade functions

    int pot_val = 0;
    bool on_flag = false;

    while (1) {
        // CHECK STATUS:
        // Set ON/new DUTY:
        if (xSemaphoreTake(status_semaphore, 0)) {
            xSemaphoreGive(status_semaphore);
            if (on_flag == false) on_flag = true;
            else {
                if (xQueueReceive(pot_q, &pot_val, pdMS_TO_TICKS(READ_TIME)) == pdTRUE) {
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 
                        pot_val, HPOINT);                                         // Change PWM by the last pot_val read
                }
            }
        }
        // Set OFF:
        else {
            if (on_flag == true) {
                on_flag = false;
                ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                    LOW, HPOINT);                                                 // Turn off
            }
            vTaskDelay(pdMS_TO_TICKS(25));
        }
    }
}


// HELPFUL FUNCTIONS:
// Check the word:
void check_word(char* word) {
    if (is_word_on(word) == false) {                                              // If word is not "ON" check "OFF"
        if (is_word_off(word) == false) {                                         // If word is not "OFF" print help
            printf("\b\b \bPlease enter ON to turn the system ON\r\n");
            printf("or OFF to turn the system OFF\r\n");
            uart_write_bytes(UART_NUM_0, "> ", 2);
        } 
    }
}

// Is the word "ON":
/*
If the word is "ON"/"On"/"on" check if the system
is already ON. If the system is already ON print 
"Already ON". If the system was OFF, turn the 
system ON and print "System ON".
Return true if the word was "ON", else return
false.
*/
bool is_word_on(char* word) {
    if (!strcmp(word, "ON") || !strcmp(word, "On") 
    || !strcmp(word, "on")) {
        // CHECK STATUS:
        if (xSemaphoreTake(status_semaphore, 0) == pdTRUE) {  
            xSemaphoreGive(status_semaphore);                                     // Already ON, give semaphore back to indicate system is ON
            uart_write_bytes(UART_NUM_0, "\b\b \b", 4);
            ESP_LOGI(TAG, "Already ON");                                              
            uart_write_bytes(UART_NUM_0, "> ", 2);
        }
        else {
            xSemaphoreGive(status_semaphore);                                     // System was OFF, turn ON and give semaphores
            xSemaphoreGive(read_semaphore);
            uart_write_bytes(UART_NUM_0, "\b\b \b", 4);
            ESP_LOGI(TAG, "System ON");                                           
            uart_write_bytes(UART_NUM_0, "> ", 2);

            if (xTimerStart(read_timer, 0) != pdPASS) {                           // Start the timer
                ESP_LOGE(TAG, "Timer start failed");
                while (1);
            }                               
        }
        return true;
    }
    return false;
}

// Is the word "OFF":
/*
If the word is "OFF"/"Off"/"off" check if the system
is already OFF. if the system is ON print 
"System OFF" and turn the system OFF. If the system 
was already OFF, print "Already OFF".
Return true if the word was "OFF", else return
false.
*/
bool is_word_off(char* word) {
    if (!strcmp(word, "OFF") || !strcmp(word, "Off") 
    || !strcmp(word, "off")) {
        if (xSemaphoreTake(status_semaphore, 0) == pdTRUE) {                      // Turned OFF by taking status semaphore
            uart_write_bytes(UART_NUM_0, "\b\b \b", 4);
            ESP_LOGI(TAG, "System OFF");                                          
            uart_write_bytes(UART_NUM_0, "> ", 2);    
                           
            if (xTimerStop(read_timer, 0) != pdPASS) {                            // Stop the timer
                ESP_LOGE(TAG, "Timer stop failed");
                while (1);
            }      
        }
        else {                                                                    // Already OFF
            uart_write_bytes(UART_NUM_0, "\b\b \b", 4);
            ESP_LOGI(TAG, "Already OFF");                                         
            uart_write_bytes(UART_NUM_0, "> ", 2);
        }
        return true;
    }
    return false;
}