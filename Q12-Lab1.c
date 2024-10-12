#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/hw_timer.h"
#include "driver/uart.h"
#include "freertos/semphr.h"
#include "freertos/FreeRTOSConfig.h"

#define UART_BUFFER_SMALL      1024
#define UART_BUFFER_LARGE      2048
#define NO_INPUT_DATA          0
#define SEMAPHORE_TIMEOUT_MS   100
#define ONE_SECOND_DELAY_MS    1000
#define HALF_SECOND_DELAY_MS   500
#define GPIO_STATE_HIGH        1
#define GPIO_STATE_LOW         0

/* Handle for the global semaphore */
SemaphoreHandle_t semaphore_handle;

/* Function to initialize the UART */
void InitializeUART(void)
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_0, &uart_config); /* Set UART configuration */
    uart_driver_install(UART_NUM_0, UART_BUFFER_LARGE, NO_INPUT_DATA, NO_INPUT_DATA, NULL, NO_INPUT_DATA);
}

/* Function to initialize GPIO */
void InitializeGPIO(void)
{
    gpio_config_t gpio_config = {
        .mode         = GPIO_MODE_DEF_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << GPIO_NUM_2) /* Configure GPIO2 pin */
    };
    gpio_config(&gpio_config); /* Apply GPIO configuration */
}

/* Task 1: Turns the LED on */
void TaskLEDOn(void *pv_parameters)
{
    while (1)
    {
        if (xSemaphoreTake(semaphore_handle, (TickType_t) SEMAPHORE_TIMEOUT_MS) == pdTRUE)
        {
            /* Set GPIO2 to high, turning the LED on */
            gpio_set_level(GPIO_NUM_2, GPIO_STATE_HIGH);
            vTaskDelay(HALF_SECOND_DELAY_MS / portTICK_PERIOD_MS); /* Wait for 500 milliseconds */
            xSemaphoreGive(semaphore_handle);
        }
        else
        {
            printf("Semaphore unavailable\n"); /* Notify when semaphore can't be acquired */
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); /* Short delay to avoid busy looping */
    }
}

/* Task 2: Turns the LED off */
void TaskLEDOff(void *pv_parameters)
{
    while (1)
    {
        if (xSemaphoreTake(semaphore_handle, (TickType_t) SEMAPHORE_TIMEOUT_MS) == pdTRUE)
        {
            /* Set GPIO2 to low, turning the LED off */
            gpio_set_level(GPIO_NUM_2, GPIO_STATE_LOW);
            vTaskDelay(ONE_SECOND_DELAY_MS / portTICK_PERIOD_MS); /* Wait for 1 second */
            xSemaphoreGive(semaphore_handle);
        }
        else
        {
            printf("Semaphore unavailable\n"); /* Notify when semaphore can't be acquired */
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); /* Short delay to avoid busy looping */
    }
}

/* Task 3: Outputs a status message */
void TaskPrintStatus(void *pv_parameters)
{
    while (1)
    {
        /* Send a status message via UART */
        printf("Task 3: System is running\n");
        vTaskDelay(ONE_SECOND_DELAY_MS / portTICK_PERIOD_MS); /* Wait for 1 second */
    }
}

/* Main application function */
void app_main(void)
{
    InitializeGPIO(); /* Set up GPIO */
    InitializeUART(); /* Set up UART */

    const int task_priority_1 = 1; // Priority of Task 1
    const int task_priority_2 = 2; // Priority of Task 2
    const int task_priority_3 = 3; // Priority of Task 3

    semaphore_handle = xSemaphoreCreateBinary(); /* Create a binary semaphore */

    if (semaphore_handle != NULL)
    {
        xSemaphoreGive(semaphore_handle); /* Give initial access to the semaphore */

        /* Create tasks with their respective priorities */
        xTaskCreate(TaskLEDOn, "Task LED On", UART_BUFFER_LARGE, NULL, task_priority_1, NULL);
        xTaskCreate(TaskLEDOff, "Task LED Off", UART_BUFFER_LARGE, NULL, task_priority_2, NULL);
        xTaskCreate(TaskPrintStatus, "Task Print Status", UART_BUFFER_LARGE, NULL, task_priority_3, NULL);
    }
}
