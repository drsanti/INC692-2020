// ex03:

#include "ecc_pic24_bsp.h"

SemaphoreHandle_t xSemaphore;

// Task Code
void vTask_A(void *pvParameters) {
    while (1)  {
        // Try to take the semaphore
        if (xSemaphoreTake(xSemaphore, (TickType_t)5000) == pdTRUE) {
            // Use the shared resource
            UART1_Write("A Take...aaaaaaaaaaaaaaaaaaaaaaa\r\n");
            // Give the semaphore
            xSemaphoreGive(xSemaphore);
        }
        else{
            UART1_Write("A failed...\r\n");
        }

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void vTask_B(void *pvParameters) {
    while (1) {
        // Try to take the semaphore
        if (xSemaphoreTake(xSemaphore, (TickType_t)5000) == pdTRUE) {
            // Use the shared resource
            UART1_Write("\tB Take...bbbbbbbbbbbbbbbbbbbbbb\r\n");
            // Give the semaphore
            xSemaphoreGive(xSemaphore);
        } else {
            UART1_Write("\tB failed...\r\n");
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

int main(void)
{
    // Hardware Setup
    System_Init();

    // Create a semaphore
    xSemaphore = xSemaphoreCreateBinary();

    // Give the semaphore
    xSemaphoreGive(xSemaphore);

    // Create Tasks
    xTaskCreate(vTask_A, NULL, 128, NULL, 0, NULL);
    xTaskCreate(vTask_B, NULL, 128, NULL, 0, NULL);

    // Start
    vTaskStartScheduler();
    return 0;
}
