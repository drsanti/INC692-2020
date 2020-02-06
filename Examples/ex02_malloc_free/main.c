// ex02: Dynamic memory allocation

#include "ecc_pic24_bsp.h"
#include <stdio.h>
#include <stdlib.h>

// Task Code
void vTask_A(void *pvParameters)
{
    int counter = 0;
    while (1)
    {
        LED0_LAT = !LED0_LAT; // Toggle LED0
        vTaskDelay(500 / portTICK_PERIOD_MS);

        // Allocate memory
        char *buffer = (char *)malloc(32);

        sprintf(buffer, "Counter: %i\r\n", counter);
        UART1_Write(buffer);

        // Free memory
        free(buffer);
    }
}

int main(void)
{
    // Hardware Setup
    System_Init();

    // Create Tasks
    xTaskCreate(vTask_A, NULL, 128, NULL, 0, NULL);

    // Start
    vTaskStartScheduler();
    return 0;
}
