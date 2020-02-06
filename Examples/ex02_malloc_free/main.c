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
        vTaskDelay(10 / portTICK_PERIOD_MS);

        // Allocate memory ---------------------
        char *buffer = (char *)malloc(100 * sizeof(char));

        // Check if the memory can be allocated
        if (buffer != NULL) {
            // Use the allocated memory
            sprintf(buffer, "Counter: %i\r\n", counter++);
            UART1_Write(buffer);

            // Free memory
            free(buffer);   // Try to comment this line to see the error

        }
        else {
            // UART1_Write("Error Mem full\r\n");
            LED3_LAT = 1;
        }
        //--------------------------------------
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
