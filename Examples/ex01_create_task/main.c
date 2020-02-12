
#include "ecc_pic24_bsp.h"

// Task Code
void vTask_LED0(void *pvParameters)
{
    while (1)
    {
        LED0_LAT = !LED0_LAT; // Toggle LED0
        vTaskDelay((TickType_t)pvParameters / portTICK_PERIOD_MS);
    }
}
void vTask_LED3(void *pvParameters)
{
    int counter = 0;
    TickType_t *ticks = (TickType_t *)pvParameters;

    char buff[32];
    while (1)
    {
        //sprintf(buff, "counter: %d\r\n", counter++);
        //UART1_Write(buff);

        LED3_LAT = !LED3_LAT; // Toggle LED3
        vTaskDelay((*ticks) / portTICK_PERIOD_MS);
    }
}

int main(void)
{
    // Hardware Setup
    System_Init();

    // Turn OFF LED<3:0>
    LED0_LAT = LED1_LAT = 1;
    LED2_LAT = LED3_LAT = 1;

    // Create Tasks
    xTaskCreate(vTask_LED0, NULL, 128, (void *)200, 0, NULL); // pass a constant value as the parameter

    TickType_t ticks = 20;
    xTaskCreate(vTask_LED3, NULL, 128, (void *)&ticks, 0, NULL); // pass an address of variable as the parameter

    // Start
    vTaskStartScheduler();

    //
    return 0;
}
