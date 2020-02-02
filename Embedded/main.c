
#include "ecc_pic24_bps.h"

// Task Code
void vTask_LED0(void *pvParameters)
{
    for (;;)
    {
        LED0_LAT = !LED0_LAT;                  // Toggle LED3
        vTaskDelay(8000 / portTICK_PERIOD_MS); // Wait for .1 seconds
    }
}
void vTask_LED3(void *pvParameters)
{
    while (1)
    {
        LED3_LAT = !LED3_LAT;                                         // Toggle LED0
        vTaskDelay(*((uint32_t *)pvParameters) / portTICK_PERIOD_MS); // Wait for T seconds
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
    xTaskCreate(vTask_LED0, NULL, 128, NULL, 0, NULL);
    xTaskCreate(vTask_LED3, NULL, 128, (void *)100, 0, NULL);

    // Start
    vTaskStartScheduler();

    //
    return 0;
}
