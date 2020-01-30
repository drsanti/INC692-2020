
#include "ecc_pic24_bps.h"

void Task1(void *pvParameters)
{
	for (;;)
	{
		LED0_LAT = !LED0_LAT;
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}

void Task2(void *pvParameters)
{
	for (;;)
	{
		LED1_LAT = !LED1_LAT;
		vTaskDelay(300 / portTICK_PERIOD_MS);
	}
}

int main(void)
{
	System_Init();

	LED2_LAT = LED3_LAT = 1;

	xTaskCreate(Task1, NULL, 128, NULL, tskIDLE_PRIORITY, NULL);
	xTaskCreate(Task2, NULL, 128, NULL, tskIDLE_PRIORITY, NULL);

	vTaskStartScheduler();

	return 0;
}
