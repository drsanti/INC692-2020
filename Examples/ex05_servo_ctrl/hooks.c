#include "ecc_pic24_bsp.h"

void vApplicationIdleHook(void)
{
    vCoRoutineSchedule();
}

void vApplicationTickHook(void)
{
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)pxTask;
    taskDISABLE_INTERRUPTS();
    while (1)
    {
    }
}
