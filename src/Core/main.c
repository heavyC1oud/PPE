
#include "board.h"

#include "FreeRTOS.h"
#include "task.h"


int main(void)
{
    initMCU();

    // start FreeRTOS scheduler
    vTaskStartScheduler();

    // program never run here
    while(1);
}
