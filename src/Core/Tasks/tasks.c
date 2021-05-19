
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "uart.h"
#include "can.h"
#include "board.h"

#include "tasks.h"

// FreeRTOS tasks
void taskUartToCan(void *pvParameters);
void taskGetTemp(void *pvParameters);
void taskSendToUart(void *pvParameters);

QueueHandle_t qStr;         // temperature string pointer queue
SemaphoreHandle_t sUartMutex;
SemaphoreHandle_t sCanMutex;


/**
 * @brief FreeRTOS tasks initialization
 */
void initTasks(void)
{
    qStr = xQueueCreate(1, sizeof(uint8_t*));
    sUartMutex = xSemaphoreCreateMutex();
    sCanMutex = xSemaphoreCreateMutex();

    xTaskCreate(taskUartToCan, "FROM UART TO CAN", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(taskGetTemp, "GET TEMPERATURE", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(taskSendToUart, "SEND DATA TO UART", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}


/**
 * @brief Retranslate received UART data to CAN interface
 */
void taskUartToCan(void *pvParameters)
{
    while(1) {
        if(rbuf.count != 0) {
            xSemaphoreTake(sCanMutex, portMAX_DELAY);

            can_putdata(rbuf.data, rbuf.count);

            rbuf.count = 0;

            xSemaphoreGive(sCanMutex);
        }
    }
}


/**
 * @brief Get temperature data
 */
void taskGetTemp(void *pvParameters)
{
    uint8_t* pStr;
    uint16_t temp;      // C = ((V25 - Vsense) / Avg_Slope) + 25;
    uint16_t volt;      // V = (4096[ADC 12 Bit] * 100[avoid fractional numbers]) / convertion value

    uint8_t str[] = {"Temperature: %xx% C | Voltage %y.yy% V\r\n"};

    pStr = str;

    while(1) {
        // division by zero avoidance
        if(conv == 0) conv = 1;

        temp = ((TEMP_SENSOR_V25 - conv) / TEMP_SENSOR_AVG_SLOPE) + 25;

        volt = ((4096 * 100) / conv);

        str[STR_TEMP_H] = (uint8_t)(temp / 10) + TEMP_STR_ASCII_ADD;
        str[STR_TEMP_L] = (uint8_t)(temp % 10) + TEMP_STR_ASCII_ADD;
        str[STR_VOLT_H] = (uint8_t)(volt / 100) + TEMP_STR_ASCII_ADD;
        str[STR_VOLT_L] = (uint8_t)((volt % 100) / 10) + TEMP_STR_ASCII_ADD;
        str[STR_VOLT_LL] = (uint8_t)(volt % 10) + TEMP_STR_ASCII_ADD;

        xQueueSend(qStr, &pStr, 3);

        vTaskDelay(TEMP_SEND_DELAY_MS);
    }
}


/**
 * @brief Send data to UART
 */
void taskSendToUart(void *pvParameters)
{
    uint8_t* pStr = 0;

    while(1) {
        xQueueReceive(qStr, &pStr, portMAX_DELAY);

        xSemaphoreTake(sUartMutex, portMAX_DELAY);

        uart_putdata(pStr, TEMP_STR_SIZE);

        // wait uart transmit complete flag
        while(getUartTF() == 0);

        xSemaphoreGive(sUartMutex);
    }
}
