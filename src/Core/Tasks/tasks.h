
#ifndef _TASKS_H_
#define _TASKS_H_

typedef enum
{
    TEMP_SEND_DELAY_MS = 6000,
    TEMP_STR_SIZE = 39,
    TEMP_STR_ASCII_ADD = 48,
} tasks_param_t;

typedef enum
{
    STR_TEMP_H = 14,
    STR_TEMP_L = (STR_TEMP_H + 1),
    STR_VOLT_H = 31,
    STR_VOLT_L = (STR_VOLT_H + 2),
    STR_VOLT_LL = (STR_VOLT_L + 1)
} str_pos_t;

typedef enum
{
    TEMP_SENSOR_V25 = 1750,             // when V25=1.41V at ref 3.3V
    TEMP_SENSOR_AVG_SLOPE = 5,          // when avg_slope=4.3mV/C at ref 3.3V
} temp_sensor_t;


void initTasks(void);

#endif  // _TASKS_H_
