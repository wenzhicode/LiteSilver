/**
******************************************************************************
* @file    scheduler.h
* @author
* @version V0.0.1
* @date    11/06/2020
* @brief   头文件，scheduler相关函数声明.
******************************************************************************
*/



#include "hardware.h"
#include "task.h"


#define GYRO_TASK_GUARD_INTERVAL_US 10
#define TASK_AVERAGE_EXECUTE_FALLBACK_US 30
#define TASK_AVERAGE_EXECUTE_PADDING_US 5
#define TASK_STATS_MOVING_SUM_COUNT 32


void schedulerInit(void);


void scheduler(void);

void rescheduleTask(taskId_e taskId, timeDelta_t newPeriodUs);

