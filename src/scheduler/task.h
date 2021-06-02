/**
******************************************************************************
* @file    task.h
* @author
* @version V0.0.1
* @date    11/06/2020
* @brief   头文件，task相关函数声明.
******************************************************************************
*/

#pragma once

#include "hardware.h"
#include "stdbool.h"


typedef uint32_t timeUs_t;
typedef int32_t timeDelta_t;


typedef struct
{

    const char *taskName;
    const char *subTaskName;

    bool (*checkFunc)(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
    void (*taskFunc)(timeUs_t currentTimeUs);
    timeDelta_t desiredPeriodUs;      // target period of execution
    const int8_t staticPriority;    // dynamicPriority grows in steps of this size

    // Scheduling
    uint16_t dynamicPriority;       // measurement of how old task was last executed, used to avoid task starvation
    uint16_t taskAgeCycles;
    timeDelta_t taskLatestDeltaTimeUs;
    timeUs_t lastExecutedAtUs;        // last time of invocation
    timeUs_t lastSignaledAtUs;        // time of invocation event for event-driven tasks
    timeUs_t lastDesiredAt;         // time of last desired execution

    float    movingAverageCycleTimeUs;
    timeUs_t movingSumExecutionTimeUs;  // moving sum over 32 samples
    timeUs_t movingSumDeltaTimeUs;  // moving sum over 32 samples
    timeUs_t maxExecutionTimeUs;
    timeUs_t totalExecutionTimeUs;    // total time consumed by task since boot

} task_t;

typedef enum
{
    /* Actual tasks */
    TASK_SYSTEM = 0,
    TASK_GYRO,
    TASK_FILTER,
    TASK_PID,
    TASK_ACCEL,
    TASK_ATTITUDE,
    TASK_RX,
    TASK_SERIAL,
    TASK_BATTERY_VOLTAGE,
    TASK_BATTERY_CURRENT,
    TASK_BARO,

    TASK_ALTITUDE,

    TASK_OSD,
    TASK_LED,
    TASK_KEY,
    /* Count of real tasks */
    TASK_COUNT,

    /* Service task IDs */
    TASK_NONE = TASK_COUNT,
    TASK_SELF
} taskId_e;


typedef enum
{
    TASK_PRIORITY_REALTIME = -1, // Task will be run outside the scheduler logic
    TASK_PRIORITY_IDLE = 0,      // Disables dynamic scheduling, task is executed only if no other task is active this cycle
    TASK_PRIORITY_LOW = 1,
    TASK_PRIORITY_MEDIUM = 3,
    TASK_PRIORITY_MEDIUM_HIGH = 4,
    TASK_PRIORITY_HIGH = 5,
    TASK_PRIORITY_MAX = 255
} taskPriority_e;



#define TASK_PERIOD_HZ(hz) (1000000 / (hz))
#define TASK_PERIOD_MS(ms) ((ms) * 1000)
#define TASK_PERIOD_US(us) (us)


static inline int16_t cmp16(uint16_t a, uint16_t b) {
    return (int16_t)(a-b);
}
static inline int32_t cmp32(uint32_t a, uint32_t b) {
    return (int32_t)(a-b);
}


void tasksInit(void);
task_t *getTask(unsigned taskId);








