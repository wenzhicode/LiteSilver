/**
******************************************************************************
* @file    scheduler.c
* @author
* @version V0.0.1
* @date    11/06/2020
* @brief   scheduler�ļ���scheduler��غ���.
******************************************************************************

�����л�

*/



#include "scheduler.h"
#include "stdbool.h"
#include "time.h"
#include "string.h"
#include "maths.h"
#include "stddef.h"



bool calculateTaskStatistics;
task_t *taskQueueArray[TASK_COUNT + 1];
task_t *currentTask = NULL;
int taskQueueSize = 0;
int taskQueuePos = 0;


timeUs_t checkFuncMaxExecutionTimeUs;
timeUs_t checkFuncTotalExecutionTimeUs;
timeUs_t checkFuncMovingSumExecutionTimeUs;
timeUs_t checkFuncMovingSumDeltaTimeUs;

uint32_t totalWaitingTasksSamples;
uint32_t totalWaitingTasks;


#define TASK_GYROPID_DESIRED_PERIOD     1000 // 1000us = 1kHz
#define SCHEDULER_DELAY_LIMIT           100

extern uint8_t pidUpdateCounter;


int periodCalculationBasisOffset = offsetof(task_t, lastExecutedAtUs);


bool gyroFilterReady(void)
{
    if (pidUpdateCounter % 2 == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool pidLoopReady(void)
{
    if ((pidUpdateCounter % 2) == 1)
    {
        return true;
    }
    return false;
}

void schedulerResetTaskStatistics(taskId_e taskId)
{
    if (taskId == TASK_SELF)
    {
        currentTask->movingSumExecutionTimeUs = 0;
        currentTask->movingSumDeltaTimeUs = 0;
        currentTask->totalExecutionTimeUs = 0;
        currentTask->maxExecutionTimeUs = 0;
    }
    else if (taskId < TASK_COUNT)
    {
        getTask(taskId)->movingSumExecutionTimeUs = 0;
        getTask(taskId)->movingSumDeltaTimeUs = 0;
        getTask(taskId)->totalExecutionTimeUs = 0;
        getTask(taskId)->maxExecutionTimeUs = 0;
    }
}


/********************************************************************************
**������Ϣ ��queueClear(void)
**�������� ������������
**������� ����
**������� ����
*********************************************************************************/
void queueClear(void)
{
    memset(taskQueueArray, 0, sizeof(taskQueueArray));
    taskQueuePos = 0;
    taskQueueSize = 0;
}


/********************************************************************************
**������Ϣ ��queueContains(task_t *task)
**�������� ����������Ƿ����
**������� ��task ������
**������� ���Ƿ����
*********************************************************************************/
bool queueContains(task_t *task)
{
    for (int ii = 0; ii < taskQueueSize; ++ii)
    {
        if (taskQueueArray[ii] == task)
        {
            return true;
        }
    }
    return false;
}


/********************************************************************************
**������Ϣ ��queueAdd(task_t *task)
**�������� ���������
**������� ��task ������
**������� ����ӳɹ����
*********************************************************************************/
bool queueAdd(task_t *task)
{
    if ((taskQueueSize >= TASK_COUNT) || queueContains(task))
    {
        return false;
    }
    for (int ii = 0; ii <= taskQueueSize; ++ii)
    {
        if (taskQueueArray[ii] == NULL || taskQueueArray[ii]->staticPriority < task->staticPriority)
        {
            memmove(&taskQueueArray[ii + 1], &taskQueueArray[ii], sizeof(task) * (taskQueueSize - ii));
            taskQueueArray[ii] = task;
            ++taskQueueSize;
            return true;
        }
    }
    return false;
}


/********************************************************************************
**������Ϣ ��queueRemove(task_t *task)
**�������� ���Ƴ�����
**������� ��task ������
**������� ���Ƴ��ɹ����
*********************************************************************************/
bool queueRemove(task_t *task)
{
    for (int ii = 0; ii < taskQueueSize; ++ii)
    {
        if (taskQueueArray[ii] == task)
        {
            memmove(&taskQueueArray[ii], &taskQueueArray[ii + 1], sizeof(task) * (taskQueueSize - ii));
            --taskQueueSize;
            return true;
        }
    }
    return false;
}

/*
 * Returns first item queue or NULL if queue empty
 */
task_t *queueFirst(void)
{
    taskQueuePos = 0;
    return taskQueueArray[0]; // guaranteed to be NULL if queue is empty
}

/*
 * Returns next item in queue or NULL if at end of queue
 */
task_t *queueNext(void)
{
    return taskQueueArray[++taskQueuePos]; // guaranteed to be NULL at end of queue
}


/********************************************************************************
**������Ϣ ��setTaskEnabled(taskId_e taskId, bool enabled)
**�������� ��ʹ������
**������� ��taskId ����ID  enabled ʹ�ܱ�־
**������� ���Ƴ��ɹ����
*********************************************************************************/
void setTaskEnabled(taskId_e taskId, bool enabled)
{
    if (taskId == TASK_SELF || taskId < TASK_COUNT)
    {
        task_t *task = taskId == TASK_SELF ? currentTask : getTask(taskId);
        if (enabled && task->taskFunc)
        {
            queueAdd(task);
        }
        else
        {
            queueRemove(task);
        }
    }
}

/********************************************************************************
**������Ϣ ��schedulerInit(void)
**�������� �������л���ʼ��
**������� ����
**������� ����
*********************************************************************************/
void schedulerInit(void)
{
    calculateTaskStatistics = true;
    queueClear();
    queueAdd(getTask(TASK_SYSTEM));

}


/********************************************************************************
**������Ϣ ��schedulerExecuteTask(task_t *selectedTask, timeUs_t currentTimeUs)
**�������� ��ִ������
**������� ��selectedTask Ҫִ�е�����  currentTimeUs ��ǰʱ��
**������� ������ִ��ʱ��
*********************************************************************************/
timeUs_t schedulerExecuteTask(task_t *selectedTask, timeUs_t currentTimeUs)
{
    timeUs_t taskExecutionTimeUs = 0;

    if (selectedTask)
    {
        currentTask = selectedTask;
        selectedTask->taskLatestDeltaTimeUs = cmpTimeUs(currentTimeUs, selectedTask->lastExecutedAtUs);

        float period = currentTimeUs - selectedTask->lastExecutedAtUs;

        selectedTask->lastExecutedAtUs = currentTimeUs;
        selectedTask->lastDesiredAt += (cmpTimeUs(currentTimeUs, selectedTask->lastDesiredAt) / selectedTask->desiredPeriodUs) * selectedTask->desiredPeriodUs;
        selectedTask->dynamicPriority = 0;

        // Execute task

        if (calculateTaskStatistics)
        {
            const timeUs_t currentTimeBeforeTaskCallUs = gettime();
            selectedTask->taskFunc(currentTimeBeforeTaskCallUs);
            taskExecutionTimeUs = gettime() - currentTimeBeforeTaskCallUs;
            selectedTask->movingSumExecutionTimeUs += taskExecutionTimeUs - selectedTask->movingSumExecutionTimeUs / 32;
            selectedTask->movingSumDeltaTimeUs += selectedTask->taskLatestDeltaTimeUs - selectedTask->movingSumDeltaTimeUs / 32;
            selectedTask->totalExecutionTimeUs += taskExecutionTimeUs;   // time consumed by scheduler + task
            selectedTask->maxExecutionTimeUs = MAX(selectedTask->maxExecutionTimeUs, taskExecutionTimeUs);
            selectedTask->movingAverageCycleTimeUs += 0.05f * (period - selectedTask->movingAverageCycleTimeUs);
        }
        else
        {
            selectedTask->taskFunc(currentTimeUs);
        }
    }

    return taskExecutionTimeUs;
}



/********************************************************************************
**������Ϣ ��getPeriodCalculationBasis(const task_t* task)
**�������� ����ȡ�����ϴ�ִ�е�ʱ���
**������� ��task ������
**������� ������ʱ���
*********************************************************************************/
inline static timeUs_t getPeriodCalculationBasis(const task_t *task)
{
    if (task->staticPriority == TASK_PRIORITY_REALTIME)
    {
        return *(timeUs_t *)((uint8_t *)task + periodCalculationBasisOffset);
    }
    else
    {
        return task->lastExecutedAtUs;
    }
}


/********************************************************************************
**������Ϣ ��rescheduleTask(taskId_e taskId, timeDelta_t newPeriodUs)
**�������� �����������ִ��ʱ��
**������� ��task ������ newPeriodUs ���
**������� ��
*********************************************************************************/
void rescheduleTask(taskId_e taskId, timeDelta_t newPeriodUs)
{
    if (taskId == TASK_SELF) {
        task_t *task = currentTask;
        task->desiredPeriodUs = MAX(SCHEDULER_DELAY_LIMIT, newPeriodUs);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
    } else if (taskId < TASK_COUNT) {
        task_t *task = getTask(taskId);
        task->desiredPeriodUs = MAX(SCHEDULER_DELAY_LIMIT, newPeriodUs);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
    }
}



/********************************************************************************
**������Ϣ ��scheduler(void)
**�������� ���л�����ִ��
**������� ����
**������� ����

ʵʱ����gyro, filter, pid

�¼�����: rxUpdate

ʱ��������accel, ...

���ȱ�֤ʵʱ�����ִ�У�Ȼ�����¼����������Ƿ����¼�����

��ʱ������������������ȼ�ִ��


*********************************************************************************/
void scheduler(void)
{
    // Cache currentTime
    const timeUs_t schedulerStartTimeUs = gettime();
    timeUs_t currentTimeUs = schedulerStartTimeUs;
    timeUs_t taskExecutionTimeUs = 0;
    task_t *selectedTask = NULL;
    uint16_t selectedTaskDynamicPriority = 0;
    uint16_t waitingTasks = 0;
    bool realtimeTaskRan = false;
    timeDelta_t gyroTaskDelayUs = 0;

    task_t *gyroTask = getTask(TASK_GYRO);

    const timeUs_t gyroExecuteTimeUs = getPeriodCalculationBasis(gyroTask) + gyroTask->desiredPeriodUs;

    gyroTaskDelayUs = cmpTimeUs(gyroExecuteTimeUs, currentTimeUs);  // time until the next expected gyro sample

    if (cmpTimeUs(currentTimeUs, gyroExecuteTimeUs) >= 0)
    {
        taskExecutionTimeUs = schedulerExecuteTask(gyroTask, currentTimeUs);
        if (gyroFilterReady())
        {
            taskExecutionTimeUs += schedulerExecuteTask(getTask(TASK_FILTER), currentTimeUs);
        }
        if (pidLoopReady())
        {
            taskExecutionTimeUs += schedulerExecuteTask(getTask(TASK_PID), currentTimeUs);
        }
        currentTimeUs = gettime();
        realtimeTaskRan = true;
    }


    if (realtimeTaskRan || (gyroTaskDelayUs > GYRO_TASK_GUARD_INTERVAL_US))
    {
        // The task to be invoked

        // Update task dynamic priorities
        for (task_t *task = queueFirst(); task != NULL; task = queueNext())
        {
            if (task->staticPriority != TASK_PRIORITY_REALTIME)
            {
                // Task has checkFunc - event driven
                if (task->checkFunc)
                {
                    const timeUs_t currentTimeBeforeCheckFuncCallUs = currentTimeUs;

                    // Increase priority for event driven tasks
                    if (task->dynamicPriority > 0)
                    {
                        task->taskAgeCycles = 1 + ((currentTimeUs - task->lastSignaledAtUs) / task->desiredPeriodUs);
                        task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                        waitingTasks++;
                    }
                    else if (task->checkFunc(currentTimeBeforeCheckFuncCallUs, cmpTimeUs(currentTimeBeforeCheckFuncCallUs, task->lastExecutedAtUs)))
                    {
                        if (calculateTaskStatistics)
                        {
                            const uint32_t checkFuncExecutionTimeUs = gettime() - currentTimeBeforeCheckFuncCallUs;
                            checkFuncMovingSumExecutionTimeUs += checkFuncExecutionTimeUs - checkFuncMovingSumExecutionTimeUs / 32;
                            checkFuncMovingSumDeltaTimeUs += task->taskLatestDeltaTimeUs - checkFuncMovingSumDeltaTimeUs / 32;
                            checkFuncTotalExecutionTimeUs += checkFuncExecutionTimeUs;   // time consumed by scheduler + task
                            checkFuncMaxExecutionTimeUs = MAX(checkFuncMaxExecutionTimeUs, checkFuncExecutionTimeUs);
                        }

                        task->lastSignaledAtUs = currentTimeBeforeCheckFuncCallUs;
                        task->taskAgeCycles = 1;
                        task->dynamicPriority = 1 + task->staticPriority;
                        waitingTasks++;
                    }
                    else
                    {
                        task->taskAgeCycles = 0;
                    }
                }
                else
                {
                    // Task is time-driven, dynamicPriority is last execution age (measured in desiredPeriods)
                    // Task age is calculated from last execution
                    task->taskAgeCycles = ((currentTimeUs - getPeriodCalculationBasis(task)) / task->desiredPeriodUs);
                    if (task->taskAgeCycles > 0)
                    {
                        task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                        waitingTasks++;
                    }
                }

                if (task->dynamicPriority > selectedTaskDynamicPriority)
                {
                    selectedTaskDynamicPriority = task->dynamicPriority;
                    selectedTask = task;
                }
            }
        }
        totalWaitingTasksSamples++;
        totalWaitingTasks += waitingTasks;

        if (selectedTask)
        {
            timeDelta_t taskRequiredTimeUs = TASK_AVERAGE_EXECUTE_FALLBACK_US;  // default average time if task statistics are not available

            if (calculateTaskStatistics)
            {
                taskRequiredTimeUs = selectedTask->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT + TASK_AVERAGE_EXECUTE_PADDING_US;
            }

            // Add in the time spent so far in check functions and the scheduler logic
            taskRequiredTimeUs += cmpTimeUs(gettime(), currentTimeUs);
            if (realtimeTaskRan || (taskRequiredTimeUs < gyroTaskDelayUs))
            {
                taskExecutionTimeUs += schedulerExecuteTask(selectedTask, currentTimeUs);
            }
            else
            {
                selectedTask = NULL;
            }
        }
    }
}

