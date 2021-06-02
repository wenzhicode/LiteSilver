/**
******************************************************************************
* @file    blackbox.c
* @author
* @version V0.0.1
* @date    4/06/2020
* @brief   blackbox文件，blackbox相关函数.
******************************************************************************
*/



#include "blackbox.h"
#include "serial_uart.h"
#include "time.h"
#include "blackbox_encoding.h"
#include "stddef.h"
#include "stdbool.h"
#include "gyro.h"
#include "acc.h"



extern float accelraw[3];
extern float gyro[3];

static int iframetemp = 0;
static int pframetemp = 0;
uint32_t blackboxIteration;

blackboxMainState_t blackboxHistoryRing[3];
blackboxMainState_t *blackboxHistory[3];

extern uint16_t bufNum;



bool blackboxShouldLogIFrame(void)
{
    // return blackboxLoopIndex == 0;

    iframetemp++;
    blackboxIteration++;

    if (iframetemp >= 32)
    {
        iframetemp = 0;

        return true;
    }
    else
    {
        return false;
    }
}


bool blackboxShouldLogPFrame(void)
{
    //return blackboxPFrameIndex == 0 && blackboxConfig()->p_ratio != 0;

    pframetemp++;
    if (pframetemp >= 2)
    {
        pframetemp = 0;
        return true;
    }
    else
    {
        return false;
    }
}



void blackboxStart(void)
{
    blackboxHistory[0] = &blackboxHistoryRing[0];
    blackboxHistory[1] = &blackboxHistoryRing[1];
    blackboxHistory[2] = &blackboxHistoryRing[2];
}

static void loadMainState(void)
{
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];

    blackboxCurrent->time = gettime();

    for (int i = 0; i < 3; i++)
    {

//        blackboxCurrent->gyroADC[i] = gyro.gyroDev.gyroADCRaw[i];

//        blackboxCurrent->accADC[i] = acc.dev.ADCRaw[i];
    }
}


static void writeIntraframe(void)
{
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];

    blackboxWrite('I');

    blackboxWriteUnsignedVB(blackboxIteration);
    blackboxWriteUnsignedVB(blackboxCurrent->time);

    blackboxWriteSigned16VBArray(blackboxCurrent->gyroADC, 3);

    blackboxWriteSigned16VBArray(blackboxCurrent->accADC, 3);

    //Rotate our history buffers:

    //The current state becomes the new "before" state
    blackboxHistory[1] = blackboxHistory[0];
    //And since we have no other history, we also use it for the "before, before" state
    blackboxHistory[2] = blackboxHistory[0];
    //And advance the current state over to a blank space ready to be filled
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;

}


static void blackboxWriteMainStateArrayUsingAveragePredictor(int arrOffsetInHistory, int count)
{
    int16_t *curr  = (int16_t *)((char *)(blackboxHistory[0]) + arrOffsetInHistory);
    int16_t *prev1 = (int16_t *)((char *)(blackboxHistory[1]) + arrOffsetInHistory);
    int16_t *prev2 = (int16_t *)((char *)(blackboxHistory[2]) + arrOffsetInHistory);

    for (int i = 0; i < count; i++)
    {
        // Predictor is the average of the previous two history states
        int32_t predictor = (prev1[i] + prev2[i]) / 2;

        blackboxWriteSignedVB(curr[i] - predictor);
    }
}


static void writeInterframe(void)
{
//    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
//    blackboxMainState_t *blackboxLast = blackboxHistory[1];

    blackboxWrite('P');

    blackboxWriteSignedVB((int32_t)(blackboxHistory[0]->time - 2 * blackboxHistory[1]->time + blackboxHistory[2]->time));

    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, gyroADC),   3);

    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, accADC), 3);

    blackboxHistory[2] = blackboxHistory[1];
    blackboxHistory[1] = blackboxHistory[0];
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;

}



void blackboxUpdate(void)
{
    bufNum = 0;
    if (blackboxShouldLogIFrame())
    {

        loadMainState();
        writeIntraframe();

        UART2_DMA_Send();
    }
    else
    {
        if (blackboxShouldLogPFrame())
        {
            loadMainState();
            writeInterframe();

            UART2_DMA_Send();
        }
    }
}



