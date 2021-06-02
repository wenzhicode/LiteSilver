/**
******************************************************************************
* @file    dshot.h
* @author
* @version V0.0.1
* @date    27/05/2020
* @brief   dshot头文件，dshot相关函数声明.
******************************************************************************
*/

#include "stdbool.h"
#include "hardware.h"
#include "targets.h"


#if defined(MOTOR0_PIN_PB0) || defined(MOTOR0_PIN_PB1) || defined(MOTOR1_PIN_PB0) \
            || defined(MOTOR1_PIN_PB1) ||defined(MOTOR2_PIN_PB0) || defined(MOTOR2_PIN_PB1) \
            || defined(MOTOR3_PIN_PB0) || defined(MOTOR3_PIN_PB1)
#define DSHOT_DMA_PHASE 2           // motor pins at both portA and portB
#else
#define DSHOT_DMA_PHASE 1          // motor pins all at portA
#endif

#define DSHOT_CMD_ROTATE_NORMAL 20
#define DSHOT_CMD_ROTATE_REVERSE 21

// motor0

#ifdef MOTOR0_PIN_PB5
#define DSHOT_PIN_0 GPIO_Pin_5
#define DSHOT_PORT_0 GPIOB
#endif


#ifdef MOTOR1_PIN_PB4
#define DSHOT_PIN_1 GPIO_Pin_4
#define DSHOT_PORT_1 GPIOB
#endif


#ifdef MOTOR2_PIN_PB1
#define DSHOT_PIN_2 GPIO_Pin_1
#define DSHOT_PORT_2 GPIOB
#endif


#ifdef MOTOR3_PIN_PB0
#define DSHOT_PIN_3 GPIO_Pin_0
#define DSHOT_PORT_3 GPIOB
#endif


void dshot_init(void);

void pwm_set(uint8_t number, float pwm);


void motor_dir(unsigned char  number, unsigned char value);

void make_packet(uint8_t number, uint16_t value, bool telemetry);

void dshot_dma_start(void);




