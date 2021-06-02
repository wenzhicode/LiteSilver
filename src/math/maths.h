/**
******************************************************************************
* @file    maths.h
* @author
* @version V0.0.1
* @date    2/06/2020
* @brief   头文件，maths相关函数声明.
******************************************************************************
*/

#pragma once

#include "hardware.h"
#include "stddef.h"


#define _sinf(val) sinf(val)
#define _cosf(val) cosf(val)

#define M_PI  3.14159265    /* pi */

#define DEGTORAD 0.017453292f
#define RADTODEG 57.29577951f

#define OCTANTIFY(_x, _y, _o)   do {                            \
    float _t;                                                   \
    _o= 0;                                                \
    if(_y<  0)  {            _x= -_x;   _y= -_y; _o += 4; }     \
    if(_x<= 0)  { _t= _x;    _x=  _y;   _y= -_t; _o += 2; }     \
    if(_x<=_y)  { _t= _y-_x; _x= _x+_y; _y=  _t; _o += 1; }     \
} while(0);


void gyro_init(void);

float atan2approx(float y, float x);

float Q_rsqrt(float number);

uint32_t castFloatBytesToInt(float f);

#define MIN(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a < _b ? _a : _b; })
#define MAX(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a > _b ? _a : _b; })
#define ABS(x) \
  __extension__ ({ __typeof__ (x) _x = (x); \
  _x > 0 ? _x : -_x; })


typedef enum
{
    USB_STATE_ATTACHED = 0,
    USB_STATE_POWERED = 1,
    USB_STATE_DEFAULT = 2,
    USB_STATE_ADDRESS = 3,
    USB_STATE_CONFIGURED = 4,
    USB_STATE_SUSPENDED = 5,
} USB_STATE_t;

// Use floating point M_PI instead explicitly.
#define M_PIf       3.14159265358979323846f

#define RAD    (M_PIf / 180.0f)
#define DEGREES_TO_DECIDEGREES(angle) ((angle) * 10)
#define DECIDEGREES_TO_DEGREES(angle) ((angle) / 10)
#define DECIDEGREES_TO_RADIANS(angle) ((angle) / 10.0f * 0.0174532925f)
#define DEGREES_TO_RADIANS(angle) ((angle) * 0.0174532925f)


float atan2_approx(float y, float x);

float cos_approx(float x);
float sin_approx(float x);


int scaleRange(int x, int srcFrom, int srcTo, int destFrom, int destTo);
float scaleRangef(float x, float srcFrom, float srcTo, float destFrom, float destTo);


