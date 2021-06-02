/**
******************************************************************************
* @file    maths.c
* @author
* @version V0.0.1
* @date    2/06/2020
* @brief   maths文件，math相关函数.
******************************************************************************
*/

#include "maths.h"
#include "math.h"

uint32_t castFloatBytesToInt(float f)
{
    union floatConvert_t
    {
        float f;
        uint32_t u;
    } floatConvert;

    floatConvert.f = f;

    return floatConvert.u;
}

float Q_rsqrt(float number)
{

    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y  = number;
    i  = * (long *) &y;
    i  = 0x5f3759df - (i >> 1);
    y  = * (float *) &i;
    y  = y * (threehalfs - (x2 * y * y));       // 1st iteration
    y  = y * (threehalfs - (x2 * y * y));       // 2nd iteration, this can be removed
//  y  = y * ( threehalfs - ( x2 * y * y ) );   // 3nd iteration, this can be removed

    return y;
}


float atan2approx(float y, float x)
{

    if (x == 0)
        x = 123e-15f;
    float phi = 0;
    float dphi;
    float t;

    OCTANTIFY(x, y, phi);

    t = (y / x);
    // atan function for 0 - 1 interval
    dphi = t * ((M_PI / 4 + 0.2447f) + t * ((-0.2447f + 0.0663f) + t * (- 0.0663f)));
    phi *= M_PI / 4;
    dphi = phi + dphi;
    if (dphi > (float) M_PI)
        dphi -= 2 * M_PI;
    return RADTODEG * dphi;
}


// http://lolengine.net/blog/2011/12/21/better-function-approximations
// Chebyshev http://stackoverflow.com/questions/345085/how-do-trigonometric-functions-work/345117#345117
// Thanks for ledvinap for making such accuracy possible! See: https://github.com/cleanflight/cleanflight/issues/940#issuecomment-110323384
// https://github.com/Crashpilot1000/HarakiriWebstore1/blob/master/src/mw.c#L1235
// sin_approx maximum absolute error = 2.305023e-06
// cos_approx maximum absolute error = 2.857298e-06
#define sinPolyCoef3 -1.666568107e-1f
#define sinPolyCoef5  8.312366210e-3f
#define sinPolyCoef7 -1.849218155e-4f
#define sinPolyCoef9  0

float sin_approx(float x)
{
    int32_t xint = x;
    if (xint < -32 || xint > 32) return 0.0f;                               // Stop here on error input (5 * 360 Deg)
    while (x >  M_PIf) x -= (2.0f * M_PIf);                                 // always wrap input angle to -PI..PI
    while (x < -M_PIf) x += (2.0f * M_PIf);
    if (x > (0.5f * M_PIf)) x = (0.5f * M_PIf) - (x - (0.5f * M_PIf));     // We just pick -90..+90 Degree
    else if (x < -(0.5f * M_PIf)) x = -(0.5f * M_PIf) - ((0.5f * M_PIf) + x);
    float x2 = x * x;
    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

float cos_approx(float x)
{
    return sin_approx(x + (0.5f * M_PIf));
}

// Initial implementation by Crashpilot1000 (https://github.com/Crashpilot1000/HarakiriWebstore1/blob/396715f73c6fcf859e0db0f34e12fe44bace6483/src/mw.c#L1292)
// Polynomial coefficients by Andor (http://www.dsprelated.com/showthread/comp.dsp/21872-1.php) optimized by Ledvinap to save one multiplication
// Max absolute error 0,000027 degree
// atan2_approx maximum absolute error = 7.152557e-07 rads (4.098114e-05 degree)
float atan2_approx(float y, float x)
{
#define atanPolyCoef1  3.14551665884836e-07f
#define atanPolyCoef2  0.99997356613987f
#define atanPolyCoef3  0.14744007058297684f
#define atanPolyCoef4  0.3099814292351353f
#define atanPolyCoef5  0.05030176425872175f
#define atanPolyCoef6  0.1471039133652469f
#define atanPolyCoef7  0.6444640676891548f

    float res, absX, absY;
    absX = fabsf(x);
    absY = fabsf(y);
    res  = MAX(absX, absY);
    if (res) res = MIN(absX, absY) / res;
    else res = 0.0f;
    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
    if (absY > absX) res = (M_PIf / 2.0f) - res;
    if (x < 0) res = M_PIf - res;
    if (y < 0) res = -res;
    return res;
}

// http://http.developer.nvidia.com/Cg/acos.html
// Handbook of Mathematical Functions
// M. Abramowitz and I.A. Stegun, Ed.
// acos_approx maximum absolute error = 6.760856e-05 rads (3.873685e-03 degree)
float acos_approx(float x)
{
    float xa = fabsf(x);
    float result = sqrtf(1.0f - xa) * (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));
    if (x < 0.0f)
        return M_PIf - result;
    else
        return result;
}


int gcd(int num, int denom)
{
    if (denom == 0)
    {
        return num;
    }

    return gcd(denom, num % denom);
}

float powerf(float base, int exp)
{
    float result = base;
    for (int count = 1; count < exp; count++) result *= base;

    return result;
}

int scaleRange(int x, int srcFrom, int srcTo, int destFrom, int destTo) {
    long int a = ((long int) destTo - (long int) destFrom) * ((long int) x - (long int) srcFrom);
    long int b = (long int) srcTo - (long int) srcFrom;
    return (a / b) + destFrom;
}

float scaleRangef(float x, float srcFrom, float srcTo, float destFrom, float destTo) {
    float a = (destTo - destFrom) * (x - srcFrom);
    float b = srcTo - srcFrom;
    return (a / b) + destFrom;
}

