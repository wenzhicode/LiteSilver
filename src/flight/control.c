/**
******************************************************************************
* @file    control.c
* @author
* @version V0.0.1
* @date    2/06/2020
* @brief   control文件，控制相关函数.
******************************************************************************
*/


#include "control.h"
#include "config.h"
#include "util.h"
#include "math.h"
#include "pid.h"
#include "defines.h"
#include "angle_pid.h"
#include "stick_vector.h"
#include "dshot.h"
#include "time.h"



float   throttle;
int idle_state;
float rxcopy[4];
float error[3];

extern int rx_ready;
int onground_long = 1;
float thrsum;

int flash_feature_1 = 0;
int flash_feature_2 = 0;
int flash_feature_3 = 0;

extern int ledcommand;
extern unsigned char motor_select;
extern float angleerror[];
extern float attitude[];
extern float rx[];
extern char aux[16];
extern float gyro[3];
extern int failsafe;
extern float pidoutput[3];
extern float setpoint[3];
extern float errorvect[];
extern float GEstG[3];
extern int armed_state;
extern int in_air;
extern int arming_release;
extern int binding_while_armed;
extern int onground;
extern unsigned int ratesValueRoll;
extern unsigned int ratesValuePitch;
extern unsigned int ratesValueYaw ;
extern unsigned char flymode;

extern void stick_vector(float rx_input[], float maxangle);
extern float errorvect[]; // level mode angle error calculated by stick_vector.c
extern float GEstG[3];
extern uint8_t crash;
float yawerror[3] = {0};
int16_t motor_value[4];

// *************************************************************************
//horizon modes tuning variables
// *************************************************************************
// 1.0 is pure angle based transition, 0.0 is pure stick defelction based transition, values inbetween are a mix of both.  Adjust from 0 to 1
#define HORIZON_SLIDER  0.3f
//leveling transitions into acro below this angle - above this angle is all acro.  DO NOT SET ABOVE 85 DEGREES!
#define HORIZON_ANGLE_TRANSITION  55.0f
//leveling transitions into acro below this stick position - beyond this stick position is all acro. Adjust from 0 to 1
#define HORIZON_STICK_TRANSITION  0.95f
// *************************************************************************
// *************************************************************************



#define SETPOINT_RATE_LIMIT 1998.0f
#define RC_RATE_INCREMENTAL 14.54f

static inline float constrainf(float amt, float low, float high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

float rcRate[3] = {1.30,1.30,1.30};
float superExpo[3] = {0.30,0.30,0.30};
float Expo[3]= {0.25,0.25,0.25};


#define power3(x) ((x)*(x)*(x))

static float calcBFRatesRad(int axis)
{
    float rcCommandf = rxcopy[axis];
    float rcCommandfAbs = fabsf(rxcopy[axis]);
    
    if(Expo[axis])
    {
        rcCommandf = rcCommandf * power3(rcCommandfAbs) * Expo[axis] + rcCommandf * (1 - Expo[axis]);
    }

    if (rcRate[axis] > 2.0f) {
        rcRate[axis] += RC_RATE_INCREMENTAL * (rcRate[axis] - 2.0f);
    }

    float angleRate = 200.0f * rcRate[axis] * rcCommandf;
    
    if (superExpo[axis]) {
        const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * superExpo[axis]), 0.01f, 1.00f));
        angleRate *= rcSuperfactor;
    }
    return constrainf(angleRate, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT) * (float)DEGTORAD;
}



void control(void)
{
    for (int i = 0 ; i < 3 ; i++)
    {
        rxcopy[i] = rx[i];

        if (fabsf(rxcopy[ i ]) <= STICKS_DEADBAND)
        {
            rxcopy[ i ] = 0.0f;
        }
        else
        {
            if (rxcopy[ i ] >= 0)
            {
                rxcopy[ i ] = mapf(rxcopy[ i ], STICKS_DEADBAND, 1, 0, 1);
            }
            else
            {
                rxcopy[ i ] = mapf(rxcopy[ i ], -STICKS_DEADBAND, -1, 0, -1);
            }
        }
    }

    pid_precalc();

    float rates[3];

//#ifndef BETAFLIGHT_RATES
//    rates[0] = rxcopy[0] * ratesValueRoll * DEGTORAD;
//    rates[1] = rxcopy[1] * ratesValuePitch * DEGTORAD;
//    rates[2] = rxcopy[2] * ratesValueYaw * DEGTORAD;
//#else
    rates[0] =  calcBFRatesRad(0);
    rates[1] =  calcBFRatesRad(1);
    rates[2] =  calcBFRatesRad(2);
//#endif

    float inclinationRoll;
    float inclinationPitch;
    float inclinationMax;
    float stickFade;
    float angleFade;
    float deflection;
    float fade;

    switch(flymode)
    {
    case 0:
        stick_vector(rxcopy, 0);
        // apply yaw from the top of the quad
        yawerror[0] = GEstG[1] * rates[2];
        yawerror[1] = - GEstG[0] * rates[2];
        yawerror[2] = GEstG[2] * rates[2];

        for (int i = 0 ; i <= 1; i++)
        {
            angleerror[i] = errorvect[i] ;
            error[i] = apid(i) + yawerror[i] - gyro[i];
        }
        // yaw
        error[2] = yawerror[2]  - gyro[2];
        break;

    case 1:
        setpoint[0] = rates[0];
        setpoint[1] = rates[1];
        setpoint[2] = rates[2];

        for (int i = 0; i < 3; i++)
        {
            error[i] = setpoint[i] - gyro[i];
        }
        break;

    case 2:
        stick_vector(rxcopy, 0);
        // apply yaw from the top of the quad
        yawerror[0] = GEstG[1] * rates[2];
        yawerror[1] = - GEstG[0] * rates[2];
        yawerror[2] = GEstG[2] * rates[2];

        inclinationRoll   = attitude[0];
        inclinationPitch = attitude[1];
        if (fabsf(inclinationRoll) >= fabsf(inclinationPitch))
        {
            inclinationMax = fabsf(inclinationRoll);
        }
        else
        {
            inclinationMax = fabsf(inclinationPitch);
        }

        // constrains acroFade variable between 0 and 1
        if (inclinationMax <= HORIZON_ANGLE_TRANSITION)
        {
            angleFade = inclinationMax / HORIZON_ANGLE_TRANSITION;
        }
        else
        {
            angleFade = 1;
        }

        deflection = fabsf(rxcopy[0]);
        if (deflection <= HORIZON_STICK_TRANSITION)
        {
            stickFade = deflection / HORIZON_STICK_TRANSITION;
        }
        else
        {
            stickFade = 1;
        }
        fade = (stickFade * (1 - HORIZON_SLIDER)) + (HORIZON_SLIDER * angleFade);
        // apply acro to roll for inverted behavior
        if (GEstG[2] < 0)
        {
            error[0] = rates[0] - gyro[0];
            error[1] = rates[1] - gyro[1];
        }
        else     // apply a transitioning mix of acro and level behavior inside of stick HORIZON_TRANSITION point and full acro beyond stick HORIZON_TRANSITION point
        {
            angleerror[0] = errorvect[0] ;
            // roll angle strength fades out as sticks approach HORIZON_TRANSITION while acro stength fades in according to value of acroFade factor
            error[0] = ((apid(0) + yawerror[0] - gyro[0]) * (1 - fade)) + (fade * (rates[0] - gyro[0]));
            //pitch is acro
            error[1] = rates[1] - gyro[1];
        }

        // yaw
        error[2] = yawerror[2]  - gyro[2];
        break;

    case 3:
        stick_vector(rxcopy, 0);
        // apply yaw from the top of the quad
        yawerror[0] = GEstG[1] * rates[2];
        yawerror[1] = - GEstG[0] * rates[2];
        yawerror[2] = GEstG[2] * rates[2];

        if (GEstG[2] < 0)    // acro on roll and pitch when inverted
        {
            error[0] = rates[0] - gyro[0];
            error[1] = rates[1] - gyro[1];
        }
        else
        {
            //roll is leveled to max angle limit
            angleerror[0] = errorvect[0] ;
            error[0] = apid(0) + yawerror[0] - gyro[0];
            //pitch is acro
            error[1] = rates[1] - gyro[1];
        }
        // yaw
        error[2] = yawerror[2] - gyro[2];
        break;

    case 4:
        stick_vector(rxcopy, 0);
        // apply yaw from the top of the quad
        yawerror[0] = GEstG[1] * rates[2];
        yawerror[1] = - GEstG[0] * rates[2];
        yawerror[2] = GEstG[2] * rates[2];

        for (int i = 0 ; i <= 1; i++)
        {
            float inclinationRoll   = attitude[0];
            float inclinationPitch = attitude[1];
            float inclinationMax;
            if (fabsf(inclinationRoll) >= fabsf(inclinationPitch))
            {
                inclinationMax = fabsf(inclinationRoll);
            }
            else
            {
                inclinationMax = fabsf(inclinationPitch);
            }
            float angleFade;
            // constrains acroFade variable between 0 and 1
            if (inclinationMax <= HORIZON_ANGLE_TRANSITION)
            {
                angleFade = inclinationMax / HORIZON_ANGLE_TRANSITION;
            }
            else
            {
                angleFade = 1;
            }
            float stickFade;
            float deflection = fabsf(rxcopy[i]);
            if (deflection <= HORIZON_STICK_TRANSITION)
            {
                stickFade = deflection / HORIZON_STICK_TRANSITION;
            }
            else
            {
                stickFade = 1;
            }
            float fade = (stickFade * (1 - HORIZON_SLIDER)) + (HORIZON_SLIDER * angleFade);
            // apply acro to roll and pitch sticks for inverted behavior
            if (GEstG[2] < 0)
            {
                error[i] = rates[i] - gyro[i];
            }
            else     // apply a transitioning mix of acro and level behavior inside of stick HORIZON_TRANSITION point and full acro beyond stick HORIZON_TRANSITION point
            {
                angleerror[i] = errorvect[i] ;
                //  angle strength fades out as sticks approach HORIZON_TRANSITION while acro stength fades in according to value of acroFade factor
                error[i] = ((apid(i) + yawerror[i] - gyro[i]) * (1 - fade)) + (fade * (rates[i] - gyro[i]));
            }
        }
        // yaw
        error[2] = yawerror[2]  - gyro[2];
        break;
    }

#ifdef YAW_FIX
    {
        rotateErrors();
        pid(0);
        pid(1);
        pid(2);
    }
#else
    {
        pid(0);
        pid(1);
        pid(2);

    }

#endif


#ifndef THROTTLE_SAFETY
#define THROTTLE_SAFETY .15f
#endif


#ifndef ARMING
    armed_state = 1;                                                                                                                                 // if arming feature is disabled - quad is always armed
#else                                                                                                                                                           // CONDITION: arming feature is enabled
    if (!aux[ARMING])                                                                                                                            //                         CONDITION: switch is DISARMED
    {
        crash =0;

        armed_state = 0;                                                                                                                              //                                                disarm the quad by setting armed state variable to zero
        if (rx_ready == 1)
        {
            binding_while_armed = 0;
            ledcommand = 4;
        }            //                        rx is bound and has been disarmed so clear binding while armed flag
    }
    else                                                                                                                                                    //                      CONDITION: switch is ARMED
    {
        if (((rx[3] > THROTTLE_SAFETY) && (arming_release == 0)) || (binding_while_armed == 1))         //                 CONDITION: (throttle is above safety limit and ARMING RELEASE FLAG IS NOT CLEARED) OR (bind just took place with transmitter armed)
        {
            armed_state = 0;
            ledcommand = 5;            //                                            override to disarmed state and rapid blink the leds
        }
        else                                                                                                                                                 //                              CONDITION: quad is being armed in a safe state
        {
            ledcommand = 4;
            armed_state = 1;                                                                                  //                                          arm the quad by setting armed state variable to 1
            arming_release = 1;                                                                                                                     //                                              clear the arming release flag - the arming release flag being cleared
        }                                                                                                                                                             //                                                                    is what stops the quad from automatically disarming again the next time
    }                                                                                                                                                                   //                                                                  throttle is raised above the safety limit
#endif


#ifndef IDLE_THR
#define IDLE_THR .001f
#endif

    if (armed_state == 0)                                                                                       // CONDITION: armed state variable is 0 so quad is DISARMED
    {
        throttle = 0;                                                                                                                                       //                      override throttle to 0
        in_air = 0;                                                                                                                                         //                      flag in air variable as NOT IN THE AIR for mix throttle increase safety
        arming_release = 0;                                                                                                                         //                      arming release flag is set to not cleared to reactivate the throttle safety limit for the next arming event

    }
    else                                                                                                    // CONDITION: armed state variable is 1 so quad is ARMED
    {

        throttle = (float) IDLE_THR + rx[3] * (1.0f - (float) IDLE_THR);                        //                                  throttle range is mapped from idle throttle value to 100%
        if ((rx[3] > THROTTLE_SAFETY) && (in_air == 0)) in_air = 1;                             //                                  change the state of in air flag when first crossing the throttle
        //                                  safety value to indicate craft has taken off for mix increase safety
    }

#ifdef STICK_TRAVEL_CHECK                                                                               //This feature completely disables throttle and allows visual feedback if control inputs reach full throws
//Stick endpoints check tied to aux channel stick gesture
    if (aux[CH_AUX1])
    {
        throttle = 0;
        if ((rx[0] <= -0.99f) || (rx[0] >= 0.99f) || (rx[1] <= -0.99f) || (rx[1] >= 0.99f) || (rx[2] <= -0.99f) || (rx[2] >= 0.99f) || (rx[3] <= 0.0f) || (rx[3] >= 0.99f))
        {
            ledcommand = 1;
        }
    }
#endif



// turn motors off if throttle is off and pitch / roll sticks are centered
    if (failsafe || (throttle < 0.001f && (!ENABLESTIX || !onground_long || aux[LEVELMODE] || (fabsf(rx[ROLL]) < (float) ENABLESTIX_TRESHOLD && fabsf(rx[PITCH]) < (float) ENABLESTIX_TRESHOLD && fabsf(rx[YAW]) < (float) ENABLESTIX_TRESHOLD))))
    {
        // motors off

        if (onground_long)
        {
            if (gettime() - onground_long > ENABLESTIX_TIMEOUT)
            {
                onground_long = 0;
            }
        }

        for (int i = 0 ; i <= 3 ; i++)
        {
            pwm_set(i, 0);
#ifdef MOTOR_FILTER
            // reset the motor filter
            motorfilter(0, i);
#endif
        }

#ifdef MOTOR_BEEPS
        extern void motorbeep(void);
        motorbeep();
#endif

#ifdef MIX_LOWER_THROTTLE
        // reset the overthrottle filter
        lpf(&overthrottlefilt, 0.0f, 0.72f);    // 50hz 1khz sample rate
        lpf(&underthrottlefilt, 0.0f, 0.72f);   // 50hz 1khz sample rate
#endif

#ifdef STOCK_TX_AUTOCENTER
        for (int i = 0 ; i < 3; i++)
        {
            if (rx[i] == lastrx[i])
            {
                consecutive[i]++;

            }
            else consecutive[i] = 0;
            lastrx[i] = rx[i];
            if (consecutive[i] > 1000 && fabsf(rx[i]) < 0.1f)
            {
                autocenter[i] = rx[i];
            }
        }
#endif


        throttle = 0;                                       //zero out throttle so it does not come back on as idle up value if enabled
        onground = 1;
        thrsum = 0;

    }
    else
    {
        // motors on - normal flight

        onground = 0;
        onground_long = gettime();

        float mix[4];

#ifdef  THROTTLE_TRANSIENT_COMPENSATION

#ifndef THROTTLE_TRANSIENT_COMPENSATION_FACTOR
#define THROTTLE_TRANSIENT_COMPENSATION_FACTOR 7.0
#endif
        extern float throttlehpf(float in);

        throttle += (float)(THROTTLE_TRANSIENT_COMPENSATION_FACTOR) * throttlehpf(throttle);
        if (throttle < 0)
            throttle = 0;
        if (throttle > 1.0f)
            throttle = 1.0f;
#endif



        // throttle angle compensation
#ifdef AUTO_THROTTLE
        if (aux[LEVELMODE])
        {
            //float autothrottle = fastcos(attitude[0] * DEGTORAD) * fastcos(attitude[1] * DEGTORAD);
            extern float GEstG[];
            float autothrottle = GEstG[2];
            float old_throttle = throttle;
            if (autothrottle <= 0.5f)
                autothrottle = 0.5f;
            throttle = throttle / autothrottle;
            // limit to 90%
            if (old_throttle < 0.9f)
                if (throttle > 0.9f)
                    throttle = 0.9f;

            if (throttle > 1.0f)
                throttle = 1.0f;

        }
#endif


#ifdef LVC_LOWER_THROTTLE

#ifdef SWITCHABLE_FEATURE_2
        extern float vbatt_comp;
        extern float vbattfilt;
        extern int flash_feature_2;
        static float throttle_i = 0.0f;
        float throttle_p = 0.0f;
        if (flash_feature_2 == 1)
        {
            // can be made into a function
            if (vbattfilt < (float) LVC_LOWER_THROTTLE_VOLTAGE_RAW)
                throttle_p = ((float) LVC_LOWER_THROTTLE_VOLTAGE_RAW - vbattfilt) * (float) LVC_LOWER_THROTTLE_KP;
            // can be made into a function
            if (vbatt_comp < (float) LVC_LOWER_THROTTLE_VOLTAGE)
                throttle_p = ((float) LVC_LOWER_THROTTLE_VOLTAGE - vbatt_comp) * (float) LVC_LOWER_THROTTLE_KP;

            if (throttle_p > 1.0f) throttle_p = 1.0f;

            if (throttle_p > 0)
            {
                throttle_i += throttle_p * 0.0001f; //ki
            }
            else throttle_i -= 0.001f;// ki on release

            if (throttle_i > 0.5f) throttle_i = 0.5f;
            if (throttle_i < 0.0f) throttle_i = 0.0f;

            throttle -= throttle_p + throttle_i;
        }
        else
        {
            //do nothing - feature is disabled via stick gesture
        }
#else
        extern float vbatt_comp;
        extern float vbattfilt;
        static float throttle_i = 0.0f;
        float throttle_p = 0.0f;
        // can be made into a function
        if (vbattfilt < (float) LVC_LOWER_THROTTLE_VOLTAGE_RAW)
            throttle_p = ((float) LVC_LOWER_THROTTLE_VOLTAGE_RAW - vbattfilt) * (float) LVC_LOWER_THROTTLE_KP;
        // can be made into a function
        if (vbatt_comp < (float) LVC_LOWER_THROTTLE_VOLTAGE)
            throttle_p = ((float) LVC_LOWER_THROTTLE_VOLTAGE - vbatt_comp) * (float) LVC_LOWER_THROTTLE_KP;

        if (throttle_p > 1.0f) throttle_p = 1.0f;

        if (throttle_p > 0)
        {
            throttle_i += throttle_p * 0.0001f; //ki
        }
        else throttle_i -= 0.001f;// ki on release

        if (throttle_i > 0.5f) throttle_i = 0.5f;
        if (throttle_i < 0.0f) throttle_i = 0.0f;

        throttle -= throttle_p + throttle_i;
#endif
#endif



#ifdef INVERTED_ENABLE
        if (pwmdir == REVERSE)
        {
            // inverted flight

            mix[MOTOR_FR] = throttle + pidoutput[ROLL] + pidoutput[PITCH] - pidoutput[YAW];     // FR
            mix[MOTOR_FL] = throttle - pidoutput[ROLL] + pidoutput[PITCH] + pidoutput[YAW];     // FL
            mix[MOTOR_BR] = throttle + pidoutput[ROLL] - pidoutput[PITCH] + pidoutput[YAW];     // BR
            mix[MOTOR_BL] = throttle - pidoutput[ROLL] - pidoutput[PITCH] - pidoutput[YAW];     // BL


        }
        else
#endif
        {
            if(motor_select)
            {
                // normal mixer
                mix[MOTOR_FR] = throttle + pidoutput[ROLL] - pidoutput[PITCH] + pidoutput[YAW];     // FR
                mix[MOTOR_FL] = throttle - pidoutput[ROLL] - pidoutput[PITCH] - pidoutput[YAW];     // FL
                mix[MOTOR_BR] = throttle + pidoutput[ROLL] + pidoutput[PITCH] - pidoutput[YAW];     // BR
                mix[MOTOR_BL] = throttle - pidoutput[ROLL] + pidoutput[PITCH] + pidoutput[YAW];     // BL
            }
            else {
                // invert mixer
                mix[MOTOR_FR] = throttle + pidoutput[ROLL] - pidoutput[PITCH] - pidoutput[YAW];     // FR
                mix[MOTOR_FL] = throttle - pidoutput[ROLL] - pidoutput[PITCH] + pidoutput[YAW];     // FL
                mix[MOTOR_BR] = throttle + pidoutput[ROLL] + pidoutput[PITCH] + pidoutput[YAW];     // BR
                mix[MOTOR_BL] = throttle - pidoutput[ROLL] + pidoutput[PITCH] - pidoutput[YAW];     // BL
            }
        }



        for (int i = 0 ; i <= 3 ; i++)
        {
#ifdef MOTOR_FILTER
            mix[i] = motorfilter(mix[i], i);
#endif

#ifdef MOTOR_FILTER2_ALPHA
            float motorlpf(float in, int x) ;
            mix[i] = motorlpf(mix[i], i);
#endif

#ifdef MOTOR_KAL
            float motor_kalman(float in, int x);
            mix[i] = motor_kalman(mix[i], i);
#endif

#ifdef TORQUE_BOOST
            float motord(float in, int x);
            mix[i] = motord(mix[i], i);
#endif
        }


// MIXER SCALING

#ifdef BRUSHLESS_MIX_SCALING
#undef MIX_LOWER_THROTTLE
#undef MIX_INCREASE_THROTTLE
#undef MIX_LOWER_THROTTLE_3
#undef MIX_INCREASE_THROTTLE_3

        static int mixScaling;
        if (onground) mixScaling = 0;
        // only enable once really in the air
        else mixScaling = in_air;
        if (mixScaling)
        {
            //ledcommand=1;
            float minMix = 1000.0f;
            float maxMix = -1000.0f;
            for (int i = 0; i < 4; i++)
            {
                if (mix[i] < minMix) minMix = mix[i];
                if (mix[i] > maxMix) maxMix = mix[i];
            }
            float mixRange = maxMix - minMix;
            float reduceAmount = 0.0f;
            if (mixRange > 1.0f)
            {
                float scale = 1.0f / mixRange;
                for (int i = 0; i < 4; i++)
                    mix[i] *= scale;
                minMix *= scale;
                reduceAmount = minMix;
            }
            else
            {
                if (maxMix > 1.0f)
                    reduceAmount = maxMix - 1.0f;
                else if (minMix < 0.0f)
                    reduceAmount = minMix;
            }
            if (reduceAmount != 0.0f)
                for (int i = 0; i < 4; i++)
                    mix[i] -= reduceAmount;
        }
#endif





#if ( defined MIX_LOWER_THROTTLE || defined MIX_INCREASE_THROTTLE)

//#define MIX_INCREASE_THROTTLE

// options for mix throttle lowering if enabled
// 0 - 100 range ( 100 = full reduction / 0 = no reduction )
#ifndef MIX_THROTTLE_REDUCTION_PERCENT
#define MIX_THROTTLE_REDUCTION_PERCENT 100
#endif
// lpf (exponential) shape if on, othewise linear
//#define MIX_THROTTLE_FILTER_LPF

// limit reduction and increase to this amount ( 0.0 - 1.0)
// 0.0 = no action
// 0.5 = reduce up to 1/2 throttle
//1.0 = reduce all the way to zero
#ifndef MIX_THROTTLE_REDUCTION_MAX
#define MIX_THROTTLE_REDUCTION_MAX 0.5
#endif

#ifndef MIX_MOTOR_MAX
#define MIX_MOTOR_MAX 1.0f
#endif


        float overthrottle = 0;
        float underthrottle = 0.001f;

        for (int i = 0; i < 4; i++)
        {
            if (mix[i] > overthrottle)
                overthrottle = mix[i];
            if (mix[i] < underthrottle)
                underthrottle = mix[i];
        }

#ifdef MIX_LOWER_THROTTLE

        overthrottle -= MIX_MOTOR_MAX ;

        if (overthrottle > (float)MIX_THROTTLE_REDUCTION_MAX)
            overthrottle = (float)MIX_THROTTLE_REDUCTION_MAX;

#ifdef MIX_THROTTLE_FILTER_LPF
        if (overthrottle > overthrottlefilt)
            lpf(&overthrottlefilt, overthrottle, 0.82); // 20hz 1khz sample rate
        else
            lpf(&overthrottlefilt, overthrottle, 0.72); // 50hz 1khz sample rate
#else
        if (overthrottle > overthrottlefilt)
            overthrottlefilt += 0.005f;
        else
            overthrottlefilt -= 0.01f;
#endif
#else
        overthrottle = 0.0f;
#endif

#ifdef MIX_INCREASE_THROTTLE
// under

        if (underthrottle < -(float)MIX_THROTTLE_REDUCTION_MAX)
            underthrottle = -(float)MIX_THROTTLE_REDUCTION_MAX;

#ifdef MIX_THROTTLE_FILTER_LPF
        if (underthrottle < underthrottlefilt)
            lpf(&underthrottlefilt, underthrottle, 0.82);   // 20hz 1khz sample rate
        else
            lpf(&underthrottlefilt, underthrottle, 0.72);   // 50hz 1khz sample rate
#else
        if (underthrottle < underthrottlefilt)
            underthrottlefilt -= 0.005f;
        else
            underthrottlefilt += 0.01f;
#endif
// under
        if (underthrottlefilt < - (float)MIX_THROTTLE_REDUCTION_MAX)
            underthrottlefilt = - (float)MIX_THROTTLE_REDUCTION_MAX;
        if (underthrottlefilt > 0.1f)
            underthrottlefilt = 0.1;

        underthrottle = underthrottlefilt;

        if (underthrottle > 0.0f)
            underthrottle = 0.0001f;

        underthrottle *= ((float)MIX_THROTTLE_REDUCTION_PERCENT / 100.0f);
#else
        underthrottle = 0.001f;
#endif
// over
        if (overthrottlefilt > (float)MIX_THROTTLE_REDUCTION_MAX)
            overthrottlefilt = (float)MIX_THROTTLE_REDUCTION_MAX;
        if (overthrottlefilt < -0.1f)
            overthrottlefilt = -0.1;


        overthrottle = overthrottlefilt;


        if (overthrottle < 0.0f)
            overthrottle = -0.0001f;


        // reduce by a percentage only, so we get an inbetween performance
        overthrottle *= ((float)MIX_THROTTLE_REDUCTION_PERCENT / 100.0f);



        if (overthrottle > 0 || underthrottle < 0)
        {
            // exceeding max motor thrust
            float temp = overthrottle + underthrottle;
            for (int i = 0; i < 4; i++)
            {
                mix[i] -= temp;
            }
        }
#endif


#ifdef MIX_LOWER_THROTTLE_3
        {
#ifndef MIX_THROTTLE_REDUCTION_MAX
#define MIX_THROTTLE_REDUCTION_MAX 0.5f
#endif

            float overthrottle = 0;

            for (int i = 0; i < 4; i++)
            {
                if (mix[i] > overthrottle)
                    overthrottle = mix[i];
            }


            overthrottle -= 1.0f;
// limit to half throttle max reduction
            if (overthrottle > (float) MIX_THROTTLE_REDUCTION_MAX)  overthrottle = (float) MIX_THROTTLE_REDUCTION_MAX;

            if (overthrottle > 0.0f)
            {
                for (int i = 0 ; i < 4 ; i++)
                    mix[i] -= overthrottle;
            }
#ifdef MIX_THROTTLE_FLASHLED
            if (overthrottle > 0.1f) ledcommand = 1;
#endif
        }
#endif


#ifdef MIX_INCREASE_THROTTLE_3
        {
#ifndef MIX_THROTTLE_INCREASE_MAX
#define MIX_THROTTLE_INCREASE_MAX 0.2f
#endif
            if (in_air == 1)
            {
                float underthrottle = 0;

                for (int i = 0; i < 4; i++)
                {
                    if (mix[i] < underthrottle)
                        underthrottle = mix[i];
                }


                // limit to half throttle max reduction
                if (underthrottle < -(float) MIX_THROTTLE_INCREASE_MAX)  underthrottle = -(float) MIX_THROTTLE_INCREASE_MAX;

                if (underthrottle < 0.0f)
                {
                    for (int i = 0 ; i < 4 ; i++)
                        mix[i] -= underthrottle;
                }
#ifdef MIX_THROTTLE_FLASHLED
                if (underthrottle < -0.01f) ledcommand = 1;
#endif
            }
        }
#endif


#ifdef MOTOR_MIN_COMMAND2    // for testing purposes:  scaling style min motor command.  scales up all mix outputs so lowest motor is at min command only when one drops below min
#ifdef BRUSHLESS_TARGET
        // do nothing - idle set by DSHOT
#else
        float motor_min_value = (float) MOTOR_MIN_COMMAND2 * 0.01f;
        float motor_min_adjust = 0;
        for (int i = 0; i < 4; i++)
        {
            if (mix[i] < 0) mix[i] = 0;                                          //Clip all mixer value to 0 before scaling
            if (mix[i] < (motor_min_value))                     //Determine maximum adjustment necessary
            {
                float motor_min_adjust_temp = motor_min_value - mix[i];
                if (motor_min_adjust_temp > motor_min_adjust) motor_min_adjust = motor_min_adjust_temp;
            }
        }
        for (int i = 0; i < 4; i++)                                             // Scale up all 4 motors by an equal amount if any command has dropped below min
        {
            mix[i] += motor_min_adjust;
        }
#endif
#endif



        thrsum = 0;     //reset throttle sum for voltage monitoring logic in main loop

//Begin for-loop to send motor commands
        for (int i = 0 ; i <= 3 ; i++)
        {

#ifdef CLIP_FF
            mix[i] = clip_ff(mix[i], i);
#endif


//***********************Motor Test Logic
#if defined(MOTORS_TO_THROTTLE) || defined(MOTORS_TO_THROTTLE_MODE)
#if defined(MOTORS_TO_THROTTLE_MODE) && !defined(MOTORS_TO_THROTTLE)
            if (aux[MOTORS_TO_THROTTLE_MODE])
            {
#endif
                mix[i] = throttle;
                if (i == MOTOR_FL && (rx[ROLL] > 0.5f || rx[PITCH] < -0.5f))
                {
                    mix[i] = 0;
                }
                if (i == MOTOR_BL && (rx[ROLL] > 0.5f || rx[PITCH] > 0.5f))
                {
                    mix[i] = 0;
                }
                if (i == MOTOR_FR && (rx[ROLL] < -0.5f || rx[PITCH] < -0.5f))
                {
                    mix[i] = 0;
                }
                if (i == MOTOR_BR && (rx[ROLL] < -0.5f || rx[PITCH] > 0.5f))
                {
                    mix[i] = 0;
                }
#if defined(MOTORS_TO_THROTTLE_MODE) && !defined(MOTORS_TO_THROTTLE)
            }
#endif

            // flash leds in valid throttle range
#ifdef MOTORS_TO_THROTTLE
            ledcommand = 1;
#warning "MOTORS TEST MODE"
#endif
#endif
//***********************End Motor Test Logic


//***********************Min Motor Command Logic
#ifdef MOTOR_MIN_COMMAND3           // clipping style min motor command
#ifdef BRUSHLESS_TARGET
            // do nothing - idle set by DSHOT
#else     //clipping style min motor command override
            if (mix[i] < (float) MOTOR_MIN_COMMAND3 * 0.01f)  mix[i] = (float) MOTOR_MIN_COMMAND3 * 0.01f;
#endif
#endif

#ifdef MOTOR_MIN_COMMAND   // for testing purposes:  mapping style min motor command.  remaps entire range of motor commands from user set min value to 1
#ifdef BRUSHLESS_TARGET
            // do nothing - idle set by DSHOT
#else
            float motor_min_value = (float) MOTOR_MIN_COMMAND * 0.01f;
            if (mix[i] < 0) mix[i] = 0;                                              //Clip all mixer values into 0 to 1 range before remapping
            if (mix[i] > 1) mix[i] = 1;
            mix[i] = motor_min_value + mix[i] * (1.0f - motor_min_value);
#endif
#endif
//***********************End Min Motor Command Logic


//***********************Send Motor PWM Command Logic
#ifndef NOMOTORS
#ifndef MOTORS_TO_THROTTLE
            //normal mode

            if (mix[i] < 0.0f)
            {
                mix[i] = 0.0;
            }
            if (mix[i] > 0.999f)
            {
                mix[i] = 0.999f;
            }
            pwm_set(i, mix[i]);

            motor_value[i] = mix[i]*2000;
#else
            // throttle test mode
            ledcommand = 1;
            pwm_set(i, mix[i]);
#endif
#else
            // no motors mode ( anti-optimization)
#warning "NO MOTORS"
            tempx[i] = motormap(mix[i]);
#endif
//***********************End Motor PWM Command Logic

//***********************Clip mmixer outputs (if not already done) before applying calculating throttle sum
            if (mix[i] < 0) mix[i] = 0;
            if (mix[i] > 1) mix[i] = 1;
            thrsum += mix[i];
        }
// end of for-loop to send motor PWM commands
        thrsum = thrsum / 4;        //calculate throttle sum for voltage monitoring logic in main loop
    }
// end motors on

}


