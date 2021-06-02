/**
******************************************************************************
* @file    config.h
* @author
* @version V0.0.1
* @date    25/05/2020
* @brief   配置文件，使能/失能相关特性.
******************************************************************************
*/


/*****************************Hardware selection******************************/
// For brushless board
#define BRUSHLESS_TARGET


#ifdef BRUSHLESS_TARGET
#define BRUSHLESS_MIX_SCALING
#undef MIX_LOWER_THROTTLE
#undef MIX_INCREASE_THROTTLE
#undef MIX_LOWER_THROTTLE_3
#undef MIX_INCREASE_THROTTLE_3
#else
#define MIX_LOWER_THROTTLE
#define MIX_THROTTLE_REDUCTION_PERCENT 10
//#define MIX_INCREASE_THROTTLE

//#define MIX_LOWER_THROTTLE_3
#define MIX_INCREASE_THROTTLE_3
#define MIX_THROTTLE_INCREASE_MAX 0.2f
#endif


/****************************Rates & Expo settings****************************/

// *************max angle for level mode
#define LEVEL_MAX_ANGLE 65.0f

// ************* low rates multiplier if rates are assigned to a channel
#define LOW_RATES_MULTI 0.5f

// *************transmitter stick adjustable deadband for roll/pitch/yaw
// *************.01f = 1% of stick range - comment out to disable
#define STICKS_DEADBAND .01f

#define MAX_RATE 660.0          //Roll & Pitch axis
#define MAX_RATEYAW 500.0       //Yaw axis (used in acro and leveling modes)



/*****************************Receiver settings*******************************/

// Radio protocol selection
//#define RX_SBUS
//#define RX_CRSF                                           //Requires tbs firmware v2.88 or newer for failsafe to operate properly
#define RX_DSMX_2048
//#define RX_DSM2_1024
//#define RX_IBUS
//#define RX_NRF24_BAYANG_TELEMETRY
//#define RX_BAYANG_BLE_APP
//#define RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND





// Transmitter Type Selection
#define USE_MULTI

#define USE_SERIAL_DSMX
#define USE_SERIAL_SBUS


/*****************************Voltage settings********************************/

// ************* Raises pids automatically as battery voltage drops in flight.  Ensure voltage is calibrated before use ****CRITICAL****.
//#define PID_VOLTAGE_COMPENSATION
#define LEVELMODE_PID_ATTENUATION 0.90f

#define LVC_LOWER_THROTTLE
#define LVC_LOWER_THROTTLE_VOLTAGE 3.30
#define LVC_LOWER_THROTTLE_VOLTAGE_RAW 2.70
#define LVC_LOWER_THROTTLE_KP 3.0

#define STOP_LOWBATTERY

#define VBATTLOW 3.5

#define ACTUAL_BATTERY_VOLTAGE 4.20
#define REPORTED_TELEMETRY_VOLTAGE 4.20



/*****************************Filter settings*********************************/

//#define WEAK_FILTERING
//#define STRONG_FILTERING
//#define VERY_STRONG_FILTERING
//#define ALIENWHOOP_ZERO_FILTERING
#define BETA_FILTERING


//Select Gyro Filter Type *** Select Only One type
#define KALMAN_GYRO
//#define PT1_GYRO


//Select Gyro Filter Cut Frequency
#define GYRO_FILTER_PASS1 HZ_90
#define GYRO_FILTER_PASS2 HZ_90

//Select D Term Filter Cut Frequency *** Select Only one
#define  DTERM_LPF_2ND_HZ 100
//#define DTERM_LPF_1ST_HZ 70



/************************* *Motor output setting******************************/
/*
About dshot: https://oscarliang.com/dshot/

DShot600 – 600,000 bits/Sec
DShot300 – 300,000 bits/Sec
DShot150 – 150,000 bits/Sec



*/

#define DSHOT600
//#define DSHOT150
//#define DSHOT300


#define MOTOR_MIN_COMMAND  2.0

//**************joelucid's yaw fix
#define YAW_FIX

//**************joelucid's transient windup protection.  Removes roll and pitch bounce back after flips
#define TRANSIENT_WINDUP_PROTECTION

#define INVERT_YAW_PID




/********************************other****************************************/
//enables use of stick accelerator and stick transition for d term lpf 1 & 2
#define ADVANCED_PID_CONTROLLER

#define THROTTLE_SAFETY .10f

#define IDLE_THR .001f

#ifdef LVC_LOWER_THROTTLE
#define SWITCHABLE_FEATURE_2
#endif

#ifdef INVERT_YAW_PID
#define SWITCHABLE_FEATURE_3
#endif


#define USE_RX_FRSKY

//#define USE_RX_SBUS_DSM_BY


