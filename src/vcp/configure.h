/**
******************************************************************************
* @file    configure.h
* @author
* @version V0.0.1
* @date    9/05/2020
* @brief   头文件，configure相关函数声明.
******************************************************************************
*/


#include "hardware.h"
#include "mavlink.h"
#include "usbprop.h"


void serialProcess(void);



typedef struct Mavlink_Messages_t {

    int sysid;
    int compid;

    mavlink_reboot_t reboot;
    mavlink_tx_t tx_value;
    mavlink_motor_value_t motor_value;
    mavlink_motor_io_t motor_io;
    mavlink_attitude_t attitude;
    mavlink_pid_t pid;
    mavlink_rx_t rx_select;
    mavlink_rates_t rates;
    mavlink_config_tab_t tab_index;
    mavlink_status_build_t status_build;
    mavlink_calibrate_t cali;

} Mavlink_Messages;


