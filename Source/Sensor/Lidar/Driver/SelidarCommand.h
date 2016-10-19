
#ifndef _SELIDAR_COMMAND_H_
#define _SELIDAR_COMMAND_H_

#include "SelidarProtocol.h"

namespace NS_Selidar{

// Commands
//-----------------------------------------

// Commands without payload and response
#define SELIDAR_CMD_STOP               0x81
#define SELIDAR_CMD_SCAN               0x41
#define SELIDAR_CMD_FORCE_SCAN         0x42
#define SELIDAR_CMD_RESET              0x82


// Commands without payload but have response
#define SELIDAR_CMD_GET_DEVICE_INFO    0x21
#define SELIDAR_CMD_GET_DEVICE_HEALTH  0x22

#define SELIDAR_CMD_GET_SAMPLERATE     0x59 //added in fw 1.17

// Commands with payload and have response
#define SELIDAR_CMD_EXPRESS_SCAN       0x43 //added in fw 1.17

//add for A2 to set SELIDAR motor pwm when using accessory board
#define SELIDAR_CMD_SET_MOTOR_PWM      0xF0
#define SELIDAR_CMD_GET_ACC_BOARD_FLAG 0xFF



// Payloads
// ------------------------------------------
#define SELIDAR_EXPRESS_SCAN_MODE_NORMAL      0 
#define SELIDAR_EXPRESS_SCAN_MODE_FIXANGLE    1
typedef struct _selidar_payload_express_scan_t {
    unsigned char   working_mode;
    unsigned int  reserved;
} __attribute__((packed)) selidar_payload_express_scan_t;

#define MAX_MOTOR_PWM               560
#define DEFAULT_MOTOR_PWM           420
typedef struct _selidar_payload_motor_pwm_t {
    unsigned short pwm_value;
} __attribute__((packed)) selidar_payload_motor_pwm_t;

typedef struct _selidar_payload_acc_board_flag_t {
    unsigned int reserved;
} __attribute__((packed)) selidar_payload_acc_board_flag_t;

// Response
// ------------------------------------------
#define SELIDAR_ANS_TYPE_DEVINFO          0x4
#define SELIDAR_ANS_TYPE_DEVHEALTH        0x6

#define SELIDAR_ANS_TYPE_MEASUREMENT                0x81
// Added in FW ver 1.17
#define SELIDAR_ANS_TYPE_MEASUREMENT_CAPSULED       0x43

// Added in FW ver 1.17
#define SELIDAR_ANS_TYPE_SAMPLE_RATE      0x15

#define SELIDAR_ANS_TYPE_ACC_BOARD_FLAG   0xFF

#define SELIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK      (0x1)
typedef struct _selidar_response_acc_board_flag_t {
    unsigned int support_flag;
} __attribute__((packed)) selidar_response_acc_board_flag_t;


#define SELIDAR_STATUS_OK                 0x0
#define SELIDAR_STATUS_WARNING            0x1
#define SELIDAR_STATUS_ERROR              0x2

#define SELIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define SELIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define SELIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define SELIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1

typedef struct _selidar_response_sample_rate_t {
    unsigned short  std_sample_duration_us;
    unsigned short  express_sample_duration_us;
} __attribute__((packed)) selidar_response_sample_rate_t;

typedef struct _selidar_response_measurement_node_t {
    unsigned char    sync_quality;      // syncbit:1;syncbit_inverse:1;quality:6;
    unsigned short   angle_q6_checkbit; // check_bit:1;angle_q6:15;
    unsigned short   distance_q2;
} __attribute__((packed)) selidar_response_measurement_node_t;

//[distance_sync flags]
#define SELIDAR_RESP_MEASUREMENT_EXP_ANGLE_MASK           (0x3)
#define SELIDAR_RESP_MEASUREMENT_EXP_DISTANCE_MASK        (0xFC)

typedef struct _selidar_response_cabin_nodes_t {
    unsigned short   distance_angle_1; // see [distance_sync flags]
    unsigned short   distance_angle_2; // see [distance_sync flags]
    unsigned char    offset_angles_q3;  
} __attribute__((packed)) selidar_response_cabin_nodes_t;   


#define SELIDAR_RESP_MEASUREMENT_EXP_SYNC_1               0xA
#define SELIDAR_RESP_MEASUREMENT_EXP_SYNC_2               0x5

#define SELIDAR_RESP_MEASUREMENT_EXP_SYNCBIT              (0x1<<15)

typedef struct _selidar_response_capsule_measurement_nodes_t {
    unsigned char                             s_checksum_1; // see [s_checksum_1]
    unsigned char                             s_checksum_2; // see [s_checksum_1]
    unsigned short                            start_angle_sync_q6;
    selidar_response_cabin_nodes_t  cabins[16];
} __attribute__((packed)) selidar_response_capsule_measurement_nodes_t;

typedef struct _selidar_response_device_info_t {
    unsigned char   model;
    unsigned short  firmware_version;
    unsigned char   hardware_version;
    unsigned char   serialnum[16];
} __attribute__((packed)) selidar_response_device_info_t;

typedef struct _selidar_response_device_health_t {
    unsigned char   status;
    unsigned short  error_code;
} __attribute__((packed)) selidar_response_device_health_t;

}

#endif
