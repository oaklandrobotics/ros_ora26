/*
 ******************************************************************************
      File Name  : ESP32_IMU_MICRO_ROS_H.h
      Author     : George Trupiano
      Date       : 
      Description:
 ******************************************************************************
 */
  
#ifndef ESP32_IMU_MICRO_ROS_H
#define ESP32_IMU_MICRO_ROS_H
  
/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */

// IMU Specific Headers
#include "SparkFun_BNO08x_Arduino_Library.h"

// MicroROS Specific Headers
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <std_srvs/srv/set_bool.h>

// General Headers
#include <stdio.h>
#include <Wire.h>

/*
 ******************************************************************************
 * DEFINES, CONSTANTS, ENUMS, STRUCTS
 ******************************************************************************
 */

// List of pins
#define HEARTBEAT_LED_PIN 2
#define AUTONOMOUS_LED_PIN 4
#define IMU_RST_PIN 13
#define IMU_INT_PIN 14
#define IMU_SDA_PIN 21
#define IMU_SCL_PIN 22

// This is the I2C address of the IMU chip itself.
#define IMU_I2C_ADDRESS 0x4B // Alternative address is 0x4A

// Number of times IMU would try to connect
#define IMU_CONNECT_RETRY_NUM 8

// Represents the 3x3 matrix for the covariances but as a 1D array
#define COVARIANCE_1D_ARRAY_SIZE 9
#define X_VARIANCE_INDEX 0
#define Y_VARIANCE_INDEX 4
#define Z_VARIANCE_INDEX 8

// MicroROS parameters
#define SYNC_SESSION_TIMEOUT_MS 1000
#define AGENT_PING_TIMEOUT_MS 1000
#define AGENT_PING_ATTEMPTS_PER_TRY 1
#define AGENT_CONNECT_RETRY_NUM 20
#define AGENT_CONNECT_RETRY_DELAY_MS 500

typedef struct
{
    float x;
    float y;
    float z;
} Vector_t;


typedef struct
{
    float x;
    float y;
    float z;
    float w;
} Quaternion_t;

typedef struct
{
    Vector_t accel;
    Vector_t gyro;
    Quaternion_t quat;
    float temp;
    unsigned long sampleTimeMS;
} IMU_t;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL FUNCTION PROTOTYPES
 ******************************************************************************
 */

bool configureIMU();
void imuOutputDataConfig();
void initMicroRos();
void imuTimerCallback(rcl_timer_t * timer, int64_t last_call_time);
void heartbeatLedTimerCallback(rcl_timer_t * timer, int64_t last_call_time);
void autonomousLedTimerCallback(rcl_timer_t * timer, int64_t last_call_time);
void autonomousLedStateServiceCallback(const void * request_msg, void * response_msg);
void setDiagonalCovariance();
void updateImuObject();
void fillImuMsgFromImuStruct();
void printImuData(bool accel, bool gyro, bool quat);
void restartSystem();
  
#endif // ESP32_IMU_MICRO_ROS_H