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

#include "MPU9250.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>

#include <stdio.h>
#include <Wire.h>

/*
 ******************************************************************************
 * DEFINES, CONSTANTS, ENUMS, STRUCTS
 ******************************************************************************
 */

#define LED_PIN 2
#define IMU_SDA_PIN 21
#define IMU_SCL_PIN 22


// This is the I2C address of the IMU chip itself.
// May change with chip variant so run the example program connection_check.ino
#define IMU_I2C_ADDRESS 0x68

// Represents the 3x3 matrix for the covariances but as a 1D array
#define COVARIANCE_1D_ARRAY_SIZE 9
#define X_VARIANCE_INDEX 0
#define Y_VARIANCE_INDEX 4
#define Z_VARIANCE_INDEX 8

// MicroROS parameters
#define SYNC_SESSION_TIMEOUT_MS 1000

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

void initIMU();
void initMicroRos();
void imuTimerCallback(rcl_timer_t * timer, int64_t last_call_time);
void ledTimerCallback(rcl_timer_t * timer, int64_t last_call_time);
void setDiagonalCovariance();
void updateImuObject();
void fillImuMsgFromImuStruct();
void printImuData(bool accel, bool gyro, bool quat, bool temp);
void restartSystem();
  
#endif // ESP32_IMU_MICRO_ROS_H