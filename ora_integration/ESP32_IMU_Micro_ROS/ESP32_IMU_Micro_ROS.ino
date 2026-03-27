/*
 **********************************************************************
 * INCLUDES
 **********************************************************************
*/

#include "ESP32_IMU_Micro_ROS.h"

/*
 **********************************************************************
 * DEFINES and CONSTANTS
 **********************************************************************
*/

// Checks return value of RCLC function. If the function failed, then either send to error state
// or (in soft checks case) ignore.
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){restartSystem();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// IMU Calibration bias values for calibration (X,Y,Z)
static const Vector_t ACCEL_BIAS = {-1558.80f, 64.50f, -890.80f};
static const Vector_t GYRO_BIAS = {-764.85f, -37.55f, -66.93f};
static const Vector_t MAG_BIAS = {214.03f, 184.00f, 114.59f};
static const Vector_t MAG_SCALE = {0.86f, 0.89f, 01.39f};

// IMU measurement covariance (accuracy)
static const Vector_t ACCEL_COV_DIAG = {0.02f, 0.02f, 0.02f};
static const Vector_t GYRO_COV_DIAG = {0.0001f, 0.000144f, 0.000081f};
static const Vector_t ORIENT_COV_DIAG = {0.01f, 0.01f, 0.01f};

// ROS expects: accel = m/s^2, gyro = rad/s
static const float G_TO_MPS2 = 9.80665f;

// Timer Interrupt timeout
const unsigned int IMU_PUBLISH_TASK_TIME_MS = 100;
const unsigned int LED_TASK_TIME_MS = 500;

/*
 **********************************************************************
 * GLOBAL VARIABLES
 **********************************************************************
*/

// ROS Client Library (RCL) Variables
// C API's for implementing ROS2 objects
rcl_publisher_t imuPublisher;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t imuPublishTimer;
rcl_timer_t ledTimer;

// ROS Client Library for C (RCLC)
// Helper functions for easier use of RCL
rclc_executor_t executor;
rclc_support_t support;

// ROS message object for IMU
sensor_msgs__msg__Imu ImuMsg;

// IMU struct and object initialization
IMU_t imu = {};
MPU9250 mpu;

/*
 **********************************************************************
 * LOCAL TYPES
 **********************************************************************
*/

/*
 **********************************************************************
 * LOCAL VARIABLES (declare as static)
 **********************************************************************
*/

/*
 **********************************************************************
 * LOCAL FUNCTION PROTOTYPES (declare as static)
 **********************************************************************
*/
  
/*
 **********************************************************************
 * GLOBAL FUNCTIONS
 **********************************************************************
*/

void setup() 
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    initIMU();
    initMicroRos();
}


void loop() 
{
    /*
        Keep IMU update running as often as possible so the fusion filter
        can continue computing quaternion values properly.
    */
    updateImuObject();

    // Run timer callback(s)
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));

    delay(1);
}


/**************************************************
 * Function Name: initIMU
 * Description: 
**************************************************/

void initIMU()
{
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);

    if(!mpu.setup(IMU_I2C_ADDRESS)) 
    {
        while (true) 
        {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    mpu.selectFilter(QuatFilterSel::MADGWICK);

    // Apply saved calibration values
    mpu.setAccBias(ACCEL_BIAS.x,  ACCEL_BIAS.y,  ACCEL_BIAS.z);
    mpu.setGyroBias(GYRO_BIAS.x, GYRO_BIAS.y, GYRO_BIAS.z);
    mpu.setMagBias(MAG_BIAS.x, MAG_BIAS.y, MAG_BIAS.z);
    mpu.setMagScale(MAG_SCALE.x, MAG_SCALE.y, MAG_SCALE.z);

    Serial.println("MPU9250 ready!");
}


/**************************************************
 * Function Name: initMicroRos
 * Description: 
**************************************************/

void initMicroRos()
{
    // Configures how ESP talks to micro ROS agent (serial, wifi, etc)
    set_microros_transports();
    
    delay(2000);

    allocator = rcl_get_default_allocator();

    // Create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create node
    RCCHECK(rclc_node_init_default(&node, "esp32", "", &support));

    // Create publisher
    RCCHECK(rclc_publisher_init_default(
        &imuPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu")
    );

    // Create timers
    // IMU publisher timermicro ros
    RCCHECK(rclc_timer_init_default(
        &imuPublishTimer,
        &support,
        RCL_MS_TO_NS(IMU_PUBLISH_TASK_TIME_MS),
        imuTimerCallback)
    );

    // Blinking LED timer
    RCCHECK(rclc_timer_init_default(
        &ledTimer,
        &support,
        RCL_MS_TO_NS(LED_TASK_TIME_MS),
        ledTimerCallback)
    );

    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &imuPublishTimer));
    RCCHECK(rclc_executor_add_timer(&executor, &ledTimer));

    // Initializing IMU frame
    static char IMU_FRAME[] = "imu_frame";
    ImuMsg.header.frame_id.data = IMU_FRAME;
    ImuMsg.header.frame_id.size = strlen(IMU_FRAME);
    ImuMsg.header.frame_id.capacity = ImuMsg.header.frame_id.size + 1;

    // Set covariances in IMU frame
    setDiagonalCovariance();

    // Sychronize timing for the microROS agent
    rmw_uros_sync_session(SYNC_SESSION_TIMEOUT_MS);
}


/**************************************************
 * Function Name: imuTimerCallback
 * Description: 
**************************************************/

void imuTimerCallback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);

  if ((timer != NULL))
  {
    fillImuMsgFromImuStruct();
    RCSOFTCHECK(rcl_publish(&imuPublisher, &ImuMsg, NULL));
  }
}


/**************************************************
 * Function Name: ledTimerCallback
 * Description: 
**************************************************/

void ledTimerCallback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) 
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}


/**************************************************
 * Function Name: setDiagonalCovariance
 * Description: 
**************************************************/

void setDiagonalCovariance()
{
    // Initialize all indices to 0
    for(int i = 0; i < COVARIANCE_1D_ARRAY_SIZE; i++)
    {
        ImuMsg.linear_acceleration_covariance[i] = 0.0;
        ImuMsg.angular_velocity_covariance[i] = 0.0;
        ImuMsg.orientation_covariance[i] = 0.0;
    }

    // Setting the variance indices to the proper weight
    ImuMsg.linear_acceleration_covariance[X_VARIANCE_INDEX] = ACCEL_COV_DIAG.x;
    ImuMsg.linear_acceleration_covariance[Y_VARIANCE_INDEX] = ACCEL_COV_DIAG.y;
    ImuMsg.linear_acceleration_covariance[Z_VARIANCE_INDEX] = ACCEL_COV_DIAG.z;

    ImuMsg.angular_velocity_covariance[X_VARIANCE_INDEX] = GYRO_COV_DIAG.x;
    ImuMsg.angular_velocity_covariance[Y_VARIANCE_INDEX] = GYRO_COV_DIAG.y;
    ImuMsg.angular_velocity_covariance[Z_VARIANCE_INDEX] = GYRO_COV_DIAG.z;
        
    ImuMsg.orientation_covariance[X_VARIANCE_INDEX] = ORIENT_COV_DIAG.x;
    ImuMsg.orientation_covariance[Y_VARIANCE_INDEX] = ORIENT_COV_DIAG.y;
    ImuMsg.orientation_covariance[Z_VARIANCE_INDEX] = ORIENT_COV_DIAG.z;
}


/**************************************************
 * Function Name: updateImuObject
 * Description:
**************************************************/

void updateImuObject()
{
    if (mpu.update())
    {
        // Copying data to IMU structure
        imu.accel.x = mpu.getAccX() * G_TO_MPS2;
        imu.accel.y = mpu.getAccY() * G_TO_MPS2;
        imu.accel.z = mpu.getAccZ() * G_TO_MPS2;

        imu.gyro.x = mpu.getGyroX() * DEG_TO_RAD;
        imu.gyro.y = mpu.getGyroY() * DEG_TO_RAD;
        imu.gyro.z = mpu.getGyroZ() * DEG_TO_RAD;

        imu.quat.w = mpu.getQuaternionW();
        imu.quat.x = mpu.getQuaternionX();
        imu.quat.y = mpu.getQuaternionY();
        imu.quat.z = mpu.getQuaternionZ();

        imu.temp = mpu.getTemperature();

        imu.sampleTimeMS = millis();
    }
}


/**************************************************
 * Function Name: fillImuMsgFromImuStruct
 * Description: 
**************************************************/

void fillImuMsgFromImuStruct()
{
    // Message Header
    if(rmw_uros_epoch_synchronized())
    {
        ImuMsg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
        ImuMsg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    }

    // Linear acceleration (m/s^2)
    ImuMsg.linear_acceleration.x = imu.accel.x;
    ImuMsg.linear_acceleration.y = imu.accel.y;
    ImuMsg.linear_acceleration.z = imu.accel.z;

    // Angular velocity (rad/s)
    ImuMsg.angular_velocity.x = imu.gyro.x;
    ImuMsg.angular_velocity.y = imu.gyro.y;
    ImuMsg.angular_velocity.z = imu.gyro.z;

    // Orientation (only if valid)
    ImuMsg.orientation.w = imu.quat.w;
    ImuMsg.orientation.x = imu.quat.x;
    ImuMsg.orientation.y = imu.quat.y;
    ImuMsg.orientation.z = imu.quat.z;
}


/**************************************************
 * Function Name: printImuData
 * Description: 
**************************************************/

void printImuData(bool accel, bool gyro, bool quat, bool temp)
{
    // Printing acceleration data
    if(accel)
    {
        Serial.print("[");
        Serial.print(imu.sampleTimeMS);
        Serial.print("] X: ");
        Serial.print(imu.accel.x);
        Serial.print(", Y: ");
        Serial.print(imu.accel.y);
        Serial.print(", Z: ");
        Serial.print(imu.accel.z);
        Serial.println(" m/s^2");
        Serial.println();
    }

    // Printing gyroscopic data
    if(gyro)
    {
        Serial.print("[");
        Serial.print(imu.sampleTimeMS);
        Serial.print("] X: ");
        Serial.print(imu.gyro.x);
        Serial.print(", Y: ");
        Serial.print(imu.gyro.y);
        Serial.print(", Z: ");
        Serial.print(imu.gyro.z);
        Serial.println(" rad/s");
        Serial.println();
    }

    // Printing quaternion data
    if(quat)
    {
        Serial.print("[");
        Serial.print(imu.sampleTimeMS);
        Serial.print("] quat (w,x,y,z): ");
        Serial.print(imu.quat.w, 6); Serial.print(", ");
        Serial.print(imu.quat.x, 6); Serial.print(", ");
        Serial.print(imu.quat.y, 6); Serial.print(", ");
        Serial.println(imu.quat.z, 6);
        Serial.println();
    }

    // Printing temperature data
    if(temp)
    {
        Serial.print("[");
        Serial.print(imu.sampleTimeMS);
        Serial.print("] Temperature: ");
        Serial.print(imu.temp);
        Serial.println(" c");
        Serial.println();
    }
    
    // Ending line if any value are requested
    if(accel || gyro || quat || temp)
    {
        Serial.println("--------------------------");
        Serial.println();
    }
}


/**************************************************
 * Function Name: restartSystem
 * Description: 
**************************************************/

void restartSystem()
{
    digitalWrite(LED_PIN, LOW);
    delay(500);

    ESP.restart();
}