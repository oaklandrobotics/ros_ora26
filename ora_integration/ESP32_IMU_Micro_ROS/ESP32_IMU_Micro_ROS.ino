/*
 **********************************************************************
 * INCLUDES
 **********************************************************************
*/

#include "ESP32_IMU_Micro_ROS.hpp"

/*
 **********************************************************************
 * DEFINES and CONSTANTS
 **********************************************************************
*/

// Checks return value of RCLC function. If the function failed, then either send to error state
// or (in soft checks case) ignore.
#define RCCHECK(fn)             \
{                               \
    rcl_ret_t temp_rc = fn;     \
                                \
    if(temp_rc != RCL_RET_OK)   \
    {                           \
        restartSystem();        \
    }                           \
}                               

#define RCSOFTCHECK(fn)         \
{                               \
    rcl_ret_t temp_rc = fn;     \
                                \
    if(temp_rc != RCL_RET_OK)   \
    {                           \
        /* Ignore for now */    \
    }                           \
}                               

// Timer Interrupt timeout
const unsigned int IMU_PUBLISH_TASK_TIME_MS = 100;
const unsigned int HEARTBEAT_LED_TASK_TIME_MS = 500;
const unsigned int AUTONOMOUS_LED_TASK_TIME_MS = 500;

// IMU measurement covariance (accuracy)
static const Vector_t ACCEL_COV_DIAG = {0.02f, 0.02f, 0.02f};
static const Vector_t GYRO_COV_DIAG = {0.0001f, 0.000144f, 0.000081f};
static const Vector_t ORIENT_COV_DIAG = {0.01f, 0.01f, 0.01f};

// IMU Report Values
static const uint8_t BNO_REPORT_SETS_PER_PUBLISH = 2;
static const uint16_t BNO_REPORT_INTERVAL_MS = (IMU_PUBLISH_TASK_TIME_MS + BNO_REPORT_SETS_PER_PUBLISH - 1) / BNO_REPORT_SETS_PER_PUBLISH; // Integer based rounding but it's basically IMU_PUBLISH_TASK_TIME_MS / BNO_REPORT_SETS_PER_PUBLISH
static const uint8_t BNO_MAX_REPORTS_PER_LOOP = 6;

// Micro-ROS Values
static const unsigned long AGENT_CHECK_PERIOD_MS = 1000;

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
rcl_timer_t heartbeatLedTimer;
rcl_timer_t autonomousLedTimer;
rcl_service_t autonomousLedStateService;
std_srvs__srv__SetBool_Request autonomousLedStateRequest;
std_srvs__srv__SetBool_Response autonomousLedStateResponse;
unsigned long lastAgentPingMS = 0;

// ROS Client Library for C (RCLC)
// Helper functions for easier use of RCL
rclc_executor_t executor;
rclc_support_t support;

// ROS message object for IMU
sensor_msgs__msg__Imu ImuMsg;

// IMU struct and object initialization
IMU_t imu = {};
BNO08x bno;

// Application Variables
bool autonomousLedState = false;

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

    delay(500);

    pinMode(HEARTBEAT_LED_PIN, OUTPUT);
    pinMode(AUTONOMOUS_LED_PIN, OUTPUT);

    digitalWrite(HEARTBEAT_LED_PIN, HIGH);
    digitalWrite(AUTONOMOUS_LED_PIN, LOW);

    // Setting up IMU
    bool imuConfigSuccessful = configureIMU();

    // Will freeze program if multiple connections to IMU don't work
    if(imuConfigSuccessful == false)
    {
        restartSystem();
    }
    else
    {
        Serial.println("IMU ready!");
    }
    delay(100);

    
    initMicroRos();
}


void loop() 
{
    // Check whether agent is still connected
    checkMicroRosAgent();

    // Run timer callback(s)
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
    
    /*
        Keep IMU update running as often as possible so the fusion filter
        can continue computing quaternion values properly.
    */
    updateImuObject();
}


/**************************************************
 * Function Name: configureIMU
 * Description: 
**************************************************/

bool configureIMU()
{
    // Configuring communication over I2C
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);   // 400 kHz I2C fast mode
    Wire.setTimeOut(50);     // prevent long I2C blocking/hanging

    delay(250);
    // TODO: Add in a multi attempt to connect to the bno
    
    uint8_t imuConnectAttempts = 0;
    bool imuConnected = false;

    while(imuConnectAttempts < IMU_CONNECT_RETRY_NUM)
    {
        // Setting up IMU with proper interrupt and reset pins
        if(bno.begin(IMU_I2C_ADDRESS, Wire, IMU_INT_PIN, IMU_RST_PIN) == false) 
        {
            Serial.println("IMU connection failed. Please check connection with examples.");
        }
        else
        {
            Serial.println("IMU connection successful");
            imuConnected = true;
            break;
        }

        imuConnectAttempts++;
        delay(500);
    }

    // Set the IMU output data configs only if the IMU is connected
    if(imuConnected == false)
    {
        return false;
    }
    else
    {
        // Configuring what data to report
        imuOutputDataConfig();
        return true;
    }
}


/**************************************************
 * Function Name: imuOutputDataConfig
 * Description: 
**************************************************/

void imuOutputDataConfig()
{
    // Accelerometer data
    bno.enableAccelerometer(BNO_REPORT_INTERVAL_MS);

    // Gyroscopic data
    bno.enableGyro(BNO_REPORT_INTERVAL_MS);

    // Rotational vector data (used for quaternions)
    bno.enableRotationVector(BNO_REPORT_INTERVAL_MS);
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
        "/esp/imu")
    );

    // Create timers
    // IMU publisher timermicro ros
    RCCHECK(rclc_timer_init_default(
        &imuPublishTimer,
        &support,
        RCL_MS_TO_NS(IMU_PUBLISH_TASK_TIME_MS),
        imuTimerCallback)
    );

    // Heartbeat LED timer
    RCCHECK(rclc_timer_init_default(
        &heartbeatLedTimer,
        &support,
        RCL_MS_TO_NS(HEARTBEAT_LED_TASK_TIME_MS),
        heartbeatLedTimerCallback)
    );

    // Autonomous LED timer
    RCCHECK(rclc_timer_init_default(
        &autonomousLedTimer,
        &support,
        RCL_MS_TO_NS(AUTONOMOUS_LED_TASK_TIME_MS),
        autonomousLedTimerCallback)
    );

    // Creating Services
    // Autonomous LED State service
    RCCHECK(rclc_service_init_default(
        &autonomousLedStateService,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
        "/navigation/set_auton")
    );

    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &imuPublishTimer));
    RCCHECK(rclc_executor_add_timer(&executor, &heartbeatLedTimer));
    RCCHECK(rclc_executor_add_timer(&executor, &autonomousLedTimer));

    // Adding Services with callback and request / response variables
    RCCHECK(rclc_executor_add_service(
        &executor,
        &autonomousLedStateService,
        &autonomousLedStateRequest,
        &autonomousLedStateResponse,
        autonomousLedStateServiceCallback)
    );

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
 * Function Name: checkMicroRosAgent
 * Description: 
**************************************************/

void checkMicroRosAgent()
{
    unsigned long currentTimeMS = millis();

    if((currentTimeMS - lastAgentPingMS) >= AGENT_CHECK_PERIOD_MS)
    {
        lastAgentPingMS = currentTimeMS;

        if(rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS_PER_TRY) != RMW_RET_OK)
        {
            restartSystem();
        }
    }
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
 * Function Name: heartbeatLedTimerCallback
 * Description: 
**************************************************/

void heartbeatLedTimerCallback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) 
  {
    digitalWrite(HEARTBEAT_LED_PIN, !digitalRead(HEARTBEAT_LED_PIN));
  }
}


/**************************************************
 * Function Name: autonomousLedTimerCallback
 * Description: 
**************************************************/

void autonomousLedTimerCallback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer != NULL) 
    {
        if(autonomousLedState)
        {
            digitalWrite(AUTONOMOUS_LED_PIN, !digitalRead(AUTONOMOUS_LED_PIN));
        }
        else
        {
            digitalWrite(AUTONOMOUS_LED_PIN, LOW);
        }
    }
}



/*************************************************************************
 * Function Name: autonomousLedStateServiceCallback
 * Description: Service for autonomous LED state. This is called whenever
 *              the client sends a request using this service.
*************************************************************************/

void autonomousLedStateServiceCallback(const void * request_msg, void * response_msg)
{
    // Cast generic pointers to the actual ROS service message types
    const std_srvs__srv__SetBool_Request * req = (const std_srvs__srv__SetBool_Request *)request_msg;

    std_srvs__srv__SetBool_Response * res = (std_srvs__srv__SetBool_Response *)response_msg;

    // Update your firmware/application state
    autonomousLedState = req->data;

    // Fill response
    res->success = true;

    // Populating the response message going back to the client based on the state
    // In addition, changing the AUTONOMOUS_LED_PIN to match up with the incoming service
    if(autonomousLedState)
    {
        const char * msg = "Autonomous LED enabled";
        res->message.data = (char *)msg;
        res->message.size = strlen(msg);
        res->message.capacity = res->message.size + 1;
        digitalWrite(AUTONOMOUS_LED_PIN, HIGH);
    }
    else
    {
        const char * msg = "Autonomous LED disabled";
        res->message.data = (char *)msg;
        res->message.size = strlen(msg);
        res->message.capacity = res->message.size + 1;
        digitalWrite(AUTONOMOUS_LED_PIN, LOW);
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
    // Reconfigure data output from BNO if chip was reset.
    if(bno.wasReset())
    {
        imuOutputDataConfig();
    }
    
    // Checking whether an event occured
    if(bno.getSensorEvent() == true)
    {
        uint8_t sensorEvent = bno.getSensorEventID();

        switch(sensorEvent)
        {
            case SENSOR_REPORTID_ACCELEROMETER:
                imu.accel.x = bno.getAccelX();
                imu.accel.y = bno.getAccelY();
                imu.accel.z = bno.getAccelZ();
            break;

            case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
                imu.gyro.x = bno.getGyroX();
                imu.gyro.y = bno.getGyroY();
                imu.gyro.z = bno.getGyroZ();
            break;

            case SENSOR_REPORTID_ROTATION_VECTOR:
                imu.quat.w = bno.getQuatReal();
                imu.quat.x = bno.getQuatI();
                imu.quat.y = bno.getQuatJ();
                imu.quat.z = bno.getQuatK();
            break;

            default:
            break;
        }
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
        int64_t time_ns = rmw_uros_epoch_nanos();

        ImuMsg.header.stamp.sec = time_ns / 1000000000;
        ImuMsg.header.stamp.nanosec = time_ns % 1000000000;
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

void printImuData(bool accel, bool gyro, bool quat)
    {
    // Printing acceleration data
    if(accel)
    {
        Serial.print("[");
        Serial.print(imu.sampleTimeMS);
        Serial.print("] Acceleration (x,y,z): ");
        Serial.print("X: ");
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
        Serial.print("] Gyroscope (x,y,z): ");
        Serial.print("X: ");
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
        Serial.print("] Quaternion (w,x,y,z): ");
        Serial.print(imu.quat.w, 6); Serial.print(", ");
        Serial.print(imu.quat.x, 6); Serial.print(", ");
        Serial.print(imu.quat.y, 6); Serial.print(", ");
        Serial.println(imu.quat.z, 6);
        Serial.println();
    }
    
    // Ending line if any value are requested
    if(accel || gyro || quat)
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
    digitalWrite(HEARTBEAT_LED_PIN, LOW);
    delay(500);

    ESP.restart();
}