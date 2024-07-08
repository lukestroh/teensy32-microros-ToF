/*
Teensy32 microROS ToF
Author(s): Luke Strohbehn

Connection reset code from: https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <teensy32_tof_msgs/msg/to_f_data_array.h>

/* microROS setup */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)){}}

rcl_publisher_t publisher;
teensy32_tof_msgs__msg__ToFDataArray msg_arr;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


/* ToF setup */
// Define which Wire objects to use, may depend on platform
// or on your configurations.
#define SENSOR1_WIRE Wire
#define SENSOR2_WIRE Wire
#if defined(WIRE_IMPLEMENT_WIRE1)
#define SENSOR3_WIRE Wire1
#define SENSOR4_WIRE Wire1
#else
#define SENSOR3_WIRE Wire
#define SENSOR4_WIRE Wire
#endif

// LED stuff
const int led_pin = 13;
unsigned long last_time = 0;

// VL53L0X setup
Adafruit_VL53L0X tof0 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof1 = Adafruit_VL53L0X();
Adafruit_VL53L0X* tofs[2] = {&tof0, &tof1}; // add more sensors here

typedef struct {
    Adafruit_VL53L0X *psensor; // pointer to sensor object
    TwoWire *pwire;
    int id;     // sensor id
    int shutdown_pin;
    int interrupt_pin;
    Adafruit_VL53L0X::VL53L0X_Sense_config_t sensor_config;
    int16_t range;
    uint8_t status;
} sensor_t;

Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2;
sensor_t sensors[] = {
    {&sensor1, &Wire, 0x2A, 11, 26, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
    {&sensor2, &Wire1, 0x2B, 12, 27, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0}
};
const uint8_t num_sensors = sizeof(sensors) / sizeof(sensors)[0];

// microROS system state
enum SystemState {
    AGENT_WAIT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} system_state;


void error_loop(void) {
    /* Error loop LED indicator */
    Serial.println(F("Error loop... "));
    uint32_t time = millis();
    while (millis() - time < 5000) {
        digitalWrite(led_pin, !digitalRead(led_pin));
        delay(50);
    }
    // reset_cpu();
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    /* Publisher timer callback method */
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg_arr, NULL));
    }
}

bool create_rcl_entities() {
    /* Create the microROS entities */
    // Init allocator
    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    // create node
    const char* _namespace = "microROS";
    const char* _node_name = "teensy32";
    RCCHECK(rclc_node_init_default(&node, _node_name, _namespace, &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(teensy32_tof_msgs, msg, ToFDataArray),
        "tof_data_array"
    ));
    
    // create timer
    const unsigned int timer_period {10};
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_period),
        timer_callback
    ));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    return true;
}

void destroy_rcl_entities() {
    /* Destroy the microROS-related entities */
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&publisher, &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

void execute_every_n_ms(int64_t ms, SystemState system_state) {
    /* Method for periodically pinging the micro_ros_agent */
    do {
        static volatile int64_t init = -1;
        if (init == -1) {
            init = uxr_millis();
        }
        if (uxr_millis() - init > ms) {
            system_state;
            init = uxr_millis();
        }
    }
    while (0);
}

void reset_tof_sensors() {
    /* Get the I2C ToF sensors ready to read data */
    // Disable everything by driving XSHUT pins low

    // shudown all sensors
    for (uint8_t i=0; i<num_sensors; ++i) {
        digitalWrite(sensors[i].shutdown_pin, LOW);
    }
    delay(10);

    // Enable sensors one by one with respective addresses
    for (uint8_t i=0; i<num_sensors; ++i) {
        digitalWrite(sensors[i].shutdown_pin, HIGH);
        delay(100);
        if (!sensors[i].psensor->begin(sensors[i].id,
                                       false,
                                       sensors[i].pwire,
                                       sensors[i].sensor_config
                                    )) {
            error_loop();
        } 
    }
}

void read_tof_sensors() {
    /* Read each sensor and store in message struct 
        TODO: Make stamped??
    */
    for (uint8_t i=0; i<num_sensors; ++i) {
        sensors[i].range = sensors[i].psensor->readRange();
        sensors[i].status = sensors[i].psensor->timeoutOccurred();
        uint32_t time = millis();

        if (sensors[i].status || sensors[i].range > 5000) { // this is a little hacky... but it gets past the ranging error of +8000
            msg_arr.data[i] = -1;
        }
        else {
            msg_arr.data[i] = static_cast<float>(sensors[i].range);
        }
    }
}

void setup() {
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, HIGH);

    Wire.setClock(400000);
    Wire1.setClock(400000);
    delay(10);
    reset_tof_sensors();

    // Serial setup
    Serial.begin(115200);
    set_microros_serial_transports(Serial); // PlatformIO

    system_state = AGENT_WAIT;

    msg_arr.data[0] = 0.0;
    msg_arr.data[1] = 0.0;
}

void loop() {
    // Read from ToF sensors
    read_tof_sensors();

    // Check system state for connection status to micro_ros_agent
    switch (system_state) {
        case AGENT_WAIT:
            execute_every_n_ms(500, system_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : AGENT_WAIT);
            break;
        case AGENT_AVAILABLE:
            system_state = (true == create_rcl_entities()) ? AGENT_CONNECTED : AGENT_WAIT;
            if (system_state == AGENT_WAIT) {
                destroy_rcl_entities();
            }
            break;
        case AGENT_CONNECTED:
            // Publish message
            execute_every_n_ms(10, system_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED);
            if (system_state == AGENT_CONNECTED) {
                RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
            }
            break;
        case AGENT_DISCONNECTED:
            destroy_rcl_entities();
            system_state = AGENT_WAIT;
            break;
        default:
            break;
    }

    // LED control
    if (system_state == AGENT_CONNECTED) {
            unsigned long now = millis();
        if (now - last_time > 9900) {
            digitalWrite(led_pin, HIGH); 
        }
        if (now - last_time > 10000) {
            digitalWrite(led_pin, LOW); 
            last_time = now;
        }
    }     
}

/* Running this script:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 ROS_DOMAIN_ID=0
*/