/*

Connection reset code from: https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
*/

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <teensy32_tof_msgs/msg/to_f_data.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)){}}

rcl_publisher_t publisher;
teensy32_tof_msgs__msg__ToFData msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

const int led_pin = 13;
unsigned long last_time = 0;

enum SystemState {
    AGENT_WAIT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} system_state;

void error_loop(void) {
    /* Error loop LED indicator */
    while (true) {
        digitalWrite(led_pin, !digitalRead(led_pin));
        delay(1000);
    }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    /* Publisher timer callback method */
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        msg.tof0 += 0.01;
        msg.tof1 += 1.0;
    }
}

bool create_rcl_entities() {
    /* Create the microROS entities */
    // Init allocator
    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    // create node
    RCCHECK(rclc_node_init_default(&node, "teensy32_microros", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(teensy32_tof_msgs, msg, ToFData),
        "microROS/tof_data"
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

void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial); // PlatformIO

    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, LOW);

    system_state = AGENT_WAIT;

    msg.tof0 = 0.0;
    msg.tof1 = 0.0;
}

void loop() {
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
            execute_every_n_ms(200, system_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED);
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