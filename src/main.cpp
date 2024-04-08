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

void error_loop(void) {
    while (true) {
        digitalWrite(led_pin, !digitalRead(led_pin));
        delay(5000);
    }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        msg.tof0 += 0.01;
        msg.tof1 += 1.0;
    }
}

void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    // set_microros_transports();

    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, LOW);
    
    delay(2000);


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

    msg.tof0 = 0.0;
    msg.tof1 = 0.0;
}

void loop() {
    // delay(100);
    unsigned long now = millis();
    if (now - last_time > 9900) {
        digitalWrite(led_pin, HIGH); 
    }
    if (now - last_time > 10000) {
        digitalWrite(led_pin, LOW); 
        last_time = now;
    }
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
       
}

/* Running this script:
After restarting the host and the board, I started it with ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0672FF3530384E5043161319-if02 -v6 ROS_DOMAIN_ID=0 and only then I plugged in the board. In another terminal window i did export ROS_DOMAIN_ID=0 followed by ros2 topic list. Now it works, thanks!
*/