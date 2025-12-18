#include <Arduino.h>
#include <MCP342x.h>
#include <Wire.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// 0x68 is the default address for all MCP342x devices
uint8_t address = 0x68;
MCP342x adc = MCP342x(address);
float tension;

// ROS Configuration
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// micro-ROS Agent Connection States
enum AgentState {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

AgentState agent_state = WAITING_AGENT;

// Timing variables
unsigned long last_read_time = 0;
const unsigned long READ_INTERVAL = 0.5; // ms (target 50ms, leave 5ms for ROS processing)
long val = 0;

// Function to map float values
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Error handling loop
void error_loop() {
  while (1) {
    delay(100);
  }
}

// Timer callback for publishing force value
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    msg.data = tension;  // Publish force value
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

// Create ROS entities
bool create_ros_entities()
{
  const char * node_name = "winch0_force_sensor_node";
  const char * ns = "";

  allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 107));  
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  RCCHECK(rclc_node_init_default(&node, node_name, ns, &support));

  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "winch0/force_sensor_data"));

  const unsigned int timer_timeout = 0.5;  // 20Hz
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
  return true;
}

// Destroy ROS entities when agent disconnects
void destroy_ros_entities()
{
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  rcl_publisher_fini(&publisher, &node);
}

void setup()
{
    // Initialize Serial & micro-ROS Transport
    Serial.begin(2000000);
    Wire.begin();
    Wire.setClock(400000); // Increase I2C clock speed to 400kHz

    // Reset devices
    MCP342x::generalCallReset();
    delay(1); // MC342x needs 300us to settle
    
    // Check device present
    Wire.requestFrom(address, (uint8_t)1);
    if (!Wire.available()) {
      while (1)
        ;
    }
  
    // Start a continuous conversion
    MCP342x::Config config(MCP342x::channel1, MCP342x::continous,
                         MCP342x::resolution16, MCP342x::gain2);
    adc.configure(config);
    
    set_microros_serial_transports(Serial);
    delay(2000);
}

void loop()
{
    // Handle micro-ROS Agent Connection
    switch (agent_state) {
        case WAITING_AGENT:
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                agent_state = AGENT_AVAILABLE;
            }
            break;

        case AGENT_AVAILABLE:
            if (create_ros_entities()) {
                agent_state = AGENT_CONNECTED;
            } else {
                agent_state = WAITING_AGENT;
            }
            break;

        case AGENT_CONNECTED:
            if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                agent_state = AGENT_DISCONNECTED;
            } else {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); // Reduced timeout
            }
            break;

        case AGENT_DISCONNECTED:
            destroy_ros_entities();
            agent_state = WAITING_AGENT;
            break;
    }

    // Read ADC at 20Hz rate
    unsigned long current_time = millis();
    if (current_time - last_read_time >= READ_INTERVAL) {
        last_read_time = current_time;
        
        // Read the value directly without reconfiguring (since we're in continuous mode)
        MCP342x::Config status;
        uint8_t err = adc.read(val, status);
        
        if (err == 0) { // Success
            tension = mapfloat(constrain(val, 5619, 24700), 5619, 24700, 0.0, 100.0);
        }
    }
}