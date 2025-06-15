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
// Configuration settings
MCP342x::Config config(MCP342x::channel1, MCP342x::oneShot,
		       MCP342x::resolution16, MCP342x::gain2);

// Configuration/status read back from the ADC
MCP342x::Config status;

// Inidicate if a new conversion should be started
bool startConversion = false;

long val=0;
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
  const char * node_name = "force_sensor_node";
  const char * ns = "";

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, node_name, ns, &support));

  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "force_sensor_data"));

  const unsigned int timer_timeout = 500;
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
    Serial.begin(115200);
    Wire.begin();
  

    // Reset devices
    MCP342x::generalCallReset();
    delay(1); // MC342x needs 300us to settle
    
    // Check device present
    Wire.requestFrom(address, (uint8_t)1);
    if (!Wire.available()) {
      while (1)
        ;
    }
  
    // First time loop() is called start a conversion
    startConversion = true;
    set_microros_serial_transports(Serial);
    delay(2000);

    // Serial.println("MCP3428 Analog to Digital Converter");

    // // Initialize Display
    // Serial.println("Initializing display...");


    // Serial.println("Display initialized.");
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
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;

        case AGENT_DISCONNECTED:
            destroy_ros_entities();

            agent_state = WAITING_AGENT;
            break;
    }

    MCP342x::Config status;
    // Initiate a conversion; convertAndRead() will wait until it can be read
    uint8_t err = adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot,
                     MCP342x::resolution16, MCP342x::gain2,
                     1000000, val, status);
    
    tension = mapfloat(constrain(val, 5813, 29390), 5813, 29390, 0.0, 100.0); // Map the value to a float range
    
    delay(500);
}