
// ROS Headers
#include "ros/ros.h"
#include "mavros_msgs/MessageInterval.h"

#include <mavlink/v2.0/common/common.hpp>
#include <mavlink/v2.0/ardupilotmega/ardupilotmega.hpp>

//======= Forward declarations
bool setMessageRate(uint32_t msg_id, float rate);
ros::ServiceClient setMessageRateClient;

// Node for setting UAV data rates

int main(int argc, char **argv) {

  // Initialise ROS
  ros::init(argc, argv, "setRates");
  ros::NodeHandle n;

  // Initialise Services
  setMessageRateClient = n.serviceClient<mavros_msgs::MessageInterval>("mavros/set_message_interval");

  // Give other systems time to boot
  ros::Duration(5.0).sleep();

  ROS_INFO("==== Set Aircraft Streaming Rates ====");

  while (ros::ok()) {

    bool rateSet_failed = 0;

    // Set data rates
    if (setMessageRate(mavlink::common::msg::GPS_GLOBAL_ORIGIN::MSG_ID,    1.0f) == 0) rateSet_failed = 1; // mavros/global_postion/gp_offset
    if (setMessageRate(mavlink::common::msg::BATTERY_STATUS::MSG_ID,       1.0f) == 0) rateSet_failed = 1; // mavros/battery
    if (setMessageRate(mavlink::common::msg::ATTITUDE::MSG_ID,            20.0f) == 0) rateSet_failed = 1; // mavros/imu/data
    if (setMessageRate(mavlink::common::msg::ATTITUDE_QUATERNION::MSG_ID, 20.0f) == 0) rateSet_failed = 1; // mavros/imu/data
    if (setMessageRate(mavlink::common::msg::RAW_IMU::MSG_ID,            200.0f) == 0) rateSet_failed = 1; // mavros/imu/data_raw
    if (setMessageRate(mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID,  20.0f) == 0) rateSet_failed = 1; // mavros/local_postion/pose
    if (setMessageRate(mavlink::common::msg::GLOBAL_POSITION_INT::MSG_ID, 20.0f) == 0) rateSet_failed = 1; // mavros/local_postion/pose
    // if (setMessageRate(mavlink::ardupilotmega::msg::RANGEFINDER::MSG_ID,   0.0f) == 0) rateSet_failed = 1; // Rangefinder message (prefer DISTANCE_SENSOR)
    // if (setMessageRate(mavlink::common::msg::DISTANCE_SENSOR::MSG_ID,     10.0f) == 0) rateSet_failed = 1;
    if (setMessageRate(mavlink::common::msg::GPS_RAW_INT::MSG_ID,          5.0f) == 0) rateSet_failed = 1;

    if (!rateSet_failed)
    {
        // All done
        ROS_INFO("Aircraft Data Rates Set");

        // Exit the program
        return 0;
    }

    // This round of setting rates failed, wait and try again
    ros::Duration(20.0).sleep();
    ros::spinOnce();

  }

  // Exit the program
  return 0;
}


bool setMessageRate(uint32_t msg_id, float rate) {
  mavros_msgs::MessageInterval cmd;

  cmd.request.message_id = msg_id;
  cmd.request.message_rate = rate;

  setMessageRateClient.call(cmd);

  if (cmd.response.success) {
    ROS_INFO("Set message rate (#%d) successful",msg_id);
  } else {
    ROS_INFO("!!! Set message rate (#%d) failed !!!",msg_id);
  }

  return (cmd.response.success);

}