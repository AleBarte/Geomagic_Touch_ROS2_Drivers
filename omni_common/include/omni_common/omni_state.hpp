#ifndef OMNI_STATE_HPP_
#define OMNI_STATE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>
#include <sstream>
#include <iostream>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "omni_msgs/msg/omni_button_event.hpp"
#include "omni_msgs/msg/omni_feedback.hpp"
#include "omni_msgs/msg/omni_state.hpp"

struct OmniState {
  hduVector3Dd position;  //3x1 vector of position
  hduVector3Dd velocity;  //3x1 vector of velocity
  hduVector3Dd body_angular_velocity;
  hduVector3Dd inp_vel1;  //3x1 history of velocity used for filtering velocity estimate
  hduVector3Dd inp_vel2;
  hduVector3Dd inp_vel3;
  hduVector3Dd out_vel1;
  hduVector3Dd out_vel2;
  hduVector3Dd out_vel3;
  hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity
  hduVector3Dd pos_hist2;
  hduQuaternion rot;
  hduVector3Dd joints;
  hduVector3Dd force;   //3 element double vector force[0], force[1], force[2]
  float thetas[7];
  int buttons[2];
  int buttons_prev[2];
  bool lock;
  bool close_gripper;
  hduVector3Dd lock_pos;
  double units_ratio;
};

class geoRos : public rclcpp::Node
{

public:

    geoRos(const std::string &node_name);
    void init(OmniState* s);

private:


    void forceCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    void publish();

    rclcpp::Publisher<omni_msgs::msg::OmniState>::SharedPtr state_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr button_publisher_;
    rclcpp::Publisher<omni_msgs::msg::OmniButtonEvent>::SharedPtr button_event_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr haptic_sub_;
    OmniState* state_;

    // Parameters
    std::string ref_frame_;
    std::string units_;
    double publishing_rate_;

    // Flags
    bool is_initialized_ = false;

    // Common timer object
    rclcpp::TimerBase::SharedPtr timer_;

};
#endif  // OMNI_STATE_HPP_
