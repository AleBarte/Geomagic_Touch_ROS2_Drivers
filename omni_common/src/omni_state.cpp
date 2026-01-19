#include "../../omni_common/include/omni_common/omni_state.hpp"

geoRos::geoRos(const std::string &node_name)
    : Node(node_name)
{

    // Parameter Declaration
    this->declare_parameter<std::string>("ref_frame", "omni_base");
    this->declare_parameter<std::string>("units", "mm");
    this->declare_parameter<double>("publishing_rate", 500.0);

    //Parameter Retrieval
    this->ref_frame_ = this->get_parameter("ref_frame").as_string();
    this->units_ = this->get_parameter("units").as_string();
    this->publishing_rate_ = this->get_parameter("publishing_rate").as_double();

    // Initialize Publishers
    std::ostringstream stream1;
    stream1 << node_name << "/button_event";
    std::string button_event_topic = std::string(stream1.str());
    this->button_event_publisher_  = this->create_publisher<omni_msgs::msg::OmniButtonEvent>(button_event_topic, 10);

    std::ostringstream stream2;
    stream2 << node_name << "/button";
    std::string button_topic = std::string(stream2.str());
    this->button_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(button_topic, 10);

    std::ostringstream stream3;
    stream3 << node_name << "/state";
    std::string state_topic_name = std::string(stream3.str());
    this->state_publisher_ = this->create_publisher<omni_msgs::msg::OmniState>(state_topic_name, 10);

    std::ostringstream stream4;
    stream4 << node_name << "/pose";
    std::string pose_topic_name = std::string(stream4.str());
    this->pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_name, 10);

    std::ostringstream stream5;
    stream5 << node_name << "/joint_states";
    std::string joint_topic_name = std::string(stream5.str());
    this->joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_topic_name, 10);

    std::ostringstream stream6;
    stream6 << node_name << "/twist";
    std::string twist_topic_name = std::string(stream6.str());
    this->twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_name, 10);

    // Initialize Subscribers
    std::ostringstream stream7;
    stream7 << node_name << "/force_feedback";
    std::string force_feedback_topic = std::string(stream7.str());
    this->haptic_sub_ = this->create_subscription<omni_msgs::msg::OmniFeedback>(
        force_feedback_topic, 10,
        std::bind(&geoRos::forceCallback, this, std::placeholders::_1));

    std::chrono::milliseconds period = std::chrono::milliseconds((int)(1000.0 / this->publishing_rate_));
    this->timer_ = this->create_wall_timer(
        period, std::bind(&geoRos::publish, this));
}

// Initialization

void geoRos::init(OmniState* s)
{
    this->state_ = s;
    this->state_->buttons[0] = 0;
    this->state_->buttons[1] = 0;
    this->state_->buttons_prev[0] = 0;
    this->state_->buttons_prev[1] = 0;
    hduVector3Dd zeros(0, 0, 0);
    this->state_->velocity = zeros;
    this->state_->inp_vel1 = zeros;
    this->state_->inp_vel2 = zeros;
    this->state_->inp_vel3 = zeros;
    this->state_->out_vel1 = zeros;
    this->state_->out_vel2 = zeros;
    this->state_->out_vel3 = zeros;
    this->state_->pos_hist1 = zeros;
    this->state_->pos_hist2 = zeros;
    this->state_->lock = false;
    this->state_->close_gripper = false;
    this->state_->lock_pos = zeros;

    if (!this->units_.compare("mm"))
        this->state_->units_ratio = 1.0;
    else if (!this->units_.compare("cm"))
        this->state_->units_ratio = 10.0;
    else if (!this->units_.compare("dm"))
        this->state_->units_ratio = 100.0;
    else if (!this->units_.compare("m"))
        this->state_->units_ratio = 1000.0;
    else
    {
        this->state_->units_ratio = 1.0;
        RCLCPP_WARN(this->get_logger(), "Unknown units [%s] using [mm]", this->units_.c_str());
        this->units_ = "mm";
    }
    RCLCPP_INFO(this->get_logger(), "Geomagic position given in [%s], ratio [%.1f]", this->units_.c_str(), this->state_->units_ratio);

    this->is_initialized_ = true;
}

//-------------------------------------------------------------------------
// Force Callback

void geoRos::forceCallback(const omni_msgs::msg::OmniFeedback::SharedPtr msg)
{

    if (!this->is_initialized_)
        return;

    //? The small damping term is added to stabilize the overall force feedback of the device.
    //? In case direct impedance matching is required, this term has to be removed.

    this->state_->force[0] = msg->force.x - 0.001 * this->state_->velocity[0];
    this->state_->force[1] = msg->force.y - 0.001 * this->state_->velocity[1];
    this->state_->force[2] = msg->force.z - 0.001 * this->state_->velocity[2];

    this->state_->lock_pos[0] = msg->position.x;
    this->state_->lock_pos[1] = msg->position.y;
    this->state_->lock_pos[2] = msg->position.z;
}

//-------------------------------------------------------------------------
// Publish Function

void geoRos::publish()
{
    if (!this->is_initialized_)
        return;

    //* Build the state msg
    omni_msgs::msg::OmniState state_msg;
    // Locked
    state_msg.locked = this->state_->lock;
    state_msg.close_gripper = this->state_->close_gripper;
    // Position
    state_msg.pose.position.x = this->state_->position[0];
    state_msg.pose.position.y = this->state_->position[1];
    state_msg.pose.position.z = this->state_->position[2];
    // Orientation
    state_msg.pose.orientation.x = this->state_->rot.v()[0];
    state_msg.pose.orientation.y = this->state_->rot.v()[1];
    state_msg.pose.orientation.z = this->state_->rot.v()[2];
    state_msg.pose.orientation.w = this->state_->rot.s();
    // Velocity
    state_msg.velocity.x= this->state_->velocity[0];
    state_msg.velocity.y = this->state_->velocity[1];
    state_msg.velocity.z = this->state_->velocity[2];
    //TODO: Add Angular Velocity
    //TODO: Purge from this insane msg definition

    state_msg.header.stamp = this->now();
    

    //Joint States
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = this->now();
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.name[0] = "waist";
    joint_state.position[0] = -this->state_->thetas[1];
    joint_state.name[1] = "shoulder";
    joint_state.position[1] = this->state_->thetas[2];
    joint_state.name[2] = "elbow";
    joint_state.position[2] = this->state_->thetas[3];
    joint_state.name[3] = "yaw";
    joint_state.position[3] = -this->state_->thetas[4] + M_PI;
    joint_state.name[4] = "pitch";
    joint_state.position[4] = -this->state_->thetas[5] - 3*M_PI/4;
    joint_state.name[5] = "roll";
    joint_state.position[5] = -this->state_->thetas[6] - M_PI;

    // Pose Message
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = state_msg.header;
    pose_msg.header.frame_id = this->ref_frame_;
    pose_msg.pose = state_msg.pose;
    pose_msg.pose.position.x /= 1000.0;
    pose_msg.pose.position.y /= 1000.0;
    pose_msg.pose.position.z /= 1000.0;

    // Twist Message
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header = state_msg.header;
    twist_msg.twist.linear.x = state_msg.velocity.x;
    twist_msg.twist.linear.y = state_msg.velocity.y;
    twist_msg.twist.linear.z = state_msg.velocity.z;
    //TODO: Fill angular velocity
    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = 0.0;

    if ((this->state_->buttons[0] != this->state_->buttons_prev[0])
        || (this->state_->buttons[1] != this->state_->buttons_prev[1]))
    {

        if (this->state_->buttons[0] == 1) {
            this->state_->close_gripper = !(this->state_->close_gripper);
        }
        if (this->state_->buttons[1] == 1) {
            this->state_->lock = !(this->state_->lock);
        }
        // Publish button event
        omni_msgs::msg::OmniButtonEvent button_event;
        button_event.grey_button = this->state_->buttons[0];
        button_event.white_button = this->state_->buttons[1];
        this->state_->buttons_prev[0] = this->state_->buttons[0];
        this->state_->buttons_prev[1] = this->state_->buttons[1];
        this->button_event_publisher_->publish(button_event);

        // Publish button state
        sensor_msgs::msg::Joy button_msg;
        button_msg.header = state_msg.header;
        button_msg.buttons.resize(2);
        button_msg.buttons[0] = this->state_->buttons[0];
        button_msg.buttons[1] = this->state_->buttons[1];
        this->button_publisher_->publish(button_msg);
    }
    // Publish all messages
    this->state_publisher_->publish(state_msg);
    this->joint_publisher_->publish(joint_state);
    this->pose_publisher_->publish(pose_msg);
    this->twist_publisher_->publish(twist_msg);

}