#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

#include "gripper/gecko/gecko_gripper.h"
#include <gripper_ros_common/PositionSetpoint.h>
#include <gripper_ros_common/VelocitySetpoint.h>
    
using namespace gripper::gecko;

class GeckoWrapper {
  public:
    GeckoWrapper(ros::NodeHandle& nh)
        : nh_(nh) {
        
        gecko_ = std::make_unique<GeckoGripper>();

        open_pos_ = gecko_->getGripperOpenPos();
        close_pos_ = gecko_->getGripperClosePos();

        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

        position_setpoint_sub_ = nh_.subscribe("position_setpoint", 1, &GeckoWrapper::positionSetpointCb, this);
        velocity_setpoint_sub_ = nh_.subscribe("velocity_setpoint", 1, &GeckoWrapper::velocitySetpointCb, this);

        initialize_srv_ = nh_.advertiseService("initialize", &GeckoWrapper::initializeCb, this);
        open_grasp_srv_ = nh_.advertiseService("open_grasp", &GeckoWrapper::openGraspCb, this);
        close_grasp_srv_ = nh_.advertiseService("close_grasp", &GeckoWrapper::closeGraspCb, this);

        ROS_INFO("Gecko Gripper ROS node started.");
    }

    void initializeHand() {
        if (!gecko_->initialize()) {
            ROS_ERROR("Failed to initialize Gecko Gripper via config.");
        }
    }

    void spinControlLoop() {
        gecko_->controlLoopCallback();
    }

    void publishState() {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name = {"gecko_gripper"};

        GripperState currentState = gecko_->getLatestState();
        
        msg.position = {currentState.position};
        msg.velocity = {currentState.velocity};
        msg.effort = {currentState.torque};

        joint_state_pub_.publish(msg);
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    std::unique_ptr<GeckoGripper> gecko_;
    
    double open_pos_;
    double close_pos_;

    ros::Subscriber position_setpoint_sub_;
    ros::Subscriber velocity_setpoint_sub_;

    ros::ServiceServer initialize_srv_;
    ros::ServiceServer open_grasp_srv_;
    ros::ServiceServer close_grasp_srv_;

    void positionSetpointCb(const gripper_ros_common::PositionSetpointConstPtr& msg) {
        gecko_->setPosition(msg->grasp);
    }

    void velocitySetpointCb(const gripper_ros_common::VelocitySetpointConstPtr& msg) {
        gecko_->setVelocity(msg->grasp);
    }

    bool initializeCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        initializeHand();
        return true;
    }

    bool openGraspCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        ROS_INFO("Opening grasp to position: %f", open_pos_);
        gecko_->setPosition(open_pos_);
        return true;
    }

    bool closeGraspCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        ROS_INFO("Closing grasp to position: %f", close_pos_);
        gecko_->setPosition(close_pos_);
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gecko_node");
    ros::NodeHandle nh("~");

    GeckoWrapper gecko_wrapper(nh);

    gecko_wrapper.initializeHand();
    ROS_INFO("Initialized");

    ros::Rate loop_rate(100); 
    
    while (ros::ok()) {
        gecko_wrapper.spinControlLoop();
        gecko_wrapper.publishState();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
