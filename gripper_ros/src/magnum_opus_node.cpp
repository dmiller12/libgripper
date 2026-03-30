#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

#include "gripper/magnum_opus/magnum_gripper.h"
#include <gripper_ros_common/PositionSetpoint.h>
#include <gripper_ros_common/VelocitySetpoint.h>

using namespace gripper::magnum_opus;

class MagnumWrapper {
  public:
    MagnumWrapper(ros::NodeHandle& nh)
        : nh_(nh) {
        
        magnum_ = std::make_unique<MagnumGripper>();

        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

        position_setpoint_sub_ = nh_.subscribe("position_setpoint", 1, &MagnumWrapper::positionSetpointCb, this);
        velocity_setpoint_sub_ = nh_.subscribe("velocity_setpoint", 1, &MagnumWrapper::velocitySetpointCb, this);

        initialize_srv_ = nh_.advertiseService("initialize", &MagnumWrapper::initializeCb, this);
        open_grasp_srv_ = nh_.advertiseService("open_grasp", &MagnumWrapper::openGraspCb, this);
        close_grasp_srv_ = nh_.advertiseService("close_grasp", &MagnumWrapper::closeGraspCb, this);

        ROS_INFO("Magnum Gripper ROS node started.");
    }

    void initializeHand() {
        if (!magnum_->initialize()) {
            ROS_ERROR("Failed to initialize Magnum Gripper via config.");
        }
    }

    void spinControlLoop() {
        magnum_->controlLoopCallback();
    }

    void publishState() {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name = {"magnum_gripper"};

        GripperState currentState = magnum_->getLatestState();
        
        msg.position = {currentState.position};
        msg.velocity = {currentState.velocity};
        msg.effort = {currentState.torque};

        joint_state_pub_.publish(msg);
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    std::unique_ptr<MagnumGripper> magnum_;
    
    double open_pos_ = magnum_->getGripperOpenPos();
    double close_pos_ = magnum_->getGripperClosePos();

    ros::Subscriber position_setpoint_sub_;
    ros::Subscriber velocity_setpoint_sub_;

    ros::ServiceServer initialize_srv_;
    ros::ServiceServer open_grasp_srv_;
    ros::ServiceServer close_grasp_srv_;

    void positionSetpointCb(const gripper_ros_common::PositionSetpointConstPtr& msg) {
        magnum_->setPosition(msg->grasp);
    }

    void velocitySetpointCb(const gripper_ros_common::VelocitySetpointConstPtr& msg) {
        magnum_->setVelocity(msg->grasp);
    }

    bool initializeCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        initializeHand();
        return true;
    }

    bool openGraspCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        ROS_INFO("Opening grasp to position: %f", open_pos_);
        magnum_->setPosition(open_pos_);
        return true;
    }

    bool closeGraspCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        ROS_INFO("Closing grasp to position: %f", close_pos_);
        magnum_->setPosition(close_pos_);
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "magnum_node");
    ros::NodeHandle nh("~");

    MagnumWrapper magnum_wrapper(nh);

    magnum_wrapper.initializeHand();
    ROS_INFO("Initialized");

    ros::Rate loop_rate(100); 
    
    while (ros::ok()) {
        magnum_wrapper.spinControlLoop();
        magnum_wrapper.publishState();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}