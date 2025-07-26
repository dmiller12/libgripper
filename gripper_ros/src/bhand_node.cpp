#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

#include <gripper/barrett/barrett_hand.h>
#include <gripper_ros_common/PositionSetpoint.h>
#include <gripper_ros_common/VelocitySetpoint.h>

using namespace gripper::barrett;

class BHandWrapper {
  public:
    BHandWrapper(ros::NodeHandle& nh)
        : nh_(nh) {
        nh_.param<std::string>("port", port, "/dev/ttyUSB0");

        bhand_ = std::make_unique<BarrettHand>();

        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

        position_setpoint_sub_ = nh_.subscribe("position_setpoint", 1, &BHandWrapper::positionSetpointCb, this);
        velocity_setpoint_sub_ = nh_.subscribe("velocity_setpoint", 1, &BHandWrapper::velocitySetpointCb, this);

        initialize_srv_ = nh_.advertiseService("initialize", &BHandWrapper::initialize, this);
        open_grasp_srv_ = nh_.advertiseService("open_grasp", &BHandWrapper::openGrasp, this);
        close_grasp_srv_ = nh_.advertiseService("close_grasp", &BHandWrapper::closeGrasp, this);
        open_spread_srv_ = nh_.advertiseService("open_spread", &BHandWrapper::openSpread, this);
        close_spread_srv_ = nh_.advertiseService("close_spread", &BHandWrapper::closeSpread, this);

        ROS_INFO("BHand ROS node started.");
    }

    void publishState() {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name = {
            "FingerOne/KuckleTwoJoint",
            "FingerTwo/KuckleTwoJoint",
            "FingerThree/KuckleTwoJoint",
            "FingerOne/KuckleOneJoint",
            "FingerTwo/KuckleOneJoint",
            "FingerOne/KuckleThreeJoint",
            "FingerTwo/KuckleThreeJoint",
            "FingerThree/KuckleThreeJoint"
        };

        HandState currentState = bhand_->getLatestState();
        msg.position.resize(8);
        // finger grasp
        msg.position[0] = currentState.joint_positions[0];
        msg.position[1] = currentState.joint_positions[1];
        msg.position[2] = currentState.joint_positions[2];

        // spread
        msg.position[3] = currentState.joint_positions[3];
        msg.position[4] = currentState.joint_positions[3];

        // distal joint
        msg.position[5] = currentState.joint_positions[0] * SECOND_LINK_RATIO;
        msg.position[6] = currentState.joint_positions[1] * SECOND_LINK_RATIO;
        msg.position[7] = currentState.joint_positions[2] * SECOND_LINK_RATIO;

        msg.velocity.resize(8);
        // finger grasp
        msg.velocity[0] = currentState.joint_velocities[0];
        msg.velocity[1] = currentState.joint_velocities[1];
        msg.velocity[2] = currentState.joint_velocities[2];

        // spread
        msg.velocity[3] = currentState.joint_velocities[3];
        msg.velocity[4] = currentState.joint_velocities[3];

        // distal joint
        msg.velocity[5] = currentState.joint_velocities[0] * SECOND_LINK_RATIO;
        msg.velocity[6] = currentState.joint_velocities[1] * SECOND_LINK_RATIO;
        msg.velocity[7] = currentState.joint_velocities[2] * SECOND_LINK_RATIO;

        joint_state_pub_.publish(msg);
    }

    void initializeHand() {
        bhand_->initialize(port);
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    std::unique_ptr<BarrettHand> bhand_;
    std::string port;
    static constexpr double SECOND_LINK_RATIO = 0.424068;

    ros::Subscriber position_setpoint_sub_;
    ros::Subscriber velocity_setpoint_sub_;

    ros::ServiceServer initialize_srv_;
    ros::ServiceServer open_grasp_srv_;
    ros::ServiceServer close_grasp_srv_;
    ros::ServiceServer open_spread_srv_;
    ros::ServiceServer close_spread_srv_;

    void positionSetpointCb(const gripper_ros_common::PositionSetpointConstPtr& msg) {
        bhand_->setPosition({msg->grasp, msg->grasp, msg->grasp, msg->spread});
    }

    void velocitySetpointCb(const gripper_ros_common::VelocitySetpointConstPtr& msg) {
        bhand_->setVelocity(msg->grasp, msg->spread);
    }

    bool initialize(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        bhand_->initialize(port);
        return true;
    }

    bool openGrasp(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        bhand_->open(MotorGroup::AllFingers);
        return true;
    }

    bool closeGrasp(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        bhand_->close(MotorGroup::AllFingers);
        return true;
    }

    bool openSpread(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        bhand_->open(MotorGroup::Spread);
        return true;
    }

    bool closeSpread(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        bhand_->close(MotorGroup::Spread);
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bhand_node");
    ros::NodeHandle nh("~");

    BHandWrapper bhand_wrapper(nh);

    bool init_prompt = false;
    nh.param<bool>("init_prompt", init_prompt, false);

    if (init_prompt) {
        ROS_INFO("Press Enter to initalize hand. Ensure it has room for full initialization: ");
        std::cin.get();
        bhand_wrapper.initializeHand();
        ROS_INFO("Initialized");
    } else {
        ROS_WARN("Run the following command in a new terminal to initialize the hand:");
        ROS_WARN("rosservice call %s/initialize", ros::this_node::getName().c_str());
    }

    ros::Rate loop_rate(25);
    while (ros::ok()) {
        bhand_wrapper.publishState();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
