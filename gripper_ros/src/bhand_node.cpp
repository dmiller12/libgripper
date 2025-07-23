#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// This is the only place you include your library's header
#include <gripper/barrett/barrett_hand.h> // Or your main header file
//
using namespace gripper::barrett;


class BHandWrapper {
public:
    BHandWrapper(ros::NodeHandle& nh) :
        nh_(nh)
    {
        // Get hardware connection details from the ROS Parameter Server
        std::string port;
        nh_.param<std::string>("port", port, "/dev/ttyUSB0");
        
        // Instantiate your actual gripper object from the library
        bhand_ = std::make_unique<BarrettHand>(); // Using your class

        if (!bhand_->initialize(port)) {
            ROS_ERROR("Failed to connect to gripper on port %s!", port.c_str());
            ros::shutdown();
            return;
        }

        // Setup a publisher for the gripper's state
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
        
        ROS_INFO("BHand ROS node started.");
    }

    // This loop is called from main() to publish feedback
    void publishState() {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name = {"J1", "J2", "J3", "J4"}; // Use your actual joint name
        HandState currentState = bhand_->getLatestState();
        for (const auto& p : currentState.joint_positions) {
            msg.position.push_back(p);
        }

        for (const auto& v : currentState.joint_velocities) {
            msg.velocity.push_back(v);
        }
        joint_state_pub_.publish(msg);
    }

private:

    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    std::unique_ptr<BarrettHand> bhand_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bhand_node");
    ros::NodeHandle nh("~");

    BHandWrapper bhand_wrapper(nh);

    ros::Rate loop_rate(50); // Publish state at 50 Hz
    while (ros::ok()) {
        bhand_wrapper.publishState();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
