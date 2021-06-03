#pragma once
#include "device_interface.h"
#include "robot_interface.h"
#include <vector>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>

class BaxterState : public DeviceState                      //BaxterState 继承于 DeviceState
{
    public:
        static constexpr unsigned int           NUM_JOINTS = 7;    //only single hand
        static const std::string                JOINT_NAMES[NUM_JOINTS];
        static double                           JOINT_THRESHOLD;

        BaxterState();

        void to_matrix(std::vector<float>& trajectory) const;

        const sensor_msgs::JointState::ConstPtr& get_message();
        
        bool valid() const;

        void set_message(const sensor_msgs::JointState::ConstPtr& message);

        bool within_threshold(const std::vector<float>& trajectory, std::size_t trajectory_idx) const;

    private:
        sensor_msgs::JointState::ConstPtr           m_message;
        bool                                        m_valid;
};

class BaxterInterface : public RobotInterface               //BaxterInterface 继承于 RobotInterface
{
    public:
        BaxterInterface(ros::NodeHandle handle);

        // Covariant return type.
        const BaxterState& get_state();

        void publish_state(const std::vector<float>& trajectory, std::size_t trajectory_idx);
    private:
        static double                               CONTROL_TIME_BUFFER;
        static int                                  CONTROL_FREQUENCY;
        static double                               MAX_ACCELERATION;
        
        ros::Subscriber                             m_stateSubscriber;
        ros::Publisher                              m_statePublisher;

        BaxterState                                 m_currentState;
        baxter_core_msgs::JointCommand              m_publishMessage;

        unsigned int                                m_control_frequency;
        float                                       m_messageTime;

        void state_callback(const sensor_msgs::JointState::ConstPtr& message);
};