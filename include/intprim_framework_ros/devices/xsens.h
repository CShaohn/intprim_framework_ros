#pragma once
#include "device_interface.h"
#include "robot_interface.h"
#include <vector>
#include "ros/ros.h"
#include <std_msgs/Float32MultiArray.h>

class XsensState : public DeviceState
{
    public:
        static constexpr unsigned int           NUM_JOINTS = 49;
        static double                           JOINT_THRESHOLD;

        XsensState();

        void to_matrix(std::vector<float>& trajectory) const;

        const std_msgs::Float32MultiArray::ConstPtr& get_message();

        bool valid() const;

        void set_message(const std_msgs::Float32MultiArray::ConstPtr& message);

        bool within_threshold(const std::vector<float>& trajectory, std::size_t trajectory_idx) const;

    private:
        std_msgs::Float32MultiArray::ConstPtr                  m_message;
        bool                                                   m_valid;
};

class XsensInterface : public RobotInterface           
{
    public:
        XsensInterface(ros::NodeHandle handle);

        // Covariant return type.
        const XsensState& get_state();

        void publish_state(const std::vector<float>& trajectory, std::size_t trajectory_idx);
    private:
        static double                               CONTROL_TIME_BUFFER;
        static int                                  CONTROL_FREQUENCY;
        static double                               MAX_ACCELERATION;

        ros::Subscriber                             m_stateSubscriber;
        ros::Publisher                              m_statePublisher;

        XsensState                                  m_currentState;
        std_msgs::Float32MultiArray                 m_publishMessage;

        unsigned int                                m_control_frequency;
        float                                       m_messageTime;

        void state_callback(const std_msgs::Float32MultiArray::ConstPtr& message);
};