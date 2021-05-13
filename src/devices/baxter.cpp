#include "devices/baxter.h"

#include <boost/bind.hpp>
#include <chrono>
#include <regex>
#include <sstream>
#include <string>
#include <thread>

double BaxterState::JOINT_THRESHOLD = 0.0;
double BaxterInterface::CONTROL_TIME_BUFFER = 0.0;
int    BaxterInterface::CONTROL_FREQUENCY = 0;
double BaxterInterface::MAX_ACCELERATION = 0.0;

BaxterState::BaxterState() :
    m_message(),
    m_valid(false)
{

}

void BaxterState::to_matrix(std::vector<float>& trajectory) const
{
    if(m_message)
    {
        for(std::size_t joint_idx = 0; joint_idx < NUM_JOINTS; ++joint_idx)
        {
            trajectory.push_back(m_message->position[joint_idx]);
        }
    }
    else
    {
        for(std::size_t joint_idx = 0; joint_idx < NUM_JOINTS; ++joint_idx)
        {
            trajectory.push_back(0.0);
        }
    }
}

const sensor_msgs::JointState::ConstPtr& BaxterState::get_message()
{
    return m_message;
}

void BaxterState::set_message(const sensor_msgs::JointState::ConstPtr& message)
{
    m_message = message;
    m_valid = true;
}

bool BaxterState::valid() const
{
    return m_valid;
}

bool BaxterState::within_threshold(const std::vector<float>& trajectory, std::size_t trajectory_idx) const
{
    if(m_message)
    {
        for(std::size_t joint_idx = 0; joint_idx < NUM_JOINTS; ++joint_idx)
        {
            if(std::abs(m_message->position[joint_idx] - trajectory[joint_idx + trajectory_idx]) > JOINT_THRESHOLD)
            {
                return false;
            }
        }
        return true;
    }
    return false;
}

BaxterInterface::BaxterInterface(ros::NodeHandle handle) :
    m_stateSubscriber(),
    m_statePublisher(),
    m_currentState(),
    m_publishMessage()
{

    handle.getParam("control/robot1/control_time_buffer", CONTROL_TIME_BUFFER);
    handle.getParam("control/robot1/control_frequency", CONTROL_FREQUENCY);
    handle.getParam("control/robot1/max_acceleration", MAX_ACCELERATION);
    handle.getParam("control/robot1/joint_distance_threshold", BaxterState::JOINT_THRESHOLD);

    std::cout << "control time buffer: " << CONTROL_TIME_BUFFER << ", control frequency: " << CONTROL_FREQUENCY << ", max accel: " << MAX_ACCELERATION << ", joint thresh: " << BaxterState::JOINT_THRESHOLD << std::endl;

    m_messageTime = (1.0 / CONTROL_FREQUENCY) * CONTROL_TIME_BUFFER;

    // m_publishMessage.acceleration = MAX_ACCELERATION;
    // m_publishMessage.blend = 0;
    // m_publishMessage.command = "speed";
    // m_publishMessage.gain = 0;
    // m_publishMessage.jointcontrol = true;
    // m_publishMessage.lookahead = 0;
    // m_publishMessage.time = m_messageTime;
    // m_publishMessage.velocity = 0;

    m_statePublisher = handle.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1);
    m_stateSubscriber = handle.subscribe("/baxter/jointstates_left", 1, &BaxterInterface::state_callback, this);
}

const BaxterState& BaxterInterface::get_state()
{
    return m_currentState;
}

void BaxterInterface::publish_state(const std::vector<float>& trajectory, std::size_t trajectory_idx)
{
    for(std::size_t joint_idx = 0; joint_idx < BaxterState::NUM_JOINTS; ++joint_idx)
    {
        m_publishMessage.mode = 1;
        m_publishMessage.command[joint_idx] = trajectory[trajectory_idx + joint_idx];
    }

    m_statePublisher.publish(m_publishMessage);
}

void BaxterInterface::state_callback(const sensor_msgs::JointState::ConstPtr& message)
{
    m_currentState.set_message(message);
}