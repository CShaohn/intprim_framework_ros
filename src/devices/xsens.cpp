#include "devices/xsens.h"
#include <boost/bind.hpp>
#include <chrono>
#include <regex>
#include <sstream>
#include <string>
#include <thread>

double XsensState::JOINT_THRESHOLD = 0.0;


XsensState::XsensState() :
    m_message(),
    m_valid(false)
{

}

void XsensState::to_matrix(std::vector<float>& trajectory) const
{

    if(m_message)
    {
        for(std::size_t joint_idx = 0; joint_idx < NUM_JOINTS; ++joint_idx)
        {
            trajectory.push_back(m_message->data[joint_idx]);
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

const std_msgs::Float32MultiArray::ConstPtr& XsensState::get_message()
{
    return m_message;
}

void XsensState::set_message(const std_msgs::Float32MultiArray::ConstPtr& message)
{
    m_message = message;
    m_valid = true;
}

bool XsensState::valid() const
{
    return m_valid;
}

bool XsensState::within_threshold(const std::vector<float>& trajectory, std::size_t trajectory_idx) const
{
    if(m_message)
    {
        for(std::size_t joint_idx = 0; joint_idx < NUM_JOINTS; ++joint_idx)
        {
              
            if(std::abs(m_message->data[joint_idx] - trajectory[joint_idx + trajectory_idx]) > JOINT_THRESHOLD)
            {
                return false;
            }
            
        }
        return true;
    }
    return false;
}

XsensInterface::XsensInterface(ros::NodeHandle handle) :
    m_stateSubscriber(),
    m_statePublisher(),
    m_currentState(),
    m_publishMessage()
{
  
 
    m_statePublisher = handle.advertise<std_msgs::Float32MultiArray>("/xsens/data", 1);
    m_stateSubscriber = handle.subscribe("/xsens_data", 1, &XsensInterface::state_callback, this);
}

const XsensState& XsensInterface::get_state()
{
    return m_currentState;
}
void XsensInterface::publish_state(const std::vector<float>& trajectory, std::size_t trajectory_idx)
{
    for(std::size_t joint_idx = 0; joint_idx < XsensState::NUM_JOINTS; ++joint_idx)
    {
          m_publishMessage.data[joint_idx] = trajectory[trajectory_idx + joint_idx];
    }

    m_statePublisher.publish(m_publishMessage);
}

void XsensInterface::state_callback(const std_msgs::Float32MultiArray::ConstPtr& message)
{
    m_currentState.set_message(message);
}