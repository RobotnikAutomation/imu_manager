#ifndef _IMU_MANAGER_STATE_MACHINE_
#define _IMU_MANAGER_STATE_MACHINE_

#include <ros/ros.h>

#include <algorithm>

//! Class to represent a state machine
class StateMachine
{
private:
  std::vector<std::string> available_states_;
  std::string current_state_;
  std::string desired_state_;

  const std::string INVALID_STATE = "";

public:
  ~StateMachine()
  {
  }

  StateMachine()
  {
    current_state_ = INVALID_STATE;
    desired_state_ = INVALID_STATE;
  }

  StateMachine(const std::vector<std::string>& available_states)
  {
    available_states_ = available_states;
    current_state_ = INVALID_STATE;
    desired_state_ = INVALID_STATE;
  }

  bool addState(const std::string& state)
  {
    if (checkStateExists(state) == true)
    {
      ROS_ERROR_STREAM("State " << state << " already exists");
      return false;
    }
    available_states_.push_back(state);
    return true;
  }

  std::string getCurrentState()
  {
    return current_state_;
  }

  std::string getDesiredState()
  {
    return desired_state_;
  }

  bool isInState(const std::string& state)
  {
    return current_state_ == state;
  }

  bool isInDesiredState()
  {
    return current_state_ != INVALID_STATE and current_state_ == desired_state_;
  }

  bool switchToState(const std::string& state)
  {
    if (checkStateExists(state) == false)
    {
      ROS_ERROR_STREAM("State " << state << " is not valid");
      return false;
    }

    if (state == current_state_)
      return true;

    ROS_INFO_STREAM("Switching from " << current_state_ << " to " << state);
    current_state_ = state;

    return true;
  }

  bool switchToDesiredState()
  {
    return switchToState(desired_state_);
  }

  bool checkStateExists(std::string state)
  {
    return state != INVALID_STATE and
           std::find(available_states_.begin(), available_states_.end(), state) != available_states_.end();
  }

  bool setDesiredState(const std::string state, const std::string& message = "")
  {
    if (checkStateExists(state) == false)
    {
      ROS_ERROR_STREAM("State " << state << " is not valid");
      return false;
    }

    desired_state_ = state;
    if (message != "")
      ROS_INFO_STREAM("Setting desired state to " << desired_state_ << " due to " << message);
    else
      ROS_INFO_STREAM("Setting desired state to " << desired_state_);

    return true;
  }
};

#endif  //_IMU_MANAGER_STATE_MACHINE_
