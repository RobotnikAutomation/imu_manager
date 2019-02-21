#include <imu_manager/imu_manager.h>

#include <imu_manager/data_utils.h>

namespace imu_manager
{
ImuManager::ImuManager(ros::NodeHandle h) : RComponent(h)
{
  component_name = pnh_.getNamespace();

  rosReadParams();
  sw_initialized_ = sw_running_ = false;
  hw_initialized_ = hw_running_ = false;
  calibration_demanded_ = false;

  calibration_state_.addState(CalibrationState::CALIBRATED);
  calibration_state_.addState(CalibrationState::MUST_CHECK);
  calibration_state_.addState(CalibrationState::CHECKING);
  calibration_state_.addState(CalibrationState::MUST_CALIBRATE);
  calibration_state_.addState(CalibrationState::CALIBRATING);
  calibration_state_.addState(CalibrationState::NOT_CALIBRATED);
  calibration_state_.addState(CalibrationState::UNKNOWN);

  calibration_state_.setDesiredState(CalibrationState::UNKNOWN);

  time_of_last_calibration_ = ros::Time(0);
  time_of_last_state_transition_ = ros::Time(0);
  temperature_at_last_calibration_ = 0.0;
  time_for_next_check_ = ros::Time::now();
}

ImuManager::~ImuManager()
{
}

void ImuManager::rosReadParams()
{
  RComponent::rosReadParams();

  bool required = true;
  max_allowed_mean_error_ = 0;
  readParam(pnh_, "max_mean_error", max_allowed_mean_error_, max_allowed_mean_error_, required);

  max_allowed_std_deviation_ = 0;
  readParam(pnh_, "max_std_dev", max_allowed_std_deviation_, max_allowed_std_deviation_, required);

  data_topic_ = "imu/data";
  readParam(pnh_, "data_topic", data_topic_, data_topic_, required);

  temperature_topic_ = "imu/temperature";
  readParam(pnh_, "temperature_topic", temperature_topic_, temperature_topic_, required);

  calibration_only_under_demand_ = false;
  readParam(pnh_, "calibration_only_under_demand", calibration_only_under_demand_, calibration_only_under_demand_,
            required);

  temperature_variation_for_calibration_ = 1;
  readParam(pnh_, "temperature_variation_for_calibration", temperature_variation_for_calibration_,
            temperature_variation_for_calibration_, required);

  double period;
  period = 10;
  readParam(pnh_, "period_between_checkings", period, period, required);
  period_between_checkings_ = ros::Duration(period);

  period = 5;
  readParam(pnh_, "period_of_data_gathering", period, period, required);
  period_of_data_gathering_ = ros::Duration(period);

  double duration;
  duration = 40;  // this is for mavros
  readParam(pnh_, "duration_of_calibration", duration, duration, required);
  duration_of_calibration_ = ros::Duration(duration);

  calibration_by_temperature_ = true;
  readParam(pnh_, "calibration_by_temperature", calibration_by_temperature_, calibration_by_temperature_, required);

  calibration_by_angular_velocity_deviation_ = false;
  readParam(pnh_, "calibration_by_angular_velocity_deviation", calibration_by_angular_velocity_deviation_,
            calibration_by_angular_velocity_deviation_, required);

  calibration_odom_constraint_ = true;
  readParam(pnh_, "calibration_odom_constraint", calibration_odom_constraint_, calibration_odom_constraint_, required);

  odom_topic_ = "robotnik_base_control/odom";
  readParam(pnh_, "odom_topic", odom_topic_, odom_topic_, required);

  readParam(pnh_, "odom_linear_hysteresis", odom_linear_hysteresis_, DEFAULT_ODOM_LINEAR_HYSTERESIS, false);
  readParam(pnh_, "odom_angular_hysteresis", odom_angular_hysteresis_, DEFAULT_ODOM_ANGULAR_HYSTERESIS, false);

  // int buffer_size_ = 100;
  // z_angular_velocity_buffer_ = boost::circular_buffer<double>(buffer_size_);
  // z_angular_velocity_buffer_
}

void ImuManager::rosPublish()
{
  std_msgs::String msg;
  msg.data = calibration_state_.getCurrentState();

  internal_state_pub_.publish(msg);

  ros::Time t_now = ros::Time::now();
  status_msg_.header.stamp = t_now;
  status_msg_.calibration_status = calibration_state_.getCurrentState();
  if (calibration_only_under_demand_ == true)
  {
    status_msg_.next_check_countdown = -1;
  }
  else
  {
    if (status_msg_.calibration_status != CalibrationState::CALIBRATING and
        status_msg_.calibration_status != CalibrationState::UNKNOWN)
    {
      status_msg_.next_check_countdown = int((time_for_next_check_ - ros::Time::now()).toSec());
    }
    else
    {
      status_msg_.next_check_countdown = -1;
    }
  }
  if (status_msg_.calibration_status == CalibrationState::CALIBRATING)
  {
    status_msg_.calibration_duration = int((ros::Time::now() - start_of_calibration_).toSec());
  }
  else
  {
    status_msg_.calibration_duration = 0;
  }
  status_msg_.imu_temperature = current_temperature_;
  status_msg_.calibrated_imu_temperature = temperature_at_last_calibration_;
  status_msg_.robot_moving = (calibration_odom_constraint_ == true and isRobotMoving() == true);

  internal_status_pub_.publish(status_msg_);

  RComponent::rosPublish();
}

int ImuManager::rosSetup()
{
  // Checks if has been initialized
  if (ros_initialized)
  {
    RCOMPONENT_INFO("Already initialized");

    return rcomponent::INITIALIZED;
  }

  calibrate_gyros_ = nh_.serviceClient<std_srvs::Trigger>("calibrate_imu_gyro");
  robot_toggle_ = nh_.serviceClient<robotnik_msgs::enable_disable>("robotnik_base_control/enable");

  calibrate_server_ = pnh_.advertiseService("trigger_calibration", &ImuManager::triggerCalibrationCallback, this);

  internal_state_pub_ = pnh_.advertise<std_msgs::String>("calibration_state", 1);
  internal_status_pub_ = pnh_.advertise<imu_manager::ImuManagerStatus>("status", 1);

  return RComponent::rosSetup();
}

void ImuManager::initState()
{
  if (startHardware() == false)
  {
    RCOMPONENT_ERROR_STREAM("Couldn't start hardware for sensor");
    switchToState(robotnik_msgs::State::FAILURE_STATE);
    return;
  }

  switchToState(robotnik_msgs::State::STANDBY_STATE);
}

void ImuManager::standbyState()
{
  if (startSoftware() == false)
  {
    RCOMPONENT_ERROR_STREAM("Couldn't start software for sensor");
    return;
  }

  switchToState(robotnik_msgs::State::READY_STATE);
}

void ImuManager::readyState()
{
  if (checkHardwareConnection() == false)
  {
    RCOMPONENT_ERROR_STREAM("Hardware for sensor is not available");
    switchToState(robotnik_msgs::State::FAILURE_STATE);
    return;
  }

  if (checkSoftwareConnection() == false)
  {
    RCOMPONENT_ERROR_STREAM("Software for sensor is not available");
    switchToState(robotnik_msgs::State::EMERGENCY_STATE);
    return;
  }

  //
  // UNKNOWN, NOT_CALIBRTED, CALIBRATED
  if (calibration_state_.getCurrentState() == CalibrationState::UNKNOWN or
      calibration_state_.getCurrentState() == CalibrationState::NOT_CALIBRATED or
      calibration_state_.getCurrentState() == CalibrationState::CALIBRATED)
  {
    calibratedSubState();
  }

  //
  // MUST CHECK
  if (calibration_state_.getCurrentState() == CalibrationState::MUST_CHECK)
  {
    mustCheckSubState();
  }

  //
  // CHECKING
  if (calibration_state_.getCurrentState() == CalibrationState::CHECKING)
  {
    checkingSubState();
  }

  //
  // MUST CALIBRATE
  if (calibration_state_.getCurrentState() == CalibrationState::MUST_CALIBRATE)
  {
    mustCalibrateSubState();
  }

  //
  // CALIBRATING
  if (calibration_state_.getCurrentState() == CalibrationState::CALIBRATING)
  {
    calibratingSubState();
  }
}

void ImuManager::emergencyState()
{
  if (getElapsedTimeSinceLastStateTransition() < WAITING_TIME_BEFORE_RECOVERY)
    return;

  stopSoftware();

  if (checkHardwareConnection() == false)
  {
    RCOMPONENT_ERROR_STREAM("Hardware for sensor is not available");
    switchToState(robotnik_msgs::State::FAILURE_STATE);
    return;
  }
  else
  {
    switchToState(robotnik_msgs::State::INIT_STATE);
  }
}

void ImuManager::failureState()
{
  if (getElapsedTimeSinceLastStateTransition() < WAITING_TIME_BEFORE_RECOVERY)
    return;

  if (hw_running_ == true)
  {
    stopHardware();
  }

  if (hw_running_ == false)
  {
    startHardware();
  }

  if (checkHardwareConnection() == true)
  {
    switchToState(robotnik_msgs::State::INIT_STATE);
  }
}

void ImuManager::allState()
{
  RComponent::allState();
  calibration_state_.switchToDesiredState();

  // Periodic drift calculation
}

bool ImuManager::canRunCalibration()
{
  // could be autonomous or triggered
  if (true == calibration_only_under_demand_ and false == calibration_demanded_)
    return false;

  return true;
}

bool ImuManager::canCheckCalibration()
{
  // could be autonomous or triggered
  if (true == calibration_only_under_demand_ and false == calibration_demanded_)
  {
    RCOMPONENT_INFO_THROTTLE(5, "calibration_only_under_demand = %d, calibration_demanded = %d",
                             calibration_only_under_demand_, calibration_demanded_);
    return false;
  }

  return true;
}

bool ImuManager::toggleRobotOperation(bool toggle)
{
  // should contact service enable/disable in robot
  robotnik_msgs::enable_disable rt;
  rt.request.value = toggle;

  bool result = robot_toggle_.call(rt);

  if (result == false)
  {
    RCOMPONENT_ERROR_STREAM("Couldn't contact service: " << robot_toggle_.getService());
    return false;
  }

  if (rt.response.ret == false)
  {
    RCOMPONENT_ERROR_STREAM("Robot could not be disabled due to an unkown reason");
    return false;
  }

  return true;
}

bool ImuManager::hasEnoughDataToCalibrate()
{
  if (data_buffer_.size() == 0)
    return false;

  if ((data_buffer_.back().header.stamp - data_buffer_.front().header.stamp) < period_of_data_gathering_)
    return false;
  RCOMPONENT_INFO("Using a buffer of %d elements gathered during %.3lf seconds", (int)data_buffer_.size(),
                  (data_buffer_.back().header.stamp - data_buffer_.front().header.stamp).toSec());
  return true;
}

bool ImuManager::checkHardwareConnection()
{
  return checkHardwareConnectionImpl();
}
bool ImuManager::checkSoftwareConnection()
{
  // return true;
  return checkSoftwareConnectionImpl();
}

// Hardware methods
bool ImuManager::startHardware()
{
  if (hw_running_ == true)
  {
    RCOMPONENT_WARN("Hardware already started!");
    return true;
  }

  bool result = startHardwareImpl();

  if (result == true)
  {
    hw_running_ = true;
    RCOMPONENT_INFO("Hardware started!");
  }
  else
  {
    RCOMPONENT_ERROR("Error while starting hardware!");
  }

  return result;
}
bool ImuManager::stopHardware()
{
  if (sw_running_ == false)
  {
    RCOMPONENT_WARN("Software still running started, impossible to stop hardware!");
    return false;
  }

  if (hw_running_ == false)
  {
    RCOMPONENT_WARN_THROTTLE(10, "Hardware not started, impossible to stop!");
    return true;
  }

  bool result = stopHardwareImpl();

  if (result == true)
  {
    hw_running_ = false;
    RCOMPONENT_INFO("Hardware stopped!");
  }
  else
  {
    RCOMPONENT_ERROR("Error while stopping hardware!");
  }

  return result;
}

// Software methods
bool ImuManager::startSoftware()
{
  if (hw_running_ == false)
  {
    RCOMPONENT_WARN("Hardware not started, impossible to start software!");
    return false;
  }

  if (sw_running_ == true)
  {
    RCOMPONENT_WARN("Software already started!");
    return true;
  }

  bool result = startSoftwareImpl();

  if (result == true)
  {
    sw_running_ = true;
    RCOMPONENT_INFO("Software started!");
  }
  else
  {
    RCOMPONENT_ERROR("Error while starting software!");
  }

  return result;
}

bool ImuManager::stopSoftware()
{
  if (sw_running_ == false)
  {
    RCOMPONENT_WARN("Software not started, impossible to stop!");
    return true;
  }

  bool result = stopSoftwareImpl();

  if (result == true)
  {
    sw_running_ = false;
    RCOMPONENT_INFO("Software stopped!");
  }
  else
  {
    RCOMPONENT_ERROR("Error while stopping software!");
  }

  return result;
}

bool ImuManager::mustRunCalibration()
{
  if (sw_running_ == false)
  {
    RCOMPONENT_WARN("Software not started, I cannot check for calibration!");
    return false;
  }

  bool is_calibrated = isCalibratedImpl();

  if (true == is_calibrated)
  {
    RCOMPONENT_INFO("Sensor is calibrated");
    return false;
  }
  else
  {
    RCOMPONENT_WARN("Sensor is NOT calibrated. Should be calibrated");
    return true;
  }
}

bool ImuManager::runCalibration()
{
  bool result = runCalibrationImpl();
  if (result == false)
    return false;

  start_of_calibration_ = ros::Time::now();
  return true;
}

bool ImuManager::isRunningCalibration()
{
  // this implementation works for mavros

  return (ros::Time::now() - start_of_calibration_) < duration_of_calibration_;
}

// Implementation methods: to be implemented when different sensor is used

bool ImuManager::checkHardwareConnectionImpl()
{
  if (hw_running_ == false)
    return false;

  return true;
}

bool ImuManager::checkSoftwareConnectionImpl()
{
  if (sw_running_ == false)
    return false;

  // if no topic is set, we assume that sw is working properly
  if (data_health_monitors_.size() == 0)
    return true;

  // XXX: it seems there is some race condition here. first time is executed, it
  // fails, but then, works nice and smooth
  bool fail = false;
  for (auto& dhm : data_health_monitors_)
  {
    if (dhm.isReceiving() == false)
    {
      fail = true;
      // RCOMPONENT_ERROR_STREAM("Topic " << dhm.getSubscriber()->getTopic() <<
      // " is not being received");
      RCOMPONENT_ERROR_STREAM_THROTTLE(5, "Topic " << dhm.getTopic() << " is not being received");
    }
  }

  // Checking service clients
  if (calibrate_gyros_.exists() == false)
  {
    fail = true;
    RCOMPONENT_ERROR_STREAM_THROTTLE(5, "Service " << calibrate_gyros_.getService() << " does not exists");
  }
  if (robot_toggle_.exists() == false)
  {
    fail = true;
    RCOMPONENT_ERROR_STREAM_THROTTLE(5, "Service " << robot_toggle_.getService() << " does not exists");
  }

  return (not fail);
}

bool ImuManager::startHardwareImpl()
{
  return true;
}
bool ImuManager::stopHardwareImpl()
{
  return true;
}

bool ImuManager::startSoftwareImpl()
{
  ros::Subscriber sub;

  // wait for message
  boost::shared_ptr<const sensor_msgs::Imu> data_received =
      ros::topic::waitForMessage<sensor_msgs::Imu>(data_topic_, gnh_, ros::Duration(1));

  if (data_received == 0)
  {
    RCOMPONENT_ERROR_STREAM("There are no publishers for " << data_topic_ << ", software cannot be started");
    return false;
  }

  // subscribe to imu/data
  data_sub_ = gnh_.subscribe(data_topic_, 1, &ImuManager::dataCallback, this);
  data_subscribers_.push_back(data_sub_);

  data_health_monitors_.push_back(TopicHealthMonitor(&data_sub_));

  // wait for message
  boost::shared_ptr<const sensor_msgs::Temperature> temperature_received =
      ros::topic::waitForMessage<sensor_msgs::Temperature>(temperature_topic_, gnh_, ros::Duration(1));

  if (temperature_received == 0)
  {
    RCOMPONENT_ERROR_STREAM("There are no publishers for " << temperature_topic_ << ", software cannot be started");
    return false;
  }

  // subscribe to imu/temperature
  temperature_sub_ = gnh_.subscribe(temperature_topic_, 1, &ImuManager::temperatureCallback, this);
  data_subscribers_.push_back(temperature_sub_);
  data_health_monitors_.push_back(TopicHealthMonitor(&temperature_sub_));

  if (calibration_odom_constraint_ == true)
  {
    boost::shared_ptr<const nav_msgs::Odometry> odom_received =
        ros::topic::waitForMessage<nav_msgs::Odometry>(odom_topic_, gnh_, ros::Duration(1));
    if (odom_received == 0)
    {
      RCOMPONENT_ERROR_STREAM("There are no publishers for " << odom_topic_ << ", software cannot be started");
      return false;
    }
    // subscribe to odom
    odom_sub_ = gnh_.subscribe(odom_topic_, 1, &ImuManager::odomCallback, this);
    data_subscribers_.push_back(odom_sub_);
    data_health_monitors_.push_back(TopicHealthMonitor(&odom_sub_));
  }

  return true;
}

bool ImuManager::stopSoftwareImpl()
{
  data_health_monitors_.clear();
  data_subscribers_.clear();

  return true;
}

bool ImuManager::triggerCalibrationCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  if (calibration_state_.getCurrentState() == CalibrationState::UNKNOWN or
      calibration_state_.getCurrentState() == CalibrationState::NOT_CALIBRATED or
      calibration_state_.getCurrentState() == CalibrationState::CALIBRATED)
  {
    calibration_demanded_ = true;
    response.success = true;
    response.message = "Calibration triggered";
  }
  else
  {
    response.success = true;
    response.message = "Calibration was running, so this call had no effect";
  }

  return true;
}

void ImuManager::dataCallback(const sensor_msgs::Imu::ConstPtr& input)
{
  if (data_health_monitors_.size() > 0)
    data_health_monitors_[0].tick();

  if (data_buffer_.size() > DEFAULT_IMU_BUFFER_SIZE)
  {
    clearBuffers();
    RCOMPONENT_WARN("Clearing buffers due to max size reached (%d)", DEFAULT_IMU_BUFFER_SIZE);
  }
  data_buffer_.push_back(*input);
  z_angular_velocity_buffer_.push_back(input->angular_velocity.z);

  // RCOMPONENT_INFO_THROTTLE(5, "buffer size = %d", (int)data_buffer_.size());
}

void ImuManager::temperatureCallback(const sensor_msgs::Temperature::ConstPtr& input)
{
  if (data_health_monitors_.size() > 1)
  {
    data_health_monitors_[1].tick();
  }
  current_temperature_ = input->temperature;
}

void ImuManager::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  if (data_health_monitors_.size() > 1)
  {
    data_health_monitors_[2].tick();
  }
  robot_odom_ = *odom;
}

double ImuManager::getMean()
{
  return data_mean_;
}

double ImuManager::getStdDev()
{
  return data_std_dev_;
}

bool ImuManager::isCalibratedImpl()
{
  bool calibrated = true;
  if (std::abs(data_mean_) > max_allowed_mean_error_)
  {
    RCOMPONENT_WARN_STREAM_THROTTLE(1, "Imu z angular velocity mean(" << data_mean_
                                                                      << ") is bigger in abs than maximum allowed ("
                                                                      << max_allowed_mean_error_ << ")");
    calibrated = false;
  }
  else
  {
    RCOMPONENT_INFO_STREAM_THROTTLE(1, "Imu z angular velocity mean (" << data_mean_
                                                                       << ") is lower in abs than maximum allowed ("
                                                                       << max_allowed_mean_error_ << ")");
  }
  if (std::abs(data_std_dev_) > max_allowed_std_deviation_)
  {
    RCOMPONENT_WARN_STREAM_THROTTLE(1, "Imu z angular velocity std dev (" << data_std_dev_
                                                                          << ") is bigger in abs than maximum allowed ("
                                                                          << max_allowed_std_deviation_ << ")");
    calibrated = false;
  }
  else
  {
    RCOMPONENT_INFO_STREAM_THROTTLE(1, "Imu z angular velocity std dev (" << data_std_dev_
                                                                          << ") is lower in abs than maximum allowed ("
                                                                          << max_allowed_std_deviation_ << ")");
  }

  return calibrated;
}

bool ImuManager::runCalibrationImpl()
{
  RCOMPONENT_WARN_STREAM("Triggering calibration");
  std_srvs::Trigger calibrate;

  bool result = calibrate_gyros_.call(calibrate);
  if (result == false)
  {
    RCOMPONENT_ERROR_STREAM("Couldn't contact service: " << calibrate_gyros_.getService());
    return false;
  }

  if (calibrate.response.success == false)
  {
    RCOMPONENT_ERROR_STREAM("Calibration process failed due to: " << calibrate.response.message);
    return false;
  }

  return true;
}

/*!	\fn void ImuManager::switchToState(int new_state)
 * 	function that switches the state of the component into the desired state
 * 	\param new_state as an integer, the new state of the component
*/
void ImuManager::switchToState(int new_state)
{
  if (new_state == state)
    return;

  // saves the previous state
  previous_state = state;
  time_of_last_state_transition_ = ros::Time::now();

  RCOMPONENT_INFO("%s -> %s", getStateString(state), getStateString(new_state));
  state = new_state;
}

/*!	\fn int ImuManager::getElapsedTimeSinceLastStateTransition()
 * 	Returns elapsed time since last state transition
*/
int ImuManager::getElapsedTimeSinceLastStateTransition()
{
  return (ros::Time::now() - time_of_last_state_transition_).toSec();
}

/*!	\fn bool ImuManager::isRobotMoving()
 * 	Returns true if robot is moving, false otherwise
*/
bool ImuManager::isRobotMoving()
{
  if (fabs(robot_odom_.twist.twist.linear.x) > odom_linear_hysteresis_ or
      fabs(robot_odom_.twist.twist.linear.y) > odom_linear_hysteresis_ or
      fabs(robot_odom_.twist.twist.angular.z) > odom_angular_hysteresis_)
  {
    return true;
  }
  return false;
}

/*!	\fn bool ImuManager::isOdomBeingReceived()
 * 	Returns true if odom is being received
*/
bool ImuManager::isOdomBeingReceived()
{
  return data_health_monitors_[2].isReceiving();
}

void ImuManager::calculateDriftValues()
{
  data_mean_ = calculateMean(z_angular_velocity_buffer_);
  data_std_dev_ = calculateStdDev(z_angular_velocity_buffer_);
}

void ImuManager::clearBuffers()
{
  data_buffer_.clear();
  z_angular_velocity_buffer_.clear();
}

void ImuManager::calibratedSubState()
{
  if (calibration_only_under_demand_ == false)
  {
    // RCOMPONENT_INFO_STREAM_THROTTLE(5, "Next calibration check in " <<
    // (time_for_next_check_ - ros::Time::now()).toSec() <<" seconds");

    if ((ros::Time::now() - time_for_next_check_) > ros::Duration(0))
    {
      calibration_state_.setDesiredState(CalibrationState::MUST_CHECK, "period between calibrations has been "
                                                                       "exceeded");
      return;
    }
  }
  else if (true == calibration_demanded_)
  {
    calibration_demanded_ = false;
    calibration_state_.setDesiredState(CalibrationState::MUST_CHECK, "calibration has been demanded");
  }
  else
  {
    RCOMPONENT_WARN_THROTTLE(10, "Waiting for calibration trigger");
  }
}

void ImuManager::mustCheckSubState()
{
  // bool can_check = canCheckCalibration();
  // bool can_check = true;
  if (true == calibration_by_temperature_ and
      std::abs(current_temperature_ - temperature_at_last_calibration_) > temperature_variation_for_calibration_)
  {
    RCOMPONENT_INFO_STREAM("Must check calibration due to a change in the IMU temperature. "
                           "Current temperature: "
                           << current_temperature_
                           << ", temperature at last calibration: " << temperature_at_last_calibration_
                           << ", variation allowed: " << temperature_variation_for_calibration_);

    calibration_state_.setDesiredState(CalibrationState::MUST_CALIBRATE, "Calibration due to temperature variation is "
                                                                         "required");
    return;
  }

  if (true == calibration_by_angular_velocity_deviation_)
  {
    // disable robot
    if (true == toggleRobotOperation(false))
    {
      if (calibration_odom_constraint_ == true)
      {
        if (isRobotMoving() == false)
        {
          // clear data
          clearBuffers();
          calibration_state_.setDesiredState(CalibrationState::CHECKING, "Calibration checking is enabled");
        }
        else
        {
          RCOMPONENT_WARN_STREAM_THROTTLE(5, "Robot is moving during " << CalibrationState::MUST_CHECK << ". Waiting");
        }
      }
      else
      {
        // clear data
        clearBuffers();
        calibration_state_.setDesiredState(CalibrationState::CHECKING, "Calibration checking is enabled");
      }
    }
    else
    {
      RCOMPONENT_WARN_THROTTLE(5, "Error disabling robot. Required to gather data");
    }
    return;
  }

  RCOMPONENT_WARN_THROTTLE(10, "No calibration is needed for now");
  time_for_next_check_ = ros::Time::now() + period_between_checkings_;
  calibration_state_.setDesiredState(calibration_state_.getPreviousState(), "Check for calibration failed");
  return;
}

void ImuManager::checkingSubState()
{
  if (true == calibration_by_angular_velocity_deviation_)
  {
    if (calibration_odom_constraint_ == true and isRobotMoving() == true)
    {
      RCOMPONENT_WARN_STREAM_THROTTLE(5, "Robot is moving during " << CalibrationState::CHECKING << ". Cancelling "
                                                                                                    "calibration");
      time_for_next_check_ = ros::Time::now() + period_between_checkings_;
      calibration_state_.setDesiredState(CalibrationState::NOT_CALIBRATED, "Calibration cancelled due to robot "
                                                                           "movement during data gathering");
      return;
    }

    if (hasEnoughDataToCalibrate() == false)
    {
      RCOMPONENT_INFO_STREAM_THROTTLE(1, "Not enough data gathered");
      return;
    }
    else
    {
      calculateDriftValues();
    }
  }
  else
  {
    time_for_next_check_ = ros::Time::now() + period_between_checkings_;
    calibration_state_.setDesiredState(CalibrationState::NOT_CALIBRATED, "Calibration by angular velocity is disabled");
    return;
  }

  bool must_calibrate = mustRunCalibration();

  if (true == must_calibrate)
  {
    // should enable robot now??
    // toggleRobotOperation(true);

    calibration_state_.setDesiredState(CalibrationState::MUST_CALIBRATE, "Imu is not calibrated");
    return;
  }
  else if (false == must_calibrate)
  {
    // set last calibration stamps
    time_of_last_calibration_ = ros::Time::now();
    temperature_at_last_calibration_ = current_temperature_;

    // enable robot
    if (false == toggleRobotOperation(true))
    {
      RCOMPONENT_ERROR("Error enabling the robot operation!!");
    }
    time_for_next_check_ = ros::Time::now() + period_between_checkings_;
    calibration_state_.setDesiredState(CalibrationState::CALIBRATED, "Imu is calibrated");
    return;
  }
  return;
}

void ImuManager::mustCalibrateSubState()
{
  /*bool can_run_calibration = canRunCalibration();

if (false == can_run_calibration)
{
RCOMPONENT_WARN_THROTTLE(10, "I need to run calibration, but I am not able to do
it");
return;
}*/

  if (false == toggleRobotOperation(false))
  {
    RCOMPONENT_ERROR("Error disabling the robot operation!!");
    time_for_next_check_ = ros::Time::now() + period_between_checkings_;
    calibration_state_.setDesiredState(CalibrationState::NOT_CALIBRATED, "Robot movement couldn't be disabled");
    return;
  }

  if (calibration_odom_constraint_ == true and isRobotMoving() == true)
  {
    RCOMPONENT_WARN_STREAM_THROTTLE(5, "Robot is moving during " << CalibrationState::MUST_CALIBRATE << ". Waiting");
    return;
  }

  bool started_calibration = runCalibration();
  if (true == started_calibration)
  {
    calibration_state_.setDesiredState(CalibrationState::CALIBRATING, "Imu is not calibrated");
    return;
  }
  if (false == started_calibration)
  {
    time_for_next_check_ = ros::Time::now() + period_between_checkings_;
    calibration_state_.setDesiredState(CalibrationState::NOT_CALIBRATED, "Calibration process could not start");
    if (false == toggleRobotOperation(true))
    {
      RCOMPONENT_ERROR("Error enabling the robot operation!!");
    }
    switchToState(robotnik_msgs::State::FAILURE_STATE);
    return;
  }
  return;
}

void ImuManager::calibratingSubState()
{
  bool running_calibration = isRunningCalibration();
  if (true == running_calibration)
  {
    if (calibration_odom_constraint_ == true and isRobotMoving() == true)
    {
      RCOMPONENT_WARN_STREAM_THROTTLE(5, "Robot is moving during " << CalibrationState::CALIBRATING << ". Cancelling");
      if (false == toggleRobotOperation(true))
      {
        RCOMPONENT_ERROR("Error enabling the robot operation!!");
      }
      calibration_state_.setDesiredState(CalibrationState::NOT_CALIBRATED, "Calibration process failed due to robot "
                                                                           "movement");
      time_for_next_check_ = ros::Time::now() + period_between_checkings_;
      return;
    }
    // RCOMPONENT_INFO_STREAM_THROTTLE(5, "Running calibration for " <<
    // (ros::Time::now() - start_of_calibration_).toSec() <<" seconds");
    return;
  }
  if (false == running_calibration)
  {
    if (false == toggleRobotOperation(true))
    {
      RCOMPONENT_ERROR("Error enabling the robot operation!!");
    }
    if (true == calibration_by_angular_velocity_deviation_)
    {
      calibration_state_.setDesiredState(CalibrationState::MUST_CHECK, "After finishing calibration");
      return;
    }
    if (true == calibration_by_temperature_)
    {
      time_of_last_calibration_ = ros::Time::now();
      temperature_at_last_calibration_ = current_temperature_;
    }

    time_for_next_check_ = ros::Time::now() + period_between_checkings_;
    calibration_state_.setDesiredState(CalibrationState::CALIBRATED, "After finishing calibration");
    return;
  }
  return;
}

}  // namespace
