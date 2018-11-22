#ifndef _IMU_MANAGER_IMU_MANAGER_
#define _IMU_MANAGER_IMU_MANAGER_

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>

#include <rcomponent/rcomponent.h>

#include <imu_manager/topic_health_monitor.h>
#include <imu_manager/state_machine.h>

#include <boost/circular_buffer.hpp>

#include <std_srvs/Trigger.h>

namespace imu_manager
{
namespace CalibrationState
{
std::string CALIBRATED = "calibrated";
std::string MUST_CHECK = "must_check";
std::string CHECKING = "checking";
std::string MUST_CALIBRATE = "must_calibrate";
std::string CALIBRATING = "calibrating";
std::string NOT_CALIBRATED = "not_calibrated";
std::string UNKNOWN = "unknown";
}

class ImuManager : public rcomponent::RComponent
{
public:
  ImuManager(ros::NodeHandle h = ros::NodeHandle("~"));
  virtual ~ImuManager();

protected:
  virtual void rosReadParams();

  //! Actions performed for all states
  virtual void initState();
  virtual void standbyState();
  virtual void readyState();
  virtual void emergencyState();
  virtual void failureState();
  virtual void allState();

  virtual int rosSetup();

  // true: hw/sw are ok
  // false: hw/sw are not ok
  virtual bool checkHardwareConnection();
  virtual bool checkSoftwareConnection();

  // true: hw/sw have been started properly
  // false: hw/sw have not been started
  virtual bool startHardware();
  virtual bool startSoftware();

  // true: hw/sw have been stoped properly
  // false: hw/sw have not been stoped
  virtual bool stopHardware();
  virtual bool stopSoftware();

  // true: must calibrate
  // false: does not need to calibrate
  virtual bool mustRunCalibration();

  // true: calibration has been started properly
  // false: calibration couldn't be started
  virtual bool runCalibration();

  // true: calibration has been triggered and haven't finished yet
  // false: calibration is not running
  virtual bool isRunningCalibration();

  // real implementation of the methods. in case we have different sensors
  virtual bool checkHardwareConnectionImpl();
  virtual bool checkSoftwareConnectionImpl();

  virtual bool startHardwareImpl();
  virtual bool stopHardwareImpl();

  virtual bool startSoftwareImpl();
  virtual bool stopSoftwareImpl();

  virtual bool isCalibratedImpl();
  virtual bool runCalibrationImpl();

  virtual double getMean();
  virtual double getStdDev();

private:
  ros::NodeHandle gnh_;

  void dataCallback(const sensor_msgs::Imu::ConstPtr& input);
  void temperatureCallback(const sensor_msgs::Temperature::ConstPtr& input);

  std::vector<double> z_angular_velocity_buffer_;
  // boost::circular_buffer<double> z_angular_velocity_buffer_;
  std::vector<sensor_msgs::Imu> data_buffer_;
  double data_mean_;
  double data_std_dev_;

  double current_temperature_;
  double temperature_at_last_calibration_;
  double temperature_variation_for_calibration_;

  double buffer_size_in_sec_;
  double max_allowed_mean_error_;
  double max_allowed_std_deviation_;

  bool sw_initialized_, sw_running_;
  bool hw_initialized_, hw_running_;

  std::vector<ros::Subscriber> data_subscribers_;
  ros::Subscriber data_sub_, temperature_sub_;
  std::vector<TopicHealthMonitor> data_health_monitors_;

  StateMachine calibration_state_;

  std::string data_topic_;
  std::string temperature_topic_;

  ros::Time time_of_last_calibration_;
  ros::Time start_of_calibration_;
  ros::Duration period_between_checkings_;
  // MAVROS
  ros::Duration period_of_calibration_;
  ros::ServiceClient calibrate_gyros_;
};
}  // namespace
#endif  // _IMU_MANAGER_IMU_MANAGER_
