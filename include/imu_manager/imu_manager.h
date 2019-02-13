#ifndef _IMU_MANAGER_IMU_MANAGER_
#define _IMU_MANAGER_IMU_MANAGER_

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <nav_msgs/Odometry.h>

#include <rcomponent/rcomponent.h>

#include <imu_manager/topic_health_monitor.h>
#include <imu_manager/state_machine.h>

#include <boost/circular_buffer.hpp>

#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <robotnik_msgs/enable_disable.h>

namespace imu_manager
{

#define WAITING_TIME_BEFORE_RECOVERY 5.0
#define DEFAULT_ODOM_LINEAR_HYSTERESIS 0.001 // m/s
#define DEFAULT_ODOM_ANGULAR_HYSTERESIS 0.001 // rads/s
	
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
  virtual void rosPublish();

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

  // true: can run calibration process
  // false: cannot run calibration process
  virtual bool canRunCalibration();

  // true: calibration has been started properly
  // false: calibration couldn't be started
  virtual bool runCalibration();

  // true: calibration has been triggered and haven't finished yet
  // false: calibration is not running
  virtual bool isRunningCalibration();

  // true: calibration can be checked, maybe automatic or triggered
  // false: calibration cannot be checked
  virtual bool canCheckCalibration();

  // true: system has enough data (depending on time of data gathering, number of data gathered) to check calibration
  // false: system need to gather more data
  virtual bool hasEnoughDataToCalibrate();

  // input
  // toggle: true, robot will be able to move
  // toggle: false, robot will not be able to move
  // output
  // true, robot operation has been set
  // false, robot operation couldn't be set
  virtual bool toggleRobotOperation(bool toggle);

  // true: calibration can be checked, maybe automatic or triggered
  // false: calibration cannot be checked

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
  
  virtual void switchToState(int new_state);
  
  int getElapsedTimeSinceLastStateTransition();
  
  virtual bool isRobotMoving();
  virtual bool isOdomBeingReceived();

private:
  ros::NodeHandle gnh_;

  void dataCallback(const sensor_msgs::Imu::ConstPtr& input);
  void temperatureCallback(const sensor_msgs::Temperature::ConstPtr& input);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  bool triggerCalibrationCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

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
  
  double odom_linear_hysteresis_;
  double odom_angular_hysteresis_;

  bool sw_initialized_, sw_running_;
  bool hw_initialized_, hw_running_;

  std::vector<ros::Subscriber> data_subscribers_;
  ros::Subscriber data_sub_, temperature_sub_, odom_sub_;
  std::vector<TopicHealthMonitor> data_health_monitors_;

  ros::Publisher internal_state_pub_;

  StateMachine calibration_state_;

  std::string data_topic_;
  std::string temperature_topic_;
  std::string odom_topic_;

  ros::Time time_of_last_calibration_;
  ros::Time start_of_calibration_;
  ros::Time time_of_last_state_transition_;
  ros::Time time_for_next_check_;
  ros::Duration period_between_checkings_;
  ros::Duration period_of_data_gathering_;

  bool calibration_only_under_demand_;
  bool calibration_demanded_;
  bool calibration_by_temperature_; // flag to enable calibration by temperature
  bool calibration_by_angular_velocity_deviation_; // flag to enable calibration by deviation in angular velocity
  bool calibration_odom_constraint_; // flag to enable the calibration by checking the odometry of the robot
  
  ros::ServiceServer calibrate_server_;

  nav_msgs::Odometry robot_odom_;
  // ROBOT
  ros::ServiceClient robot_toggle_;

  // MAVROS
  ros::Duration duration_of_calibration_;
  ros::ServiceClient calibrate_gyros_;
};
}  // namespace
#endif  // _IMU_MANAGER_IMU_MANAGER_
