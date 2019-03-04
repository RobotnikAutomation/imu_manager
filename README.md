# imu_manager

Package that manages imu calibration and its state.

Will check current calibration state of the IMU and in case it needs a recalibration, and in that case will run the calibration method.

To check calibration and run calibration method robot must be stopped. In order to check that the robot is not moving, it can be set to read the odometry from it.

Calibration can be triggered by:
- a period has passed between calibrations.
- under demand.

In case calibration is triggered, different behaviours can be configured:

* it will only be completed if a drift is  measured in the angular velocity. Drift is measured along a configurable period of time.
* the temperature changes

## subscribed topics

**imu/data** (sensor_msgs/Imu)

Imu data stream


**imu/temperature** (sensor_msgs/Temperature)

Temperature data stream

**robotnik_base_control/odom** (nav_msgs/Odometry)

Robot odometry (optional)

**mavros/data** (mavros_msgs/State)

Mavros State

## published topics

**~/calibration_state** (std_msgs/String)

Current state of calibration (*calibrated, must_check, checking, must_calibrate, calibrating, not_calibrated, unknown*)

**~/state** (robotnik_msgs/State)

Current state of the internal state machine

**~/status** (imu_manager/ImuManagerStatus)

Current status of the calibration along with robot & sensor information.

Example:

```
header:
  seq: 10488
  stamp:
    secs: 666
    nsecs: 590000000
  frame_id: ''
calibration_status: "calibrated"
next_check_countdown: 49
calibration_duration: 0
imu_temperature: 38.719997406
calibrated_imu_temperature: 37.6599998474
robot_moving: False

```

## required services
**robotnik_base_control/enable** (robotnik_msgs::enable_disable)

To toggle robot operation before and after calibration

**calibrate_imu_gyro** (std_srvs::Trigger)

To call gyroscopes calibration using Robotnik Localization Utils

## advertised services

**~/trigger_calibration** (std_srvs/Trigger)

Triggers calibration under user demand

## parameters

**data_topic** (string, default: "imu/data")

To change input topic to imu/data

**temperature_topic** (string, default: "imu/temperature")

To change input topic to imu/temperature

**max_mean_error** (double, default: 0)

Maximum error from 0 in measured yaw velocity to accept current calibration. 0 means always calibrate.

**max_std_dev** (double, default: 0)

Maximum standard deviation in measured yaw velocity to accept current calibration. 0 means always calibrate.

**temperature_variation_for_calibration** (double, default: 1)

Maximum temperature change between calibrations.

**calibration_only_under_demand** (bool, default: false)

Do not trigger automatic calibration.

**period_between_checkings** (double, default: 25)

Period between calibration checkings, in case automatic calibration is enabled.

**period_of_data_gathering** (double, default: 5)

Period while the robot will gather data to check current calibration.

**duration_of_calibration** (double, default: 40)

Duration of calibration process. In case calibration method is not blocking (as in MAVROS/Pixhawk).

**calibration_by_temperature** (bool, default: true)

It allows the calibration by temperature variation.

**calibration_by_angular_velocity_deviation** (bool, default: false)

It allows the calibration if a drift is detected.

**calibration_odom_constraint** (bool, default: true)

It adds a constraint in the calibration process, taking into consideration the current speed of the robot. In case is moving, any calibration process is cancelled.

**odom_topic** (string, default: robotnik_base_control/odom)

Topic to get the robot odometry

**odom_linear_hysteresis** (double, default: 0.001)

Hysteresis for the linear velocity

**odom_angular_hysteresis** (double, default: 0.001)

Hysteresis for the angular velocity

**period_of_robot_without_moving** (double, default: 5.0)

Period to consider that the robot is not moving after it has really stopped

## TODO

- Add interfaces for different IMUS.
- Move custom msgs into imu_manager_msgs?
