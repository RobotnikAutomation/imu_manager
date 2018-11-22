# imu_manager

Package that manages imu calibration and its state.

Will check current calibration state of the IMU and in case it needs a recalibration, and in that case will run the calibration method. 

To check calibration and run calibration method robot must be stopped. 

Calibration can be triggered by:
- a temperature change,
- a period has passed between calibrations.
- under demand.

In case calibration is triggered, it will only be completed if a drift is  measured in the angular velocity. Drift is measured along a configurable period of time.

## subscribed topics

**imu/data** (sensor_msgs/Imu)

Imu data stream


**imu/temperature** (sensor_msgs/Temperature)

Temperature data stream


## published topics

**~/calibration_state** (std_msgs/String)

Current state of the internal state machine

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


## TODO

- Add interfaces for different IMUS.
