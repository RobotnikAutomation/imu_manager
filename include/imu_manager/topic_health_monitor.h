#ifndef _IMU_MANAGER_TOPIC_HEALTH_MONITOR_
#define _IMU_MANAGER_TOPIC_HEALTH_MONITOR_

#include <ros/ros.h>

//! Class to check the health of a topic

class TopicHealthMonitor
{
private:
  //! timeout to consider that the topic is not receiving
  double topic_timeout_;
  //! saves the ros time of the last msg received
  ros::Time last_msg_time_;
  //! reference to the subscriber
  ros::Subscriber* sub_;
  std::string topic_;

public:
  ~TopicHealthMonitor()
  {
  }

  TopicHealthMonitor(ros::Subscriber* subscriber = 0, double timeout = 5.0)
  {
    sub_ = subscriber;
    if (sub_ != 0)
      topic_ = subscriber->getTopic();
    topic_timeout_ = timeout;
    last_msg_time_ = ros::Time::now();
  }

  //!
  void tick()
  {
    last_msg_time_ = ros::Time::now();
  }
  //! returns true if the topic is receiving data
  bool isReceiving()
  {
    return isReceiving(topic_timeout_);
  }

  bool isReceiving(double timeout)
  {
    //  if (sub_ == 0 or sub_->getNumPublishers() == 0)
    //    return false;

    if ((ros::Time::now() - last_msg_time_).toSec() > timeout)
      return false;

    return true;
  }

  void setSubscriber(ros::Subscriber* subscriber)
  {
    sub_ = subscriber;
  }

  ros::Subscriber* getSubscriber()
  {
    return sub_;
  }

  void setTimeout(double timeout)
  {
    topic_timeout_ = timeout;
  }

  double getTimeout()
  {
    return topic_timeout_;
  }

  std::string getTopic()
  {
    return topic_;
  }

};  // class

#endif  // _IMU_MANAGER_TOPIC_HEALTH_MONITOR_
