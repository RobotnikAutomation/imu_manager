#include <imu_manager/imu_manager.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_manager");
  ros::NodeHandle n;

  imu_manager::ImuManager manager(n);
  manager.start();
}
