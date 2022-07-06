#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <lio_sam/lio_samConfig.h>

void callback(lio_sam::lio_samConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f", config.edgeThreshold,
           config.surfThreshold);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_lio");

  dynamic_reconfigure::Server<lio_sam::lio_samConfig> server;
  dynamic_reconfigure::Server<lio_sam::lio_samConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}