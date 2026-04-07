#include <ros/ros.h>
#include "rm_deploy_stream/video_encoder_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rm_deploy_stream");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  rm_deploy_stream::VideoEncoderCore core(nh, pnh);

  ros::AsyncSpinner spinner(3);
  spinner.start();
  ROS_INFO("All nodes started, spinning...");
  ros::waitForShutdown();
  return 0;
}
