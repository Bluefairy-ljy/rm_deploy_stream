#include <ros/ros.h>
#include "rm_deploy_stream/video_encoder_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_encoder");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ROS_INFO("ok");
  rm_deploy_stream::VideoEncoderCore core(nh, pnh);
  ros::spin();
  return 0;
}
