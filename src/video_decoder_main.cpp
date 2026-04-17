//
// Created by ljyi on 2026/4/8.
//

#include <ros/ros.h>
#include "rm_deploy_stream/video_decoder_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_decoder");
  printf("bbb\n");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  rm_deploy_stream::VideoDecoderNode node(nh, pnh);
  ros::spin();
  return 0;
}
