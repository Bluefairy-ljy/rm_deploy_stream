//
// Created by ljyi on 2026/4/5.
//

#ifndef RM_DEPLOY_STREAM_SERIAL_SENDER_NODE_H
#define RM_DEPLOY_STREAM_SERIAL_SENDER_NODE_H

#include <ros/ros.h>
#include <rm_deploy_stream/VideoPacket.h>
#include <serial/serial.h>

namespace rm_deploy_stream
{
class SerialSenderNode
{
public:
  SerialSenderNode();
  ~SerialSenderNode() = default;

private:
  void VideoPacketCB(const VideoPacketConstPtr& msg);
  ros::NodeHandle nh_;
  ros::Subscriber video_packet_sub_;
  serial::Serial serial_;
  int packet_size_;
};

}  // namespace rm_deploy_stream

#endif
