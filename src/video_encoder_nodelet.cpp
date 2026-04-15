/*
 * @Author: Daxiaolizi
 * @Date: 2026-04-05 04:36:06
 */
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "rm_deploy_stream/video_encoder_node.h"

namespace rm_deploy_stream
{
class VideoEncoderNodelet : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    core_.reset(new VideoEncoderCore(getNodeHandle(), getPrivateNodeHandle()));
  }

private:
  std::unique_ptr<VideoEncoderCore> core_;
};
}  // namespace rm_deploy_stream

PLUGINLIB_EXPORT_CLASS(rm_deploy_stream::VideoEncoderNodelet, nodelet::Nodelet)
