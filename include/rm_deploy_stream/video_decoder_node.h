#ifndef RM_DEPLOY_STREAM_VIDEO_DECODER_NODE_H
#define RM_DEPLOY_STREAM_VIDEO_DECODER_NODE_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include <mutex>
#include <thread>
#include <vector>
#include <atomic>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <libavformat/avformat.h>
}

#include <mosquitto.h>
#include "referee.pb.h"

namespace rm_deploy_stream
{
class VideoDecoderNode
{
public:
  VideoDecoderNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~VideoDecoderNode();
  void onMqttMessage(const void* payload, int len);

private:
  // 初始化函数
  void initMqtt();
  void initDecoder();
  void freeDecoder();

  // 解码与显示线程
  void decodeThread();

  // FFmpeg 相关
  AVCodecContext* codec_ctx_;
  AVCodecParserContext* parser_ctx_;
  AVFrame* frame_;      // YUV帧
  AVFrame* rgb_frame_;  // RGB/BGR帧
  SwsContext* sws_ctx_;

  // MQTT 客户端
  mosquitto* mosq_;

  // 数据流缓冲
  std::vector<uint8_t> stream_buffer_;

  // 线程与运行标志
  std::atomic<bool> display_running_;
  std::thread display_thread_;

  // 帧队列 + 互斥锁
  std::queue<cv::Mat> frame_queue_;
  std::mutex mtx_;
  int last_seq_;
};

}  // namespace rm_deploy_stream

#endif
