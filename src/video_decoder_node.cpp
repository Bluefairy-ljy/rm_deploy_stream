//
// Created by ljyi on 2026/4/8.
//

#include "rm_deploy_stream/video_decoder_node.h"

namespace rm_deploy_stream
{
// MQTT 配置
#define MQTT_HOST "192.168.1.2"
#define MQTT_PORT 3333
#define MQTT_TOPIC "CustomByteBlock"
#define MQTT_CLIENT_ID "101"

static void mqtt_message_callback(struct mosquitto* mosq, void* userdata, const struct mosquitto_message* msg)
{
  if (!msg || !msg->payload || msg->payloadlen <= 0)
    return;
  auto* node = (VideoDecoderNode*)userdata;
  node->onMqttMessage(msg->payload, msg->payloadlen);
}

static void mqtt_connect_callback(struct mosquitto* mosq, void* userdata, int result)
{
  if (result == 0)
  {
    mosquitto_subscribe(mosq, nullptr, MQTT_TOPIC, 1);
    printf("MQTT is ok，listen: %s\n", MQTT_TOPIC);
  }
  else
  {
    printf("MQTT manbaout");
  }
}

VideoDecoderNode::VideoDecoderNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : codec_ctx_(nullptr)
  , parser_ctx_(nullptr)
  , frame_(nullptr)
  , rgb_frame_(nullptr)
  , sws_ctx_(nullptr)
  , mosq_(nullptr)
  , last_seq_(-1)
  , display_running_(true)
{
  initDecoder();
  initMqtt();
  display_thread_ = std::thread(&VideoDecoderNode::decodeThread, this);
  ROS_INFO("aaa");
}

VideoDecoderNode::~VideoDecoderNode()
{
  display_running_ = false;
  if (display_thread_.joinable())
    display_thread_.join();

  mosquitto_destroy(mosq_);
  mosquitto_lib_cleanup();
  freeDecoder();
}

// 初始化 MQTT
void VideoDecoderNode::initMqtt()
{
  ROS_INFO("Init mqtt");
  mosquitto_lib_init();
  mosq_ = mosquitto_new(MQTT_CLIENT_ID, true, this);

  mosquitto_connect_callback_set(mosq_, mqtt_connect_callback);
  mosquitto_message_callback_set(mosq_, mqtt_message_callback);

  mosquitto_connect_async(mosq_, MQTT_HOST, MQTT_PORT, 60);
  mosquitto_loop_start(mosq_);
}
// 反序列化 CustomByteBlock
void VideoDecoderNode::onMqttMessage(const void* payload, int len)
{
  referee::CustomByteBlock pb;
  if (!pb.ParseFromArray(payload, len))
  {
    printf("Protobuf manbaout");
    return;
  }
  // 反序列化后的数据
  const std::string& data = pb.data();
  if (data.empty())
    return;
  // pull current package 8-bit serial number
  uint8_t current_seq = static_cast<uint8_t>(data[0]);
  // serial number continuity check
  if (last_seq_ != -1)
  {
    uint8_t expected_seq = static_cast<uint8_t>((last_seq_ + 1) & 0xFF);
    if (current_seq != expected_seq)
    {
      ROS_WARN("Packet lost/recorded! Expected %d, got %d. Dropping corrupted buffer.", expected_seq, current_seq);

      stream_buffer_.clear();
      // FFmpeg throw trash and waiting next start code
      if (parser_ctx_)
      {
        av_parser_close(parser_ctx_);
        parser_ctx_ = av_parser_init(AV_CODEC_ID_H264);
      }
    }
  }
  last_seq_ = current_seq;
  // 调试打印
  printf("[MQTT 包] data_len: %-4lu | 第一个字节(seq): 0x%02X | 前8字节: ", data.size(), (uint8_t)data[0]);
  for (int i = 0; i < std::min(8, (int)data.size()); i++)
  {
    printf("%02X ", (uint8_t)data[i]);
  }
  printf("\n");
  stream_buffer_.insert(stream_buffer_.end(), (const uint8_t*)data.data() + 1,
                        (const uint8_t*)data.data() + data.size());

  AVPacket* pkt = av_packet_alloc();
  uint8_t* ptr = stream_buffer_.data();
  int remain = stream_buffer_.size();

  if (data.size() > 1)
  {
    stream_buffer_.insert(stream_buffer_.end(), (const uint8_t*)data.data() + 1,
                          (const uint8_t*)data.data() + data.size());
  }

  while (remain > 0)
  {
    int ret = av_parser_parse2(parser_ctx_, codec_ctx_, &pkt->data, &pkt->size, ptr, remain, 0, 0, 0);
    if (ret <= 0)
      break;

    ptr += ret;
    remain -= ret;

    if (pkt->size && avcodec_send_packet(codec_ctx_, pkt) == 0)
    {
      while (avcodec_receive_frame(codec_ctx_, frame_) == 0)
      {
        sws_scale(sws_ctx_, frame_->data, frame_->linesize, 0, 320, rgb_frame_->data, rgb_frame_->linesize);
        cv::Mat img(320, 320, CV_8UC3, rgb_frame_->data[0]);
        std::lock_guard<std::mutex> lk(mtx_);
        if (frame_queue_.size() > 2)
          frame_queue_.pop();
        frame_queue_.push(img.clone());
      }
    }
  }

  stream_buffer_.assign(ptr, ptr + remain);
  av_packet_free(&pkt);
}

void VideoDecoderNode::initDecoder()
{
  ROS_INFO("Init decoder");
  const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  parser_ctx_ = av_parser_init(AV_CODEC_ID_H264);
  codec_ctx_ = avcodec_alloc_context3(codec);
  codec_ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY;
  codec_ctx_->thread_count = 1;
  avcodec_open2(codec_ctx_, codec, nullptr);

  frame_ = av_frame_alloc();
  rgb_frame_ = av_frame_alloc();
  rgb_frame_->width = 320;
  rgb_frame_->height = 320;
  rgb_frame_->format = AV_PIX_FMT_BGR24;
  av_frame_get_buffer(rgb_frame_, 0);

  sws_ctx_ =
      sws_getContext(320, 320, AV_PIX_FMT_YUV420P, 320, 320, AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr, nullptr);
}

void VideoDecoderNode::freeDecoder()
{
  sws_freeContext(sws_ctx_);
  av_frame_free(&frame_);
  av_frame_free(&rgb_frame_);
  avcodec_free_context(&codec_ctx_);
  av_parser_close(parser_ctx_);
}

void VideoDecoderNode::decodeThread()
{
  cv::namedWindow("Video", cv::WINDOW_NORMAL);
  while (display_running_ && ros::ok())
  {
    cv::Mat img;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!frame_queue_.empty())
      {
        img = frame_queue_.front();
        frame_queue_.pop();
      }
    }

    if (!img.empty())
    {
      cv::imshow("Video", img);
      cv::waitKey(1);
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  cv::destroyAllWindows();
}

}  // namespace rm_deploy_stream
