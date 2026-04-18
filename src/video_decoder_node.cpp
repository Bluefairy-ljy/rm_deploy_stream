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
  , yuv_frame_(nullptr)
  , rgb_frame_(nullptr)
  , sws_ctx_(nullptr)
  , mqtt_client_(nullptr)
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

  mosquitto_destroy(mqtt_client_);
  mosquitto_lib_cleanup();
  freeDecoder();
}

// 初始化 MQTT
void VideoDecoderNode::initMqtt()
{
  ROS_INFO("Init mqtt");
  mosquitto_lib_init();
    mqtt_client_ = mosquitto_new(MQTT_CLIENT_ID, true, this);

  mosquitto_connect_callback_set(mqtt_client_, mqtt_connect_callback);
  mosquitto_message_callback_set(mqtt_client_, mqtt_message_callback);

  mosquitto_connect_async(mqtt_client_, MQTT_HOST, MQTT_PORT, 60);
  mosquitto_loop_start(mqtt_client_);
}
void VideoDecoderNode::onMqttMessage(const void* payload, int len)
{
  referee::CustomByteBlock proto_msg;
  // 把 MQTT 收到的二进制数据还原成可用的 H264 视频流
  if (!proto_msg.ParseFromArray(payload, len))
  {
    printf("Protobuf manbaout");
    return;
  }
  // 反序列化后的数据
  const std::string& data = proto_msg.data();
  if (data.empty())
    return;
  // 每个包第一个字节是包序号
  auto current_seq = static_cast<uint8_t>(data[0]);
  if (last_seq_ != -1)
  {
    auto expected_seq = static_cast<uint8_t>((last_seq_ + 1) & 0xFF);
      // 丢包乱序时
      if (current_seq != expected_seq)
    {
      ROS_WARN("Packet lost/recorded! Expected %d, got %d. Dropping corrupted buffer.", expected_seq, current_seq);
      // 清空损坏的视频数据
      stream_buffer_.clear();
      // 重置解码器，等待下一个关键帧重新开始
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
  // 把 H.264 数据写入缓冲区，第2到300字节
  if (data.size() > 1)
  {
    stream_buffer_.insert(stream_buffer_.end(), (const uint8_t*)data.data() + 1,
                          (const uint8_t*)data.data() + data.size());
  }
  // 分配一个 AVPacket 用于存放切分出来的 H.264 帧
  AVPacket* pkt = av_packet_alloc();
  // buffer_pos 指向当前读取位置
  uint8_t* buffer_pos = stream_buffer_.data();
  // 缓冲区字节数
  int remaining_bytes = static_cast<int>(stream_buffer_.size());
  while (remaining_bytes > 0)
  {
    // FFmpeg 解析 H.264 流，把拼接好的 H264 流切成完整的帧。parse_result 为消耗的字节数
    int parse_result = av_parser_parse2(parser_ctx_, codec_ctx_, &pkt->data, &pkt->size, buffer_pos, remaining_bytes, 0, 0, 0);
    if (parse_result <= 0)
      break;
    buffer_pos += parse_result;
    remaining_bytes -= parse_result;
    // 把一帧完整的 H.264 交给解码器
    if (pkt->size && avcodec_send_packet(codec_ctx_, pkt) == 0)
    {
      // 接收解码后的图像
      while (avcodec_receive_frame(codec_ctx_, yuv_frame_) == 0)
      {
        // 格式转换从YUV到BGR
        sws_scale(sws_ctx_, yuv_frame_->data, yuv_frame_->linesize, 0, 320, rgb_frame_->data, rgb_frame_->linesize);
        cv::Mat img(320, 320, CV_8UC3, rgb_frame_->data[0]);
        std::lock_guard<std::mutex> lock(mtx_);
        if (frame_queue_.size() > 2)
          frame_queue_.pop();
        // 图像送入队列
        frame_queue_.push(img.clone());
      }
    }
  }
  // 留下未解析数据，清掉解析过的
  stream_buffer_.assign(buffer_pos, buffer_pos + remaining_bytes);
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

    yuv_frame_ = av_frame_alloc();
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
  av_frame_free(&yuv_frame_);
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
      std::lock_guard<std::mutex> lock(mtx_);
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
