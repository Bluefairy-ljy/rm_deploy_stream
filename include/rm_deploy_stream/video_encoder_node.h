#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>
#include <string>
#include <utility>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

#include "rm_deploy_stream/VideoPacket.h"

namespace rm_deploy_stream
{

class VideoEncoderCore
{
public:
  VideoEncoderCore(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~VideoEncoderCore();

private:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  cv::Mat preprocessImage(const cv::Mat& input, cv::Mat* roi_downsample, cv::Mat* static_removed);
  void initializeGstreamer();
  void shutdownGstreamer();
  void pushFrameToGstreamer(const cv::Mat& frame);
  void pullStreamAndPacketize();
  void displayLoop();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher packet_pub_;

  GstElement* pipeline_{nullptr};
  GstElement* appsrc_{nullptr};
  GstElement* appsink_{nullptr};
  GstBus* bus_{nullptr};

  std::mutex frame_mutex_;
  cv::Mat display_raw_frame_;
  cv::Mat display_roi_frame_;
  cv::Mat display_static_frame_;
  cv::Mat display_frame_;
  std::thread display_thread_;
  std::atomic<bool> display_running_{false};

  std::mutex buffer_mutex_;
  std::vector<uint8_t> stream_buffer_;
  std::deque<std::pair<int64_t, size_t>> sent_window_;
  size_t sent_window_bytes_{0};

  uint64_t packet_sequence_id_{0};
  uint64_t frame_count_{0};
  uint64_t display_frame_counter_{0};
  uint64_t dropped_bytes_{0};
  uint32_t dropped_events_{0};
  int64_t last_telemetry_ns_{0};
  int64_t last_encode_stamp_ns_{0};

  cv::Mat background_gray_f32_;
  cv::Mat motion_erode_kernel_;
  cv::Mat motion_dilate_kernel_;
  std::deque<cv::Mat> motion_mask_history_;
  std::deque<cv::Mat> trail_frame_history_;

  std::string input_topic_;
  int crop_size_{800};
  int output_size_{400};
  int output_fps_{60};
  int target_bitrate_{40};
  int packet_size_{150};
  bool static_simplify_{true};
  int motion_threshold_{14};
  int motion_erode_px_{1};
  int motion_dilate_px_{2};
  int motion_trail_frames_{3};
  double trail_disable_motion_ratio_{0.30};
  double bg_update_alpha_{0.01};
  double bg_blur_sigma_{1.2};
  int center_clear_size_{100};
  bool force_monochrome_{false};
  double bandwidth_limit_kbytes_{7.0};
  double bandwidth_window_s_{2.0};
  double max_tx_delay_s_{1.0};
  bool enable_display_{true};
  std::string x264_preset_{"auto"};

  bool debug_dump_enable_{false};
  int debug_dump_every_n_frames_{20};
  bool debug_dump_save_raw_{true};
  bool debug_dump_save_roi_{true};
  bool debug_dump_save_static_{true};
  bool debug_dump_save_final_{true};
  std::string debug_dump_dir_{"sniper_debug_imgs"};
};

}  // namespace rm_deploy_stream
