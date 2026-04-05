/*
 * @Author: Daxiaolizi
 * @Date: 2026-04-05 02:38:06
 */
#include "rm_deploy_stream/video_encoder_node.h"
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace rm_deploy_stream
{

VideoEncoderCore::VideoEncoderCore(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
: nh_(nh), pnh_(pnh), it_(nh_)
{
  constexpr int kVideoPacketBytes = 300;

  pnh_.param<std::string>("input_topic", input_topic_, "/hk_camera/image_raw");
  pnh_.param("crop_size", crop_size_, 800);
  pnh_.param("output_size", output_size_, 400);
  pnh_.param("output_fps", output_fps_, 60);
  pnh_.param("target_bitrate", target_bitrate_, 40);
  pnh_.param("packet_size", packet_size_, kVideoPacketBytes);
  pnh_.param("static_simplify", static_simplify_, true);
  pnh_.param("motion_threshold", motion_threshold_, 14);
  pnh_.param("motion_erode_px", motion_erode_px_, 1);
  pnh_.param("motion_dilate_px", motion_dilate_px_, 2);
  pnh_.param("motion_trail_frames", motion_trail_frames_, 3);
  pnh_.param("trail_disable_motion_ratio", trail_disable_motion_ratio_, 0.30);
  pnh_.param("bg_update_alpha", bg_update_alpha_, 0.01);
  pnh_.param("bg_blur_sigma", bg_blur_sigma_, 1.2);
  pnh_.param("center_clear_size", center_clear_size_, 100);
  pnh_.param("force_monochrome", force_monochrome_, false);
  pnh_.param("bandwidth_limit_kbytes", bandwidth_limit_kbytes_, 7.0);
  pnh_.param("bandwidth_window_s", bandwidth_window_s_, 2.0);
  pnh_.param("max_tx_delay_s", max_tx_delay_s_, 1.0);
  pnh_.param("enable_display", enable_display_, true);
  pnh_.param<std::string>("x264_preset", x264_preset_, "auto");
  pnh_.param("debug_dump_enable", debug_dump_enable_, false);
  pnh_.param("debug_dump_every_n_frames", debug_dump_every_n_frames_, 20);
  pnh_.param("debug_dump_save_raw", debug_dump_save_raw_, true);
  pnh_.param("debug_dump_save_roi", debug_dump_save_roi_, true);
  pnh_.param("debug_dump_save_static", debug_dump_save_static_, true);
  pnh_.param("debug_dump_save_final", debug_dump_save_final_, true);
  pnh_.param<std::string>("debug_dump_dir", debug_dump_dir_, "sniper_debug_imgs");

  output_fps_ = std::clamp(output_fps_, 1, 60);
  motion_trail_frames_ = std::clamp(motion_trail_frames_, 0, 15);
  motion_erode_px_ = std::clamp(motion_erode_px_, 0, 20);
  motion_dilate_px_ = std::clamp(motion_dilate_px_, 0, 20);
  trail_disable_motion_ratio_ = std::clamp(trail_disable_motion_ratio_, 0.0, 1.0);
  bandwidth_limit_kbytes_ = std::max(1.0, bandwidth_limit_kbytes_);
  bandwidth_window_s_ = std::max(0.2, bandwidth_window_s_);
  max_tx_delay_s_ = std::max(0.05, max_tx_delay_s_);
  debug_dump_every_n_frames_ = std::max(1, debug_dump_every_n_frames_);

  if (packet_size_ != kVideoPacketBytes) {
    ROS_WARN("VideoPacket fixed 300 bytes, override packet_size %d -> 300", packet_size_);
    packet_size_ = kVideoPacketBytes;
  }

  if (debug_dump_enable_) {
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(debug_dump_dir_) / "encoder", ec);
    if (ec) {
      ROS_WARN("Create dump dir failed: %s", ec.message().c_str());
      debug_dump_enable_ = false;
    }
  }

  packet_pub_ = nh_.advertise<rm_deploy_stream::VideoPacket>("/video_stream", 3000);
  image_sub_ = it_.subscribe(input_topic_, 10, &VideoEncoderCore::imageCallback, this);

  initializeGstreamer();

  if (enable_display_) {
    display_running_ = true;
    display_thread_ = std::thread(&VideoEncoderCore::displayLoop, this);
  }

  ROS_INFO("video_encoder ready. sub=%s out=%dx%d@%dfps bitrate=%dkbps",
           input_topic_.c_str(), output_size_, output_size_, output_fps_, target_bitrate_);
}

VideoEncoderCore::~VideoEncoderCore()
{
  display_running_ = false;
  if (display_thread_.joinable()) display_thread_.join();
  cv::destroyAllWindows();
  shutdownGstreamer();
}

void VideoEncoderCore::initializeGstreamer()
{
  static bool inited = false;
  if (!inited) {
    gst_init(nullptr, nullptr);
    inited = true;
  }

  pipeline_ = gst_pipeline_new("encoder_pipe");
  appsrc_ = gst_element_factory_make("appsrc", "source");
  appsink_ = gst_element_factory_make("appsink", "sink");
  GstElement* convert = gst_element_factory_make("videoconvert", "convert");
  GstElement* encoder = gst_element_factory_make("x264enc", "encoder");
  GstElement* parser = gst_element_factory_make("h264parse", "parser");

  if (!pipeline_ || !appsrc_ || !appsink_ || !convert || !encoder || !parser) {
    ROS_FATAL("GStreamer element creation failed");
    return;
  }

  GstCaps* caps = gst_caps_new_simple("video/x-raw",
                                      "format", G_TYPE_STRING, "BGR",
                                      "width", G_TYPE_INT, output_size_,
                                      "height", G_TYPE_INT, output_size_,
                                      "framerate", GST_TYPE_FRACTION, output_fps_, 1,
                                      nullptr);
  g_object_set(G_OBJECT(appsrc_),
               "caps", caps,
               "stream-type", 0,
               "format", GST_FORMAT_TIME,
               "is-live", TRUE,
               "do-timestamp", TRUE,
               nullptr);
  gst_caps_unref(caps);

  const bool low_bitrate_mode = (target_bitrate_ <= 80);
  const int key_int = std::max(8 * output_fps_, 30);

  g_object_set(G_OBJECT(encoder),
               "bitrate", target_bitrate_,
               "byte-stream", TRUE,
               "key-int-max", key_int,
               "aud", TRUE,
               nullptr);

  if (low_bitrate_mode) {
    g_object_set(G_OBJECT(encoder),
                 "speed-preset", 9, "bframes", 4, "rc-lookahead", 40, "ref", 5,
                 "option-string", "repeat-headers=1:scenecut=0:aq-mode=2:mbtree=1:force-cfr=1",
                 nullptr);
  } else {
    g_object_set(G_OBJECT(encoder),
                 "speed-preset", 3, "tune", 0x00000004, "bframes", 0, "rc-lookahead", 0,
                 "option-string", "repeat-headers=1:scenecut=0:ref=1:force-cfr=1",
                 nullptr);
  }

  g_object_set(G_OBJECT(parser), "config-interval", -1, "disable-passthrough", TRUE, nullptr);

  GstCaps* h264_caps = gst_caps_new_simple("video/x-h264",
                                           "stream-format", G_TYPE_STRING, "byte-stream",
                                           "alignment", G_TYPE_STRING, "au",
                                           nullptr);
  g_object_set(G_OBJECT(appsink_), "caps", h264_caps, "max-buffers", 5, "drop", FALSE,
               "emit-signals", FALSE, "sync", FALSE, nullptr);
  gst_caps_unref(h264_caps);

  gst_bin_add_many(GST_BIN(pipeline_), appsrc_, convert, encoder, parser, appsink_, nullptr);
  if (!gst_element_link_many(appsrc_, convert, encoder, parser, appsink_, nullptr)) {
    ROS_FATAL("GStreamer pipeline link failed");
    return;
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    ROS_FATAL("GStreamer pipeline start failed");
    return;
  }

  bus_ = gst_element_get_bus(pipeline_);
  ROS_INFO("GStreamer ready");
}

void VideoEncoderCore::shutdownGstreamer()
{
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    if (bus_) gst_object_unref(bus_);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
}

cv::Mat VideoEncoderCore::preprocessImage(const cv::Mat& input, cv::Mat* roi_downsample, cv::Mat* static_removed)
{
  int x = std::max(0, (input.cols - crop_size_) / 2);
  int y = std::max(0, (input.rows - crop_size_) / 2);
  int w = std::min(crop_size_, input.cols - x);
  int h = std::min(crop_size_, input.rows - y);

  cv::Mat cropped = input(cv::Rect(x, y, w, h));
  cv::Mat resized;
  cv::resize(cropped, resized, cv::Size(output_size_, output_size_), 0, 0, cv::INTER_LINEAR);
  if (roi_downsample) resized.copyTo(*roi_downsample);

  cv::Mat working = resized;
  if (force_monochrome_) {
    cv::Mat gray;
    cv::cvtColor(working, gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(gray, working, cv::COLOR_GRAY2BGR);
  }

  if (!static_simplify_) {
    if (static_removed) working.copyTo(*static_removed);
    return working;
  }

  cv::Mat gray;
  cv::cvtColor(working, gray, cv::COLOR_BGR2GRAY);
  if (background_gray_f32_.empty()) {
    gray.convertTo(background_gray_f32_, CV_32F);
    return working;
  }

  cv::Mat bg_u8, diff, motion_mask;
  cv::convertScaleAbs(background_gray_f32_, bg_u8);
  cv::absdiff(gray, bg_u8, diff);
  cv::threshold(diff, motion_mask, motion_threshold_, 255, cv::THRESH_BINARY);

  if (motion_erode_px_ > 0) {
    if (motion_erode_kernel_.empty()) {
      int k = 2 * motion_erode_px_ + 1;
      motion_erode_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    }
    cv::erode(motion_mask, motion_mask, motion_erode_kernel_);
  }
  if (motion_dilate_px_ > 0) {
    if (motion_dilate_kernel_.empty()) {
      int k = 2 * motion_dilate_px_ + 1;
      motion_dilate_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    }
    cv::dilate(motion_mask, motion_mask, motion_dilate_kernel_);
  }

  if (center_clear_size_ > 0) {
    int cs = std::min({center_clear_size_, working.cols, working.rows});
    int x0 = std::max(0, working.cols / 2 - cs / 2);
    int y0 = std::max(0, working.rows / 2 - cs / 2);
    cv::rectangle(motion_mask, cv::Rect(x0, y0, cs, cs), cv::Scalar(255), cv::FILLED);
  }

  cv::Mat static_base = working.clone();
  if (!force_monochrome_ && target_bitrate_ <= 80) {
    cv::Mat g;
    cv::cvtColor(static_base, g, cv::COLOR_BGR2GRAY);
    cv::cvtColor(g, static_base, cv::COLOR_GRAY2BGR);
  }

  cv::Mat blurred_static;
  cv::GaussianBlur(static_base, blurred_static, cv::Size(), std::max(0.0, bg_blur_sigma_));
  cv::Mat focused = blurred_static.clone();
  working.copyTo(focused, motion_mask);
  if (static_removed) focused.copyTo(*static_removed);

  if (motion_trail_frames_ > 0) {
    motion_mask_history_.push_back(motion_mask.clone());
    trail_frame_history_.push_back(working.clone());
    size_t max_history = static_cast<size_t>(motion_trail_frames_ + 1);
    while (motion_mask_history_.size() > max_history) motion_mask_history_.pop_front();
    while (trail_frame_history_.size() > max_history) trail_frame_history_.pop_front();

    double motion_ratio = static_cast<double>(cv::countNonZero(motion_mask)) / motion_mask.total();
    bool suppress_trail = motion_ratio >= trail_disable_motion_ratio_;

    if (!suppress_trail && motion_mask_history_.size() > 1 &&
        motion_mask_history_.size() == trail_frame_history_.size()) {
      cv::Mat trail_mask = motion_mask.clone();
      cv::Mat trail_img = working.clone();
      for (size_t i = 0; i + 1 < motion_mask_history_.size(); ++i) {
        cv::bitwise_or(trail_mask, motion_mask_history_[i], trail_mask);
        cv::max(trail_img, trail_frame_history_[i], trail_img);
      }
      trail_img.copyTo(focused, trail_mask);
    }
  } else {
    motion_mask_history_.clear();
    trail_frame_history_.clear();
  }

  cv::accumulateWeighted(gray, background_gray_f32_, std::clamp(bg_update_alpha_, 0.001, 0.2));
  return focused;
}

void VideoEncoderCore::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try {
    if (output_fps_ < 60) {
      int64_t now_ns = (msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp).toNSec();
      int64_t interval_ns = 1000000000LL / std::max(1, output_fps_);
      if (last_encode_stamp_ns_ > 0 && (now_ns - last_encode_stamp_ns_) < interval_ns) return;
      last_encode_stamp_ns_ = now_ns;
    }

    cv::Mat input = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat roi_downsample, static_removed;
    cv::Mat processed = preprocessImage(input, &roi_downsample, &static_removed);

    if (enable_display_) {
      cv::Mat raw_preview;
      cv::resize(input, raw_preview, cv::Size(std::max(1, input.cols / 2), std::max(1, input.rows / 2)));
      std::lock_guard<std::mutex> lk(frame_mutex_);
      raw_preview.copyTo(display_raw_frame_);
      roi_downsample.copyTo(display_roi_frame_);
      static_removed.copyTo(display_static_frame_);
      processed.copyTo(display_frame_);
    }

    pushFrameToGstreamer(processed);
    pullStreamAndPacketize();
    frame_count_++;
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge error: %s", e.what());
  }
}

void VideoEncoderCore::pushFrameToGstreamer(const cv::Mat& frame)
{
  if (!appsrc_ || frame.empty()) return;
  size_t size = frame.total() * frame.elemSize();
  GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);

  GstMapInfo map;
  if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
    std::memcpy(map.data, frame.data, size);
    gst_buffer_unmap(buffer, &map);
    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
    if (ret != GST_FLOW_OK) ROS_WARN("push-buffer failed: %d", ret);
  }
  gst_buffer_unref(buffer);
}

void VideoEncoderCore::pullStreamAndPacketize()
{
  if (!appsink_) return;

  const size_t packet_bytes = static_cast<size_t>(packet_size_);
  const int64_t window_ns = static_cast<int64_t>(bandwidth_window_s_ * 1e9);
  const size_t window_limit_bytes = static_cast<size_t>(bandwidth_limit_kbytes_ * 1000.0 * bandwidth_window_s_);
  const size_t max_backlog_bytes = static_cast<size_t>(bandwidth_limit_kbytes_ * 1000.0 * max_tx_delay_s_);

  while (true) {
    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), 0);
    if (!sample) break;

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
      gst_sample_unref(sample);
      continue;
    }

    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      std::lock_guard<std::mutex> lk(buffer_mutex_);

      size_t old = stream_buffer_.size();
      stream_buffer_.resize(old + map.size);
      std::memcpy(stream_buffer_.data() + old, map.data, map.size);

      while (stream_buffer_.size() >= packet_bytes) {
        int64_t now_ns = ros::Time::now().toNSec();
        while (!sent_window_.empty() && (now_ns - sent_window_.front().first) > window_ns) {
          sent_window_bytes_ -= sent_window_.front().second;
          sent_window_.pop_front();
        }
        if (sent_window_bytes_ + packet_bytes > window_limit_bytes) break;

        rm_deploy_stream::VideoPacket pkt;
        pkt.seq = static_cast<uint32_t>(packet_sequence_id_ & 0xFFu);
        packet_sequence_id_++;
        pkt.timestamp_ns = static_cast<uint64_t>(now_ns);
        pkt.data.fill(0);
        std::memcpy(pkt.data.data(), stream_buffer_.data(), packet_size_);
        packet_pub_.publish(pkt);

        sent_window_.emplace_back(now_ns, packet_bytes);
        sent_window_bytes_ += packet_bytes;

        std::memmove(stream_buffer_.data(), stream_buffer_.data() + packet_size_, stream_buffer_.size() - packet_size_);
        stream_buffer_.resize(stream_buffer_.size() - packet_size_);
      }

      if (stream_buffer_.size() > max_backlog_bytes) {
        size_t target_drop = stream_buffer_.size() - max_backlog_bytes;
        size_t drop_bytes = target_drop;
        for (size_t i = target_drop; i + 4 < stream_buffer_.size(); ++i) {
          bool sc3 = stream_buffer_[i] == 0 && stream_buffer_[i + 1] == 0 && stream_buffer_[i + 2] == 1;
          bool sc4 = stream_buffer_[i] == 0 && stream_buffer_[i + 1] == 0 && stream_buffer_[i + 2] == 0 && stream_buffer_[i + 3] == 1;
          if (sc3 || sc4) { drop_bytes = i; break; }
        }
        std::memmove(stream_buffer_.data(), stream_buffer_.data() + drop_bytes, stream_buffer_.size() - drop_bytes);
        stream_buffer_.resize(stream_buffer_.size() - drop_bytes);
        dropped_bytes_ += drop_bytes;
        dropped_events_++;
      }

      int64_t now_ns = ros::Time::now().toNSec();
      if (now_ns - last_telemetry_ns_ > 1000000000LL) {
        double window_kb = static_cast<double>(sent_window_bytes_) / 1000.0;
        ROS_INFO("TX stats: window=%.2f/%.2fKB avg=%.2fKB/s backlog=%zuB dropped=%luB",
                 window_kb, static_cast<double>(window_limit_bytes) / 1000.0,
                 window_kb / bandwidth_window_s_, stream_buffer_.size(), dropped_bytes_);
        last_telemetry_ns_ = now_ns;
      }

      gst_buffer_unmap(buffer, &map);
    }

    gst_sample_unref(sample);
  }
}

void VideoEncoderCore::displayLoop()
{
  cv::namedWindow("Doorlock Sniper Raw", cv::WINDOW_NORMAL);
  cv::namedWindow("Doorlock Sniper ROI", cv::WINDOW_NORMAL);
  cv::namedWindow("Doorlock Sniper Static", cv::WINDOW_NORMAL);
  cv::namedWindow("Doorlock Sniper", cv::WINDOW_NORMAL);

  while (display_running_ && ros::ok()) {
    cv::Mat raw, roi, sta, fin;
    {
      std::lock_guard<std::mutex> lk(frame_mutex_);
      if (!display_raw_frame_.empty()) display_raw_frame_.copyTo(raw);
      if (!display_roi_frame_.empty()) display_roi_frame_.copyTo(roi);
      if (!display_static_frame_.empty()) display_static_frame_.copyTo(sta);
      if (!display_frame_.empty()) display_frame_.copyTo(fin);
    }

    if (!raw.empty()) cv::imshow("Doorlock Sniper Raw", raw);
    if (!roi.empty()) cv::imshow("Doorlock Sniper ROI", roi);
    if (!sta.empty()) cv::imshow("Doorlock Sniper Static", sta);
    if (!fin.empty()) cv::imshow("Doorlock Sniper", fin);

    if (debug_dump_enable_ && !fin.empty()) {
      display_frame_counter_++;
      if ((display_frame_counter_ % static_cast<uint64_t>(debug_dump_every_n_frames_)) == 0U) {
        std::filesystem::path dump_dir = std::filesystem::path(debug_dump_dir_) / "encoder";
        std::ostringstream ss;
        ss << std::setw(8) << std::setfill('0') << display_frame_counter_;
        const std::string id = ss.str();
        if (debug_dump_save_raw_ && !raw.empty()) cv::imwrite((dump_dir / ("raw_" + id + ".png")).string(), raw);
        if (debug_dump_save_roi_ && !roi.empty()) cv::imwrite((dump_dir / ("roi_" + id + ".png")).string(), roi);
        if (debug_dump_save_static_ && !sta.empty()) cv::imwrite((dump_dir / ("static_" + id + ".png")).string(), sta);
        if (debug_dump_save_final_) cv::imwrite((dump_dir / ("final_" + id + ".png")).string(), fin);
      }
    }

    cv::waitKey(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
}

}  // namespace rm_deploy_stream
