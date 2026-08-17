#include "semantic_detection/ros/YoloNode.hpp"

#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include "semantic_detection/backends/CpuBackend.hpp"
#include "semantic_detection/utils/YamlParser.hpp"
#ifdef SEMANTIC_DETECTION_WITH_QNN_HTP
#include "semantic_detection/backends/QnnHtpBackend.hpp"
#endif

namespace semantic_detection {

YoloNode::YoloNode() : Node("yolo_node") {
  declareParameters();
  buildDetector();

  // Best-effort, depth-1 QoS: if inference falls behind the camera's
  // publishing rate we drop frames rather than queue them. This node runs
  // as its own process/subscriber, entirely independent of OKVIS's own
  // image subscription, so a slow detector cannot block or delay SLAM.
  const auto image_qos = rclcpp::SensorDataQoS().keep_last(1);

  image_sub_ = image_transport::create_subscription(
      this, image_topic_,
      std::bind(&YoloNode::imageCallback, this, std::placeholders::_1), "raw",
      image_qos.get_rmw_qos_profile());

  detections_pub_ =
      create_publisher<vision_msgs::msg::Detection2DArray>("~/detections", rclcpp::QoS(10));

  if (publish_debug_image_) {
    debug_image_pub_ = image_transport::create_publisher(this, "~/debug_image");
  }

  RCLCPP_INFO(get_logger(), "semantic_detection: backend='%s' subscribed to '%s'",
              detector_->backendName().c_str(), image_topic_.c_str());
}

void YoloNode::declareParameters() {
  image_topic_ = declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw");
  backend_name_ = declare_parameter<std::string>("backend", "cpu");
  model_path_ = declare_parameter<std::string>("model_path", "");
  data_yaml_path_ = declare_parameter<std::string>("data_yaml_path", "");
  input_size_ = declare_parameter<int>("input_size", 640);
  score_threshold_ = declare_parameter<double>("score_threshold", 0.25);
  nms_threshold_ = declare_parameter<double>("nms_threshold", 0.45);
  publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);

  qnn_backend_lib_ = declare_parameter<std::string>("qnn_backend_lib", "libQnnHtp.so");
  qnn_graph_name_ = declare_parameter<std::string>("qnn_graph_name", "");
  qnn_input_tensor_name_ = declare_parameter<std::string>("qnn_input_tensor_name", "images");
  qnn_output_tensor_name_ = declare_parameter<std::string>("qnn_output_tensor_name", "output0");
  qnn_output_channels_ = declare_parameter<int>("qnn_output_channels", 84);
  qnn_output_boxes_ = declare_parameter<int>("qnn_output_boxes", 8400);
  qnn_input_quantized_ = declare_parameter<bool>("qnn_input_quantized", true);
  qnn_output_quantized_ = declare_parameter<bool>("qnn_output_quantized", true);
  qnn_input_bits_ = declare_parameter<int>("qnn_input_bits", 16);
  qnn_output_bits_ = declare_parameter<int>("qnn_output_bits", 16);
  qnn_input_scale_ = declare_parameter<double>("qnn_input_scale", 1.0);
  qnn_input_offset_ = declare_parameter<int>("qnn_input_offset", 0);
  qnn_output_scale_ = declare_parameter<double>("qnn_output_scale", 1.0);
  qnn_output_offset_ = declare_parameter<int>("qnn_output_offset", 0);
}

void YoloNode::buildDetector() {
  std::vector<std::string> class_names;
  if (!data_yaml_path_.empty()) {
    class_names = loadClassNames(data_yaml_path_);
  } else {
    RCLCPP_WARN(get_logger(),
                "semantic_detection: no data_yaml_path set -- detections will be labeled by "
                "numeric class id only");
  }

  std::unique_ptr<InferenceBackend> backend;
  if (backend_name_ == "htp") {
#ifdef SEMANTIC_DETECTION_WITH_QNN_HTP
    QnnHtpConfig config;
    config.input_size = input_size_;
    config.backend_lib_path = qnn_backend_lib_;
    config.graph_name = qnn_graph_name_;
    config.input.name = qnn_input_tensor_name_;
    config.input.dims = {1, static_cast<uint32_t>(input_size_), static_cast<uint32_t>(input_size_), 3};
    config.input.quantized = qnn_input_quantized_;
    config.input.bits = qnn_input_bits_;
    config.input.scale = static_cast<float>(qnn_input_scale_);
    config.input.offset = qnn_input_offset_;
    config.output.name = qnn_output_tensor_name_;
    config.output.dims = {1, static_cast<uint32_t>(qnn_output_channels_),
                         static_cast<uint32_t>(qnn_output_boxes_)};
    config.output.quantized = qnn_output_quantized_;
    config.output.bits = qnn_output_bits_;
    config.output.scale = static_cast<float>(qnn_output_scale_);
    config.output.offset = qnn_output_offset_;
    RCLCPP_INFO(get_logger(),
                "semantic_detection: HTP graph='%s' input(name='%s' dims=[1,%d,%d,3] bits=%d "
                "scale=%.10g offset=%d) output(name='%s' dims=[1,%d,%d] bits=%d scale=%.10g "
                "offset=%d)",
                qnn_graph_name_.c_str(), qnn_input_tensor_name_.c_str(), input_size_, input_size_,
                qnn_input_bits_, qnn_input_scale_, qnn_input_offset_,
                qnn_output_tensor_name_.c_str(), qnn_output_channels_, qnn_output_boxes_,
                qnn_output_bits_, qnn_output_scale_, qnn_output_offset_);
    backend = std::make_unique<QnnHtpBackend>(config);
#else
    RCLCPP_ERROR(get_logger(),
                "semantic_detection: backend='htp' requested but this build was compiled "
                "without WITH_QNN_HTP (QNN SDK not found at configure time). Falling back to "
                "the CPU backend.");
    backend = std::make_unique<CpuBackend>(input_size_);
#endif
  } else {
    backend = std::make_unique<CpuBackend>(input_size_);
  }

  const size_t num_classes = class_names.size();
  detector_ = std::make_unique<YOLODetector>(std::move(backend), std::move(class_names),
                                             YOLODetectorParams{
                                                 static_cast<float>(score_threshold_),
                                                 static_cast<float>(nms_threshold_)});

  const auto load_start = std::chrono::steady_clock::now();
  detector_->loadModel(model_path_);
  const auto load_end = std::chrono::steady_clock::now();
  const double load_ms =
      std::chrono::duration<double, std::milli>(load_end - load_start).count();
  load_ms_ = load_ms;
  RCLCPP_INFO(get_logger(),
              "semantic_detection: model loaded in %.2f ms (backend='%s' model_path='%s' "
              "input_size=%d score_threshold=%.2f nms_threshold=%.2f num_classes=%zu)",
              load_ms, detector_->backendName().c_str(), model_path_.c_str(), input_size_,
              score_threshold_, nms_threshold_, num_classes);
}

void YoloNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "semantic_detection: cv_bridge conversion failed: %s", e.what());
    return;
  }

  const auto callback_start = std::chrono::steady_clock::now();
  double interval_ms = 0.0;
  if (have_last_frame_time_) {
    interval_ms =
        std::chrono::duration<double, std::milli>(callback_start - last_frame_time_).count();
  }
  last_frame_time_ = callback_start;
  have_last_frame_time_ = true;

  DetectionResult result;
  try {
    result = detector_->detect(cv_ptr->image);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "semantic_detection: inference failed: %s", e.what());
    return;
  }

  publishDetections(msg->header, result.detections);
  if (publish_debug_image_) {
    publishDebugImage(msg->header, cv_ptr->image, result.detections);
  }

  const double total_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - callback_start)
          .count();

  preprocess_stats_.add(result.preprocess_ms);
  inference_stats_.add(result.inference_ms);
  postprocess_stats_.add(result.postprocess_ms);
  total_stats_.add(total_ms);

  last_preprocess_ms_ = result.preprocess_ms;
  last_inference_ms_ = result.inference_ms;
  last_postprocess_ms_ = result.postprocess_ms;
  last_total_ms_ = total_ms;

  RCLCPP_INFO(get_logger(), "----- frame (stamp %u.%09u) -----", msg->header.stamp.sec,
              msg->header.stamp.nanosec);
  if (interval_ms > 0.0) {
    RCLCPP_INFO(get_logger(), "  camera interval : %6.2f ms  (%.1f FPS in)", interval_ms,
                1000.0 / interval_ms);
  }
  RCLCPP_INFO(get_logger(), "  preprocess      : %6.2f ms", result.preprocess_ms);
  RCLCPP_INFO(get_logger(), "  inference       : %6.2f ms", result.inference_ms);
  RCLCPP_INFO(get_logger(), "  postprocess     : %6.2f ms", result.postprocess_ms);
  RCLCPP_INFO(get_logger(), "  callback total  : %6.2f ms  (%.1f FPS out)", total_ms,
              1000.0 / total_ms);
  RCLCPP_INFO(get_logger(), "  detections      : %zu", result.detections.size());
  for (const auto & det : result.detections) {
    RCLCPP_INFO(get_logger(), "    - %-16s score=%.3f box=[x=%d y=%d w=%d h=%d]",
                det.label.c_str(), det.score, det.box.x, det.box.y, det.box.width, det.box.height);
  }
}

void YoloNode::publishDetections(const std_msgs::msg::Header & header,
                                 const std::vector<Detection> & detections) {
  vision_msgs::msg::Detection2DArray msg;
  msg.header = header;
  msg.detections.reserve(detections.size());

  for (const auto & det : detections) {
    vision_msgs::msg::Detection2D d;
    d.header = header;

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = det.label;
    hyp.hypothesis.score = det.score;
    d.results.push_back(hyp);

    d.bbox.center.position.x = det.box.x + det.box.width / 2.0;
    d.bbox.center.position.y = det.box.y + det.box.height / 2.0;
    d.bbox.size_x = det.box.width;
    d.bbox.size_y = det.box.height;

    msg.detections.push_back(std::move(d));
  }

  detections_pub_->publish(msg);
}

void YoloNode::publishDebugImage(const std_msgs::msg::Header & header, const cv::Mat & bgr_image,
                                 const std::vector<Detection> & detections) {
  cv::Mat debug_image = bgr_image.clone();
  for (const auto & det : detections) {
    cv::rectangle(debug_image, det.box, cv::Scalar(0, 255, 0), 2);
    const std::string caption = det.label + " " + cv::format("%.2f", det.score);
    int baseline = 0;
    const cv::Size text_size = cv::getTextSize(caption, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    const cv::Point text_origin(det.box.x, std::max(det.box.y - 4, text_size.height));
    cv::rectangle(debug_image,
                  text_origin + cv::Point(0, baseline + 2),
                  text_origin + cv::Point(text_size.width, -text_size.height - 2),
                  cv::Scalar(0, 255, 0), cv::FILLED);
    cv::putText(debug_image, caption, text_origin, cv::FONT_HERSHEY_SIMPLEX, 0.5,
               cv::Scalar(0, 0, 0), 1);
  }
  auto out_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, debug_image).toImageMsg();
  debug_image_pub_.publish(out_msg);
}

void YoloNode::printSummary() const {
  const auto print_stats = [this](const char * label, const TimingStats & stats) {
    if (stats.count == 0) {
      RCLCPP_INFO(get_logger(), "  %-12s: no frames processed", label);
      return;
    }
    RCLCPP_INFO(get_logger(), "  %-12s: avg=%7.2f ms  min=%7.2f ms  max=%7.2f ms", label,
                stats.mean_ms(), stats.min_ms, stats.max_ms);
  };

  RCLCPP_INFO(get_logger(), "================ semantic_detection benchmark summary ================");
  RCLCPP_INFO(get_logger(), "  model load   : %7.2f ms", load_ms_);
  RCLCPP_INFO(get_logger(), "  frames       : %zu", total_stats_.count);

  if (total_stats_.count > 0) {
    RCLCPP_INFO(get_logger(), "  -- last frame --");
    RCLCPP_INFO(get_logger(), "  preprocess   : %7.2f ms", last_preprocess_ms_);
    RCLCPP_INFO(get_logger(), "  inference    : %7.2f ms", last_inference_ms_);
    RCLCPP_INFO(get_logger(), "  postprocess  : %7.2f ms", last_postprocess_ms_);
    RCLCPP_INFO(get_logger(), "  total        : %7.2f ms", last_total_ms_);
  }

  RCLCPP_INFO(get_logger(), "  -- over all frames --");
  print_stats("preprocess", preprocess_stats_);
  print_stats("inference", inference_stats_);
  print_stats("postprocess", postprocess_stats_);
  print_stats("total/frame", total_stats_);
  if (total_stats_.count > 0) {
    RCLCPP_INFO(get_logger(), "  avg FPS      : %7.2f", 1000.0 / total_stats_.mean_ms());
  }
  RCLCPP_INFO(get_logger(), "========================================================================");
}

}  // namespace semantic_detection
