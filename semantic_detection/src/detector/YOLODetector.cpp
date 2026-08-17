#include "semantic_detection/detector/YOLODetector.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

#include "semantic_detection/utils/NMS.hpp"

namespace semantic_detection {

YOLODetector::YOLODetector(std::unique_ptr<InferenceBackend> backend,
                           std::vector<std::string> class_names, YOLODetectorParams params)
    : backend_(std::move(backend)), class_names_(std::move(class_names)), params_(params) {
  if (!backend_) {
    throw std::invalid_argument("YOLODetector: backend must not be null");
  }
  backend_name_ = backend_->backendName();
}

void YOLODetector::loadModel(const std::string & model_path) { backend_->loadModel(model_path); }

std::vector<Detection> YOLODetector::decode(const RawOutput & raw, const LetterboxInfo & info,
                                            const cv::Size & original_size) const {
  const int num_boxes = raw.num_boxes;
  const int num_classes = raw.num_channels - 4;
  if (num_classes <= 0) {
    throw std::runtime_error("YOLODetector: raw output has fewer than 5 channels; expected "
                              "[cx, cy, w, h, class_0..N]");
  }

  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  std::vector<int> class_ids;
  boxes.reserve(num_boxes);
  scores.reserve(num_boxes);
  class_ids.reserve(num_boxes);

  const float * data = raw.data.data();
  auto at = [&](int channel, int box) { return data[static_cast<size_t>(channel) * num_boxes + box]; };

  for (int j = 0; j < num_boxes; ++j) {
    int best_class = -1;
    float best_score = params_.score_threshold;
    for (int c = 0; c < num_classes; ++c) {
      const float score = at(4 + c, j);
      if (score > best_score) {
        best_score = score;
        best_class = c;
      }
    }
    if (best_class < 0) {
      continue;
    }

    const float cx = at(0, j);
    const float cy = at(1, j);
    const float w = at(2, j);
    const float h = at(3, j);

    // Undo letterbox: from network-input space back to the original image.
    const float x1 = (cx - w / 2.0f - info.pad_x) / info.scale;
    const float y1 = (cy - h / 2.0f - info.pad_y) / info.scale;
    const float box_w = w / info.scale;
    const float box_h = h / info.scale;

    cv::Rect box(static_cast<int>(std::round(x1)), static_cast<int>(std::round(y1)),
                 static_cast<int>(std::round(box_w)), static_cast<int>(std::round(box_h)));
    box &= cv::Rect(0, 0, original_size.width, original_size.height);
    if (box.width <= 0 || box.height <= 0) {
      continue;
    }

    boxes.push_back(box);
    scores.push_back(best_score);
    class_ids.push_back(best_class);
  }

  const std::vector<int> keep =
      perClassNMS(boxes, scores, class_ids, params_.score_threshold, params_.nms_threshold);

  std::vector<Detection> detections;
  detections.reserve(keep.size());
  for (int idx : keep) {
    Detection det;
    det.class_id = class_ids[idx];
    det.label = (det.class_id >= 0 && det.class_id < static_cast<int>(class_names_.size()))
                    ? class_names_[det.class_id]
                    : ("class_" + std::to_string(det.class_id));
    det.score = scores[idx];
    det.box = boxes[idx];
    detections.push_back(std::move(det));
  }
  return detections;
}

namespace {
double elapsedMs(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}
}  // namespace

DetectionResult YOLODetector::detect(const cv::Mat & bgr_image) const {
  const auto t0 = std::chrono::steady_clock::now();
  LetterboxInfo info;
  const cv::Mat input = letterbox(bgr_image, backend_->inputSize(), info);
  const auto t1 = std::chrono::steady_clock::now();

  const RawOutput raw = backend_->infer(input);
  const auto t2 = std::chrono::steady_clock::now();

  std::vector<Detection> detections = decode(raw, info, bgr_image.size());
  const auto t3 = std::chrono::steady_clock::now();

  DetectionResult result;
  result.detections = std::move(detections);
  result.preprocess_ms = elapsedMs(t0, t1);
  result.inference_ms = elapsedMs(t1, t2);
  result.postprocess_ms = elapsedMs(t2, t3);
  return result;
}

}  // namespace semantic_detection
