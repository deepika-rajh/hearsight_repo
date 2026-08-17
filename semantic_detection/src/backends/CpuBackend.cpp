#include "semantic_detection/backends/CpuBackend.hpp"

#include <stdexcept>

namespace semantic_detection {

CpuBackend::CpuBackend(int input_size) : input_size_(input_size) {}

void CpuBackend::loadModel(const std::string & model_path) {
  net_ = cv::dnn::readNetFromONNX(model_path);
  if (net_.empty()) {
    throw std::runtime_error("CpuBackend: failed to load ONNX model at " + model_path);
  }
  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

RawOutput CpuBackend::infer(const cv::Mat & letterboxed_bgr) {
  cv::Mat blob = cv::dnn::blobFromImage(
      letterboxed_bgr, 1.0 / 255.0, cv::Size(input_size_, input_size_), cv::Scalar(), true, false);
  net_.setInput(blob);
  cv::Mat out = net_.forward();

  // Ultralytics YOLOv8 ONNX export shape is [1, 4+num_classes, num_boxes].
  // out.size may report 3 dims (1, C, N); reshape to a flat 2D [C x N] view.
  if (out.dims == 3) {
    out = out.reshape(1, {out.size[1], out.size[2]});
  }

  RawOutput result;
  result.num_channels = out.rows;
  result.num_boxes = out.cols;
  result.data.assign(reinterpret_cast<float *>(out.data),
                      reinterpret_cast<float *>(out.data) + out.total());
  return result;
}

}  // namespace semantic_detection
