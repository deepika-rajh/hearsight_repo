#include "semantic_detection/utils/ImageUtils.hpp"

#include <cmath>

#include <opencv2/imgproc.hpp>

namespace semantic_detection {

cv::Mat letterbox(const cv::Mat & image, int input_size, LetterboxInfo & info) {
  const float scale =
      std::min(static_cast<float>(input_size) / image.cols, static_cast<float>(input_size) / image.rows);
  const int scaled_w = static_cast<int>(std::round(image.cols * scale));
  const int scaled_h = static_cast<int>(std::round(image.rows * scale));

  cv::Mat resized;
  cv::resize(image, resized, cv::Size(scaled_w, scaled_h), 0, 0, cv::INTER_LINEAR);

  info.scale = scale;
  info.pad_x = (input_size - scaled_w) / 2;
  info.pad_y = (input_size - scaled_h) / 2;

  cv::Mat padded(input_size, input_size, image.type(), cv::Scalar(114, 114, 114));
  resized.copyTo(padded(cv::Rect(info.pad_x, info.pad_y, scaled_w, scaled_h)));
  return padded;
}

}  // namespace semantic_detection
