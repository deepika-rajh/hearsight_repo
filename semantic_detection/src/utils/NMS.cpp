#include "semantic_detection/utils/NMS.hpp"

#include <map>

#include <opencv2/dnn.hpp>

namespace semantic_detection {

std::vector<int> perClassNMS(const std::vector<cv::Rect> & boxes, const std::vector<float> & scores,
                             const std::vector<int> & class_ids, float score_threshold,
                             float nms_threshold) {
  // cv::dnn::NMSBoxesBatched would do this in one call but isn't available
  // in every OpenCV version this package targets, so group by class here.
  std::map<int, std::vector<int>> indices_by_class;
  for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
    indices_by_class[class_ids[i]].push_back(i);
  }

  std::vector<int> keep;
  for (const auto & [class_id, indices] : indices_by_class) {
    std::vector<cv::Rect> class_boxes;
    std::vector<float> class_scores;
    class_boxes.reserve(indices.size());
    class_scores.reserve(indices.size());
    for (int i : indices) {
      class_boxes.push_back(boxes[i]);
      class_scores.push_back(scores[i]);
    }
    std::vector<int> class_keep;
    cv::dnn::NMSBoxes(class_boxes, class_scores, score_threshold, nms_threshold, class_keep);
    for (int k : class_keep) {
      keep.push_back(indices[k]);
    }
  }
  return keep;
}

}  // namespace semantic_detection
