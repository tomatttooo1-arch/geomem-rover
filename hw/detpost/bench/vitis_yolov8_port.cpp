// AMD Vitis AI Library 的 YOLOv8 后处理，按原样移植成不依赖 VART 的独立函数，只作 A53 计时对照。
// 来源：https://github.com/Xilinx/Vitis-AI/blob/v3.5/src/vai_library/xnnpp/src/yolov8.cpp
//       与 .../xnnpp/src/apply_nms.cpp（Apache-2.0，Copyright 2022-2023 Advanced Micro Devices Inc.）
// 只改了三处，其余逐行保留（包括 vector 的逐格分配、对全部格子先做 DFL 再判阈值、int8 阈值截断）：
//   1. 原代码 output_dim 写死 144（80 类）；这里改成 64+nc。
//   2. 输入从 vitis::ai::library::OutputTensor 改成「缓冲起点 + 三个头的偏移」，尺度顺序 P3/P4/P5。
//   3. 删掉调试打印与 profiling 宏；坐标映射回原图的一步保留（scale=1、padding=0）。
// 与 ultralytics 的语义差别（不影响计时，影响结果，README §2.2 已列）：
//   每个超过阈值的类别都出一个框（多标签），不是取最大类别；阈值换算 (int8)(logit/scale) 向零截断，
//   负阈值时比精确值高一档；NMS 抑制条件是 IoU >= 阈值。
#include <math.h>
#include <stdint.h>
#include <algorithm>
#include <functional>
#include <numeric>
#include <utility>
#include <vector>

namespace vaiport {
using namespace std;

static vector<float> softmax(const std::vector<float>& input) {
  auto output = std::vector<float>(input.size());
  std::transform(input.begin(), input.end(), output.begin(), expf);
  auto sum = accumulate(output.begin(), output.end(), 0.0f, std::plus<float>());
  std::transform(output.begin(), output.end(), output.begin(), [sum](float v) { return v / sum; });
  return output;
}

static vector<float> conv(const vector<vector<float>>& input) {
  vector<float> output(4, 0.0f);
  for (int row = 0; row < 4; row++)
    for (int col = 0; col < 16; col++) output[row] += input[row][col] * col;
  return output;
}

static vector<vector<float>> make_anchors(int w, int h) {
  vector<vector<float>> anchor_points;
  anchor_points.reserve(w * h);
  for (int i = 0; i < w; ++i) {
    float sy = i + 0.5f;
    for (int j = 0; j < h; ++j) {
      float sx = j + 0.5f;
      vector<float> anchor(2);
      anchor[0] = sx;
      anchor[1] = sy;
      anchor_points.emplace_back(anchor);
    }
  }
  return anchor_points;
}

static vector<float> dist2bbox(const vector<float>& distance, const vector<float>& point, const float stride) {
  vector<float> box;
  box.resize(4);
  float x1 = point[0] - distance[0];
  float y1 = point[1] - distance[1];
  float x2 = point[0] + distance[2];
  float y2 = point[1] + distance[3];
  box[0] = (x1 + x2) / 2.0f * stride;
  box[1] = (y1 + y2) / 2.0f * stride;
  box[2] = (x2 - x1) * stride;
  box[3] = (y2 - y1) * stride;
  return box;
}

static float overlap(float x1, float w1, float x2, float w2) {
  float left = max(x1 - w1 / 2.0, x2 - w2 / 2.0);
  float right = min(x1 + w1 / 2.0, x2 + w2 / 2.0);
  return right - left;
}

static float cal_iou(vector<float> box, vector<float> truth) {
  float w = overlap(box[0], box[2], truth[0], truth[2]);
  float h = overlap(box[1], box[3], truth[1], truth[3]);
  if (w < 0 || h < 0) return 0;
  float inter_area = w * h;
  float union_area = box[2] * box[3] + truth[2] * truth[3] - inter_area;
  return inter_area * 1.0 / union_area;
}

static void applyNMS(const vector<vector<float>>& boxes, const vector<float>& scores, const float nms,
                     const float conf, vector<size_t>& res, bool stable) {
  const size_t count = boxes.size();
  vector<pair<float, size_t>> order;
  for (size_t i = 0; i < count; ++i) order.push_back({scores[i], i});
  if (stable)
    stable_sort(order.begin(), order.end(),
                [](const pair<float, size_t>& ls, const pair<float, size_t>& rs) { return ls.first > rs.first; });
  else
    sort(order.begin(), order.end(),
         [](const pair<float, size_t>& ls, const pair<float, size_t>& rs) { return ls.first > rs.first; });
  vector<size_t> ordered;
  transform(order.begin(), order.end(), back_inserter(ordered), [](auto& km) { return km.second; });
  vector<bool> exist_box(count, true);
  for (size_t _i = 0; _i < count; ++_i) {
    size_t i = ordered[_i];
    if (!exist_box[i]) continue;
    if (scores[i] < conf) { exist_box[i] = false; continue; }
    res.push_back(i);
    for (size_t _j = _i + 1; _j < count; ++_j) {
      size_t j = ordered[_j];
      if (!exist_box[j]) continue;
      float ovr = cal_iou(boxes[j], boxes[i]);
      if (ovr >= nms) exist_box[j] = false;
    }
  }
}

// 返回框表：每个元素 {x1, y1, x2, y2, label, score}（letterbox 像素）
vector<vector<float>> yolov8_post_process(const int8_t* buf, const uint32_t offset[3], int nc, int fix_point,
                                          float conf_thresh, float nms_thresh, size_t max_boxes_num,
                                          size_t max_nms_num) {
  const int num_classes = nc;
  const float stride[3] = {8, 16, 32};
  const int grid[3] = {80, 40, 20};
  const int output_dim = 64 + nc;  // 原代码：144（80 + 16 * 4）
  vector<vector<float>> boxes;
  for (int i = 0; i < 3; i++) {
    int width = grid[i], height = grid[i];
    auto anchor_points = make_anchors(height, width);
    int sizeOut = width * height;
    boxes.reserve(boxes.size() + sizeOut);
    const int8_t* det_out = buf + offset[i];
    float det_scale = ldexpf(1.0f, -fix_point);
    int8_t conf_thresh_inverse = -std::log(1.0f / conf_thresh - 1) / det_scale;
    for (auto n = 0; n < sizeOut; ++n) {
      vector<vector<float>> pre_output_unit;
      pre_output_unit.resize(4);
      for (auto t = 0; t < 4; t++) {
        vector<float> softmax_;
        softmax_.reserve(16);
        for (auto m = 0; m < 16; m++) softmax_.emplace_back(det_out[n * output_dim + t * 16 + m] * det_scale);
        pre_output_unit[t] = softmax(softmax_);
      }
      auto distance = conv(pre_output_unit);
      auto dbox = dist2bbox(distance, anchor_points[n], stride[i]);
      for (auto m = 0; m < num_classes; ++m) {
        if (det_out[n * output_dim + 64 + m] > conf_thresh_inverse) {
          vector<float> box(6);
          for (int j = 0; j < 4; j++) box[j] = dbox[j];
          float cls_score = 1.0 / (1 + exp(-1.0f * det_out[n * output_dim + 64 + m] * det_scale));
          box[4] = m;
          box[5] = cls_score;
          boxes.emplace_back(box);
        }
      }
    }
  }
  auto compare = [=](vector<float>& lhs, vector<float>& rhs) { return lhs[5] > rhs[5]; };
  if (boxes.size() > max_boxes_num) {
    std::partial_sort(boxes.begin(), boxes.begin() + max_boxes_num, boxes.end(), compare);
    boxes.resize(max_boxes_num);
  } else {
    std::sort(boxes.begin(), boxes.end(), compare);
  }
  vector<vector<vector<float>>> boxes_for_nms(num_classes);
  vector<vector<float>> scores(num_classes);
  for (const auto& box : boxes) {
    boxes_for_nms[box[4]].push_back(box);
    scores[box[4]].push_back(box[5]);
  }
  vector<vector<float>> res;
  for (auto i = 0; i < num_classes; i++) {
    vector<size_t> result_k;
    applyNMS(boxes_for_nms[i], scores[i], nms_thresh, 0, result_k, true);
    res.reserve(res.size() + result_k.size());
    transform(result_k.begin(), result_k.end(), back_inserter(res), [&](auto& k) { return boxes_for_nms[i][k]; });
  }
  if (res.size() > max_nms_num) {
    std::partial_sort(res.begin(), res.begin() + max_nms_num, res.end(), compare);
    res.resize(max_nms_num);
  } else {
    std::sort(res.begin(), res.end(), compare);
  }
  vector<vector<float>> results;
  for (const auto& r : res) {
    vector<float> b(6);
    b[0] = r[0] - r[2] / 2.0f;
    b[1] = r[1] - r[3] / 2.0f;
    b[2] = b[0] + r[2];
    b[3] = b[1] + r[3];
    b[4] = r[4];
    b[5] = r[5];
    results.push_back(b);
  }
  return results;
}

}  // namespace vaiport
