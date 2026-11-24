#pragma once

#include <iomanip>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <taskgraph/NodeData.hpp>

namespace pg {

/** \brief 图里传递的 4x4 变换矩阵。 */
class TransformData final : public tg::NodeData {
 public:
  TransformData() = default;
  explicit TransformData(const Eigen::Matrix4f& matrix) : _matrix(matrix) {}

  tg::NodeDataType type() const override {
    return tg::NodeDataType{"transform", "Transform"};
  }

  const Eigen::Matrix4f& matrix() const { return _matrix; }
  void setMatrix(const Eigen::Matrix4f& matrix) { _matrix = matrix; }

  /** \brief 日志用摘要：平移、旋转角（度）、旋转轴。只读矩阵。 */
  std::string debugSummary() const override {
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kRadiansToDegrees = 180.0f / kPi;
    constexpr int kTranslationDecimals = 2;
    constexpr int kAngleDecimals = 1;
    constexpr int kAxisDecimals = 2;
    const Eigen::AngleAxisf rotation(_matrix.block<3, 3>(0, 0));
    const float angle_degrees = rotation.angle() * kRadiansToDegrees;
    std::ostringstream text;
    text << std::fixed << std::setprecision(kTranslationDecimals)
         << "Transform(t=(" << _matrix(0, 3) << ',' << _matrix(1, 3) << ','
         << _matrix(2, 3) << "), angle=" << std::setprecision(kAngleDecimals)
         << angle_degrees << "deg, axis=(" << std::setprecision(kAxisDecimals)
         << rotation.axis().x() << ',' << rotation.axis().y() << ','
         << rotation.axis().z() << "))";
    return text.str();
  }

 private:
  Eigen::Matrix4f _matrix = Eigen::Matrix4f::Identity();
};

}  // namespace pg
