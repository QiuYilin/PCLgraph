#pragma once

#include <Eigen/Core>

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

 private:
  Eigen::Matrix4f _matrix = Eigen::Matrix4f::Identity();
};

}  // namespace pg
