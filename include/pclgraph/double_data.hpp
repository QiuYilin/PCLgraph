#pragma once

#include <taskgraph/NodeData.hpp>

namespace pg {

/** \brief 图里传递的单个 double。 */
class DoubleData final : public tg::NodeData {
 public:
  DoubleData() = default;
  explicit DoubleData(double value) : _value(value) {}

  tg::NodeDataType type() const override {
    return tg::NodeDataType{"double", "Double"};
  }

  double value() const { return _value; }
  void setValue(double value) { _value = value; }

 private:
  double _value = 0.0;
};

}  // namespace pg
