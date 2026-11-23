#include "pclgraph/nodes/nodes.hpp"

#include <memory>
#include <utility>

#include <taskgraph/Error.hpp>

namespace pg {

PointCloudConvertNode::PointCloudConvertNode()
    : inport{tg::In<CloudData>{this, kInIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = AllTypes();
  _input_type = addProperty("input_type", 0,
                            PointTypeHints(_types, false));
  _input_type_index = static_cast<int>(getProperties().size()) - 1;
  _output_type = addProperty("output_type", 1,
                             PointTypeHints(_types, false));
  _output_type_index = static_cast<int>(getProperties().size()) - 1;
}

const PointTypeDesc* PointCloudConvertNode::InputType() const {
  return PointTypeAt(std::get<int>(_input_type->value), _types, false);
}

const PointTypeDesc* PointCloudConvertNode::OutputType() const {
  return PointTypeAt(std::get<int>(_output_type->value), _types, false);
}

tg::NodeDataType PointCloudConvertNode::fixedDataType(
    tg::PortType port_type, unsigned int) const {
  if (port_type == tg::PortType::In) {
    return DataTypeOf(InputType());
  }
  if (port_type == tg::PortType::Out) {
    return DataTypeOf(OutputType());
  }
  return {};
}

int PointCloudConvertNode::portTypeProperty(tg::PortType port_type,
                                            unsigned int index) const {
  if (port_type == tg::PortType::In && index == kInIndex) {
    return _input_type_index;
  }
  if (port_type == tg::PortType::Out && index == kOutIndex) {
    return _output_type_index;
  }
  return -1;
}

const char* PointCloudConvertNode::fixedPortId(tg::PortType port_type,
                                               unsigned int) const {
  if (port_type == tg::PortType::In) {
    return "in";
  }
  if (port_type == tg::PortType::Out) {
    return "out";
  }
  return "";
}

void PointCloudConvertNode::compute() {
  const auto input = _in.lock();
  if (input == nullptr) {
    throw tg::NodeError("PointCloudConvert: missing input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(input);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError("PointCloudConvert: input is not a point cloud");
  }
  const PointTypeDesc* input_type = InputType();
  const PointTypeDesc* output_type = OutputType();
  if (input_type == nullptr || output_type == nullptr) {
    throw tg::NodeError("PointCloudConvert: point type is not selected");
  }
  if (cloud->desc()->id != input_type->id) {
    throw tg::NodeError("PointCloudConvert: input point type mismatch");
  }
  if (input_type->id == output_type->id) {
    _out = std::make_shared<CloudData>(cloud->cloud(), output_type);
    return;
  }

  pcl::PCLPointCloud2 blob;
  input_type->to_cloud2(cloud->cloud(), blob);
  auto converted = output_type->from_cloud2(blob);
  if (converted == nullptr) {
    throw tg::NodeError("PointCloudConvert: conversion failed");
  }
  _out = std::make_shared<CloudData>(std::move(converted), output_type);
}

}  // namespace pg
