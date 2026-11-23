#include "pclgraph/nodes/nodes.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <pcl/common/point_tests.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <taskgraph/Error.hpp>

namespace pg {
namespace {

const PointTypeDesc* Desc(const char* id) {
  return PointTypeRegistry::Instance().Find(id);
}

}  // namespace

CalcModelResoNode::CalcModelResoNode()
    : inport{tg::In<CloudData>{this, kInIndex}},
      outport{tg::Out<DoubleData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _k = addProperty("k", 10);
}

unsigned int CalcModelResoNode::nFixedPorts(
    tg::PortType port_type) const {
  if (port_type == tg::PortType::In) {
    return 1u;
  }
  if (port_type == tg::PortType::Out) {
    return 1u;
  }
  return 0u;
}

tg::NodeDataType CalcModelResoNode::fixedDataType(
    tg::PortType port_type, unsigned int) const {
  if (port_type == tg::PortType::In) {
    return DataTypeOf(Desc(kPointTypeXyz));
  }
  if (port_type == tg::PortType::Out) {
    return DoubleData{}.type();
  }
  return {};
}

const char* CalcModelResoNode::fixedPortId(tg::PortType port_type,
                                           unsigned int) const {
  if (port_type == tg::PortType::In) {
    return "cloud";
  }
  if (port_type == tg::PortType::Out) {
    return "resolution";
  }
  return "";
}

int CalcModelResoNode::portTypeProperty(tg::PortType,
                                        unsigned int) const {
  return -1;
}

void CalcModelResoNode::onSetInPortData(
    unsigned int index, std::shared_ptr<tg::NodeData> data) {
  if (index == kInIndex) {
    _in = std::move(data);
  }
}

void CalcModelResoNode::compute() {
  const auto input_data = _in.lock();
  if (input_data == nullptr) {
    throw tg::NodeError("CalcModelReso: missing input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(input_data);
  if (cloud == nullptr || cloud->desc() == nullptr ||
      cloud->desc()->id != kPointTypeXyz) {
    throw tg::NodeError("CalcModelReso: input must be a PointXYZ cloud");
  }
  const auto src = cloud->as<pcl::PointXYZ>();
  if (src == nullptr || src->empty()) {
    throw tg::NodeError("CalcModelReso: input cloud is empty");
  }

  const int k = std::max(2, std::get<int>(_k->value));
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(src);
  pcl::Indices indices;
  std::vector<float> sqr_distances;

  double sum = 0.0;
  int count = 0;
  for (std::size_t i = 0; i < src->size(); ++i) {
    if (!pcl::isFinite(src->at(i))) {
      continue;
    }
    const int found = tree.nearestKSearch(src->at(i), k, indices,
                                          sqr_distances);
    if (found <= 0) {
      continue;
    }
    for (const float distance : sqr_distances) {
      if (distance != 0.0f) {
        sum += distance;
        break;
      }
    }
    ++count;
  }

  const double resolution =
      count > 0 ? std::sqrt(sum / static_cast<double>(count)) : 0.0;
  _value_out = std::make_shared<DoubleData>(resolution);
}

}  // namespace pg
