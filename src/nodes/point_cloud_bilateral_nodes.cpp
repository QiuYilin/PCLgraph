#include "pclgraph/nodes/nodes.hpp"

#include <algorithm>
#include <memory>
#include <utility>

#include <pcl/filters/bilateral.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/impl/instantiate.hpp>
#include <taskgraph/Error.hpp>

#include "pclgraph/algorithm_registry.hpp"

namespace pg {
namespace {

tg::PropertyHints DoubleHints(double min_value, double max_value, double step,
                              int decimals) {
  tg::PropertyHints hints;
  hints.editor = tg::PropertyEditor::Double;
  hints.min = min_value;
  hints.max = max_value;
  hints.step = step;
  hints.decimals = decimals;
  return hints;
}

std::shared_ptr<CloudData> Wrap(std::shared_ptr<const void> cloud,
                                const PointTypeDesc* desc) {
  return std::make_shared<CloudData>(std::move(cloud), desc);
}

struct BilateralParams {
  double sigma_s = 15.0;
  double sigma_r = 0.05;
};

template <typename PointT>
std::shared_ptr<CloudData> BilateralOp(const CloudData& input,
                                       const void* raw_params) {
  const auto* params = static_cast<const BilateralParams*>(raw_params);
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError("BilateralFilter: input point type mismatch");
  }
  auto output = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::BilateralFilter<PointT> filter;
  filter.setInputCloud(cloud);
  filter.setHalfSize(params->sigma_s);
  filter.setStdDev(params->sigma_r);
  filter.filter(*output);
  return Wrap(std::shared_ptr<const void>(std::move(output)), input.desc());
}

struct FastBilateralParams {
  double sigma_s = 15.0;
  double sigma_r = 0.05;
  unsigned int threads = 1;
};

template <typename PointT>
std::shared_ptr<CloudData> FastBilateralOp(const CloudData& input,
                                           const void* raw_params) {
  const auto* params = static_cast<const FastBilateralParams*>(raw_params);
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError("FastBilateralFilter: input point type mismatch");
  }
  if (!cloud->isOrganized()) {
    throw tg::NodeError(
        "FastBilateralFilter: input point cloud is not organized");
  }
  auto output = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::FastBilateralFilterOMP<PointT> filter;
  filter.setInputCloud(cloud);
  filter.setSigmaS(params->sigma_s);
  filter.setSigmaR(params->sigma_r);
  filter.setNumberOfThreads(params->threads);
  filter.filter(*output);
  return Wrap(std::shared_ptr<const void>(std::move(output)), input.desc());
}

#define PCL_INSTANTIATE_PG_BILATERAL(T) \
  PG_INSTANTIATE("bilateral_filter", kPointTypeId<T>, &BilateralOp<T>);
#define PCL_INSTANTIATE_PG_FAST_BILATERAL(T) \
  PG_INSTANTIATE("fast_bilateral_filter", kPointTypeId<T>, \
                 &FastBilateralOp<T>);

struct BilateralRegistrations {
  BilateralRegistrations() {
    PCL_INSTANTIATE(PG_BILATERAL, (pcl::PointXYZI)(pcl::PointXYZINormal))
    PCL_INSTANTIATE(PG_FAST_BILATERAL,
                    (pcl::PointXYZ)(pcl::PointXYZRGB)(pcl::PointXYZRGBA))
  }
};

const BilateralRegistrations kBilateralRegistrations;

}  // namespace

BilateralFilterNode::BilateralFilterNode()
    : inport{tg::In<CloudData>{this, kInIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("bilateral_filter");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  _sigma_s = addProperty("sigma_s", 15.0,
                         DoubleHints(0.0, 1.0e9, 0.1, 6));
  _sigma_r = addProperty("sigma_r", 0.05,
                         DoubleHints(0.0, 1.0e9, 0.001, 6));
}

const PointTypeDesc* BilateralFilterNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void BilateralFilterNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType BilateralFilterNode::fixedDataType(
    tg::PortType port_type, unsigned int) const {
  if (port_type != tg::PortType::In && port_type != tg::PortType::Out) {
    return {};
  }
  return DataTypeOf(SelectedType());
}

const char* BilateralFilterNode::fixedPortId(tg::PortType port_type,
                                             unsigned int) const {
  return port_type == tg::PortType::In || port_type == tg::PortType::Out
             ? "cloud"
             : "";
}

int BilateralFilterNode::portTypeProperty(tg::PortType port_type,
                                          unsigned int index) const {
  return (port_type == tg::PortType::In || port_type == tg::PortType::Out) &&
                 index == kInIndex
             ? _point_type_index
             : -1;
}

void BilateralFilterNode::compute() {
  const auto input = _in.lock();
  if (input == nullptr) {
    throw tg::NodeError("BilateralFilter: missing input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(input);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError("BilateralFilter: input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError("BilateralFilter: input point type mismatch");
  }
  BilateralParams params;
  params.sigma_s = std::get<double>(_sigma_s->value);
  params.sigma_r = std::get<double>(_sigma_r->value);
  _out = cloud->dispatch("bilateral_filter", &params);
}

FastBilateralFilterNode::FastBilateralFilterNode()
    : inport{tg::In<CloudData>{this, kInIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("fast_bilateral_filter");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  _sigma_s = addProperty("sigma_s", 15.0,
                         DoubleHints(0.0, 1.0e9, 0.1, 6));
  _sigma_r = addProperty("sigma_r", 0.05,
                         DoubleHints(0.0, 1.0e9, 0.001, 6));
  _threads = addProperty("threads", 1);
}

const PointTypeDesc* FastBilateralFilterNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void FastBilateralFilterNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType FastBilateralFilterNode::fixedDataType(
    tg::PortType port_type, unsigned int) const {
  if (port_type != tg::PortType::In && port_type != tg::PortType::Out) {
    return {};
  }
  return DataTypeOf(SelectedType());
}

const char* FastBilateralFilterNode::fixedPortId(tg::PortType port_type,
                                                 unsigned int) const {
  return port_type == tg::PortType::In || port_type == tg::PortType::Out
             ? "cloud"
             : "";
}

int FastBilateralFilterNode::portTypeProperty(tg::PortType port_type,
                                              unsigned int index) const {
  return (port_type == tg::PortType::In || port_type == tg::PortType::Out) &&
                 index == kInIndex
             ? _point_type_index
             : -1;
}

void FastBilateralFilterNode::compute() {
  const auto input = _in.lock();
  if (input == nullptr) {
    throw tg::NodeError("FastBilateralFilter: missing input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(input);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError("FastBilateralFilter: input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError("FastBilateralFilter: input point type mismatch");
  }
  FastBilateralParams params;
  params.sigma_s = std::get<double>(_sigma_s->value);
  params.sigma_r = std::get<double>(_sigma_r->value);
  params.threads = static_cast<unsigned int>(
      std::max(0, std::get<int>(_threads->value)));
  _out = cloud->dispatch("fast_bilateral_filter", &params);
}

}  // namespace pg
