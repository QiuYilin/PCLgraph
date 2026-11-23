#include "pclgraph/nodes/nodes.hpp"

#include <memory>
#include <utility>

#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/shadowpoints.h>
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

struct ShadowParams {
  pcl::PointCloud<pcl::Normal>::ConstPtr normals;
  float threshold = 0.0f;
  bool keep_organized = false;
};

template <typename PointT>
std::shared_ptr<CloudData> ShadowOp(const CloudData& input,
                                    const void* raw_params) {
  const auto* params = static_cast<const ShadowParams*>(raw_params);
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError("ShadowPoints: input point type mismatch");
  }
  if (params->normals == nullptr ||
      params->normals->size() != cloud->size()) {
    throw tg::NodeError("ShadowPoints: normals size does not match input");
  }
  auto normals =
      std::const_pointer_cast<pcl::PointCloud<pcl::Normal>>(params->normals);
  auto output = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::ShadowPoints<PointT, pcl::Normal> filter;
  filter.setInputCloud(cloud);
  filter.setNormals(normals);
  filter.setThreshold(params->threshold);
  filter.setKeepOrganized(params->keep_organized);
  filter.filter(*output);
  return Wrap(std::shared_ptr<const void>(std::move(output)), input.desc());
}

struct CovarianceParams {
  pcl::PointCloud<pcl::Normal>::ConstPtr normals;
  unsigned int samples = 5;
};

template <typename PointT>
std::shared_ptr<CloudData> CovarianceOp(const CloudData& input,
                                        const void* raw_params) {
  const auto* params = static_cast<const CovarianceParams*>(raw_params);
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError("CovarianceSampling: input point type mismatch");
  }
  if (params->normals == nullptr ||
      params->normals->size() != cloud->size()) {
    throw tg::NodeError(
        "CovarianceSampling: normals size does not match input");
  }
  if (params->samples == 0 || params->samples > cloud->size()) {
    throw tg::NodeError(
        "CovarianceSampling: number_samples exceeds input point count");
  }
  auto output = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::CovarianceSampling<PointT, pcl::Normal> filter;
  filter.setInputCloud(cloud);
  filter.setNormals(params->normals);
  filter.setNumberOfSamples(params->samples);
  filter.filter(*output);
  return Wrap(std::shared_ptr<const void>(std::move(output)), input.desc());
}

#define PCL_INSTANTIATE_PG_SHADOW(T) \
  PG_INSTANTIATE("shadow_points", kPointTypeId<T>, &ShadowOp<T>);
#define PCL_INSTANTIATE_PG_COVARIANCE(T) \
  PG_INSTANTIATE("covariance_sampling", kPointTypeId<T>, &CovarianceOp<T>);

struct NormalsFilterRegistrations {
  NormalsFilterRegistrations() {
    PCL_INSTANTIATE(PG_SHADOW, PCL_XYZ_POINT_TYPES)
    PCL_INSTANTIATE(PG_COVARIANCE,
                    (pcl::PointXYZ)(pcl::PointNormal)(pcl::PointXYZRGB)
                    (pcl::PointXYZRGBA)(pcl::PointXYZRGBNormal))
  }
};

const NormalsFilterRegistrations kNormalsFilterRegistrations;

const PointTypeDesc* NormalType() {
  return PointTypeRegistry::Instance().Find(kPointTypeNormal);
}

}  // namespace

ShadowPointsNode::ShadowPointsNode()
    : inport{tg::In<CloudData>{this, kInCloudIndex},
             tg::In<CloudData>{this, kInNormalsIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("shadow_points");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  _threshold = addProperty("threshold", 0.0,
                           DoubleHints(-1.0e9, 1.0e9, 0.01, 6));
  _keep_organized = addProperty("keep_organized", false);
}

unsigned int ShadowPointsNode::nFixedPorts(tg::PortType port_type) const {
  if (port_type == tg::PortType::In) {
    return 2u;
  }
  if (port_type == tg::PortType::Out) {
    return 1u;
  }
  return 0u;
}

const PointTypeDesc* ShadowPointsNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void ShadowPointsNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType ShadowPointsNode::fixedDataType(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return DataTypeOf(index == kInNormalsIndex ? NormalType()
                                               : SelectedType());
  }
  if (port_type == tg::PortType::Out) {
    return DataTypeOf(SelectedType());
  }
  return {};
}

const char* ShadowPointsNode::fixedPortId(tg::PortType port_type,
                                          unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return index == kInNormalsIndex ? "normals" : "cloud";
  }
  if (port_type == tg::PortType::Out) {
    return "cloud";
  }
  return "";
}

int ShadowPointsNode::portTypeProperty(tg::PortType port_type,
                                       unsigned int index) const {
  if (port_type == tg::PortType::In && index == kInNormalsIndex) {
    return -1;
  }
  if ((port_type == tg::PortType::In && index == kInCloudIndex) ||
      port_type == tg::PortType::Out) {
    return _point_type_index;
  }
  return -1;
}

void ShadowPointsNode::onSetInPortData(
    unsigned int index, std::shared_ptr<tg::NodeData> data) {
  if (index == kInCloudIndex) {
    _in_cloud = std::move(data);
  } else if (index == kInNormalsIndex) {
    _in_normals = std::move(data);
  }
}

void ShadowPointsNode::compute() {
  const auto cloud_data = _in_cloud.lock();
  const auto normals_data = _in_normals.lock();
  if (cloud_data == nullptr) {
    throw tg::NodeError("ShadowPoints: missing cloud input");
  }
  if (normals_data == nullptr) {
    throw tg::NodeError("ShadowPoints: missing normals input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(cloud_data);
  const auto normals = std::dynamic_pointer_cast<CloudData>(normals_data);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError("ShadowPoints: cloud input is not a point cloud");
  }
  if (normals == nullptr || normals->desc() == nullptr ||
      normals->desc()->id != kPointTypeNormal) {
    throw tg::NodeError("ShadowPoints: normals input must be a Normal cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError("ShadowPoints: input point type mismatch");
  }
  ShadowParams params;
  params.normals = normals->as<pcl::Normal>();
  if (params.normals == nullptr) {
    throw tg::NodeError("ShadowPoints: normals input must be a Normal cloud");
  }
  params.threshold = static_cast<float>(std::get<double>(_threshold->value));
  params.keep_organized = std::get<bool>(_keep_organized->value);
  _out = cloud->dispatch("shadow_points", &params);
}

CovarianceSamplingNode::CovarianceSamplingNode()
    : inport{tg::In<CloudData>{this, kInCloudIndex},
             tg::In<CloudData>{this, kInNormalsIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("covariance_sampling");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  _number_samples = addProperty("number_samples", 5);
}

unsigned int CovarianceSamplingNode::nFixedPorts(
    tg::PortType port_type) const {
  if (port_type == tg::PortType::In) {
    return 2u;
  }
  if (port_type == tg::PortType::Out) {
    return 1u;
  }
  return 0u;
}

const PointTypeDesc* CovarianceSamplingNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void CovarianceSamplingNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType CovarianceSamplingNode::fixedDataType(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return DataTypeOf(index == kInNormalsIndex ? NormalType()
                                               : SelectedType());
  }
  if (port_type == tg::PortType::Out) {
    return DataTypeOf(SelectedType());
  }
  return {};
}

const char* CovarianceSamplingNode::fixedPortId(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return index == kInNormalsIndex ? "normals" : "cloud";
  }
  if (port_type == tg::PortType::Out) {
    return "cloud";
  }
  return "";
}

int CovarianceSamplingNode::portTypeProperty(tg::PortType port_type,
                                             unsigned int index) const {
  if (port_type == tg::PortType::In && index == kInNormalsIndex) {
    return -1;
  }
  if ((port_type == tg::PortType::In && index == kInCloudIndex) ||
      port_type == tg::PortType::Out) {
    return _point_type_index;
  }
  return -1;
}

void CovarianceSamplingNode::onSetInPortData(
    unsigned int index, std::shared_ptr<tg::NodeData> data) {
  if (index == kInCloudIndex) {
    _in_cloud = std::move(data);
  } else if (index == kInNormalsIndex) {
    _in_normals = std::move(data);
  }
}

void CovarianceSamplingNode::compute() {
  const auto cloud_data = _in_cloud.lock();
  const auto normals_data = _in_normals.lock();
  if (cloud_data == nullptr) {
    throw tg::NodeError("CovarianceSampling: missing cloud input");
  }
  if (normals_data == nullptr) {
    throw tg::NodeError("CovarianceSampling: missing normals input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(cloud_data);
  const auto normals = std::dynamic_pointer_cast<CloudData>(normals_data);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError(
        "CovarianceSampling: cloud input is not a point cloud");
  }
  if (normals == nullptr || normals->desc() == nullptr ||
      normals->desc()->id != kPointTypeNormal) {
    throw tg::NodeError(
        "CovarianceSampling: normals input must be a Normal cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError("CovarianceSampling: input point type mismatch");
  }
  CovarianceParams params;
  params.normals = normals->as<pcl::Normal>();
  if (params.normals == nullptr) {
    throw tg::NodeError(
        "CovarianceSampling: normals input must be a Normal cloud");
  }
  const int samples = std::get<int>(_number_samples->value);
  if (samples <= 0) {
    throw tg::NodeError(
        "CovarianceSampling: number_samples must be positive");
  }
  params.samples = static_cast<unsigned int>(samples);
  _out = cloud->dispatch("covariance_sampling", &params);
}

}  // namespace pg
