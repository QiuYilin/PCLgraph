#include "pclgraph/nodes/nodes.hpp"

#include <algorithm>
#include <memory>
#include <utility>

#include <pcl/features/normal_3d.h>
#include <taskgraph/Error.hpp>

#include "pclgraph/algorithm_registry.hpp"

namespace pg {
namespace {

struct NormalParams {
  int k_search = 10;
  double radius_search = 0.0;
};

const PointTypeDesc* NormalTypeFor(const PointTypeDesc* input) {
  if (input == nullptr) {
    return nullptr;
  }
  auto& registry = PointTypeRegistry::Instance();
  if (input->id == kPointTypeXyz || input->id == kPointTypeXyzi ||
      input->id == kPointTypeXyzNormal || input->id == kPointTypeSurfel) {
    return registry.Find(kPointTypeXyzNormal);
  }
  if (input->id == kPointTypeXyzRgb || input->id == kPointTypeXyzRgba) {
    return registry.Find(kPointTypeXyzRgbNormal);
  }
  return nullptr;
}

template <typename PointT, typename OutT>
void CopySharedFields(const pcl::PointCloud<PointT>& input,
                      pcl::PointCloud<OutT>& output) {
  const std::size_t count = std::min(input.size(), output.size());
  for (std::size_t i = 0; i < count; ++i) {
    if constexpr (pcl::traits::has_field<PointT, pcl::fields::x>::value &&
                  pcl::traits::has_field<OutT, pcl::fields::x>::value) {
      output[i].x = input[i].x;
      output[i].y = input[i].y;
      output[i].z = input[i].z;
    }
    if constexpr (pcl::traits::has_field<PointT, pcl::fields::intensity>::value &&
                  pcl::traits::has_field<OutT, pcl::fields::intensity>::value) {
      output[i].intensity = input[i].intensity;
    }
    if constexpr (pcl::traits::has_field<PointT, pcl::fields::rgb>::value &&
                  pcl::traits::has_field<OutT, pcl::fields::rgb>::value) {
      output[i].rgb = input[i].rgb;
    }
    if constexpr (pcl::traits::has_field<PointT, pcl::fields::rgba>::value &&
                  pcl::traits::has_field<OutT, pcl::fields::rgba>::value) {
      output[i].rgba = input[i].rgba;
    }
    if constexpr (pcl::traits::has_field<PointT, pcl::fields::rgba>::value &&
                  !pcl::traits::has_field<OutT, pcl::fields::rgba>::value &&
                  pcl::traits::has_field<OutT, pcl::fields::rgb>::value) {
      output[i].rgb = input[i].rgba;
    }
  }
}

template <typename PointT, typename OutT>
std::shared_ptr<CloudData> NormalEstimationOp(const CloudData& input,
                                              const void* raw_params) {
  const auto* params = static_cast<const NormalParams*>(raw_params);
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError("NormalEstimation: input point type mismatch");
  }
  auto output = std::make_shared<pcl::PointCloud<OutT>>();
  pcl::NormalEstimation<PointT, OutT> estimator;
  estimator.setInputCloud(cloud);
  if (params->radius_search > 0.0) {
    estimator.setRadiusSearch(params->radius_search);
  } else {
    estimator.setKSearch(params->k_search > 0 ? params->k_search : 10);
  }
  estimator.compute(*output);
  CopySharedFields(*cloud, *output);

  const PointTypeDesc* out_desc = NormalTypeFor(input.desc());
  if (out_desc == nullptr) {
    throw tg::NodeError("NormalEstimation: unsupported output point type");
  }
  return std::make_shared<CloudData>(
      std::shared_ptr<const void>(std::move(output)), out_desc);
}

struct FeatureRegistrations {
  FeatureRegistrations() {
    auto& registry = AlgorithmRegistry::Instance();
    registry.Add("normal_estimation", kPointTypeXyz,
                 &NormalEstimationOp<pcl::PointXYZ, pcl::PointNormal>);
    registry.Add("normal_estimation", kPointTypeXyzi,
                 &NormalEstimationOp<pcl::PointXYZI, pcl::PointNormal>);
    registry.Add("normal_estimation", kPointTypeXyzNormal,
                 &NormalEstimationOp<pcl::PointNormal, pcl::PointNormal>);
    registry.Add("normal_estimation", kPointTypeSurfel,
                 &NormalEstimationOp<pcl::PointSurfel, pcl::PointNormal>);
    registry.Add(
        "normal_estimation", kPointTypeXyzRgb,
        &NormalEstimationOp<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>);
    registry.Add(
        "normal_estimation", kPointTypeXyzRgba,
        &NormalEstimationOp<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal>);
  }
};

const FeatureRegistrations kFeatureRegistrations;

}  // namespace

NormalEstimationNode::NormalEstimationNode()
    : inport{tg::In<CloudData>{this, kInIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("normal_estimation");
  _input_type = addProperty("input_type", 0,
                            PointTypeHints(_types, false));
  _input_type_index = static_cast<int>(getProperties().size()) - 1;
  _k_search = addProperty("k_search", 10);
  tg::PropertyHints radius_hints;
  radius_hints.editor = tg::PropertyEditor::Double;
  radius_hints.min = 0.0;
  radius_hints.max = 1.0e6;
  radius_hints.step = 0.01;
  radius_hints.decimals = 4;
  _radius_search = addProperty("radius_search", 0.0, radius_hints);
}

const PointTypeDesc* NormalEstimationNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_input_type->value), _types, false);
}

const PointTypeDesc* NormalEstimationNode::OutputType() const {
  return NormalTypeFor(SelectedType());
}

tg::NodeDataType NormalEstimationNode::fixedDataType(
    tg::PortType port_type, unsigned int) const {
  if (port_type == tg::PortType::In) {
    return DataTypeOf(SelectedType());
  }
  if (port_type == tg::PortType::Out) {
    return DataTypeOf(OutputType());
  }
  return {};
}

int NormalEstimationNode::portTypeProperty(tg::PortType port_type,
                                           unsigned int index) const {
  return port_type == tg::PortType::In && index == kInIndex
             ? _input_type_index
             : -1;
}

const char* NormalEstimationNode::fixedPortId(tg::PortType port_type,
                                              unsigned int) const {
  if (port_type == tg::PortType::In) {
    return "cloud";
  }
  if (port_type == tg::PortType::Out) {
    return "normals";
  }
  return "";
}

void NormalEstimationNode::compute() {
  const auto input = _in.lock();
  if (input == nullptr) {
    throw tg::NodeError("NormalEstimation: missing input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(input);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError("NormalEstimation: input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError("NormalEstimation: input point type mismatch");
  }

  NormalParams params;
  params.k_search = std::get<int>(_k_search->value);
  params.radius_search = std::get<double>(_radius_search->value);
  _out = cloud->dispatch("normal_estimation", &params);
}

}  // namespace pg
