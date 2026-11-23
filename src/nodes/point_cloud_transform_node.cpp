#include "pclgraph/nodes/nodes.hpp"

#include <memory>
#include <utility>

#include <pcl/common/transforms.h>
#include <pcl/impl/instantiate.hpp>
#include <taskgraph/Error.hpp>

#include "pclgraph/algorithm_registry.hpp"

namespace pg {
namespace {

std::shared_ptr<CloudData> Wrap(std::shared_ptr<const void> cloud,
                                const PointTypeDesc* desc) {
  return std::make_shared<CloudData>(std::move(cloud), desc);
}

struct TransformParams {
  const Eigen::Matrix4f* matrix = nullptr;
};

template <typename PointT>
std::shared_ptr<CloudData> TransformOp(const CloudData& input,
                                       const void* raw_params) {
  const auto* params = static_cast<const TransformParams*>(raw_params);
  const auto src = input.as<PointT>();
  if (src == nullptr) {
    throw tg::NodeError("TransformPointCloud: point type mismatch");
  }
  if (params->matrix == nullptr) {
    throw tg::NodeError("TransformPointCloud: missing transform");
  }
  auto output = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::transformPointCloud(*src, *output, *params->matrix);
  return Wrap(std::shared_ptr<const void>(std::move(output)), input.desc());
}

#define PCL_INSTANTIATE_PG_TRANSFORM(T) \
  PG_INSTANTIATE("transform_point_cloud", kPointTypeId<T>, &TransformOp<T>);

struct TransformRegistrations {
  TransformRegistrations() {
    PCL_INSTANTIATE(PG_TRANSFORM, PCL_XYZ_POINT_TYPES)
  }
};

const TransformRegistrations kTransformRegistrations;

}  // namespace

TransformPointCloudNode::TransformPointCloudNode()
    : inport{tg::In<CloudData>{this, kInCloudIndex},
             tg::In<TransformData>{this, kInTransformIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("transform_point_cloud");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
}

unsigned int TransformPointCloudNode::nFixedPorts(
    tg::PortType port_type) const {
  if (port_type == tg::PortType::In) {
    return 2u;
  }
  if (port_type == tg::PortType::Out) {
    return 1u;
  }
  return 0u;
}

const PointTypeDesc* TransformPointCloudNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void TransformPointCloudNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType TransformPointCloudNode::fixedDataType(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    if (index == kInTransformIndex) {
      return tg::NodeDataType{"transform", "Transform"};
    }
    return DataTypeOf(SelectedType());
  }
  if (port_type == tg::PortType::Out) {
    return DataTypeOf(SelectedType());
  }
  return {};
}

const char* TransformPointCloudNode::fixedPortId(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return index == kInTransformIndex ? "transform" : "cloud";
  }
  if (port_type == tg::PortType::Out) {
    return "cloud";
  }
  return "";
}

int TransformPointCloudNode::portTypeProperty(tg::PortType port_type,
                                              unsigned int index) const {
  if (port_type == tg::PortType::In && index == kInCloudIndex) {
    return _point_type_index;
  }
  if (port_type == tg::PortType::Out) {
    return _point_type_index;
  }
  return -1;
}

void TransformPointCloudNode::onSetInPortData(
    unsigned int index, std::shared_ptr<tg::NodeData> data) {
  if (index == kInCloudIndex) {
    _in_cloud = std::move(data);
  } else if (index == kInTransformIndex) {
    _in_transform = std::move(data);
  }
}

void TransformPointCloudNode::compute() {
  const auto cloud_data = _in_cloud.lock();
  const auto transform_data = _in_transform.lock();
  if (cloud_data == nullptr) {
    throw tg::NodeError("TransformPointCloud: missing cloud input");
  }
  if (transform_data == nullptr) {
    throw tg::NodeError("TransformPointCloud: missing transform input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(cloud_data);
  const auto transform =
      std::dynamic_pointer_cast<TransformData>(transform_data);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError("TransformPointCloud: input is not a point cloud");
  }
  if (transform == nullptr) {
    throw tg::NodeError(
        "TransformPointCloud: input is not a Transform");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError("TransformPointCloud: input point type mismatch");
  }
  TransformParams params;
  params.matrix = &transform->matrix();
  _out = cloud->dispatch("transform_point_cloud", &params);
}

}  // namespace pg
