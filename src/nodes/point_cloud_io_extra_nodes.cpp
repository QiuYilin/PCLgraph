#include "pclgraph/nodes/nodes.hpp"

#include <memory>
#include <utility>

#include <pcl/common/io.h>
#include <pcl/impl/instantiate.hpp>
#include <taskgraph/Error.hpp>

#include "pclgraph/algorithm_registry.hpp"

namespace pg {
namespace {

std::shared_ptr<CloudData> Wrap(std::shared_ptr<const void> cloud,
                                const PointTypeDesc* desc) {
  return std::make_shared<CloudData>(std::move(cloud), desc);
}

struct ConcatenateParams {
  pcl::PointCloud<pcl::Normal>::ConstPtr normals;
};

template <typename PointT, typename OutT>
std::shared_ptr<CloudData> ConcatenateOp(const CloudData& input,
                                         const void* raw_params) {
  const auto* params = static_cast<const ConcatenateParams*>(raw_params);
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError("ConcatenateFields: input point type mismatch");
  }
  if (params->normals == nullptr ||
      params->normals->size() != cloud->size()) {
    throw tg::NodeError(
        "ConcatenateFields: normals size does not match input");
  }
  auto output = std::make_shared<pcl::PointCloud<OutT>>();
  pcl::concatenateFields(*cloud, *params->normals, *output);
  const PointTypeDesc* desc =
      PointTypeRegistry::Instance().Find(kPointTypeId<OutT>);
  if (desc == nullptr) {
    throw tg::NodeError(
        "ConcatenateFields: output point type is not registered");
  }
  return Wrap(std::shared_ptr<const void>(std::move(output)), desc);
}

const PointTypeDesc* ConcatenateOutputType(const PointTypeDesc* input) {
  if (input == nullptr) {
    return nullptr;
  }
  const char* id = nullptr;
  if (input->id == kPointTypeXyz) {
    id = kPointTypeXyzNormal;
  } else if (input->id == kPointTypeXyzi) {
    id = kPointTypeXyziNormal;
  } else if (input->id == kPointTypeXyzRgb) {
    id = kPointTypeXyzRgbNormal;
  } else if (input->id == kPointTypeXyzL) {
    id = kPointTypeXyzLNormal;
  }
  return id == nullptr ? nullptr :
                         PointTypeRegistry::Instance().Find(id);
}

struct MergeParams {
  const CloudData* second = nullptr;
};

template <typename PointT>
std::shared_ptr<CloudData> MergeOp(const CloudData& input,
                                   const void* raw_params) {
  const auto* params = static_cast<const MergeParams*>(raw_params);
  const auto first = input.as<PointT>();
  if (first == nullptr) {
    throw tg::NodeError("Merge: input point type mismatch");
  }
  if (params->second == nullptr) {
    throw tg::NodeError("Merge: missing second input");
  }
  const auto second = params->second->as<PointT>();
  if (second == nullptr) {
    throw tg::NodeError("Merge: second point type mismatch");
  }
  pcl::PointCloud<PointT> first_copy = *first;
  auto output =
      std::make_shared<pcl::PointCloud<PointT>>(first_copy + *second);
  return Wrap(std::shared_ptr<const void>(std::move(output)), input.desc());
}

#define PCL_INSTANTIATE_PG_MERGE(T) \
  PG_INSTANTIATE("merge", kPointTypeId<T>, &MergeOp<T>);

struct IoExtraRegistrations {
  IoExtraRegistrations() {
    auto& registry = AlgorithmRegistry::Instance();
    registry.Add("concatenate_fields", kPointTypeXyz,
                 &ConcatenateOp<pcl::PointXYZ, pcl::PointNormal>);
    registry.Add("concatenate_fields", kPointTypeXyzi,
                 &ConcatenateOp<pcl::PointXYZI, pcl::PointXYZINormal>);
    registry.Add(
        "concatenate_fields", kPointTypeXyzRgb,
        &ConcatenateOp<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>);
    registry.Add(
        "concatenate_fields", kPointTypeXyzL,
        &ConcatenateOp<pcl::PointXYZL, pcl::PointXYZLNormal>);
    PCL_INSTANTIATE(PG_MERGE, PCL_XYZ_POINT_TYPES)
  }
};

const IoExtraRegistrations kIoExtraRegistrations;

const PointTypeDesc* NormalType() {
  return PointTypeRegistry::Instance().Find(kPointTypeNormal);
}

}  // namespace

ConcatenateFieldsNode::ConcatenateFieldsNode()
    : inport{tg::In<CloudData>{this, kInCloudIndex},
             tg::In<CloudData>{this, kInNormalsIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("concatenate_fields");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
}

unsigned int ConcatenateFieldsNode::nFixedPorts(
    tg::PortType port_type) const {
  if (port_type == tg::PortType::In) {
    return 2u;
  }
  if (port_type == tg::PortType::Out) {
    return 1u;
  }
  return 0u;
}

const PointTypeDesc* ConcatenateFieldsNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

const PointTypeDesc* ConcatenateFieldsNode::OutputType() const {
  return ConcatenateOutputType(SelectedType());
}

void ConcatenateFieldsNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType ConcatenateFieldsNode::fixedDataType(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return DataTypeOf(index == kInNormalsIndex ? NormalType()
                                               : SelectedType());
  }
  if (port_type == tg::PortType::Out) {
    return DataTypeOf(OutputType());
  }
  return {};
}

const char* ConcatenateFieldsNode::fixedPortId(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return index == kInNormalsIndex ? "normals" : "cloud";
  }
  if (port_type == tg::PortType::Out) {
    return "cloud";
  }
  return "";
}

int ConcatenateFieldsNode::portTypeProperty(tg::PortType port_type,
                                            unsigned int index) const {
  if (port_type == tg::PortType::In && index == kInCloudIndex) {
    return _point_type_index;
  }
  return -1;
}

void ConcatenateFieldsNode::onSetInPortData(
    unsigned int index, std::shared_ptr<tg::NodeData> data) {
  if (index == kInCloudIndex) {
    _in_cloud = std::move(data);
  } else if (index == kInNormalsIndex) {
    _in_normals = std::move(data);
  }
}

void ConcatenateFieldsNode::compute() {
  const auto cloud_data = _in_cloud.lock();
  const auto normals_data = _in_normals.lock();
  if (cloud_data == nullptr) {
    throw tg::NodeError("ConcatenateFields: missing cloud input");
  }
  if (normals_data == nullptr) {
    throw tg::NodeError("ConcatenateFields: missing normals input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(cloud_data);
  const auto normals = std::dynamic_pointer_cast<CloudData>(normals_data);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError(
        "ConcatenateFields: cloud input is not a point cloud");
  }
  if (normals == nullptr || normals->desc() == nullptr ||
      normals->desc()->id != kPointTypeNormal) {
    throw tg::NodeError(
        "ConcatenateFields: normals input must be a Normal cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError("ConcatenateFields: input point type mismatch");
  }
  ConcatenateParams params;
  params.normals = normals->as<pcl::Normal>();
  if (params.normals == nullptr) {
    throw tg::NodeError(
        "ConcatenateFields: normals input must be a Normal cloud");
  }
  _out = cloud->dispatch("concatenate_fields", &params);
}

PointCloudMergeNode::PointCloudMergeNode()
    : inport{tg::In<CloudData>{this, kInFirstIndex},
             tg::In<CloudData>{this, kInSecondIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("merge");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
}

unsigned int PointCloudMergeNode::nFixedPorts(
    tg::PortType port_type) const {
  if (port_type == tg::PortType::In) {
    return 2u;
  }
  if (port_type == tg::PortType::Out) {
    return 1u;
  }
  return 0u;
}

const PointTypeDesc* PointCloudMergeNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void PointCloudMergeNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType PointCloudMergeNode::fixedDataType(
    tg::PortType port_type, unsigned int) const {
  if (port_type == tg::PortType::In || port_type == tg::PortType::Out) {
    return DataTypeOf(SelectedType());
  }
  return {};
}

const char* PointCloudMergeNode::fixedPortId(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return index == kInSecondIndex ? "second" : "first";
  }
  if (port_type == tg::PortType::Out) {
    return "cloud";
  }
  return "";
}

int PointCloudMergeNode::portTypeProperty(tg::PortType port_type,
                                          unsigned int) const {
  if (port_type == tg::PortType::In || port_type == tg::PortType::Out) {
    return _point_type_index;
  }
  return -1;
}

void PointCloudMergeNode::onSetInPortData(
    unsigned int index, std::shared_ptr<tg::NodeData> data) {
  if (index == kInFirstIndex) {
    _in_first = std::move(data);
  } else if (index == kInSecondIndex) {
    _in_second = std::move(data);
  }
}

void PointCloudMergeNode::compute() {
  const auto first_data = _in_first.lock();
  const auto second_data = _in_second.lock();
  if (first_data == nullptr) {
    throw tg::NodeError("Merge: missing first input");
  }
  if (second_data == nullptr) {
    throw tg::NodeError("Merge: missing second input");
  }
  const auto first = std::dynamic_pointer_cast<CloudData>(first_data);
  const auto second = std::dynamic_pointer_cast<CloudData>(second_data);
  if (first == nullptr || first->desc() == nullptr) {
    throw tg::NodeError("Merge: first input is not a point cloud");
  }
  if (second == nullptr || second->desc() == nullptr) {
    throw tg::NodeError("Merge: second input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != first->desc()->id ||
      desc->id != second->desc()->id) {
    throw tg::NodeError("Merge: input point type mismatch");
  }
  MergeParams params;
  params.second = second.get();
  _out = first->dispatch("merge", &params);
}

}  // namespace pg
