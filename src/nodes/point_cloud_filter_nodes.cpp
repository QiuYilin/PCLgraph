#include "pclgraph/nodes/nodes.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
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

struct VoxelGridParams {
  float leaf_x = 0.01f;
  float leaf_y = 0.01f;
  float leaf_z = 0.01f;
  unsigned int min_points = 1;
  std::string field = "x";
  bool limits_enabled = false;
  double limit_min = 0.0;
  double limit_max = 0.0;
};

template <typename PointT>
std::shared_ptr<CloudData> VoxelGridOp(const CloudData& input,
                                       const void* raw_params) {
  const auto* params = static_cast<const VoxelGridParams*>(raw_params);
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError("VoxelGrid: input point type mismatch");
  }
  auto output = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::VoxelGrid<PointT> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(params->leaf_x, params->leaf_y, params->leaf_z);
  filter.setMinimumPointsNumberPerVoxel(params->min_points);
  if (params->limits_enabled) {
    filter.setFilterFieldName(params->field);
    filter.setFilterLimits(params->limit_min, params->limit_max);
  }
  filter.filter(*output);
  return std::make_shared<CloudData>(
      std::shared_ptr<const void>(std::move(output)), input.desc());
}

struct PassThroughParams {
  std::string field = "z";
  double limit_min = 0.0;
  double limit_max = 0.0;
  bool negative = false;
  bool keep_organized = false;
};

template <typename PointT>
std::shared_ptr<CloudData> PassThroughOp(const CloudData& input,
                                         const void* raw_params) {
  const auto* params = static_cast<const PassThroughParams*>(raw_params);
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError("PassThrough: input point type mismatch");
  }
  auto output = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::PassThrough<PointT> filter;
  filter.setInputCloud(cloud);
  filter.setFilterFieldName(params->field);
  filter.setFilterLimits(params->limit_min, params->limit_max);
  filter.setNegative(params->negative);
  filter.setKeepOrganized(params->keep_organized);
  filter.filter(*output);
  return std::make_shared<CloudData>(
      std::shared_ptr<const void>(std::move(output)), input.desc());
}

#define PCL_INSTANTIATE_PG_VOXEL_GRID(T) \
  PG_INSTANTIATE("voxel_grid", kPointTypeId<T>, &VoxelGridOp<T>);
#define PCL_INSTANTIATE_PG_PASS_THROUGH(T) \
  PG_INSTANTIATE("pass_through", kPointTypeId<T>, &PassThroughOp<T>);

struct FilterRegistrations {
  FilterRegistrations() {
    PCL_INSTANTIATE(PG_VOXEL_GRID, PCL_XYZ_POINT_TYPES)
    PCL_INSTANTIATE(PG_PASS_THROUGH, PCL_XYZ_POINT_TYPES)
  }
};

const FilterRegistrations kFilterRegistrations;

}  // namespace

VoxelGridNode::VoxelGridNode()
    : inport{tg::In<CloudData>{this, kInIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("voxel_grid");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  _leaf_x = addProperty("leaf_x", 0.01, DoubleHints(0.0001, 100.0, 0.01, 4));
  _leaf_y = addProperty("leaf_y", 0.01, DoubleHints(0.0001, 100.0, 0.01, 4));
  _leaf_z = addProperty("leaf_z", 0.01, DoubleHints(0.0001, 100.0, 0.01, 4));
  _min_points = addProperty("min_points", 1);
  _field = addProperty("filter_field", 0, EnumHints({"x", "y", "z"}));
  _limits_enabled = addProperty("limits_enabled", false);
  _limit_min = addProperty("limit_min", 0.0,
                           DoubleHints(-1.0e9, 1.0e9, 0.1, 4));
  _limit_max = addProperty("limit_max", 0.0,
                           DoubleHints(-1.0e9, 1.0e9, 0.1, 4));
}

const PointTypeDesc* VoxelGridNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void VoxelGridNode::setLeafSize(double x, double y, double z) {
  _leaf_x->value = x;
  _leaf_y->value = y;
  _leaf_z->value = z;
}

void VoxelGridNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType VoxelGridNode::fixedDataType(tg::PortType port_type,
                                              unsigned int) const {
  if (port_type != tg::PortType::In && port_type != tg::PortType::Out) {
    return {};
  }
  return DataTypeOf(SelectedType());
}

int VoxelGridNode::portTypeProperty(tg::PortType port_type,
                                    unsigned int index) const {
  if ((port_type == tg::PortType::In || port_type == tg::PortType::Out) &&
      index == kInIndex) {
    return _point_type_index;
  }
  return -1;
}

const char* VoxelGridNode::fixedPortId(tg::PortType port_type,
                                       unsigned int) const {
  return port_type == tg::PortType::In || port_type == tg::PortType::Out
             ? "cloud"
             : "";
}

void VoxelGridNode::compute() {
  const auto input = _in.lock();
  if (input == nullptr) {
    throw tg::NodeError("VoxelGrid: missing input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(input);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError("VoxelGrid: input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError("VoxelGrid: input point type mismatch");
  }

  VoxelGridParams params;
  params.leaf_x = static_cast<float>(std::get<double>(_leaf_x->value));
  params.leaf_y = static_cast<float>(std::get<double>(_leaf_y->value));
  params.leaf_z = static_cast<float>(std::get<double>(_leaf_z->value));
  params.min_points =
      static_cast<unsigned int>(std::max(0, std::get<int>(_min_points->value)));
  static const char* kFields[] = {"x", "y", "z"};
  params.field = kFields[std::clamp(std::get<int>(_field->value), 0, 2)];
  params.limits_enabled = std::get<bool>(_limits_enabled->value);
  params.limit_min = std::get<double>(_limit_min->value);
  params.limit_max = std::get<double>(_limit_max->value);

  _out = cloud->dispatch("voxel_grid", &params);
}

PassThroughNode::PassThroughNode()
    : inport{tg::In<CloudData>{this, kInIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("pass_through");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  _field = addProperty("field_name", 2, EnumHints({"x", "y", "z"}));
  _limit_min = addProperty("limit_min", 0.0,
                           DoubleHints(-1.0e9, 1.0e9, 0.1, 4));
  _limit_max = addProperty("limit_max", 0.0,
                           DoubleHints(-1.0e9, 1.0e9, 0.1, 4));
  _negative = addProperty("negative", false);
  _keep_organized = addProperty("keep_organized", false);
}

const PointTypeDesc* PassThroughNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

tg::NodeDataType PassThroughNode::fixedDataType(tg::PortType port_type,
                                                unsigned int) const {
  if (port_type != tg::PortType::In && port_type != tg::PortType::Out) {
    return {};
  }
  return DataTypeOf(SelectedType());
}

int PassThroughNode::portTypeProperty(tg::PortType port_type,
                                      unsigned int index) const {
  if ((port_type == tg::PortType::In || port_type == tg::PortType::Out) &&
      index == kInIndex) {
    return _point_type_index;
  }
  return -1;
}

const char* PassThroughNode::fixedPortId(tg::PortType port_type,
                                         unsigned int) const {
  return port_type == tg::PortType::In || port_type == tg::PortType::Out
             ? "cloud"
             : "";
}

void PassThroughNode::compute() {
  const auto input = _in.lock();
  if (input == nullptr) {
    throw tg::NodeError("PassThrough: missing input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(input);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError("PassThrough: input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError("PassThrough: input point type mismatch");
  }

  PassThroughParams params;
  static const char* kFields[] = {"x", "y", "z"};
  params.field = kFields[std::clamp(std::get<int>(_field->value), 0, 2)];
  params.limit_min = std::get<double>(_limit_min->value);
  params.limit_max = std::get<double>(_limit_max->value);
  params.negative = std::get<bool>(_negative->value);
  params.keep_organized = std::get<bool>(_keep_organized->value);

  _out = cloud->dispatch("pass_through", &params);
}

}  // namespace pg
