#include "pclgraph/nodes/nodes.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/impl/normal_space.hpp>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/search/kdtree.h>
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

struct MedianParams {
  int window_size = 5;
  float max_movement = 0.0f;
};

template <typename PointT>
std::shared_ptr<CloudData> MedianOp(const CloudData& input,
                                    const void* raw_params) {
  const auto* params = static_cast<const MedianParams*>(raw_params);
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError("MedianFilter: input point type mismatch");
  }
  if (!cloud->isOrganized()) {
    throw tg::NodeError("MedianFilter: input point cloud is not organized");
  }
  auto output = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::MedianFilter<PointT> filter;
  filter.setInputCloud(cloud);
  filter.setWindowSize(params->window_size);
  filter.setMaxAllowedMovement(params->max_movement);
  filter.filter(*output);
  return Wrap(std::shared_ptr<const void>(std::move(output)), input.desc());
}

struct RadiusOutlierParams {
  double radius = 0.02;
  int min_neighbors = 1;
  bool negative = false;
  bool keep_organized = false;
};

template <typename PointT>
std::shared_ptr<CloudData> RadiusOutlierOp(const CloudData& input,
                                           const void* raw_params) {
  const auto* params = static_cast<const RadiusOutlierParams*>(raw_params);
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError("RadiusOutlierRemoval: input point type mismatch");
  }
  auto output = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::RadiusOutlierRemoval<PointT> filter;
  filter.setInputCloud(cloud);
  filter.setRadiusSearch(params->radius);
  filter.setMinNeighborsInRadius(params->min_neighbors);
  filter.setNegative(params->negative);
  filter.setKeepOrganized(params->keep_organized);
  filter.filter(*output);
  return Wrap(std::shared_ptr<const void>(std::move(output)), input.desc());
}

struct CropBoxParams {
  float min[3] = {-1.0f, -1.0f, -1.0f};
  float max[3] = {1.0f, 1.0f, 1.0f};
  float rotation[3] = {0.0f, 0.0f, 0.0f};
  float translation[3] = {0.0f, 0.0f, 0.0f};
  bool keep_organized = false;
  bool negative = false;
};

template <typename PointT>
std::shared_ptr<CloudData> CropBoxOp(const CloudData& input,
                                     const void* raw_params) {
  const auto* params = static_cast<const CropBoxParams*>(raw_params);
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError("CropBox: input point type mismatch");
  }
  auto output = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::CropBox<PointT> filter;
  filter.setInputCloud(cloud);
  filter.setMin(Eigen::Vector4f(params->min[0], params->min[1], params->min[2],
                                1.0f));
  filter.setMax(Eigen::Vector4f(params->max[0], params->max[1], params->max[2],
                                1.0f));
  filter.setRotation(Eigen::Vector3f(
      params->rotation[0], params->rotation[1], params->rotation[2]));
  filter.setTranslation(Eigen::Vector3f(params->translation[0],
                                        params->translation[1],
                                        params->translation[2]));
  filter.setKeepOrganized(params->keep_organized);
  filter.setNegative(params->negative);
  filter.filter(*output);
  return Wrap(std::shared_ptr<const void>(std::move(output)), input.desc());
}

struct ApproximateVoxelParams {
  float leaf[3] = {0.01f, 0.01f, 0.01f};
};

template <typename PointT>
std::shared_ptr<CloudData> ApproximateVoxelOp(const CloudData& input,
                                              const void* raw_params) {
  const auto* params =
      static_cast<const ApproximateVoxelParams*>(raw_params);
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError(
        "ApproximateVoxelGrid: input point type mismatch");
  }
  auto output = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::ApproximateVoxelGrid<PointT> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(params->leaf[0], params->leaf[1], params->leaf[2]);
  filter.filter(*output);
  return Wrap(std::shared_ptr<const void>(std::move(output)), input.desc());
}

struct UniformSamplingParams {
  double radius = 0.01;
  int min_points_per_voxel = 1;
  bool negative = false;
  bool keep_organized = false;
};

template <typename PointT>
std::shared_ptr<CloudData> UniformSamplingOp(const CloudData& input,
                                            const void* raw_params) {
  const auto* params = static_cast<const UniformSamplingParams*>(raw_params);
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError("UniformSampling: input point type mismatch");
  }
  auto output = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::UniformSampling<PointT> filter;
  filter.setInputCloud(cloud);
  filter.setRadiusSearch(params->radius);
  filter.setMinimumPointsNumberPerVoxel(
      static_cast<unsigned int>(std::max(1, params->min_points_per_voxel)));
  filter.setNegative(params->negative);
  filter.setKeepOrganized(params->keep_organized);
  filter.filter(*output);
  return Wrap(std::shared_ptr<const void>(std::move(output)), input.desc());
}

template <typename PointT>
std::shared_ptr<CloudData> RemoveNanOp(const CloudData& input,
                                       const void* /*params*/) {
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError("RemoveNan: input point type mismatch");
  }
  auto output = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(*cloud, *output, indices);
  return Wrap(std::shared_ptr<const void>(std::move(output)), input.desc());
}

struct NormalSpaceParams {
  pcl::PointCloud<pcl::Normal>::ConstPtr normals;
  unsigned int sample = 0;
  unsigned int seed = 0;
  unsigned int bins[3] = {10, 10, 10};
  bool keep_organized = false;
};

template <typename PointT>
std::shared_ptr<CloudData> NormalSpaceOp(const CloudData& input,
                                         const void* raw_params) {
  const auto* params = static_cast<const NormalSpaceParams*>(raw_params);
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError(
        "NormalSpaceSampling: input point type mismatch");
  }
  if (params->normals == nullptr ||
      params->normals->size() != cloud->size()) {
    throw tg::NodeError(
        "NormalSpaceSampling: normals size does not match input");
  }
  if (params->bins[0] == 0 || params->bins[1] == 0 ||
      params->bins[2] == 0) {
    throw tg::NodeError("NormalSpaceSampling: bins must be positive");
  }
  const unsigned int sample =
      params->sample == 0
          ? static_cast<unsigned int>(cloud->size())
          : std::min(params->sample,
                     static_cast<unsigned int>(cloud->size()));
  auto output = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::NormalSpaceSampling<PointT, pcl::Normal> filter;
  filter.setInputCloud(cloud);
  filter.setNormals(params->normals);
  filter.setSample(sample);
  filter.setSeed(params->seed);
  filter.setBins(params->bins[0], params->bins[1], params->bins[2]);
  filter.setKeepOrganized(params->keep_organized);
  filter.filter(*output);
  return Wrap(std::shared_ptr<const void>(std::move(output)), input.desc());
}

#define PCL_INSTANTIATE_PG_MEDIAN_FILTER(T) \
  PG_INSTANTIATE("median_filter", kPointTypeId<T>, &MedianOp<T>);
#define PCL_INSTANTIATE_PG_RADIUS_OUTLIER(T) \
  PG_INSTANTIATE("radius_outlier_removal", kPointTypeId<T>, \
                 &RadiusOutlierOp<T>);
#define PCL_INSTANTIATE_PG_CROP_BOX(T) \
  PG_INSTANTIATE("crop_box", kPointTypeId<T>, &CropBoxOp<T>);
#define PCL_INSTANTIATE_PG_APPROXIMATE_VOXEL(T) \
  PG_INSTANTIATE("approximate_voxel_grid", kPointTypeId<T>, \
                 &ApproximateVoxelOp<T>);
#define PCL_INSTANTIATE_PG_UNIFORM_SAMPLING(T) \
  PG_INSTANTIATE("uniform_sampling", kPointTypeId<T>, \
                 &UniformSamplingOp<T>);
#define PCL_INSTANTIATE_PG_REMOVE_NAN(T) \
  PG_INSTANTIATE("remove_nan", kPointTypeId<T>, &RemoveNanOp<T>);
#define PCL_INSTANTIATE_PG_NORMAL_SPACE(T) \
  PG_INSTANTIATE("normal_space_sampling", kPointTypeId<T>, \
                 &NormalSpaceOp<T>);

struct ExtraFilterRegistrations {
  ExtraFilterRegistrations() {
    PCL_INSTANTIATE(PG_MEDIAN_FILTER, PCL_XYZ_POINT_TYPES)
    PCL_INSTANTIATE(PG_RADIUS_OUTLIER, PCL_XYZ_POINT_TYPES)
    PCL_INSTANTIATE(PG_CROP_BOX, PCL_XYZ_POINT_TYPES)
    PCL_INSTANTIATE(PG_APPROXIMATE_VOXEL, PCL_XYZ_POINT_TYPES)
    PCL_INSTANTIATE(PG_UNIFORM_SAMPLING, PCL_XYZ_POINT_TYPES)
    PCL_INSTANTIATE(PG_REMOVE_NAN, PCL_XYZ_POINT_TYPES)
    PCL_INSTANTIATE(PG_NORMAL_SPACE,
                    (pcl::PointXYZ)(pcl::PointXYZI))
  }
};

const ExtraFilterRegistrations kExtraFilterRegistrations;

}  // namespace

MedianFilterNode::MedianFilterNode()
    : inport{tg::In<CloudData>{this, kInIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("median_filter");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  _window_size = addProperty("window_size", 5);
  _max_allowed_movement =
      addProperty("max_allowed_movement", 0.0,
                  DoubleHints(0.0, 1.0e9, 0.01, 6));
}

const PointTypeDesc* MedianFilterNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void MedianFilterNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType MedianFilterNode::fixedDataType(
    tg::PortType port_type, unsigned int) const {
  if (port_type != tg::PortType::In && port_type != tg::PortType::Out) {
    return {};
  }
  return DataTypeOf(SelectedType());
}

const char* MedianFilterNode::fixedPortId(tg::PortType port_type,
                                          unsigned int) const {
  return port_type == tg::PortType::In || port_type == tg::PortType::Out
             ? "cloud"
             : "";
}

int MedianFilterNode::portTypeProperty(tg::PortType port_type,
                                       unsigned int index) const {
  return (port_type == tg::PortType::In || port_type == tg::PortType::Out) &&
                 index == kInIndex
             ? _point_type_index
             : -1;
}

void MedianFilterNode::compute() {
  const auto input = _in.lock();
  if (input == nullptr) {
    throw tg::NodeError("MedianFilter: missing input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(input);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError("MedianFilter: input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError("MedianFilter: input point type mismatch");
  }
  MedianParams params;
  params.window_size = std::get<int>(_window_size->value);
  params.max_movement =
      static_cast<float>(std::get<double>(_max_allowed_movement->value));
  _out = cloud->dispatch("median_filter", &params);
}

RadiusOutlierRemovalNode::RadiusOutlierRemovalNode()
    : inport{tg::In<CloudData>{this, kInIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("radius_outlier_removal");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  _search_radius = addProperty("search_radius", 0.02,
                               DoubleHints(0.0001, 1.0e6, 0.01, 6));
  _min_neighbors = addProperty("min_neighbors", 1);
  _negative = addProperty("negative", false);
  _keep_organized = addProperty("keep_organized", false);
}

const PointTypeDesc* RadiusOutlierRemovalNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void RadiusOutlierRemovalNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType RadiusOutlierRemovalNode::fixedDataType(
    tg::PortType port_type, unsigned int) const {
  if (port_type != tg::PortType::In && port_type != tg::PortType::Out) {
    return {};
  }
  return DataTypeOf(SelectedType());
}

const char* RadiusOutlierRemovalNode::fixedPortId(tg::PortType port_type,
                                                  unsigned int) const {
  return port_type == tg::PortType::In || port_type == tg::PortType::Out
             ? "cloud"
             : "";
}

int RadiusOutlierRemovalNode::portTypeProperty(tg::PortType port_type,
                                               unsigned int index) const {
  return (port_type == tg::PortType::In || port_type == tg::PortType::Out) &&
                 index == kInIndex
             ? _point_type_index
             : -1;
}

void RadiusOutlierRemovalNode::compute() {
  const auto input = _in.lock();
  if (input == nullptr) {
    throw tg::NodeError("RadiusOutlierRemoval: missing input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(input);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError("RadiusOutlierRemoval: input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError("RadiusOutlierRemoval: input point type mismatch");
  }
  RadiusOutlierParams params;
  params.radius = std::get<double>(_search_radius->value);
  params.min_neighbors = std::get<int>(_min_neighbors->value);
  params.negative = std::get<bool>(_negative->value);
  params.keep_organized = std::get<bool>(_keep_organized->value);
  _out = cloud->dispatch("radius_outlier_removal", &params);
}

CropBoxNode::CropBoxNode()
    : inport{tg::In<CloudData>{this, kInIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("crop_box");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  const auto bounds_hints = DoubleHints(-1.0e9, 1.0e9, 0.1, 6);
  _min_x = addProperty("min_x", -1.0, bounds_hints);
  _min_y = addProperty("min_y", -1.0, bounds_hints);
  _min_z = addProperty("min_z", -1.0, bounds_hints);
  _max_x = addProperty("max_x", 1.0, bounds_hints);
  _max_y = addProperty("max_y", 1.0, bounds_hints);
  _max_z = addProperty("max_z", 1.0, bounds_hints);
  const auto angle_hints = DoubleHints(-3.15, 3.15, 0.01, 6);
  _rotation_x = addProperty("rotation_x", 0.0, angle_hints);
  _rotation_y = addProperty("rotation_y", 0.0, angle_hints);
  _rotation_z = addProperty("rotation_z", 0.0, angle_hints);
  _translation_x =
      addProperty("translation_x", 0.0, DoubleHints(-1.0e9, 1.0e9, 0.1, 6));
  _translation_y =
      addProperty("translation_y", 0.0, DoubleHints(-1.0e9, 1.0e9, 0.1, 6));
  _translation_z =
      addProperty("translation_z", 0.0, DoubleHints(-1.0e9, 1.0e9, 0.1, 6));
  _keep_organized = addProperty("keep_organized", false);
  _negative = addProperty("negative", false);
}

const PointTypeDesc* CropBoxNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void CropBoxNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType CropBoxNode::fixedDataType(tg::PortType port_type,
                                            unsigned int) const {
  if (port_type != tg::PortType::In && port_type != tg::PortType::Out) {
    return {};
  }
  return DataTypeOf(SelectedType());
}

const char* CropBoxNode::fixedPortId(tg::PortType port_type,
                                     unsigned int) const {
  return port_type == tg::PortType::In || port_type == tg::PortType::Out
             ? "cloud"
             : "";
}

int CropBoxNode::portTypeProperty(tg::PortType port_type,
                                  unsigned int index) const {
  return (port_type == tg::PortType::In || port_type == tg::PortType::Out) &&
                 index == kInIndex
             ? _point_type_index
             : -1;
}

void CropBoxNode::compute() {
  const auto input = _in.lock();
  if (input == nullptr) {
    throw tg::NodeError("CropBox: missing input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(input);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError("CropBox: input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError("CropBox: input point type mismatch");
  }
  CropBoxParams params;
  params.min[0] = static_cast<float>(std::get<double>(_min_x->value));
  params.min[1] = static_cast<float>(std::get<double>(_min_y->value));
  params.min[2] = static_cast<float>(std::get<double>(_min_z->value));
  params.max[0] = static_cast<float>(std::get<double>(_max_x->value));
  params.max[1] = static_cast<float>(std::get<double>(_max_y->value));
  params.max[2] = static_cast<float>(std::get<double>(_max_z->value));
  params.rotation[0] = static_cast<float>(std::get<double>(_rotation_x->value));
  params.rotation[1] = static_cast<float>(std::get<double>(_rotation_y->value));
  params.rotation[2] = static_cast<float>(std::get<double>(_rotation_z->value));
  params.translation[0] =
      static_cast<float>(std::get<double>(_translation_x->value));
  params.translation[1] =
      static_cast<float>(std::get<double>(_translation_y->value));
  params.translation[2] =
      static_cast<float>(std::get<double>(_translation_z->value));
  params.keep_organized = std::get<bool>(_keep_organized->value);
  params.negative = std::get<bool>(_negative->value);
  _out = cloud->dispatch("crop_box", &params);
}

ApproximateVoxelGridNode::ApproximateVoxelGridNode()
    : inport{tg::In<CloudData>{this, kInIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("approximate_voxel_grid");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  const auto hints = DoubleHints(0.0001, 1.0e6, 0.01, 6);
  _leaf_x = addProperty("leaf_x", 0.01, hints);
  _leaf_y = addProperty("leaf_y", 0.01, hints);
  _leaf_z = addProperty("leaf_z", 0.01, hints);
}

const PointTypeDesc* ApproximateVoxelGridNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void ApproximateVoxelGridNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType ApproximateVoxelGridNode::fixedDataType(
    tg::PortType port_type, unsigned int) const {
  if (port_type != tg::PortType::In && port_type != tg::PortType::Out) {
    return {};
  }
  return DataTypeOf(SelectedType());
}

const char* ApproximateVoxelGridNode::fixedPortId(tg::PortType port_type,
                                                  unsigned int) const {
  return port_type == tg::PortType::In || port_type == tg::PortType::Out
             ? "cloud"
             : "";
}

int ApproximateVoxelGridNode::portTypeProperty(tg::PortType port_type,
                                               unsigned int index) const {
  return (port_type == tg::PortType::In || port_type == tg::PortType::Out) &&
                 index == kInIndex
             ? _point_type_index
             : -1;
}

void ApproximateVoxelGridNode::compute() {
  const auto input = _in.lock();
  if (input == nullptr) {
    throw tg::NodeError("ApproximateVoxelGrid: missing input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(input);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError(
        "ApproximateVoxelGrid: input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError(
        "ApproximateVoxelGrid: input point type mismatch");
  }
  ApproximateVoxelParams params;
  params.leaf[0] = static_cast<float>(std::get<double>(_leaf_x->value));
  params.leaf[1] = static_cast<float>(std::get<double>(_leaf_y->value));
  params.leaf[2] = static_cast<float>(std::get<double>(_leaf_z->value));
  _out = cloud->dispatch("approximate_voxel_grid", &params);
}

UniformSamplingNode::UniformSamplingNode()
    : inport{tg::In<CloudData>{this, kInIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("uniform_sampling");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  _search_radius = addProperty("search_radius", 0.01,
                               DoubleHints(0.0001, 1.0e6, 0.01, 6));
  _min_points_per_voxel = addProperty("min_points_per_voxel", 1);
  _negative = addProperty("negative", false);
  _keep_organized = addProperty("keep_organized", false);
}

const PointTypeDesc* UniformSamplingNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void UniformSamplingNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType UniformSamplingNode::fixedDataType(
    tg::PortType port_type, unsigned int) const {
  if (port_type != tg::PortType::In && port_type != tg::PortType::Out) {
    return {};
  }
  return DataTypeOf(SelectedType());
}

const char* UniformSamplingNode::fixedPortId(tg::PortType port_type,
                                             unsigned int) const {
  return port_type == tg::PortType::In || port_type == tg::PortType::Out
             ? "cloud"
             : "";
}

int UniformSamplingNode::portTypeProperty(tg::PortType port_type,
                                          unsigned int index) const {
  return (port_type == tg::PortType::In || port_type == tg::PortType::Out) &&
                 index == kInIndex
             ? _point_type_index
             : -1;
}

void UniformSamplingNode::compute() {
  const auto input = _in.lock();
  if (input == nullptr) {
    throw tg::NodeError("UniformSampling: missing input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(input);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError("UniformSampling: input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError("UniformSampling: input point type mismatch");
  }
  UniformSamplingParams params;
  params.radius = std::get<double>(_search_radius->value);
  params.min_points_per_voxel = std::get<int>(_min_points_per_voxel->value);
  params.negative = std::get<bool>(_negative->value);
  params.keep_organized = std::get<bool>(_keep_organized->value);
  _out = cloud->dispatch("uniform_sampling", &params);
}

RemoveNanNode::RemoveNanNode()
    : inport{tg::In<CloudData>{this, kInIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("remove_nan");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
}

const PointTypeDesc* RemoveNanNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void RemoveNanNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType RemoveNanNode::fixedDataType(tg::PortType port_type,
                                              unsigned int) const {
  if (port_type != tg::PortType::In && port_type != tg::PortType::Out) {
    return {};
  }
  return DataTypeOf(SelectedType());
}

const char* RemoveNanNode::fixedPortId(tg::PortType port_type,
                                       unsigned int) const {
  return port_type == tg::PortType::In || port_type == tg::PortType::Out
             ? "cloud"
             : "";
}

int RemoveNanNode::portTypeProperty(tg::PortType port_type,
                                    unsigned int index) const {
  return (port_type == tg::PortType::In || port_type == tg::PortType::Out) &&
                 index == kInIndex
             ? _point_type_index
             : -1;
}

void RemoveNanNode::compute() {
  const auto input = _in.lock();
  if (input == nullptr) {
    throw tg::NodeError("RemoveNan: missing input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(input);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError("RemoveNan: input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError("RemoveNan: input point type mismatch");
  }
  _out = cloud->dispatch("remove_nan", nullptr);
}

NormalSpaceSamplingNode::NormalSpaceSamplingNode()
    : inport{tg::In<CloudData>{this, kInCloudIndex},
             tg::In<CloudData>{this, kInNormalsIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("normal_space_sampling");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  _sample = addProperty("sample", 0);
  _seed = addProperty("seed", 0);
  _bins_x = addProperty("bins_x", 10);
  _bins_y = addProperty("bins_y", 10);
  _bins_z = addProperty("bins_z", 10);
  _keep_organized = addProperty("keep_organized", false);
}

unsigned int NormalSpaceSamplingNode::nFixedPorts(
    tg::PortType port_type) const {
  if (port_type == tg::PortType::In) {
    return 2u;
  }
  if (port_type == tg::PortType::Out) {
    return 1u;
  }
  return 0u;
}

const PointTypeDesc* NormalSpaceSamplingNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void NormalSpaceSamplingNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType NormalSpaceSamplingNode::fixedDataType(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return DataTypeOf(index == kInNormalsIndex
                          ? PointTypeRegistry::Instance().Find(
                                kPointTypeNormal)
                          : SelectedType());
  }
  if (port_type == tg::PortType::Out) {
    return DataTypeOf(SelectedType());
  }
  return {};
}

const char* NormalSpaceSamplingNode::fixedPortId(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return index == kInNormalsIndex ? "normals" : "cloud";
  }
  if (port_type == tg::PortType::Out) {
    return "cloud";
  }
  return "";
}

int NormalSpaceSamplingNode::portTypeProperty(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In && index == kInCloudIndex) {
    return _point_type_index;
  }
  if (port_type == tg::PortType::Out) {
    return _point_type_index;
  }
  return -1;
}

void NormalSpaceSamplingNode::onSetInPortData(
    unsigned int index, std::shared_ptr<tg::NodeData> data) {
  if (index == kInCloudIndex) {
    _in_cloud = std::move(data);
  } else if (index == kInNormalsIndex) {
    _in_normals = std::move(data);
  }
}

void NormalSpaceSamplingNode::compute() {
  const auto cloud_data = _in_cloud.lock();
  const auto normals_data = _in_normals.lock();
  if (cloud_data == nullptr) {
    throw tg::NodeError("NormalSpaceSampling: missing cloud input");
  }
  if (normals_data == nullptr) {
    throw tg::NodeError("NormalSpaceSampling: missing normals input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(cloud_data);
  const auto normals = std::dynamic_pointer_cast<CloudData>(normals_data);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError(
        "NormalSpaceSampling: cloud input is not a point cloud");
  }
  if (normals == nullptr || normals->desc() == nullptr ||
      normals->desc()->id != kPointTypeNormal) {
    throw tg::NodeError(
        "NormalSpaceSampling: normals input must be a Normal cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError(
        "NormalSpaceSampling: input point type mismatch");
  }
  NormalSpaceParams params;
  params.normals = normals->as<pcl::Normal>();
  if (params.normals == nullptr) {
    throw tg::NodeError(
        "NormalSpaceSampling: normals input must be a Normal cloud");
  }
  const int sample = std::get<int>(_sample->value);
  params.sample = sample <= 0 ? 0u : static_cast<unsigned int>(sample);
  params.seed = static_cast<unsigned int>(
      std::max(0, std::get<int>(_seed->value)));
  params.bins[0] = static_cast<unsigned int>(
      std::max(1, std::get<int>(_bins_x->value)));
  params.bins[1] = static_cast<unsigned int>(
      std::max(1, std::get<int>(_bins_y->value)));
  params.bins[2] = static_cast<unsigned int>(
      std::max(1, std::get<int>(_bins_z->value)));
  params.keep_organized = std::get<bool>(_keep_organized->value);
  _out = cloud->dispatch("normal_space_sampling", &params);
}

}  // namespace pg
