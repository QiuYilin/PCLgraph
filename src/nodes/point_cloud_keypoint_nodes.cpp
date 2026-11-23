#include "pclgraph/nodes/nodes.hpp"

#include <algorithm>
#include <memory>
#include <utility>

#include <pcl/common/point_tests.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/search/kdtree.h>
#include <taskgraph/Error.hpp>

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

std::shared_ptr<CloudData> WrapCloud(std::shared_ptr<const void> cloud,
                                     const PointTypeDesc* desc) {
  return std::make_shared<CloudData>(std::move(cloud), desc);
}

const PointTypeDesc* Desc(const char* id) {
  return PointTypeRegistry::Instance().Find(id);
}

}  // namespace

SiftKeypointNode::SiftKeypointNode()
    : inport{tg::In<CloudData>{this, kInIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _min_scale =
      addProperty("min_scale", 0.01, DoubleHints(0.0, 1.0e9, 0.001, 4));
  _nr_octaves = addProperty("nr_octaves", 4);
  _nr_scales_per_octave = addProperty("nr_scales_per_octave", 5);
  _min_contrast =
      addProperty("min_contrast", 0.001, DoubleHints(0.0, 1.0e9, 0.001, 6));
}

unsigned int SiftKeypointNode::nFixedPorts(
    tg::PortType port_type) const {
  if (port_type == tg::PortType::In) {
    return 1u;
  }
  if (port_type == tg::PortType::Out) {
    return 1u;
  }
  return 0u;
}

tg::NodeDataType SiftKeypointNode::fixedDataType(
    tg::PortType port_type, unsigned int) const {
  if (port_type == tg::PortType::In) {
    return DataTypeOf(Desc(kPointTypeXyzi));
  }
  if (port_type == tg::PortType::Out) {
    return DataTypeOf(Desc(kPointTypeXyz));
  }
  return {};
}

const char* SiftKeypointNode::fixedPortId(tg::PortType port_type,
                                          unsigned int) const {
  if (port_type == tg::PortType::In) {
    return "cloud";
  }
  if (port_type == tg::PortType::Out) {
    return "keypoints";
  }
  return "";
}

int SiftKeypointNode::portTypeProperty(tg::PortType, unsigned int) const {
  return -1;
}

void SiftKeypointNode::onSetInPortData(
    unsigned int index, std::shared_ptr<tg::NodeData> data) {
  if (index == kInIndex) {
    _in = std::move(data);
  }
}

void SiftKeypointNode::compute() {
  const auto input_data = _in.lock();
  if (input_data == nullptr) {
    throw tg::NodeError("SIFT: missing input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(input_data);
  if (cloud == nullptr || cloud->desc() == nullptr ||
      cloud->desc()->id != kPointTypeXyzi) {
    throw tg::NodeError("SIFT: input must be a PointXYZI cloud");
  }
  const auto src = cloud->as<pcl::PointXYZI>();
  if (src == nullptr || src->empty()) {
    throw tg::NodeError("SIFT: input cloud is empty");
  }

  auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointXYZ> sift;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>());
  sift.setSearchMethod(tree);
  sift.setScales(static_cast<float>(std::get<double>(_min_scale->value)),
                 std::get<int>(_nr_octaves->value),
                 std::get<int>(_nr_scales_per_octave->value));
  sift.setMinimumContrast(
      static_cast<float>(std::get<double>(_min_contrast->value)));
  sift.setInputCloud(src);
  sift.compute(*output);

  const PointTypeDesc* desc = Desc(kPointTypeXyz);
  if (desc == nullptr) {
    throw tg::NodeError("SIFT: PointXYZ is not registered");
  }
  _out = WrapCloud(std::shared_ptr<const void>(std::move(output)), desc);
}

IssKeypoint3DNode::IssKeypoint3DNode()
    : inport{tg::In<CloudData>{this, kInIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _salient_radius =
      addProperty("salient_radius", 0.1, DoubleHints(0.0, 1.0e9, 0.01, 6));
  _non_max_radius =
      addProperty("non_max_radius", 0.05, DoubleHints(0.0, 1.0e9, 0.01, 6));
  _normal_radius =
      addProperty("normal_radius", 0.1, DoubleHints(0.0, 1.0e9, 0.01, 6));
  _border_radius =
      addProperty("border_radius", 0.05, DoubleHints(0.0, 1.0e9, 0.01, 6));
  _gamma_21 =
      addProperty("gamma_21", 0.975, DoubleHints(0.0, 1.0, 0.001, 6));
  _gamma_32 =
      addProperty("gamma_32", 0.975, DoubleHints(0.0, 1.0, 0.001, 6));
  _min_neighbors = addProperty("min_neighbors", 5);
  _threads = addProperty("threads", 1);
}

unsigned int IssKeypoint3DNode::nFixedPorts(
    tg::PortType port_type) const {
  if (port_type == tg::PortType::In) {
    return 1u;
  }
  if (port_type == tg::PortType::Out) {
    return 1u;
  }
  return 0u;
}

tg::NodeDataType IssKeypoint3DNode::fixedDataType(
    tg::PortType port_type, unsigned int) const {
  if (port_type == tg::PortType::In || port_type == tg::PortType::Out) {
    return DataTypeOf(Desc(kPointTypeXyz));
  }
  return {};
}

const char* IssKeypoint3DNode::fixedPortId(tg::PortType port_type,
                                           unsigned int) const {
  if (port_type == tg::PortType::In) {
    return "cloud";
  }
  if (port_type == tg::PortType::Out) {
    return "keypoints";
  }
  return "";
}

int IssKeypoint3DNode::portTypeProperty(tg::PortType, unsigned int) const {
  return -1;
}

void IssKeypoint3DNode::onSetInPortData(
    unsigned int index, std::shared_ptr<tg::NodeData> data) {
  if (index == kInIndex) {
    _in = std::move(data);
  }
}

void IssKeypoint3DNode::compute() {
  const auto input_data = _in.lock();
  if (input_data == nullptr) {
    throw tg::NodeError("ISS3D: missing input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(input_data);
  if (cloud == nullptr || cloud->desc() == nullptr ||
      cloud->desc()->id != kPointTypeXyz) {
    throw tg::NodeError("ISS3D: input must be a PointXYZ cloud");
  }
  const auto src = cloud->as<pcl::PointXYZ>();
  if (src == nullptr || src->empty()) {
    throw tg::NodeError("ISS3D: input cloud is empty");
  }

  auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  iss.setSearchMethod(tree);
  iss.setInputCloud(src);
  iss.setSalientRadius(std::get<double>(_salient_radius->value));
  iss.setNonMaxRadius(std::get<double>(_non_max_radius->value));
  iss.setNormalRadius(std::get<double>(_normal_radius->value));
  iss.setBorderRadius(std::get<double>(_border_radius->value));
  iss.setThreshold21(std::get<double>(_gamma_21->value));
  iss.setThreshold32(std::get<double>(_gamma_32->value));
  iss.setMinNeighbors(std::get<int>(_min_neighbors->value));
  iss.setNumberOfThreads(static_cast<unsigned int>(
      std::max(0, std::get<int>(_threads->value))));
  iss.compute(*output);

  const PointTypeDesc* desc = Desc(kPointTypeXyz);
  if (desc == nullptr) {
    throw tg::NodeError("ISS3D: PointXYZ is not registered");
  }
  _out = WrapCloud(std::shared_ptr<const void>(std::move(output)), desc);
}

HarrisKeypoint3DNode::HarrisKeypoint3DNode()
    : inport{tg::In<CloudData>{this, kInCloudIndex},
             tg::In<CloudData>{this, kInNormalsIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _method = addProperty(
      "method", 0,
      EnumHints({"HARRIS", "NOBLE", "LOWE", "TOMASI", "CURVATURE"}));
  _radius = addProperty("radius", 0.02,
                        DoubleHints(0.0, 1.0e9, 0.001, 6));
  _threshold =
      addProperty("threshold", 0.0, DoubleHints(0.0, 1.0e9, 0.001, 6));
  _non_max_suppression = addProperty("non_max_suppression", true);
  _refine = addProperty("refine", false);
  _threads = addProperty("threads", 1);
}

unsigned int HarrisKeypoint3DNode::nFixedPorts(
    tg::PortType port_type) const {
  if (port_type == tg::PortType::In) {
    return 2u;
  }
  if (port_type == tg::PortType::Out) {
    return 1u;
  }
  return 0u;
}

tg::NodeDataType HarrisKeypoint3DNode::fixedDataType(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return DataTypeOf(index == kInNormalsIndex ? Desc(kPointTypeNormal)
                                               : Desc(kPointTypeXyz));
  }
  if (port_type == tg::PortType::Out) {
    return DataTypeOf(Desc(kPointTypeXyzi));
  }
  return {};
}

const char* HarrisKeypoint3DNode::fixedPortId(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return index == kInNormalsIndex ? "normals" : "cloud";
  }
  if (port_type == tg::PortType::Out) {
    return "keypoints";
  }
  return "";
}

int HarrisKeypoint3DNode::portTypeProperty(tg::PortType,
                                            unsigned int) const {
  return -1;
}

void HarrisKeypoint3DNode::onSetInPortData(
    unsigned int index, std::shared_ptr<tg::NodeData> data) {
  if (index == kInCloudIndex) {
    _in_cloud = std::move(data);
  } else if (index == kInNormalsIndex) {
    _in_normals = std::move(data);
  }
}

void HarrisKeypoint3DNode::compute() {
  const auto cloud_data = _in_cloud.lock();
  const auto normals_data = _in_normals.lock();
  if (cloud_data == nullptr) {
    throw tg::NodeError("Harris3D: missing cloud input");
  }
  if (normals_data == nullptr) {
    throw tg::NodeError("Harris3D: missing normals input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(cloud_data);
  const auto normals = std::dynamic_pointer_cast<CloudData>(normals_data);
  if (cloud == nullptr || cloud->desc() == nullptr ||
      cloud->desc()->id != kPointTypeXyz) {
    throw tg::NodeError("Harris3D: cloud input must be a PointXYZ cloud");
  }
  if (normals == nullptr || normals->desc() == nullptr ||
      normals->desc()->id != kPointTypeNormal) {
    throw tg::NodeError("Harris3D: normals input must be a Normal cloud");
  }
  const auto src = cloud->as<pcl::PointXYZ>();
  const auto normal_cloud = normals->as<pcl::Normal>();
  if (src == nullptr || src->empty()) {
    throw tg::NodeError("Harris3D: input cloud is empty");
  }
  if (normal_cloud == nullptr ||
      normal_cloud->size() != src->size()) {
    throw tg::NodeError("Harris3D: normals size does not match cloud");
  }

  auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> harris;
  harris.setInputCloud(src);
  harris.setNormals(normal_cloud);
  harris.setRadius(static_cast<float>(std::get<double>(_radius->value)));
  harris.setThreshold(
      static_cast<float>(std::get<double>(_threshold->value)));
  harris.setNonMaxSupression(
      std::get<bool>(_non_max_suppression->value));
  harris.setRefine(std::get<bool>(_refine->value));
  harris.setNumberOfThreads(static_cast<unsigned int>(
      std::max(0, std::get<int>(_threads->value))));
  switch (std::get<int>(_method->value)) {
    case 1:
      harris.setMethod(harris.NOBLE);
      break;
    case 2:
      harris.setMethod(harris.LOWE);
      break;
    case 3:
      harris.setMethod(harris.TOMASI);
      break;
    case 4:
      harris.setMethod(harris.CURVATURE);
      break;
    default:
      harris.setMethod(harris.HARRIS);
      break;
  }
  harris.compute(*output);

  const PointTypeDesc* desc = Desc(kPointTypeXyzi);
  if (desc == nullptr) {
    throw tg::NodeError("Harris3D: PointXYZI is not registered");
  }
  _out = WrapCloud(std::shared_ptr<const void>(std::move(output)), desc);
}

}  // namespace pg
