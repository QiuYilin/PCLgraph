#include "pclgraph/nodes/nodes.hpp"

#include <algorithm>
#include <memory>
#include <utility>

#include <pcl/features/fpfh_omp.h>
#include <pcl/impl/instantiate.hpp>
#include <taskgraph/Error.hpp>

#include "pclgraph/algorithm_registry.hpp"

namespace pg {
namespace {

std::shared_ptr<CloudData> Wrap(std::shared_ptr<const void> cloud,
                                const PointTypeDesc* desc) {
  return std::make_shared<CloudData>(std::move(cloud), desc);
}

const PointTypeDesc* NormalType() {
  return PointTypeRegistry::Instance().Find(kPointTypeNormal);
}

const PointTypeDesc* SignatureType() {
  return PointTypeRegistry::Instance().Find(kPointTypeFpfhSignature33);
}

struct FpfhParams {
  int k = 10;
  int bins[3] = {11, 11, 11};
  unsigned int threads = 1;
  pcl::PointCloud<pcl::Normal>::ConstPtr normals;
};

template <typename PointT>
std::shared_ptr<CloudData> FpfhOp(const CloudData& input,
                                  const void* raw_params) {
  const auto* params = static_cast<const FpfhParams*>(raw_params);
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError("FPFH: input point type mismatch");
  }
  if (params->normals == nullptr ||
      params->normals->size() != cloud->size()) {
    throw tg::NodeError("FPFH: normals size does not match input");
  }
  auto output = std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
  pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud(cloud);
  fpfh.setInputNormals(params->normals);
  fpfh.setKSearch(params->k);
  fpfh.setNrSubdivisions(params->bins[0], params->bins[1], params->bins[2]);
  fpfh.setNumberOfThreads(params->threads);
  fpfh.compute(*output);

  const PointTypeDesc* desc = SignatureType();
  if (desc == nullptr) {
    throw tg::NodeError("FPFH: FPFHSignature33 type is not registered");
  }
  return Wrap(std::shared_ptr<const void>(std::move(output)), desc);
}

#define PCL_INSTANTIATE_PG_FPFH(T) \
  PG_INSTANTIATE("fpfh", kPointTypeId<T>, &FpfhOp<T>);

struct FpfhRegistrations {
  FpfhRegistrations() {
    PCL_INSTANTIATE(PG_FPFH,
                    (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)
                    (pcl::PointXYZRGBA)(pcl::PointNormal))
  }
};

const FpfhRegistrations kFpfhRegistrations;

}  // namespace

FpfhEstimationNode::FpfhEstimationNode()
    : inport{tg::In<CloudData>{this, kInCloudIndex},
             tg::In<CloudData>{this, kInNormalsIndex}},
      outport{tg::Out<CloudData>{this, kOutIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("fpfh");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  _k = addProperty("k", 10);
  _bins_f1 = addProperty("nr_bins_f1", 11);
  _bins_f2 = addProperty("nr_bins_f2", 11);
  _bins_f3 = addProperty("nr_bins_f3", 11);
  _threads = addProperty("threads", 1);
}

unsigned int FpfhEstimationNode::nFixedPorts(
    tg::PortType port_type) const {
  if (port_type == tg::PortType::In) {
    return 2u;
  }
  if (port_type == tg::PortType::Out) {
    return 1u;
  }
  return 0u;
}

const PointTypeDesc* FpfhEstimationNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void FpfhEstimationNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType FpfhEstimationNode::fixedDataType(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return DataTypeOf(index == kInNormalsIndex ? NormalType()
                                               : SelectedType());
  }
  if (port_type == tg::PortType::Out) {
    return DataTypeOf(SignatureType());
  }
  return {};
}

const char* FpfhEstimationNode::fixedPortId(tg::PortType port_type,
                                            unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return index == kInNormalsIndex ? "normals" : "cloud";
  }
  if (port_type == tg::PortType::Out) {
    return "signature";
  }
  return "";
}

int FpfhEstimationNode::portTypeProperty(tg::PortType port_type,
                                         unsigned int index) const {
  if (port_type == tg::PortType::In && index == kInCloudIndex) {
    return _point_type_index;
  }
  return -1;
}

void FpfhEstimationNode::onSetInPortData(
    unsigned int index, std::shared_ptr<tg::NodeData> data) {
  if (index == kInCloudIndex) {
    _in_cloud = std::move(data);
  } else if (index == kInNormalsIndex) {
    _in_normals = std::move(data);
  }
}

void FpfhEstimationNode::compute() {
  const auto cloud_data = _in_cloud.lock();
  const auto normals_data = _in_normals.lock();
  if (cloud_data == nullptr) {
    throw tg::NodeError("FPFH: missing cloud input");
  }
  if (normals_data == nullptr) {
    throw tg::NodeError("FPFH: missing normals input");
  }
  const auto cloud = std::dynamic_pointer_cast<CloudData>(cloud_data);
  const auto normals = std::dynamic_pointer_cast<CloudData>(normals_data);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    throw tg::NodeError("FPFH: cloud input is not a point cloud");
  }
  if (normals == nullptr || normals->desc() == nullptr ||
      normals->desc()->id != kPointTypeNormal) {
    throw tg::NodeError("FPFH: normals input must be a Normal cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != cloud->desc()->id) {
    throw tg::NodeError("FPFH: input point type mismatch");
  }

  FpfhParams params;
  params.k = std::get<int>(_k->value);
  params.bins[0] = std::get<int>(_bins_f1->value);
  params.bins[1] = std::get<int>(_bins_f2->value);
  params.bins[2] = std::get<int>(_bins_f3->value);
  params.threads = static_cast<unsigned int>(
      std::max(0, std::get<int>(_threads->value)));
  params.normals = normals->as<pcl::Normal>();
  if (params.normals == nullptr) {
    throw tg::NodeError("FPFH: normals input must be a Normal cloud");
  }
  _out = cloud->dispatch("fpfh", &params);
}

}  // namespace pg
