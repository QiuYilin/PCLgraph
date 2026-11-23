#include "pclgraph/nodes/nodes.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include <Eigen/Eigenvalues>
#include <pcl/common/point_tests.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/registration/ia_fpcs.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
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

struct IcpParams {
  const CloudData* target = nullptr;
  Eigen::Matrix4f* transform = nullptr;
  int max_iterations = 10;
  double max_correspondence_distance = 0.05;
  double transformation_epsilon = 0.0;
  double euclidean_fitness_epsilon = -1.0e9;
  bool use_reciprocal = false;
  unsigned int threads = 1;
};

template <typename PointT>
std::shared_ptr<CloudData> IcpOp(const CloudData& source,
                                 const void* raw_params) {
  const auto* params = static_cast<const IcpParams*>(raw_params);
  const auto src = source.as<PointT>();
  if (src == nullptr) {
    throw tg::NodeError("ICP: source point type mismatch");
  }
  if (params->target == nullptr) {
    throw tg::NodeError("ICP: missing target cloud");
  }
  const auto tgt = params->target->as<PointT>();
  if (tgt == nullptr) {
    throw tg::NodeError("ICP: target point type mismatch");
  }
  auto aligned = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setInputSource(src);
  icp.setInputTarget(tgt);
  icp.setMaximumIterations(params->max_iterations);
  icp.setMaxCorrespondenceDistance(params->max_correspondence_distance);
  icp.setTransformationEpsilon(params->transformation_epsilon);
  icp.setEuclideanFitnessEpsilon(params->euclidean_fitness_epsilon);
  icp.setUseReciprocalCorrespondences(params->use_reciprocal);
  icp.setNumberOfThreads(params->threads);
  icp.align(*aligned);
  if (params->transform != nullptr) {
    *params->transform = icp.getFinalTransformation();
  }
  return Wrap(std::shared_ptr<const void>(std::move(aligned)), source.desc());
}

struct NdtParams {
  const CloudData* target = nullptr;
  Eigen::Matrix4f* transform = nullptr;
  double resolution = 1.0;
  int min_points_per_voxel = 3;
  double step_size = 0.1;
  int max_iterations = 10;
  double transformation_epsilon = 0.0;
  double euclidean_fitness_epsilon = -1.0e9;
};

template <typename PointT>
std::shared_ptr<CloudData> NdtOp(const CloudData& source,
                                 const void* raw_params) {
  const auto* params = static_cast<const NdtParams*>(raw_params);
  const auto src = source.as<PointT>();
  if (src == nullptr) {
    throw tg::NodeError("NDT: source point type mismatch");
  }
  if (params->target == nullptr) {
    throw tg::NodeError("NDT: missing target cloud");
  }
  const auto tgt = params->target->as<PointT>();
  if (tgt == nullptr) {
    throw tg::NodeError("NDT: target point type mismatch");
  }
  auto aligned = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::NormalDistributionsTransform<PointT, PointT> ndt;
  ndt.setInputSource(src);
  ndt.setInputTarget(tgt);
  ndt.setResolution(static_cast<float>(params->resolution));
  ndt.setMinPointPerVoxel(params->min_points_per_voxel);
  ndt.setStepSize(params->step_size);
  ndt.setMaximumIterations(params->max_iterations);
  ndt.setTransformationEpsilon(params->transformation_epsilon);
  ndt.setEuclideanFitnessEpsilon(params->euclidean_fitness_epsilon);
  ndt.align(*aligned);
  if (params->transform != nullptr) {
    *params->transform = ndt.getFinalTransformation();
  }
  return nullptr;
}

inline constexpr double kDefaultSacIaErrorThreshold = 100.0;

struct SacIaParams {
  const CloudData* target = nullptr;
  const CloudData* source_features = nullptr;
  const CloudData* target_features = nullptr;
  Eigen::Matrix4f* transform = nullptr;
  int transform_estimation = 0;
  int max_iterations = 10;
  double min_sample_distance = 0.0;
  int nr_samples = 3;
  int k_correspondences = 3;
  int error_functor = 0;
  double error_threshold = kDefaultSacIaErrorThreshold;
};

const PointTypeDesc* SignatureType() {
  return PointTypeRegistry::Instance().Find(kPointTypeFpfhSignature33);
}

template <typename PointT>
std::shared_ptr<CloudData> SacIaOp(const CloudData& source,
                                   const void* raw_params) {
  const auto* params = static_cast<const SacIaParams*>(raw_params);
  const auto src = source.as<PointT>();
  if (src == nullptr) {
    throw tg::NodeError("SAC-IA: source point type mismatch");
  }
  if (params->target == nullptr) {
    throw tg::NodeError("SAC-IA: missing target cloud");
  }
  const auto tgt = params->target->as<PointT>();
  if (tgt == nullptr) {
    throw tg::NodeError("SAC-IA: target point type mismatch");
  }
  if (params->source_features == nullptr ||
      params->source_features->desc() == nullptr ||
      params->source_features->desc()->id != kPointTypeFpfhSignature33) {
    throw tg::NodeError("SAC-IA: source features must be FPFH");
  }
  if (params->target_features == nullptr ||
      params->target_features->desc() == nullptr ||
      params->target_features->desc()->id != kPointTypeFpfhSignature33) {
    throw tg::NodeError("SAC-IA: target features must be FPFH");
  }
  const auto source_features =
      params->source_features->as<pcl::FPFHSignature33>();
  const auto target_features =
      params->target_features->as<pcl::FPFHSignature33>();
  if (source_features == nullptr || target_features == nullptr) {
    throw tg::NodeError("SAC-IA: feature point type mismatch");
  }
  if (source_features->size() != src->size() ||
      target_features->size() != tgt->size()) {
    throw tg::NodeError(
        "SAC-IA: feature count does not match point count");
  }

  pcl::SampleConsensusInitialAlignment<PointT, PointT,
                                       pcl::FPFHSignature33>
      sac_ia;
  sac_ia.setInputSource(src);
  sac_ia.setInputTarget(tgt);
  sac_ia.setSourceFeatures(source_features);
  sac_ia.setTargetFeatures(target_features);

  switch (params->transform_estimation) {
    case 1:
      sac_ia.setTransformationEstimation(
          std::make_shared<
              pcl::registration::TransformationEstimationDualQuaternion<
                  PointT, PointT>>());
      break;
    case 2:
      sac_ia.setTransformationEstimation(
          std::make_shared<pcl::registration::TransformationEstimation2D<
              PointT, PointT>>());
      break;
    case 3:
      sac_ia.setTransformationEstimation(
          std::make_shared<pcl::registration::TransformationEstimationLM<
              PointT, PointT>>());
      break;
    default:
      sac_ia.setTransformationEstimation(
          std::make_shared<pcl::registration::TransformationEstimationSVD<
              PointT, PointT>>());
      break;
  }

  sac_ia.setMaximumIterations(std::max(1, params->max_iterations));
  sac_ia.setMinSampleDistance(
      static_cast<float>(params->min_sample_distance));
  sac_ia.setNumberOfSamples(std::max(3, params->nr_samples));
  sac_ia.setCorrespondenceRandomness(
      std::max(1, params->k_correspondences));
  if (params->error_threshold > 0.0) {
    const auto threshold = static_cast<float>(params->error_threshold);
    if (params->error_functor == 1) {
      sac_ia.setErrorFunction(
          std::make_shared<typename pcl::SampleConsensusInitialAlignment<
              PointT, PointT, pcl::FPFHSignature33>::HuberPenalty>(
              threshold));
    } else {
      sac_ia.setErrorFunction(
          std::make_shared<typename pcl::SampleConsensusInitialAlignment<
              PointT, PointT, pcl::FPFHSignature33>::TruncatedError>(
              threshold));
    }
  }

  auto aligned = std::make_shared<pcl::PointCloud<PointT>>();
  sac_ia.align(*aligned);
  if (params->transform != nullptr) {
    *params->transform = sac_ia.getFinalTransformation();
  }
  return Wrap(std::shared_ptr<const void>(std::move(aligned)), source.desc());
}

struct FpcsParams {
  const CloudData* target = nullptr;
  const CloudData* source_normals = nullptr;
  const CloudData* target_normals = nullptr;
  Eigen::Matrix4f* transform = nullptr;
  int ransac_iterations = 1000;
  double delta = 1.0;
  double approx_overlap = 0.5;
  double score_threshold = 0.1;
  int nr_samples = 3;
  bool use_normals = false;
  double max_norm_diff = 90.0;
  int max_runtime = 0;
  int threads = 1;
};

template <typename PointT>
std::shared_ptr<CloudData> FpcsOp(const CloudData& source,
                                  const void* raw_params) {
  const auto* params = static_cast<const FpcsParams*>(raw_params);
  const auto src = source.as<PointT>();
  if (src == nullptr) {
    throw tg::NodeError("FPCS: source point type mismatch");
  }
  if (params->target == nullptr) {
    throw tg::NodeError("FPCS: missing target cloud");
  }
  const auto tgt = params->target->as<PointT>();
  if (tgt == nullptr) {
    throw tg::NodeError("FPCS: target point type mismatch");
  }

  pcl::registration::FPCSInitialAlignment<PointT, PointT, pcl::Normal> fpcs;
  fpcs.setInputSource(src);
  fpcs.setInputTarget(tgt);
  fpcs.setRANSACIterations(std::max(0, params->ransac_iterations));
  fpcs.setDelta(static_cast<float>(params->delta), true);
  fpcs.setApproxOverlap(static_cast<float>(params->approx_overlap));
  fpcs.setScoreThreshold(static_cast<float>(params->score_threshold));
  fpcs.setNumberOfSamples(std::max(3, params->nr_samples));
  fpcs.setMaxNormalDifference(static_cast<float>(params->max_norm_diff));
  fpcs.setMaxComputationTime(std::max(0, params->max_runtime));
  fpcs.setNumberOfThreads(std::max(0, params->threads));

  if (params->use_normals) {
    if (params->source_normals == nullptr ||
        params->target_normals == nullptr) {
      throw tg::NodeError("FPCS: normals inputs are required");
    }
    const auto source_normals =
        params->source_normals->as<pcl::Normal>();
    const auto target_normals =
        params->target_normals->as<pcl::Normal>();
    if (source_normals == nullptr || target_normals == nullptr ||
        source_normals->size() != src->size() ||
        target_normals->size() != tgt->size()) {
      throw tg::NodeError("FPCS: normals size does not match clouds");
    }
    fpcs.setSourceNormals(source_normals);
    fpcs.setTargetNormals(target_normals);
  }

  auto aligned = std::make_shared<pcl::PointCloud<PointT>>();
  fpcs.align(*aligned);
  if (params->transform != nullptr) {
    *params->transform = fpcs.getFinalTransformation();
  }
  return Wrap(std::shared_ptr<const void>(std::move(aligned)), source.desc());
}

struct IcpPointToPlaneParams {
  const CloudData* target = nullptr;
  Eigen::Matrix4f* transform = nullptr;
  int max_iterations = 10;
  double max_correspondence_distance = 0.05;
  double transformation_epsilon = 0.0;
  double rotation_epsilon = 0.0;
  double euclidean_fitness_epsilon = -1.0e9;
  int ransac_iterations = 0;
  double inlier_threshold = 0.05;
  bool use_reciprocal = false;
  bool use_symmetric_objective = false;
  bool enforce_same_direction_normals = false;
  unsigned int threads = 1;
};

template <typename PointT>
std::shared_ptr<CloudData> IcpPointToPlaneOp(
    const CloudData& source, const void* raw_params) {
  const auto* params =
      static_cast<const IcpPointToPlaneParams*>(raw_params);
  const auto src = source.as<PointT>();
  if (src == nullptr) {
    throw tg::NodeError("ICP Point-to-Plane: source point type mismatch");
  }
  if (params->target == nullptr) {
    throw tg::NodeError("ICP Point-to-Plane: missing target cloud");
  }
  const auto tgt = params->target->as<PointT>();
  if (tgt == nullptr) {
    throw tg::NodeError("ICP Point-to-Plane: target point type mismatch");
  }
  if (src->empty() || tgt->empty()) {
    throw tg::NodeError("ICP Point-to-Plane: empty input cloud");
  }
  for (const auto& point : *src) {
    if (!pcl::isFinite(point) || !std::isfinite(point.normal_x) ||
        !std::isfinite(point.normal_y) ||
        !std::isfinite(point.normal_z)) {
      throw tg::NodeError(
          "ICP Point-to-Plane: source contains invalid normals");
    }
  }
  for (const auto& point : *tgt) {
    if (!pcl::isFinite(point) || !std::isfinite(point.normal_x) ||
        !std::isfinite(point.normal_y) ||
        !std::isfinite(point.normal_z)) {
      throw tg::NodeError(
          "ICP Point-to-Plane: target contains invalid normals");
    }
  }

  constexpr float kMinNormalRankRatio = 1.0e-3f;
  Eigen::Matrix3f normal_covariance = Eigen::Matrix3f::Zero();
  std::size_t valid_normals = 0;
  for (const auto& point : *tgt) {
    Eigen::Vector3f normal(point.normal_x, point.normal_y, point.normal_z);
    const float length = normal.norm();
    if (length <= 0.0f) {
      continue;
    }
    normal /= length;
    normal_covariance.noalias() += normal * normal.transpose();
    ++valid_normals;
  }
  if (valid_normals < 3) {
    throw tg::NodeError(
        "ICP Point-to-Plane: target needs at least three normals");
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(
      normal_covariance);
  if (solver.info() != Eigen::Success ||
      solver.eigenvalues()(0) <
          kMinNormalRankRatio * solver.eigenvalues()(2)) {
    throw tg::NodeError(
        "ICP Point-to-Plane: target normals are degenerate");
  }

  auto aligned = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::IterativeClosestPointWithNormals<PointT, PointT> icp;
  icp.setInputSource(src);
  icp.setInputTarget(tgt);
  icp.setUseSymmetricObjective(params->use_symmetric_objective);
  icp.setEnforceSameDirectionNormals(
      params->enforce_same_direction_normals);
  icp.setMaximumIterations(std::max(0, params->max_iterations));
  icp.setMaxCorrespondenceDistance(
      params->max_correspondence_distance);
  icp.setTransformationEpsilon(params->transformation_epsilon);
  icp.setTransformationRotationEpsilon(params->rotation_epsilon);
  icp.setEuclideanFitnessEpsilon(params->euclidean_fitness_epsilon);
  icp.setRANSACIterations(std::max(0, params->ransac_iterations));
  icp.setRANSACOutlierRejectionThreshold(params->inlier_threshold);
  icp.setUseReciprocalCorrespondences(params->use_reciprocal);
  icp.setNumberOfThreads(params->threads);
  icp.align(*aligned);
  if (params->transform != nullptr) {
    *params->transform = icp.getFinalTransformation();
  }
  return Wrap(std::shared_ptr<const void>(std::move(aligned)), source.desc());
}

#define PCL_INSTANTIATE_PG_ICP(T) \
  PG_INSTANTIATE("icp_point_to_point", kPointTypeId<T>, &IcpOp<T>);
#define PCL_INSTANTIATE_PG_NDT(T) \
  PG_INSTANTIATE("ndt", kPointTypeId<T>, &NdtOp<T>);
#define PCL_INSTANTIATE_PG_SAC_IA(T) \
  PG_INSTANTIATE("sac_ia", kPointTypeId<T>, &SacIaOp<T>);
#define PCL_INSTANTIATE_PG_FPCS(T) \
  PG_INSTANTIATE("fpcs", kPointTypeId<T>, &FpcsOp<T>);
#define PCL_INSTANTIATE_PG_ICP_P2PLANE(T) \
  PG_INSTANTIATE("icp_point_to_plane", kPointTypeId<T>, \
                 &IcpPointToPlaneOp<T>);

struct RegistrationRegistrations {
  RegistrationRegistrations() {
    PCL_INSTANTIATE(PG_ICP,
                    (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB))
    PCL_INSTANTIATE(PG_NDT,
                    (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB))
    PCL_INSTANTIATE(PG_SAC_IA,
                    (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)
                    (pcl::PointXYZRGBA)(pcl::PointNormal))
    PCL_INSTANTIATE(PG_FPCS, (pcl::PointXYZ)(pcl::PointXYZI))
    PCL_INSTANTIATE(PG_ICP_P2PLANE,
                    (pcl::PointNormal)(pcl::PointXYZINormal))
  }
};

const RegistrationRegistrations kRegistrationRegistrations;

}  // namespace

IcpPointToPointNode::IcpPointToPointNode()
    : inport{tg::In<CloudData>{this, kInSourceIndex},
             tg::In<CloudData>{this, kInTargetIndex}},
      outport{tg::Out<CloudData>{this, kOutAlignedIndex},
              tg::Out<TransformData>{this, kOutTransformIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("icp_point_to_point");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  _max_iterations = addProperty("max_iterations", 10);
  _max_correspondence_distance =
      addProperty("max_correspondence_distance", 0.05,
                  DoubleHints(0.0, 1.0e9, 0.01, 6));
  _transformation_epsilon =
      addProperty("transformation_epsilon", 0.0,
                  DoubleHints(0.0, 1.0e9, 0.000001, 9));
  _euclidean_fitness_epsilon =
      addProperty("euclidean_fitness_epsilon", -1.0e9,
                  DoubleHints(-1.0e9, 1.0e9, 0.000001, 9));
  _use_reciprocal = addProperty("use_reciprocal_correspondence", false);
  _threads = addProperty("threads", 1);
}

unsigned int IcpPointToPointNode::nFixedPorts(
    tg::PortType port_type) const {
  if (port_type == tg::PortType::In) {
    return 2u;
  }
  if (port_type == tg::PortType::Out) {
    return 2u;
  }
  return 0u;
}

const PointTypeDesc* IcpPointToPointNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void IcpPointToPointNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType IcpPointToPointNode::fixedDataType(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return DataTypeOf(SelectedType());
  }
  if (port_type == tg::PortType::Out) {
    if (index == kOutTransformIndex) {
      return tg::NodeDataType{"transform", "Transform"};
    }
    return DataTypeOf(SelectedType());
  }
  return {};
}

const char* IcpPointToPointNode::fixedPortId(tg::PortType port_type,
                                             unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return index == kInTargetIndex ? "target" : "source";
  }
  if (port_type == tg::PortType::Out) {
    return index == kOutTransformIndex ? "transform" : "aligned";
  }
  return "";
}

int IcpPointToPointNode::portTypeProperty(tg::PortType port_type,
                                          unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return _point_type_index;
  }
  if (port_type == tg::PortType::Out && index == kOutAlignedIndex) {
    return _point_type_index;
  }
  return -1;
}

void IcpPointToPointNode::onSetInPortData(
    unsigned int index, std::shared_ptr<tg::NodeData> data) {
  if (index == kInSourceIndex) {
    _in_source = std::move(data);
  } else if (index == kInTargetIndex) {
    _in_target = std::move(data);
  }
}

std::shared_ptr<tg::NodeData> IcpPointToPointNode::getOutPortData(
    unsigned int index) {
  if (index == kOutAlignedIndex) {
    return _out;
  }
  if (index == kOutTransformIndex) {
    return _transform_out;
  }
  return nullptr;
}

void IcpPointToPointNode::compute() {
  const auto source_data = _in_source.lock();
  const auto target_data = _in_target.lock();
  if (source_data == nullptr) {
    throw tg::NodeError("ICP: missing source input");
  }
  if (target_data == nullptr) {
    throw tg::NodeError("ICP: missing target input");
  }
  const auto source = std::dynamic_pointer_cast<CloudData>(source_data);
  const auto target = std::dynamic_pointer_cast<CloudData>(target_data);
  if (source == nullptr || source->desc() == nullptr) {
    throw tg::NodeError("ICP: source input is not a point cloud");
  }
  if (target == nullptr || target->desc() == nullptr) {
    throw tg::NodeError("ICP: target input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != source->desc()->id ||
      desc->id != target->desc()->id) {
    throw tg::NodeError("ICP: source/target point type mismatch");
  }

  IcpParams params;
  params.target = target.get();
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  params.transform = &transform;
  params.max_iterations = std::get<int>(_max_iterations->value);
  params.max_correspondence_distance =
      std::get<double>(_max_correspondence_distance->value);
  params.transformation_epsilon =
      std::get<double>(_transformation_epsilon->value);
  params.euclidean_fitness_epsilon =
      std::get<double>(_euclidean_fitness_epsilon->value);
  params.use_reciprocal = std::get<bool>(_use_reciprocal->value);
  params.threads = static_cast<unsigned int>(
      std::max(0, std::get<int>(_threads->value)));
  _out = source->dispatch("icp_point_to_point", &params);
  _transform_out = std::make_shared<TransformData>(transform);
}

NdtNode::NdtNode()
    : inport{tg::In<CloudData>{this, kInSourceIndex},
             tg::In<CloudData>{this, kInTargetIndex}},
      outport{tg::Out<TransformData>{this, kOutTransformIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("ndt");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  _resolution = addProperty("resolution", 1.0,
                            DoubleHints(0.0001, 1.0e9, 0.1, 6));
  _min_points_per_voxel = addProperty("min_points_per_voxel", 3);
  _step_size = addProperty("step_size", 0.1,
                           DoubleHints(0.000001, 1.0e9, 0.01, 6));
  _max_iterations = addProperty("max_iterations", 10);
  _transformation_epsilon =
      addProperty("transformation_epsilon", 0.0,
                  DoubleHints(0.0, 1.0e9, 0.000001, 9));
  _euclidean_fitness_epsilon =
      addProperty("euclidean_fitness_epsilon", -1.0e9,
                  DoubleHints(-1.0e9, 1.0e9, 0.000001, 9));
}

unsigned int NdtNode::nFixedPorts(tg::PortType port_type) const {
  if (port_type == tg::PortType::In) {
    return 2u;
  }
  if (port_type == tg::PortType::Out) {
    return 1u;
  }
  return 0u;
}

const PointTypeDesc* NdtNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void NdtNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType NdtNode::fixedDataType(tg::PortType port_type,
                                        unsigned int) const {
  if (port_type == tg::PortType::In) {
    return DataTypeOf(SelectedType());
  }
  if (port_type == tg::PortType::Out) {
    return tg::NodeDataType{"transform", "Transform"};
  }
  return {};
}

const char* NdtNode::fixedPortId(tg::PortType port_type,
                                 unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return index == kInTargetIndex ? "target" : "source";
  }
  if (port_type == tg::PortType::Out) {
    return "transform";
  }
  return "";
}

int NdtNode::portTypeProperty(tg::PortType port_type,
                              unsigned int) const {
  return port_type == tg::PortType::In ? _point_type_index : -1;
}

void NdtNode::onSetInPortData(unsigned int index,
                              std::shared_ptr<tg::NodeData> data) {
  if (index == kInSourceIndex) {
    _in_source = std::move(data);
  } else if (index == kInTargetIndex) {
    _in_target = std::move(data);
  }
}

void NdtNode::compute() {
  const auto source_data = _in_source.lock();
  const auto target_data = _in_target.lock();
  if (source_data == nullptr) {
    throw tg::NodeError("NDT: missing source input");
  }
  if (target_data == nullptr) {
    throw tg::NodeError("NDT: missing target input");
  }
  const auto source = std::dynamic_pointer_cast<CloudData>(source_data);
  const auto target = std::dynamic_pointer_cast<CloudData>(target_data);
  if (source == nullptr || source->desc() == nullptr) {
    throw tg::NodeError("NDT: source input is not a point cloud");
  }
  if (target == nullptr || target->desc() == nullptr) {
    throw tg::NodeError("NDT: target input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != source->desc()->id ||
      desc->id != target->desc()->id) {
    throw tg::NodeError("NDT: source/target point type mismatch");
  }

  NdtParams params;
  params.target = target.get();
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  params.transform = &transform;
  params.resolution = std::get<double>(_resolution->value);
  params.min_points_per_voxel =
      std::get<int>(_min_points_per_voxel->value);
  params.step_size = std::get<double>(_step_size->value);
  params.max_iterations = std::get<int>(_max_iterations->value);
  params.transformation_epsilon =
      std::get<double>(_transformation_epsilon->value);
  params.euclidean_fitness_epsilon =
      std::get<double>(_euclidean_fitness_epsilon->value);
  source->dispatch("ndt", &params);
  _transform_out = std::make_shared<TransformData>(transform);
}

SacIaNode::SacIaNode()
    : inport{tg::In<CloudData>{this, kInSourceIndex},
             tg::In<CloudData>{this, kInSourceFeaturesIndex},
             tg::In<CloudData>{this, kInTargetIndex},
             tg::In<CloudData>{this, kInTargetFeaturesIndex}},
      outport{tg::Out<CloudData>{this, kOutAlignedIndex},
              tg::Out<TransformData>{this, kOutTransformIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("sac_ia");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  _transform_estimation = addProperty(
      "transform_estimation", 0,
      EnumHints({"SVD", "DualQuaternion", "2D", "LM"}));
  _max_iterations = addProperty("max_iterations", 10);
  _min_sample_distance = addProperty(
      "min_sample_distance", 0.0, DoubleHints(0.0, 1.0e9, 0.01, 6));
  _nr_samples = addProperty("nr_samples", 3);
  _k_correspondences = addProperty("k_correspondences", 3);
  _error_functor =
      addProperty("error_functor", 0,
                  EnumHints({"TruncatedError", "HuberPenalty"}));
  _error_threshold = addProperty(
      "error_threshold", kDefaultSacIaErrorThreshold,
      DoubleHints(0.0, 1.0e9, 0.01, 6));
}

unsigned int SacIaNode::nFixedPorts(tg::PortType port_type) const {
  if (port_type == tg::PortType::In) {
    return 4u;
  }
  if (port_type == tg::PortType::Out) {
    return 2u;
  }
  return 0u;
}

const PointTypeDesc* SacIaNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void SacIaNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType SacIaNode::fixedDataType(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    if (index == kInSourceFeaturesIndex ||
        index == kInTargetFeaturesIndex) {
      return DataTypeOf(SignatureType());
    }
    return DataTypeOf(SelectedType());
  }
  if (port_type == tg::PortType::Out) {
    if (index == kOutTransformIndex) {
      return tg::NodeDataType{"transform", "Transform"};
    }
    return DataTypeOf(SelectedType());
  }
  return {};
}

const char* SacIaNode::fixedPortId(tg::PortType port_type,
                                   unsigned int index) const {
  if (port_type == tg::PortType::In) {
    switch (index) {
      case kInSourceIndex:
        return "source";
      case kInSourceFeaturesIndex:
        return "source_features";
      case kInTargetIndex:
        return "target";
      case kInTargetFeaturesIndex:
        return "target_features";
      default:
        return "";
    }
  }
  if (port_type == tg::PortType::Out) {
    return index == kOutTransformIndex ? "transform" : "aligned";
  }
  return "";
}

int SacIaNode::portTypeProperty(tg::PortType port_type,
                                unsigned int index) const {
  if (port_type == tg::PortType::In &&
      (index == kInSourceIndex || index == kInTargetIndex)) {
    return _point_type_index;
  }
  if (port_type == tg::PortType::Out && index == kOutAlignedIndex) {
    return _point_type_index;
  }
  return -1;
}

void SacIaNode::onSetInPortData(
    unsigned int index, std::shared_ptr<tg::NodeData> data) {
  if (index == kInSourceIndex) {
    _in_source = std::move(data);
  } else if (index == kInSourceFeaturesIndex) {
    _in_source_features = std::move(data);
  } else if (index == kInTargetIndex) {
    _in_target = std::move(data);
  } else if (index == kInTargetFeaturesIndex) {
    _in_target_features = std::move(data);
  }
}

std::shared_ptr<tg::NodeData> SacIaNode::getOutPortData(
    unsigned int index) {
  if (index == kOutAlignedIndex) {
    return _out;
  }
  if (index == kOutTransformIndex) {
    return _transform_out;
  }
  return nullptr;
}

void SacIaNode::compute() {
  const auto source_data = _in_source.lock();
  const auto source_features_data = _in_source_features.lock();
  const auto target_data = _in_target.lock();
  const auto target_features_data = _in_target_features.lock();
  if (source_data == nullptr) {
    throw tg::NodeError("SAC-IA: missing source input");
  }
  if (source_features_data == nullptr) {
    throw tg::NodeError("SAC-IA: missing source features input");
  }
  if (target_data == nullptr) {
    throw tg::NodeError("SAC-IA: missing target input");
  }
  if (target_features_data == nullptr) {
    throw tg::NodeError("SAC-IA: missing target features input");
  }
  const auto source = std::dynamic_pointer_cast<CloudData>(source_data);
  const auto source_features =
      std::dynamic_pointer_cast<CloudData>(source_features_data);
  const auto target = std::dynamic_pointer_cast<CloudData>(target_data);
  const auto target_features =
      std::dynamic_pointer_cast<CloudData>(target_features_data);
  if (source == nullptr || source->desc() == nullptr) {
    throw tg::NodeError("SAC-IA: source input is not a point cloud");
  }
  if (target == nullptr || target->desc() == nullptr) {
    throw tg::NodeError("SAC-IA: target input is not a point cloud");
  }
  if (source_features == nullptr || source_features->desc() == nullptr ||
      source_features->desc()->id != kPointTypeFpfhSignature33) {
    throw tg::NodeError("SAC-IA: source features must be FPFH");
  }
  if (target_features == nullptr || target_features->desc() == nullptr ||
      target_features->desc()->id != kPointTypeFpfhSignature33) {
    throw tg::NodeError("SAC-IA: target features must be FPFH");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != source->desc()->id ||
      desc->id != target->desc()->id) {
    throw tg::NodeError("SAC-IA: source/target point type mismatch");
  }

  SacIaParams params;
  params.target = target.get();
  params.source_features = source_features.get();
  params.target_features = target_features.get();
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  params.transform = &transform;
  params.transform_estimation =
      std::get<int>(_transform_estimation->value);
  params.max_iterations = std::get<int>(_max_iterations->value);
  params.min_sample_distance =
      std::get<double>(_min_sample_distance->value);
  params.nr_samples = std::get<int>(_nr_samples->value);
  params.k_correspondences = std::get<int>(_k_correspondences->value);
  params.error_functor = std::get<int>(_error_functor->value);
  params.error_threshold = std::get<double>(_error_threshold->value);
  _out = source->dispatch("sac_ia", &params);
  _transform_out = std::make_shared<TransformData>(transform);
}

FpcsNode::FpcsNode()
    : inport{tg::In<CloudData>{this, kInSourceIndex},
             tg::In<CloudData>{this, kInTargetIndex},
             tg::In<CloudData>{this, kInSourceNormalsIndex},
             tg::In<CloudData>{this, kInTargetNormalsIndex}},
      outport{tg::Out<CloudData>{this, kOutAlignedIndex},
              tg::Out<TransformData>{this, kOutTransformIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("fpcs");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  _ransac_iterations = addProperty("ransac_iterations", 1000);
  _delta = addProperty("delta", 1.0,
                       DoubleHints(0.0, 1.0e9, 0.01, 6));
  _approx_overlap =
      addProperty("approx_overlap", 0.5,
                  DoubleHints(0.0, 1.0, 0.01, 6));
  _score_threshold =
      addProperty("score_threshold", 0.1,
                  DoubleHints(0.0, 1.0, 0.001, 6));
  _nr_samples = addProperty("nr_samples", 3);
  _use_normals = addProperty("use_normals", false);
  _max_norm_diff =
      addProperty("max_norm_diff", 90.0,
                  DoubleHints(0.0, 180.0, 1.0, 3));
  _max_runtime = addProperty("max_runtime", 0);
  _threads = addProperty("threads", 1);
}

unsigned int FpcsNode::nFixedPorts(tg::PortType port_type) const {
  if (port_type == tg::PortType::In) {
    return 4u;
  }
  if (port_type == tg::PortType::Out) {
    return 2u;
  }
  return 0u;
}

const PointTypeDesc* FpcsNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void FpcsNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType FpcsNode::fixedDataType(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    if (index == kInSourceNormalsIndex ||
        index == kInTargetNormalsIndex) {
      return DataTypeOf(
          PointTypeRegistry::Instance().Find(kPointTypeNormal));
    }
    return DataTypeOf(SelectedType());
  }
  if (port_type == tg::PortType::Out) {
    if (index == kOutTransformIndex) {
      return tg::NodeDataType{"transform", "Transform"};
    }
    return DataTypeOf(SelectedType());
  }
  return {};
}

const char* FpcsNode::fixedPortId(tg::PortType port_type,
                                  unsigned int index) const {
  if (port_type == tg::PortType::In) {
    switch (index) {
      case kInSourceIndex:
        return "source";
      case kInTargetIndex:
        return "target";
      case kInSourceNormalsIndex:
        return "source_normals";
      case kInTargetNormalsIndex:
        return "target_normals";
      default:
        return "";
    }
  }
  if (port_type == tg::PortType::Out) {
    return index == kOutTransformIndex ? "transform" : "aligned";
  }
  return "";
}

int FpcsNode::portTypeProperty(tg::PortType port_type,
                               unsigned int index) const {
  if (port_type == tg::PortType::In &&
      (index == kInSourceIndex || index == kInTargetIndex)) {
    return _point_type_index;
  }
  if (port_type == tg::PortType::Out && index == kOutAlignedIndex) {
    return _point_type_index;
  }
  return -1;
}

void FpcsNode::onSetInPortData(
    unsigned int index, std::shared_ptr<tg::NodeData> data) {
  if (index == kInSourceIndex) {
    _in_source = std::move(data);
  } else if (index == kInTargetIndex) {
    _in_target = std::move(data);
  } else if (index == kInSourceNormalsIndex) {
    _in_source_normals = std::move(data);
  } else if (index == kInTargetNormalsIndex) {
    _in_target_normals = std::move(data);
  }
}

std::shared_ptr<tg::NodeData> FpcsNode::getOutPortData(
    unsigned int index) {
  if (index == kOutAlignedIndex) {
    return _out;
  }
  if (index == kOutTransformIndex) {
    return _transform_out;
  }
  return nullptr;
}

void FpcsNode::compute() {
  const auto source_data = _in_source.lock();
  const auto target_data = _in_target.lock();
  if (source_data == nullptr) {
    throw tg::NodeError("FPCS: missing source input");
  }
  if (target_data == nullptr) {
    throw tg::NodeError("FPCS: missing target input");
  }
  const auto source = std::dynamic_pointer_cast<CloudData>(source_data);
  const auto target = std::dynamic_pointer_cast<CloudData>(target_data);
  if (source == nullptr || source->desc() == nullptr) {
    throw tg::NodeError("FPCS: source input is not a point cloud");
  }
  if (target == nullptr || target->desc() == nullptr) {
    throw tg::NodeError("FPCS: target input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != source->desc()->id ||
      desc->id != target->desc()->id) {
    throw tg::NodeError("FPCS: source/target point type mismatch");
  }

  FpcsParams params;
  params.target = target.get();
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  params.transform = &transform;
  params.ransac_iterations = std::get<int>(_ransac_iterations->value);
  params.delta = std::get<double>(_delta->value);
  params.approx_overlap = std::get<double>(_approx_overlap->value);
  params.score_threshold = std::get<double>(_score_threshold->value);
  params.nr_samples = std::get<int>(_nr_samples->value);
  params.use_normals = std::get<bool>(_use_normals->value);
  params.max_norm_diff = std::get<double>(_max_norm_diff->value);
  params.max_runtime = std::get<int>(_max_runtime->value);
  params.threads = std::get<int>(_threads->value);
  if (params.use_normals) {
    const auto source_normals = _in_source_normals.lock();
    const auto target_normals = _in_target_normals.lock();
    if (source_normals == nullptr || target_normals == nullptr) {
      throw tg::NodeError("FPCS: missing normals inputs");
    }
    const auto source_normals_data =
        std::dynamic_pointer_cast<CloudData>(source_normals);
    const auto target_normals_data =
        std::dynamic_pointer_cast<CloudData>(target_normals);
    if (source_normals_data == nullptr ||
        target_normals_data == nullptr ||
        source_normals_data->desc() == nullptr ||
        target_normals_data->desc() == nullptr ||
        source_normals_data->desc()->id != kPointTypeNormal ||
        target_normals_data->desc()->id != kPointTypeNormal) {
      throw tg::NodeError("FPCS: normals inputs must be Normal clouds");
    }
    params.source_normals = source_normals_data.get();
    params.target_normals = target_normals_data.get();
  }
  _out = source->dispatch("fpcs", &params);
  _transform_out = std::make_shared<TransformData>(transform);
}

IcpPointToPlaneNode::IcpPointToPlaneNode()
    : inport{tg::In<CloudData>{this, kInSourceIndex},
             tg::In<CloudData>{this, kInTargetIndex}},
      outport{tg::Out<CloudData>{this, kOutAlignedIndex},
              tg::Out<TransformData>{this, kOutTransformIndex}} {
  setTypeId(kTypeId);
  _types = SupportedTypesFor("icp_point_to_plane");
  _point_type = addProperty("point_type", 0,
                            PointTypeHints(_types, false));
  _point_type_index = static_cast<int>(getProperties().size()) - 1;
  _max_iterations = addProperty("max_iterations", 10);
  _max_correspondence_distance =
      addProperty("max_correspondence_distance", 0.05,
                  DoubleHints(0.0, 1.0e9, 0.01, 6));
  _transformation_epsilon =
      addProperty("transformation_epsilon", 0.0,
                  DoubleHints(0.0, 1.0e9, 0.000001, 9));
  _rotation_epsilon =
      addProperty("rotation_epsilon", 0.0,
                  DoubleHints(0.0, 1.0e9, 0.000001, 9));
  _euclidean_fitness_epsilon =
      addProperty("euclidean_fitness_epsilon", -1.0e9,
                  DoubleHints(-1.0e9, 1.0e9, 0.000001, 9));
  _ransac_iterations = addProperty("ransac_iterations", 0);
  _inlier_threshold =
      addProperty("inlier_threshold", 0.05,
                  DoubleHints(0.0, 1.0e9, 0.01, 6));
  _use_reciprocal = addProperty("use_reciprocal", false);
  _use_symmetric_objective =
      addProperty("use_symmetric_objective", false);
  _enforce_same_direction_normals =
      addProperty("enforce_same_direction_normals", false);
  _threads = addProperty("threads", 1);
}

unsigned int IcpPointToPlaneNode::nFixedPorts(
    tg::PortType port_type) const {
  if (port_type == tg::PortType::In) {
    return 2u;
  }
  if (port_type == tg::PortType::Out) {
    return 2u;
  }
  return 0u;
}

const PointTypeDesc* IcpPointToPlaneNode::SelectedType() const {
  return PointTypeAt(std::get<int>(_point_type->value), _types, false);
}

void IcpPointToPlaneNode::setPointType(int index) {
  _point_type->value = index;
  refreshPortTypes();
}

tg::NodeDataType IcpPointToPlaneNode::fixedDataType(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return DataTypeOf(SelectedType());
  }
  if (port_type == tg::PortType::Out) {
    if (index == kOutTransformIndex) {
      return tg::NodeDataType{"transform", "Transform"};
    }
    return DataTypeOf(SelectedType());
  }
  return {};
}

const char* IcpPointToPlaneNode::fixedPortId(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return index == kInTargetIndex ? "target" : "source";
  }
  if (port_type == tg::PortType::Out) {
    return index == kOutTransformIndex ? "transform" : "aligned";
  }
  return "";
}

int IcpPointToPlaneNode::portTypeProperty(
    tg::PortType port_type, unsigned int index) const {
  if (port_type == tg::PortType::In) {
    return _point_type_index;
  }
  if (port_type == tg::PortType::Out && index == kOutAlignedIndex) {
    return _point_type_index;
  }
  return -1;
}

void IcpPointToPlaneNode::onSetInPortData(
    unsigned int index, std::shared_ptr<tg::NodeData> data) {
  if (index == kInSourceIndex) {
    _in_source = std::move(data);
  } else if (index == kInTargetIndex) {
    _in_target = std::move(data);
  }
}

std::shared_ptr<tg::NodeData> IcpPointToPlaneNode::getOutPortData(
    unsigned int index) {
  if (index == kOutAlignedIndex) {
    return _out;
  }
  if (index == kOutTransformIndex) {
    return _transform_out;
  }
  return nullptr;
}

void IcpPointToPlaneNode::compute() {
  const auto source_data = _in_source.lock();
  const auto target_data = _in_target.lock();
  if (source_data == nullptr) {
    throw tg::NodeError("ICP Point-to-Plane: missing source input");
  }
  if (target_data == nullptr) {
    throw tg::NodeError("ICP Point-to-Plane: missing target input");
  }
  const auto source = std::dynamic_pointer_cast<CloudData>(source_data);
  const auto target = std::dynamic_pointer_cast<CloudData>(target_data);
  if (source == nullptr || source->desc() == nullptr) {
    throw tg::NodeError(
        "ICP Point-to-Plane: source input is not a point cloud");
  }
  if (target == nullptr || target->desc() == nullptr) {
    throw tg::NodeError(
        "ICP Point-to-Plane: target input is not a point cloud");
  }
  const PointTypeDesc* desc = SelectedType();
  if (desc == nullptr || desc->id != source->desc()->id ||
      desc->id != target->desc()->id) {
    throw tg::NodeError(
        "ICP Point-to-Plane: source/target point type mismatch");
  }

  IcpPointToPlaneParams params;
  params.target = target.get();
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  params.transform = &transform;
  params.max_iterations = std::get<int>(_max_iterations->value);
  params.max_correspondence_distance =
      std::get<double>(_max_correspondence_distance->value);
  params.transformation_epsilon =
      std::get<double>(_transformation_epsilon->value);
  params.rotation_epsilon = std::get<double>(_rotation_epsilon->value);
  params.euclidean_fitness_epsilon =
      std::get<double>(_euclidean_fitness_epsilon->value);
  params.ransac_iterations = std::get<int>(_ransac_iterations->value);
  params.inlier_threshold = std::get<double>(_inlier_threshold->value);
  params.use_reciprocal = std::get<bool>(_use_reciprocal->value);
  params.use_symmetric_objective =
      std::get<bool>(_use_symmetric_objective->value);
  params.enforce_same_direction_normals =
      std::get<bool>(_enforce_same_direction_normals->value);
  params.threads = static_cast<unsigned int>(
      std::max(0, std::get<int>(_threads->value)));
  _out = source->dispatch("icp_point_to_plane", &params);
  _transform_out = std::make_shared<TransformData>(transform);
}

}  // namespace pg
