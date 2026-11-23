#include "pclgraph/nodes/register_nodes.hpp"

#include <taskgraph/Controller.hpp>

#include "pclgraph/nodes/node_categories.hpp"
#include "pclgraph/nodes/nodes.hpp"

namespace pg {

void RegisterPclNodes(tg::Controller& controller) {
  controller.registerNodeType<PointCloudReaderNode>("PointCloud Reader",
                                                    kCategoryIo);
  controller.registerNodeType<PointCloudWriterNode>("PointCloud Writer",
                                                    kCategoryIo);
  controller.registerNodeType<ConcatenateFieldsNode>("Concatenate Fields",
                                                     kCategoryIo);
  controller.registerNodeType<PointCloudMergeNode>("Merge", kCategoryIo);
  controller.registerNodeType<VoxelGridNode>("VoxelGrid",
                                             kCategoryFilterSampling);
  controller.registerNodeType<PassThroughNode>("PassThrough",
                                               kCategoryFilterRoi);
  controller.registerNodeType<MedianFilterNode>("MedianFilter3d",
                                                kCategoryFilterDenoising);
  controller.registerNodeType<RadiusOutlierRemovalNode>(
      "RadiusOutlierRemoval", kCategoryFilterDenoising);
  controller.registerNodeType<CropBoxNode>("CropBox", kCategoryFilterRoi);
  controller.registerNodeType<ApproximateVoxelGridNode>(
      "ApproximateVoxelGrid", kCategoryFilterSampling);
  controller.registerNodeType<UniformSamplingNode>("UniformSampling",
                                                   kCategoryFilterSampling);
  controller.registerNodeType<RemoveNanNode>("RemoveNan",
                                             kCategoryFilterTool);
  controller.registerNodeType<BilateralFilterNode>("BilateralFilter",
                                                   kCategoryFilterDenoising);
  controller.registerNodeType<FastBilateralFilterNode>(
      "FastBilateralFilter", kCategoryFilterDenoising);
  controller.registerNodeType<ShadowPointsNode>("ShadowPoints",
                                                kCategoryFilterDenoising);
  controller.registerNodeType<CovarianceSamplingNode>(
      "CovarianceSampling", kCategoryFilterSampling);
  controller.registerNodeType<NormalSpaceSamplingNode>(
      "NormalSpaceSampling", kCategoryFilterSampling);
  controller.registerNodeType<SiftKeypointNode>("SIFT", kCategoryKeypoints);
  controller.registerNodeType<IssKeypoint3DNode>("ISS3D",
                                                kCategoryKeypoints);
  controller.registerNodeType<HarrisKeypoint3DNode>(
      "Harris3D", kCategoryKeypoints);
  controller.registerNodeType<NormalEstimationNode>("NormalEstimation",
                                                    kCategoryFeatures);
  controller.registerNodeType<FpfhEstimationNode>("FPFH", kCategoryFeatures);
  controller.registerNodeType<PointCloudConvertNode>("PointCloud Convert",
                                                     kCategoryTool);
  controller.registerNodeType<IcpPointToPointNode>("ICP Point-to-Point",
                                                   kCategoryRegistration);
  controller.registerNodeType<IcpPointToPlaneNode>(
      "ICP Point-to-Plane", kCategoryRegistration);
  controller.registerNodeType<NdtNode>("NDT", kCategoryRegistration);
  controller.registerNodeType<SacIaNode>("SAC-IA", kCategoryRegistration);
  controller.registerNodeType<FpcsNode>("FPCS", kCategoryRegistration);
  controller.registerNodeType<TransformPointCloudNode>(
      "Transform PointCloud", kCategoryTool);
  controller.registerNodeType<CalcModelResoNode>("Calc Model Reso",
                                                 kCategoryTool);
}

}  // namespace pg
