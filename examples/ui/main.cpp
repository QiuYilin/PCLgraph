#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pclgraph/pclgraph.hpp>
#include <taskgraph/taskgraph.hpp>
#include <Ui.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "point_cloud_view.hpp"

namespace {

constexpr const char* kChefPcdName = "chef.pcd";
constexpr const char* kSourcePcdName = "source.pcd";
constexpr const char* kTargetPcdName = "target.pcd";
constexpr const char* kAlignedPcdName = "aligned.pcd";
constexpr const char* kSourceKeypointNodeName = "Source ISS3D";
constexpr const char* kTargetKeypointNodeName = "Target ISS3D";

constexpr float kDemoExtent = 1.0f;
constexpr float kHalfSpan = 0.5f;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kTargetRotationZDegrees = 11.0;
constexpr double kTargetRotationYDegrees = 4.0;
constexpr double kTargetTranslationX = 0.10;
constexpr double kTargetTranslationY = -0.06;
constexpr double kTargetTranslationZ = 0.08;
constexpr double kMaxMeanAlignmentError = 0.02;
constexpr std::size_t kMinimumKeypointCount = 12;

// 可执行文件所在目录（argv[0] 解析）；解析失败则退回当前目录。
std::filesystem::path ExecutableDirectory(const char* argv0) {
  std::error_code error;
  std::filesystem::path invoked(argv0 != nullptr ? argv0 : "");
  if (invoked.empty()) {
    return std::filesystem::current_path(error);
  }
  const std::filesystem::path absolute =
      std::filesystem::absolute(invoked, error);
  if (error) {
    return std::filesystem::current_path(error);
  }
  return absolute.parent_path();
}

bool PrepareDemoData(const std::filesystem::path& demo_root,
                     std::string& error) {
  const std::filesystem::path chef_path = demo_root / kChefPcdName;
  if (!std::filesystem::exists(chef_path)) {
    error = "missing " + chef_path.string() +
            "; rebuild with PCLGRAPH_DEMO_PCD set or allow the demo PCD download";
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ> loaded;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(chef_path.string(), loaded) < 0) {
    error = "failed to read " + chef_path.string();
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ> finite;
  finite.reserve(loaded.size());
  for (const pcl::PointXYZ& point : loaded) {
    if (std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z)) {
      finite.push_back(point);
    }
  }
  if (finite.empty()) {
    error = "demo point cloud has no finite points";
    return false;
  }

  float min_x = finite.front().x;
  float min_y = finite.front().y;
  float min_z = finite.front().z;
  float max_x = finite.front().x;
  float max_y = finite.front().y;
  float max_z = finite.front().z;
  for (const pcl::PointXYZ& point : finite) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    min_z = std::min(min_z, point.z);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
    max_z = std::max(max_z, point.z);
  }
  const float center_x = (min_x + max_x) * kHalfSpan;
  const float center_y = (min_y + max_y) * kHalfSpan;
  const float center_z = (min_z + max_z) * kHalfSpan;
  const float max_extent =
      std::max({max_x - min_x, max_y - min_y, max_z - min_z});
  if (!(max_extent > 0.0f)) {
    error = "demo point cloud has zero extent";
    return false;
  }
  const float scale = kDemoExtent / max_extent;
  for (pcl::PointXYZ& point : finite) {
    point.x = (point.x - center_x) * scale;
    point.y = (point.y - center_y) * scale;
    point.z = (point.z - center_z) * scale;
  }

  const std::filesystem::path source_path = demo_root / kSourcePcdName;
  if (pcl::io::savePCDFileASCII(source_path.string(), finite) != 0) {
    error = "failed to write " + source_path.string();
    return false;
  }

  const Eigen::Affine3f target_transform =
      Eigen::Translation3f(static_cast<float>(kTargetTranslationX),
                           static_cast<float>(kTargetTranslationY),
                           static_cast<float>(kTargetTranslationZ)) *
      Eigen::AngleAxisf(
          static_cast<float>(kTargetRotationZDegrees * kDegreesToRadians),
          Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(
          static_cast<float>(kTargetRotationYDegrees * kDegreesToRadians),
          Eigen::Vector3f::UnitY());
  pcl::PointCloud<pcl::PointXYZ> target;
  pcl::transformPointCloud(finite, target, target_transform.matrix());

  const std::filesystem::path target_path = demo_root / kTargetPcdName;
  if (pcl::io::savePCDFileASCII(target_path.string(), target) != 0) {
    error = "failed to write " + target_path.string();
    return false;
  }
  return true;
}

std::size_t CountCloudPoints(tg::Controller& project,
                             std::string_view node_name) {
  for (const std::shared_ptr<tg::Graph>& graph : project.getGraphs()) {
    if (graph == nullptr) {
      continue;
    }
    for (const std::shared_ptr<tg::Node>& node : graph->getNodes()) {
      if (node == nullptr || std::string_view(node->getName()) != node_name) {
        continue;
      }
      const auto cloud = std::dynamic_pointer_cast<pg::CloudData>(
          node->getOutPortData(0));
      if (cloud == nullptr || cloud->desc() == nullptr) {
        return 0;
      }
      const auto points = cloud->as<pcl::PointXYZ>();
      return points != nullptr ? points->size() : 0;
    }
  }
  return 0;
}

bool EvaluateAlignment(const std::filesystem::path& aligned_path,
                       const std::filesystem::path& target_path,
                       double& mean_error, std::string& error) {
  auto aligned = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto target = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(aligned_path.string(), *aligned) <
      0) {
    error = "failed to read " + aligned_path.string();
    return false;
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(target_path.string(), *target) < 0) {
    error = "failed to read " + target_path.string();
    return false;
  }
  if (aligned->empty() || target->empty()) {
    error = "alignment check needs two non-empty clouds";
    return false;
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(target);
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  double distance_sum = 0.0;
  std::size_t count = 0;
  for (const pcl::PointXYZ& point : *aligned) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    if (tree.nearestKSearch(point, 1, indices, squared_distances) > 0) {
      distance_sum += std::sqrt(static_cast<double>(squared_distances[0]));
      ++count;
    }
  }
  if (count == 0) {
    error = "alignment check found no finite aligned points";
    return false;
  }
  mean_error = distance_sum / static_cast<double>(count);
  return true;
}

// --check 不开窗时也走一遍点云到 VTK 的转换，确认数据链路可显示。
void PrintComputedClouds(tg::Controller& project) {
  for (const auto& graph : project.getGraphs()) {
    if (graph == nullptr) {
      continue;
    }
    for (const auto& node : graph->getNodes()) {
      if (node == nullptr) {
        continue;
      }
      for (unsigned int port = 0; port < node->outputPortCount(); ++port) {
        const auto cloud = std::dynamic_pointer_cast<pg::CloudData>(
            node->getOutPortData(port));
        if (cloud == nullptr || cloud->desc() == nullptr ||
            cloud->cloud() == nullptr) {
          continue;
        }
        const PointCloudActorResult result = BuildPointCloudActor(*cloud);
        if (result.actor == nullptr) {
          continue;
        }
        const char* caption =
            node->fixedPortCaption(tg::PortType::Out, port);
        std::cout << "cloud: " << graph->getName() << " / " << node->getName()
                  << " / "
                  << (caption != nullptr && caption[0] != '\0' ? caption
                                                               : "cloud")
                  << " points=" << result.point_count << '\n';
      }
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  tg::Controller project;
  pg::RegisterPclNodes(project);

  const char* argv0 = argc > 0 ? argv[0] : nullptr;
  const std::filesystem::path demo_root =
      ExecutableDirectory(argv0) / "demo-project";
  std::string prepare_error;
  if (!PrepareDemoData(demo_root, prepare_error)) {
    std::cerr << "Failed to prepare demo data: " << prepare_error << '\n';
    return 1;
  }
  if (!project.openProject(demo_root)) {
    std::cerr << "Failed to open demo project: " << demo_root << '\n';
    return 1;
  }

  // --check：不开窗口，只验证 demo 工程能载入、跑通并对齐。
  if (argc > 1 && std::string(argv[1]) == "--check") {
    std::cout << "project: " << project.projectName() << '\n';
    for (const auto& graph : project.getGraphs()) {
      std::cout << "graph: " << graph->getName() << '\n';
      if (const auto error = project.tryRun(*graph)) {
        std::cerr << "run failed: " << *error << '\n';
        return 1;
      }
    }
    PrintComputedClouds(project);

    const std::size_t source_keypoints =
        CountCloudPoints(project, kSourceKeypointNodeName);
    const std::size_t target_keypoints =
        CountCloudPoints(project, kTargetKeypointNodeName);
    std::cout << "keypoints: source=" << source_keypoints
              << " target=" << target_keypoints << '\n';
    if (source_keypoints < kMinimumKeypointCount ||
        target_keypoints < kMinimumKeypointCount) {
      std::cerr << "keypoint count below " << kMinimumKeypointCount
                << "; tune ISS3D radii or use a denser PCL sample\n";
      return 1;
    }

    const std::filesystem::path output = demo_root / kAlignedPcdName;
    std::cout << "output: " << output.string()
              << " (exists=" << std::filesystem::exists(output) << ")\n";
    double mean_error = 0.0;
    if (!EvaluateAlignment(output, demo_root / kTargetPcdName, mean_error,
                           prepare_error)) {
      std::cerr << "alignment check failed: " << prepare_error << '\n';
      return 1;
    }
    std::cout << "alignment: mean_nearest_neighbor=" << mean_error
              << " limit=" << kMaxMeanAlignmentError << '\n';
    return mean_error <= kMaxMeanAlignmentError ? 0 : 1;
  }

  Ui ui(project);
  // 视图存进 Ui 后由 ViewWindow 的 shared_ptr 持有，随 Ui 在 GL 上下文之前析构。
  auto point_cloud = std::make_shared<PointCloudPanel>(
      project, [&ui] { return ui.SelectedNode(); });
  ui.AddViewWindow(point_cloud->MakeViewWindow());
  ui.Run();
  return 0;
}
