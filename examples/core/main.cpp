#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <system_error>

#include <pcl/io/pcd_io.h>
#include <pclgraph/pclgraph.hpp>
#include <taskgraph/Controller.hpp>

int main() {
  using std::filesystem::path;

  const path dir =
      std::filesystem::temp_directory_path() / "pclgraph-core-example";
  std::error_code error;
  std::filesystem::create_directories(dir, error);
  const path input_path = dir / "points.pcd";
  const path output_path = dir / "downsampled.pcd";

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int i = 0; i < 12; ++i) {
    cloud->push_back(pcl::PointXYZ{static_cast<float>(i % 2),
                                   static_cast<float>(i % 3),
                                   static_cast<float>(i % 4)});
  }
  pcl::PCDWriter writer;
  if (writer.write(input_path.string(), *cloud, false) != 0) {
    std::cerr << "failed to write " << input_path << '\n';
    return EXIT_FAILURE;
  }

  tg::Controller project;
  auto graph = project.createGraph();
  auto reader = graph->createNode<pg::PointCloudReaderNode>();
  auto grid = graph->createNode<pg::VoxelGridNode>();
  auto sink = graph->createNode<pg::PointCloudWriterNode>();

  reader->setPath(input_path.string());
  grid->setLeafSize(2.0, 2.0, 2.0);
  sink->setPath(output_path.string());

  if (!graph->connect<pg::CloudData>(reader->outport.cloud,
                                     grid->inport.cloud)) {
    std::cerr << "failed to connect reader -> voxel grid\n";
    return EXIT_FAILURE;
  }
  if (!graph->connect<pg::CloudData>(grid->outport.cloud,
                                     sink->inport.cloud)) {
    std::cerr << "failed to connect voxel grid -> writer\n";
    return EXIT_FAILURE;
  }

  if (const auto run_error = project.tryRun(*graph)) {
    std::cerr << "run failed: " << *run_error << '\n';
    return EXIT_FAILURE;
  }

  const auto downsampled = grid->outport.cloud.nodeData();
  const std::size_t output_size =
      downsampled != nullptr ? downsampled->as<pcl::PointXYZ>()->size() : 0;

  std::cout << "input:  " << input_path.string() << " (" << cloud->size()
            << " points)\n";
  std::cout << "voxel:  " << output_size << " points\n";
  std::cout << "output: " << output_path.string()
            << " (exists=" << std::filesystem::exists(output_path) << ")\n";
  return EXIT_SUCCESS;
}
