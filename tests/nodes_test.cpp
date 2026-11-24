#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pclgraph/algorithm_registry.hpp>
#include <pclgraph/nodes/node_categories.hpp>
#include <pclgraph/nodes/nodes.hpp>
#include <pclgraph/nodes/register_nodes.hpp>
#include <pclgraph/point_type.hpp>
#include <taskgraph/Controller.hpp>
#include <taskgraph/Error.hpp>
#include <taskgraph/Graph.hpp>
#include <taskgraph/Log.hpp>

namespace {

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> MakeXyzCloud() {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int i = 0; i < 12; ++i) {
    cloud->push_back(pcl::PointXYZ{static_cast<float>(i % 2),
                                   static_cast<float>(i % 3),
                                   static_cast<float>(i % 4)});
  }
  return cloud;
}

std::shared_ptr<pg::CloudData> MakeXyzData() {
  return std::make_shared<pg::CloudData>(
      MakeXyzCloud(), pg::PointTypeRegistry::Instance().Find(pg::kPointTypeXyz));
}

std::shared_ptr<pg::CloudData> MakeCloudData(
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud) {
  return std::make_shared<pg::CloudData>(
      std::move(cloud),
      pg::PointTypeRegistry::Instance().Find(pg::kPointTypeXyz));
}

std::shared_ptr<pg::CloudData> MakeNormalData(std::size_t count) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  for (std::size_t i = 0; i < count; ++i) {
    pcl::Normal normal;
    normal.normal_x = 0.0f;
    normal.normal_y = 0.0f;
    normal.normal_z = 1.0f;
    normal.curvature = 0.0f;
    cloud->push_back(normal);
  }
  return std::make_shared<pg::CloudData>(
      cloud, pg::PointTypeRegistry::Instance().Find(pg::kPointTypeNormal));
}

std::shared_ptr<pg::CloudData> MakeFpfhData(std::size_t count) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
  for (std::size_t i = 0; i < count; ++i) {
    pcl::FPFHSignature33 signature{};
    for (std::size_t h = 0; h < 33; ++h) {
      signature.histogram[h] = static_cast<float>(h);
    }
    cloud->push_back(signature);
  }
  return std::make_shared<pg::CloudData>(
      cloud, pg::PointTypeRegistry::Instance().Find(
                 pg::kPointTypeFpfhSignature33));
}

bool SetProp(tg::Node& node, std::string_view name, tg::PropertyValue value) {
  const auto& properties = node.getProperties();
  for (std::size_t i = 0; i < properties.size(); ++i) {
    if (properties[i]->name == name) {
      node.setProperty(static_cast<int>(i), std::move(value));
      return true;
    }
  }
  return false;
}

bool WriteXyzPcd(const std::filesystem::path& path) {
  pcl::PCDWriter writer;
  return writer.write(path.string(), *MakeXyzCloud(), false) == 0;
}

bool HasNodeSelector(const tg::Node& node, std::string_view name) {
  for (const auto& property : node.getProperties()) {
    if (property->name == name) {
      return property->hints.placement == tg::PropertyPlacement::Port;
    }
  }
  return false;
}

int PropertyIndex(const tg::Node& node, std::string_view name) {
  const auto& properties = node.getProperties();
  for (std::size_t index = 0; index < properties.size(); ++index) {
    if (properties[index]->name == name) {
      return static_cast<int>(index);
    }
  }
  return -1;
}

std::string CategoryOf(const tg::Controller& controller,
                       std::string_view type_id) {
  for (const auto& type : controller.nodeTypes()) {
    if (type.type_id == type_id) {
      return type.category;
    }
  }
  return {};
}

int RegistryIndex(std::string_view id) {
  const auto& types = pg::PointTypeRegistry::Instance().All();
  for (std::size_t index = 0; index < types.size(); ++index) {
    if (types[index].id == id) {
      return static_cast<int>(index);
    }
  }
  return -1;
}

int AlgorithmTypeIndex(std::string_view algo_id, std::string_view type_id) {
  const auto types = pg::AlgorithmRegistry::Instance().TypesFor(algo_id);
  for (std::size_t index = 0; index < types.size(); ++index) {
    if (types[index] == type_id) {
      return static_cast<int>(index);
    }
  }
  return -1;
}

const tg::LogRecord* FindLogRecord(const std::vector<tg::LogRecord>& records,
                                   tg::LogCategory category,
                                   std::string_view needle) {
  for (const auto& record : records) {
    if (record.category == category &&
        record.text.find(needle) != std::string::npos) {
      return &record;
    }
  }
  return nullptr;
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> MakeOrganizedXyzCloud() {
  constexpr int kOrganizedSize = 4;
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->width = kOrganizedSize;
  cloud->height = kOrganizedSize;
  cloud->is_dense = true;
  cloud->resize(kOrganizedSize * kOrganizedSize);
  for (int y = 0; y < kOrganizedSize; ++y) {
    for (int x = 0; x < kOrganizedSize; ++x) {
      cloud->at(x, y) = pcl::PointXYZ{static_cast<float>(x),
                                      static_cast<float>(y), 0.0f};
    }
  }
  return cloud;
}

class NodesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = std::filesystem::temp_directory_path() / "pclgraph_nodes_tests";
    std::error_code error;
    std::filesystem::remove_all(dir_, error);
    std::filesystem::create_directories(dir_, error);
  }

  std::filesystem::path dir_;
};

TEST_F(NodesTest, ReaderInfersPointTypeFromHeader) {
  const auto path = dir_ / "cloud.pcd";
  ASSERT_TRUE(WriteXyzPcd(path));

  pg::PointCloudReaderNode reader;
  ASSERT_NE(reader.outputPort(0), nullptr);
  EXPECT_TRUE(reader.outputPort(0)->data_type.id.empty());

  ASSERT_TRUE(SetProp(reader, "path", path.string()));
  ASSERT_NE(reader.outputPort(0), nullptr);
  EXPECT_EQ(reader.outputPort(0)->data_type.id, pg::kPointTypeXyz);
  EXPECT_EQ(reader.outputPort(0)->data_type.name, "PointXYZ");

  reader.compute();
  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(reader.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  ASSERT_EQ(output->as<pcl::PointXYZ>()->size(), 12u);
}

TEST_F(NodesTest, VoxelGridDownsamples) {
  pg::VoxelGridNode grid;
  ASSERT_TRUE(SetProp(grid, "leaf_x", 2.0));
  ASSERT_TRUE(SetProp(grid, "leaf_y", 2.0));
  ASSERT_TRUE(SetProp(grid, "leaf_z", 2.0));
  ASSERT_TRUE(SetProp(grid, "min_points", 1));

  const auto input = MakeXyzData();
  grid.setInPortData(0, input);
  grid.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(grid.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type().id, pg::kPointTypeXyz);
  EXPECT_GT(output->as<pcl::PointXYZ>()->size(), 0u);
  EXPECT_LT(output->as<pcl::PointXYZ>()->size(), 12u);
}

TEST_F(NodesTest, PassThroughFiltersZRange) {
  pg::PassThroughNode pass;
  ASSERT_TRUE(SetProp(pass, "field_name", 2));
  ASSERT_TRUE(SetProp(pass, "limit_min", -0.5));
  ASSERT_TRUE(SetProp(pass, "limit_max", 2.5));

  const auto input = MakeXyzData();
  pass.setInPortData(0, input);
  pass.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(pass.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->as<pcl::PointXYZ>()->size(), 9u);
}

TEST_F(NodesTest, NormalEstimationProducesPointNormal) {
  pg::NormalEstimationNode normal;
  const auto input = MakeXyzData();
  normal.setInPortData(0, input);
  normal.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(normal.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type().id, pg::kPointTypeXyzNormal);
  ASSERT_EQ(output->as<pcl::PointNormal>()->size(), 12u);
  EXPECT_FLOAT_EQ(output->as<pcl::PointNormal>()->at(1).x, 1.0f);
  EXPECT_FLOAT_EQ(output->as<pcl::PointNormal>()->at(1).z, 1.0f);
}

TEST_F(NodesTest, ConvertChangesPointType) {
  pg::PointCloudConvertNode convert;
  const int output_index = RegistryIndex(pg::kPointTypeXyzNormal);
  ASSERT_GE(output_index, 0);
  ASSERT_TRUE(SetProp(convert, "output_type", output_index));
  const auto input = MakeXyzData();
  convert.setInPortData(0, input);
  convert.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(convert.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type().id, pg::kPointTypeXyzNormal);
  ASSERT_EQ(output->as<pcl::PointNormal>()->size(), 12u);
}

TEST_F(NodesTest, WriterWritesPcd) {
  const auto path = dir_ / "out.pcd";
  pg::PointCloudWriterNode writer;
  ASSERT_TRUE(SetProp(writer, "path", path.string()));
  ASSERT_TRUE(SetProp(writer, "input_type", 0));
  const auto input = MakeXyzData();
  writer.setInPortData(0, input);
  writer.compute();
  EXPECT_TRUE(std::filesystem::exists(path));
}

TEST_F(NodesTest, ReaderInfersFromPlyRoundTrip) {
  const auto path = dir_ / "out.ply";
  pg::PointCloudWriterNode writer;
  ASSERT_TRUE(SetProp(writer, "path", path.string()));
  ASSERT_TRUE(SetProp(writer, "input_type", 0));
  ASSERT_TRUE(SetProp(writer, "format", 2));
  const auto input = MakeXyzData();
  writer.setInPortData(0, input);
  writer.compute();
  ASSERT_TRUE(std::filesystem::exists(path));

  pg::PointCloudReaderNode reader;
  ASSERT_TRUE(SetProp(reader, "path", path.string()));
  ASSERT_NE(reader.outputPort(0), nullptr);
  EXPECT_EQ(reader.outputPort(0)->data_type.id, pg::kPointTypeXyz);
  reader.compute();
  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(reader.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->as<pcl::PointXYZ>()->size(), 12u);
}

TEST_F(NodesTest, TypeSelectorsUsePortPlacement) {
  pg::PointCloudReaderNode reader;
  EXPECT_TRUE(HasNodeSelector(reader, "point_type"));
  EXPECT_EQ(reader.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(reader, "point_type"));
  EXPECT_EQ(reader.portTypeProperty(tg::PortType::In, 0), -1);

  pg::PointCloudWriterNode writer;
  EXPECT_TRUE(HasNodeSelector(writer, "input_type"));
  EXPECT_EQ(writer.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(writer, "input_type"));
  EXPECT_EQ(writer.portTypeProperty(tg::PortType::Out, 0), -1);

  pg::VoxelGridNode grid;
  EXPECT_TRUE(HasNodeSelector(grid, "point_type"));
  EXPECT_EQ(grid.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(grid, "point_type"));
  EXPECT_EQ(grid.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(grid, "point_type"));

  pg::PassThroughNode pass;
  EXPECT_TRUE(HasNodeSelector(pass, "point_type"));
  EXPECT_EQ(pass.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(pass, "point_type"));
  EXPECT_EQ(pass.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(pass, "point_type"));

  pg::NormalEstimationNode normal;
  EXPECT_TRUE(HasNodeSelector(normal, "input_type"));
  EXPECT_EQ(normal.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(normal, "input_type"));
  EXPECT_EQ(normal.portTypeProperty(tg::PortType::Out, 0), -1);

  pg::PointCloudConvertNode convert;
  EXPECT_TRUE(HasNodeSelector(convert, "input_type"));
  EXPECT_TRUE(HasNodeSelector(convert, "output_type"));
  EXPECT_EQ(convert.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(convert, "input_type"));
  EXPECT_EQ(convert.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(convert, "output_type"));

  pg::MedianFilterNode median;
  EXPECT_TRUE(HasNodeSelector(median, "point_type"));
  EXPECT_EQ(median.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(median, "point_type"));
  EXPECT_EQ(median.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(median, "point_type"));

  pg::RadiusOutlierRemovalNode radius;
  EXPECT_TRUE(HasNodeSelector(radius, "point_type"));
  EXPECT_EQ(radius.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(radius, "point_type"));
  EXPECT_EQ(radius.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(radius, "point_type"));

  pg::CropBoxNode crop;
  EXPECT_TRUE(HasNodeSelector(crop, "point_type"));
  EXPECT_EQ(crop.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(crop, "point_type"));
  EXPECT_EQ(crop.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(crop, "point_type"));

  pg::ApproximateVoxelGridNode approx;
  EXPECT_TRUE(HasNodeSelector(approx, "point_type"));
  EXPECT_EQ(approx.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(approx, "point_type"));
  EXPECT_EQ(approx.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(approx, "point_type"));

  pg::UniformSamplingNode uniform;
  EXPECT_TRUE(HasNodeSelector(uniform, "point_type"));
  EXPECT_EQ(uniform.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(uniform, "point_type"));
  EXPECT_EQ(uniform.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(uniform, "point_type"));

  pg::RemoveNanNode remove_nan;
  EXPECT_TRUE(HasNodeSelector(remove_nan, "point_type"));
  EXPECT_EQ(remove_nan.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(remove_nan, "point_type"));
  EXPECT_EQ(remove_nan.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(remove_nan, "point_type"));

  pg::BilateralFilterNode bilateral;
  EXPECT_TRUE(HasNodeSelector(bilateral, "point_type"));
  EXPECT_EQ(bilateral.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(bilateral, "point_type"));
  EXPECT_EQ(bilateral.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(bilateral, "point_type"));

  pg::FastBilateralFilterNode fast_bilateral;
  EXPECT_TRUE(HasNodeSelector(fast_bilateral, "point_type"));
  EXPECT_EQ(fast_bilateral.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(fast_bilateral, "point_type"));
  EXPECT_EQ(fast_bilateral.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(fast_bilateral, "point_type"));

  pg::ShadowPointsNode shadow;
  EXPECT_TRUE(HasNodeSelector(shadow, "point_type"));
  EXPECT_EQ(shadow.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(shadow, "point_type"));
  EXPECT_EQ(shadow.portTypeProperty(tg::PortType::In, 1), -1);
  EXPECT_EQ(shadow.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(shadow, "point_type"));

  pg::CovarianceSamplingNode covariance;
  EXPECT_TRUE(HasNodeSelector(covariance, "point_type"));
  EXPECT_EQ(covariance.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(covariance, "point_type"));
  EXPECT_EQ(covariance.portTypeProperty(tg::PortType::In, 1), -1);
  EXPECT_EQ(covariance.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(covariance, "point_type"));

  pg::FpfhEstimationNode fpfh;
  EXPECT_TRUE(HasNodeSelector(fpfh, "point_type"));
  EXPECT_EQ(fpfh.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(fpfh, "point_type"));
  EXPECT_EQ(fpfh.portTypeProperty(tg::PortType::In, 1), -1);
  EXPECT_EQ(fpfh.portTypeProperty(tg::PortType::Out, 0), -1);

  pg::IcpPointToPointNode icp;
  EXPECT_TRUE(HasNodeSelector(icp, "point_type"));
  EXPECT_EQ(icp.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(icp, "point_type"));
  EXPECT_EQ(icp.portTypeProperty(tg::PortType::In, 1),
            PropertyIndex(icp, "point_type"));
  EXPECT_EQ(icp.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(icp, "point_type"));
  EXPECT_EQ(icp.portTypeProperty(tg::PortType::Out, 1), -1);

  pg::NdtNode ndt;
  EXPECT_TRUE(HasNodeSelector(ndt, "point_type"));
  EXPECT_EQ(ndt.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(ndt, "point_type"));
  EXPECT_EQ(ndt.portTypeProperty(tg::PortType::In, 1),
            PropertyIndex(ndt, "point_type"));
  EXPECT_EQ(ndt.portTypeProperty(tg::PortType::Out, 0), -1);

  pg::TransformPointCloudNode transform_cloud;
  EXPECT_TRUE(HasNodeSelector(transform_cloud, "point_type"));
  EXPECT_EQ(transform_cloud.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(transform_cloud, "point_type"));
  EXPECT_EQ(transform_cloud.portTypeProperty(tg::PortType::In, 1), -1);
  EXPECT_EQ(transform_cloud.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(transform_cloud, "point_type"));

  pg::ConcatenateFieldsNode concatenate;
  EXPECT_TRUE(HasNodeSelector(concatenate, "point_type"));
  EXPECT_EQ(concatenate.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(concatenate, "point_type"));
  EXPECT_EQ(concatenate.portTypeProperty(tg::PortType::In, 1), -1);
  EXPECT_EQ(concatenate.portTypeProperty(tg::PortType::Out, 0), -1);

  pg::PointCloudMergeNode merge;
  EXPECT_TRUE(HasNodeSelector(merge, "point_type"));
  EXPECT_EQ(merge.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(merge, "point_type"));
  EXPECT_EQ(merge.portTypeProperty(tg::PortType::In, 1),
            PropertyIndex(merge, "point_type"));
  EXPECT_EQ(merge.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(merge, "point_type"));

  pg::CalcModelResoNode calc_reso;
  EXPECT_EQ(calc_reso.portTypeProperty(tg::PortType::In, 0), -1);
  EXPECT_EQ(calc_reso.portTypeProperty(tg::PortType::Out, 0), -1);

  pg::SacIaNode sac_ia;
  EXPECT_TRUE(HasNodeSelector(sac_ia, "point_type"));
  EXPECT_EQ(sac_ia.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(sac_ia, "point_type"));
  EXPECT_EQ(sac_ia.portTypeProperty(tg::PortType::In, 1), -1);
  EXPECT_EQ(sac_ia.portTypeProperty(tg::PortType::In, 2),
            PropertyIndex(sac_ia, "point_type"));
  EXPECT_EQ(sac_ia.portTypeProperty(tg::PortType::In, 3), -1);
  EXPECT_EQ(sac_ia.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(sac_ia, "point_type"));
  EXPECT_EQ(sac_ia.portTypeProperty(tg::PortType::Out, 1), -1);

  pg::NormalSpaceSamplingNode normal_space;
  EXPECT_TRUE(HasNodeSelector(normal_space, "point_type"));
  EXPECT_EQ(normal_space.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(normal_space, "point_type"));
  EXPECT_EQ(normal_space.portTypeProperty(tg::PortType::In, 1), -1);
  EXPECT_EQ(normal_space.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(normal_space, "point_type"));

  pg::FpcsNode fpcs;
  EXPECT_TRUE(HasNodeSelector(fpcs, "point_type"));
  EXPECT_EQ(fpcs.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(fpcs, "point_type"));
  EXPECT_EQ(fpcs.portTypeProperty(tg::PortType::In, 1),
            PropertyIndex(fpcs, "point_type"));
  EXPECT_EQ(fpcs.portTypeProperty(tg::PortType::In, 2), -1);
  EXPECT_EQ(fpcs.portTypeProperty(tg::PortType::In, 3), -1);
  EXPECT_EQ(fpcs.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(fpcs, "point_type"));
  EXPECT_EQ(fpcs.portTypeProperty(tg::PortType::Out, 1), -1);

  pg::IcpPointToPlaneNode icp_p2plane;
  EXPECT_TRUE(HasNodeSelector(icp_p2plane, "point_type"));
  EXPECT_EQ(icp_p2plane.portTypeProperty(tg::PortType::In, 0),
            PropertyIndex(icp_p2plane, "point_type"));
  EXPECT_EQ(icp_p2plane.portTypeProperty(tg::PortType::In, 1),
            PropertyIndex(icp_p2plane, "point_type"));
  EXPECT_EQ(icp_p2plane.portTypeProperty(tg::PortType::Out, 0),
            PropertyIndex(icp_p2plane, "point_type"));
  EXPECT_EQ(icp_p2plane.portTypeProperty(tg::PortType::Out, 1), -1);

  pg::SiftKeypointNode sift;
  EXPECT_FALSE(HasNodeSelector(sift, "point_type"));
  EXPECT_EQ(sift.portTypeProperty(tg::PortType::In, 0), -1);
  EXPECT_EQ(sift.portTypeProperty(tg::PortType::Out, 0), -1);

  pg::IssKeypoint3DNode iss;
  EXPECT_FALSE(HasNodeSelector(iss, "point_type"));
  EXPECT_EQ(iss.portTypeProperty(tg::PortType::In, 0), -1);
  EXPECT_EQ(iss.portTypeProperty(tg::PortType::Out, 0), -1);

  pg::HarrisKeypoint3DNode harris;
  EXPECT_FALSE(HasNodeSelector(harris, "point_type"));
  EXPECT_EQ(harris.portTypeProperty(tg::PortType::In, 0), -1);
  EXPECT_EQ(harris.portTypeProperty(tg::PortType::In, 1), -1);
  EXPECT_EQ(harris.portTypeProperty(tg::PortType::Out, 0), -1);
}

TEST_F(NodesTest, MedianFilterRequiresOrganized) {
  pg::MedianFilterNode node;
  ASSERT_TRUE(SetProp(node, "window_size", 3));

  const auto flat = MakeXyzData();
  node.setInPortData(0, flat);
  EXPECT_THROW(node.compute(), tg::NodeError);

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->width = 4;
  cloud->height = 4;
  cloud->is_dense = true;
  cloud->resize(16);
  for (int y = 0; y < 4; ++y) {
    for (int x = 0; x < 4; ++x) {
      cloud->at(x, y) = pcl::PointXYZ{static_cast<float>(x),
                                      static_cast<float>(y), 0.0f};
    }
  }
  cloud->at(1, 1).z = 10.0f;
  const auto organized = MakeCloudData(cloud);
  node.setInPortData(0, organized);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->as<pcl::PointXYZ>()->size(), 16u);
}

TEST_F(NodesTest, RadiusOutlierRemovalDropsIsolatedPoint) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int i = 0; i < 5; ++i) {
    cloud->push_back(pcl::PointXYZ{0.001f * static_cast<float>(i), 0.0f, 0.0f});
  }
  cloud->push_back(pcl::PointXYZ{10.0f, 10.0f, 10.0f});

  pg::RadiusOutlierRemovalNode node;
  ASSERT_TRUE(SetProp(node, "search_radius", 0.2));
  ASSERT_TRUE(SetProp(node, "min_neighbors", 2));
  const auto input = MakeCloudData(cloud);
  node.setInPortData(0, input);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->as<pcl::PointXYZ>()->size(), 5u);
}

TEST_F(NodesTest, CropBoxFiltersKnownBounds) {
  pg::CropBoxNode node;
  ASSERT_TRUE(SetProp(node, "min_x", 0.0));
  ASSERT_TRUE(SetProp(node, "min_y", 0.0));
  ASSERT_TRUE(SetProp(node, "min_z", 0.0));
  ASSERT_TRUE(SetProp(node, "max_x", 0.5));
  ASSERT_TRUE(SetProp(node, "max_y", 1.5));
  ASSERT_TRUE(SetProp(node, "max_z", 1.5));
  const auto input = MakeXyzData();
  node.setInPortData(0, input);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->as<pcl::PointXYZ>()->size(), 2u);
}

TEST_F(NodesTest, ApproximateVoxelGridDownsamples) {
  pg::ApproximateVoxelGridNode node;
  ASSERT_TRUE(SetProp(node, "leaf_x", 2.0));
  ASSERT_TRUE(SetProp(node, "leaf_y", 2.0));
  ASSERT_TRUE(SetProp(node, "leaf_z", 2.0));
  const auto input = MakeXyzData();
  node.setInPortData(0, input);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_GT(output->as<pcl::PointXYZ>()->size(), 0u);
  EXPECT_LT(output->as<pcl::PointXYZ>()->size(), 12u);
}

TEST_F(NodesTest, UniformSamplingDownsamples) {
  pg::UniformSamplingNode node;
  ASSERT_TRUE(SetProp(node, "search_radius", 1.5));
  const auto input = MakeXyzData();
  node.setInPortData(0, input);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_GT(output->as<pcl::PointXYZ>()->size(), 0u);
  EXPECT_LT(output->as<pcl::PointXYZ>()->size(), 12u);
}

TEST_F(NodesTest, RemoveNanDropsInvalidPoints) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->is_dense = false;
  cloud->push_back(pcl::PointXYZ{0.0f, 0.0f, 0.0f});
  cloud->push_back(pcl::PointXYZ{std::numeric_limits<float>::quiet_NaN(), 0.0f,
                                 0.0f});

  pg::RemoveNanNode node;
  const auto input = MakeCloudData(cloud);
  node.setInPortData(0, input);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->as<pcl::PointXYZ>()->size(), 1u);
}

TEST_F(NodesTest, FilterSupportsPclExtendedTypes) {
  auto& registry = pg::PointTypeRegistry::Instance();

  auto xyz_l_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZL>>();
  xyz_l_cloud->push_back(pcl::PointXYZL{0.0f, 0.0f, 0.0f, 7u});
  xyz_l_cloud->push_back(pcl::PointXYZL{1.0f, 1.0f, 1.0f, 8u});
  const auto xyz_l_data = std::make_shared<pg::CloudData>(
      xyz_l_cloud, registry.Find(pg::kPointTypeXyzL));

  pg::VoxelGridNode grid;
  ASSERT_TRUE(SetProp(grid, "point_type",
                      AlgorithmTypeIndex("voxel_grid", pg::kPointTypeXyzL)));
  grid.setInPortData(0, xyz_l_data);
  grid.compute();
  const auto xyz_l_out =
      std::dynamic_pointer_cast<pg::CloudData>(grid.getOutPortData(0));
  ASSERT_NE(xyz_l_out, nullptr);
  EXPECT_EQ(xyz_l_out->type().id, pg::kPointTypeXyzL);

  auto surfel_cloud = std::make_shared<pcl::PointCloud<pcl::PointSurfel>>();
  pcl::PointSurfel surfel;
  surfel.x = 0.0f;
  surfel.y = 0.0f;
  surfel.z = 0.0f;
  surfel.normal_x = 0.0f;
  surfel.normal_y = 0.0f;
  surfel.normal_z = 1.0f;
  surfel.rgba = 0xffffffffu;
  surfel.radius = 0.01f;
  surfel.confidence = 1.0f;
  surfel.curvature = 0.0f;
  surfel_cloud->push_back(surfel);
  const auto surfel_data = std::make_shared<pg::CloudData>(
      surfel_cloud, registry.Find(pg::kPointTypeSurfel));

  pg::VoxelGridNode surfel_grid;
  ASSERT_TRUE(SetProp(
      surfel_grid, "point_type",
      AlgorithmTypeIndex("voxel_grid", pg::kPointTypeSurfel)));
  surfel_grid.setInPortData(0, surfel_data);
  surfel_grid.compute();
  const auto surfel_out =
      std::dynamic_pointer_cast<pg::CloudData>(surfel_grid.getOutPortData(0));
  ASSERT_NE(surfel_out, nullptr);
  EXPECT_EQ(surfel_out->type().id, pg::kPointTypeSurfel);
}

TEST_F(NodesTest, NormalEstimationUsesPclPrecompiledPairs) {
  auto& registry = pg::PointTypeRegistry::Instance();
  const auto ids =
      pg::AlgorithmRegistry::Instance().TypesFor("normal_estimation");
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyzi), ids.end());
  EXPECT_EQ(std::find(ids.begin(), ids.end(), pg::kPointTypeXyziNormal),
            ids.end());

  auto xyzi_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  auto rgb_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  for (int i = 0; i < 12; ++i) {
    xyzi_cloud->push_back(
        pcl::PointXYZI{static_cast<float>(i % 2), static_cast<float>(i % 3),
                       static_cast<float>(i % 4), 1.0f});
    pcl::PointXYZRGB point;
    point.x = static_cast<float>(i % 2);
    point.y = static_cast<float>(i % 3);
    point.z = static_cast<float>(i % 4);
    point.r = 10;
    point.g = 20;
    point.b = 30;
    rgb_cloud->push_back(point);
  }

  const auto xyzi_data = std::make_shared<pg::CloudData>(
      xyzi_cloud, registry.Find(pg::kPointTypeXyzi));
  pg::NormalEstimationNode xyzi_node;
  ASSERT_TRUE(SetProp(
      xyzi_node, "input_type",
      AlgorithmTypeIndex("normal_estimation", pg::kPointTypeXyzi)));
  xyzi_node.setInPortData(0, xyzi_data);
  xyzi_node.compute();
  const auto xyzi_out =
      std::dynamic_pointer_cast<pg::CloudData>(xyzi_node.getOutPortData(0));
  ASSERT_NE(xyzi_out, nullptr);
  EXPECT_EQ(xyzi_out->type().id, pg::kPointTypeXyzNormal);

  const auto rgb_data = std::make_shared<pg::CloudData>(
      rgb_cloud, registry.Find(pg::kPointTypeXyzRgb));
  pg::NormalEstimationNode rgb_node;
  ASSERT_TRUE(SetProp(
      rgb_node, "input_type",
      AlgorithmTypeIndex("normal_estimation", pg::kPointTypeXyzRgb)));
  rgb_node.setInPortData(0, rgb_data);
  rgb_node.compute();
  const auto rgb_out =
      std::dynamic_pointer_cast<pg::CloudData>(rgb_node.getOutPortData(0));
  ASSERT_NE(rgb_out, nullptr);
  EXPECT_EQ(rgb_out->type().id, pg::kPointTypeXyzRgbNormal);
}

TEST_F(NodesTest, ReaderInfersXyzLFromHeader) {
  const auto path = dir_ / "labeled.pcd";
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZL>>();
  cloud->push_back(pcl::PointXYZL{0.0f, 0.0f, 0.0f, 1u});
  cloud->push_back(pcl::PointXYZL{1.0f, 1.0f, 1.0f, 2u});
  pcl::PCDWriter writer;
  ASSERT_EQ(writer.write(path.string(), *cloud, false), 0);

  pg::PointCloudReaderNode reader;
  ASSERT_TRUE(SetProp(reader, "path", path.string()));
  ASSERT_NE(reader.outputPort(0), nullptr);
  EXPECT_EQ(reader.outputPort(0)->data_type.id, pg::kPointTypeXyzL);
  reader.compute();
  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(reader.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->as<pcl::PointXYZL>()->size(), 2u);
}

TEST_F(NodesTest, BilateralFilterUsesPclTypeList) {
  const auto ids =
      pg::AlgorithmRegistry::Instance().TypesFor("bilateral_filter");
  ASSERT_EQ(ids.size(), 2u);
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyzi), ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyziNormal),
            ids.end());

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  cloud->width = 4;
  cloud->height = 4;
  cloud->is_dense = true;
  cloud->resize(16);
  for (int y = 0; y < 4; ++y) {
    for (int x = 0; x < 4; ++x) {
      cloud->at(x, y) = pcl::PointXYZI{static_cast<float>(x),
                                       static_cast<float>(y), 0.0f, 1.0f};
    }
  }
  const auto input = std::make_shared<pg::CloudData>(
      cloud, pg::PointTypeRegistry::Instance().Find(pg::kPointTypeXyzi));

  pg::BilateralFilterNode node;
  ASSERT_TRUE(SetProp(
      node, "point_type",
      AlgorithmTypeIndex("bilateral_filter", pg::kPointTypeXyzi)));
  node.setInPortData(0, input);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type().id, pg::kPointTypeXyzi);
  EXPECT_EQ(output->as<pcl::PointXYZI>()->size(), 16u);
}

TEST_F(NodesTest, FastBilateralFilterRequiresOrganized) {
  const auto ids =
      pg::AlgorithmRegistry::Instance().TypesFor("fast_bilateral_filter");
  ASSERT_EQ(ids.size(), 3u);
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyz), ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyzRgb),
            ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyzRgba),
            ids.end());

  pg::FastBilateralFilterNode node;
  const auto flat = MakeXyzData();
  node.setInPortData(0, flat);
  EXPECT_THROW(node.compute(), tg::NodeError);

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->width = 4;
  cloud->height = 4;
  cloud->is_dense = true;
  cloud->resize(16);
  for (int y = 0; y < 4; ++y) {
    for (int x = 0; x < 4; ++x) {
      cloud->at(x, y) = pcl::PointXYZ{static_cast<float>(x),
                                      static_cast<float>(y), 0.0f};
    }
  }
  const auto input = std::make_shared<pg::CloudData>(
      cloud, pg::PointTypeRegistry::Instance().Find(pg::kPointTypeXyz));
  node.setInPortData(0, input);
  node.compute();
  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type().id, pg::kPointTypeXyz);
  EXPECT_EQ(output->as<pcl::PointXYZ>()->size(), 16u);
}

TEST_F(NodesTest, ShadowPointsFiltersUsingNormals) {
  const auto ids =
      pg::AlgorithmRegistry::Instance().TypesFor("shadow_points");
  ASSERT_EQ(ids.size(), 18u);
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyzi), ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeSurfel),
            ids.end());

  pg::ShadowPointsNode node;
  ASSERT_TRUE(SetProp(node, "threshold", 0.0));
  const auto cloud = MakeXyzData();
  const auto normals = MakeNormalData(12);
  node.setInPortData(0, cloud);
  node.setInPortData(1, normals);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type().id, pg::kPointTypeXyz);
  EXPECT_EQ(output->as<pcl::PointXYZ>()->size(), 12u);
}

TEST_F(NodesTest, CovarianceSamplingSelectsRequestedCount) {
  const auto ids =
      pg::AlgorithmRegistry::Instance().TypesFor("covariance_sampling");
  ASSERT_EQ(ids.size(), 5u);
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyz), ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyzRgba),
            ids.end());

  pg::CovarianceSamplingNode node;
  ASSERT_TRUE(SetProp(node, "number_samples", 5));
  const auto cloud = MakeXyzData();
  const auto normals = MakeNormalData(12);
  node.setInPortData(0, cloud);
  node.setInPortData(1, normals);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type().id, pg::kPointTypeXyz);
  EXPECT_EQ(output->as<pcl::PointXYZ>()->size(), 5u);
}

TEST_F(NodesTest, NormalsInputsAreValidated) {
  const auto cloud = MakeXyzData();

  pg::ShadowPointsNode shadow;
  shadow.setInPortData(0, cloud);
  EXPECT_THROW(shadow.compute(), tg::NodeError);

  const auto short_normals = MakeNormalData(11);
  shadow.setInPortData(1, short_normals);
  EXPECT_THROW(shadow.compute(), tg::NodeError);

  pg::CovarianceSamplingNode covariance;
  ASSERT_TRUE(SetProp(covariance, "number_samples", 100));
  const auto normals = MakeNormalData(12);
  covariance.setInPortData(0, cloud);
  covariance.setInPortData(1, normals);
  EXPECT_THROW(covariance.compute(), tg::NodeError);
}

TEST_F(NodesTest, FpfhEstimationComputesSignatures) {
  const auto ids = pg::AlgorithmRegistry::Instance().TypesFor("fpfh");
  ASSERT_EQ(ids.size(), 5u);
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyz), ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyzi), ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyzRgb),
            ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyzRgba),
            ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyzNormal),
            ids.end());
  EXPECT_EQ(std::find(ids.begin(), ids.end(), pg::kPointTypeSurfel),
            ids.end());

  pg::FpfhEstimationNode node;
  const auto cloud = MakeXyzData();
  const auto normals = MakeNormalData(12);
  node.setInPortData(0, cloud);
  node.setInPortData(1, normals);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type().id, pg::kPointTypeFpfhSignature33);
  EXPECT_EQ(output->as<pcl::FPFHSignature33>()->size(), 12u);
}

TEST_F(NodesTest, FpfhValidatesNormals) {
  pg::FpfhEstimationNode node;
  node.setInPortData(0, MakeXyzData());
  EXPECT_THROW(node.compute(), tg::NodeError);

  const auto short_normals = MakeNormalData(11);
  node.setInPortData(1, short_normals);
  EXPECT_THROW(node.compute(), tg::NodeError);
}

TEST(TransformData, ExposesTypeAndMatrix) {
  pg::TransformData data;
  EXPECT_EQ(data.type().id, "transform");
  EXPECT_EQ(data.type().name, "Transform");
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix(0, 3) = 0.5f;
  data.setMatrix(matrix);
  EXPECT_FLOAT_EQ(data.matrix()(0, 3), 0.5f);
}

TEST(TransformData, SummarizesTranslationAndRotation) {
  constexpr float kHalfTurn = 3.14159265358979323846f / 2.0f;
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix.block<3, 1>(0, 3) << 0.1f, -0.06f, 0.08f;
  matrix.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(kHalfTurn, Eigen::Vector3f::UnitZ()).toRotationMatrix();

  const pg::TransformData data(matrix);
  const std::string text = data.debugSummary();
  EXPECT_NE(text.find("t=(0.10,-0.06,0.08)"), std::string::npos) << text;
  EXPECT_NE(text.find("angle=90.0deg"), std::string::npos) << text;
  EXPECT_NE(text.find("axis=("), std::string::npos) << text;
}

TEST(PointTypeRegistry, RegistersSummaryForEveryType) {
  for (const auto& desc : pg::PointTypeRegistry::Instance().All()) {
    EXPECT_FALSE(desc.display_name.empty()) << desc.id;
    EXPECT_NE(desc.summary, nullptr) << desc.id;
  }
}

TEST(CloudDataSummary, ReportsPointsAndFields) {
  const auto xyz = MakeXyzData();
  EXPECT_EQ(xyz->pointCount(), 12u);
  const std::string xyz_text = xyz->debugSummary();
  EXPECT_NE(xyz_text.find("PointXYZ"), std::string::npos) << xyz_text;
  EXPECT_NE(xyz_text.find("12 pts"), std::string::npos) << xyz_text;
  EXPECT_NE(xyz_text.find("unorganized"), std::string::npos) << xyz_text;
  EXPECT_NE(xyz_text.find("fields=xyz"), std::string::npos) << xyz_text;

  const auto normals = MakeNormalData(12);
  EXPECT_EQ(normals->pointCount(), 12u);
  const std::string normal_text = normals->debugSummary();
  EXPECT_NE(normal_text.find("12 pts"), std::string::npos) << normal_text;
  EXPECT_NE(normal_text.find("normal"), std::string::npos) << normal_text;
  EXPECT_NE(normal_text.find("curvature"), std::string::npos) << normal_text;

  const auto fpfh = MakeFpfhData(7);
  EXPECT_EQ(fpfh->pointCount(), 7u);
  const std::string fpfh_text = fpfh->debugSummary();
  EXPECT_NE(fpfh_text.find("FPFHSignature33"), std::string::npos) << fpfh_text;
  EXPECT_NE(fpfh_text.find("7 features"), std::string::npos) << fpfh_text;
  EXPECT_NE(fpfh_text.find("fields=fpfh33"), std::string::npos) << fpfh_text;
}

TEST(CloudDataSummary, ReportsOrganizedGrid) {
  const auto organized = MakeCloudData(MakeOrganizedXyzCloud());
  EXPECT_EQ(organized->pointCount(), 16u);
  const std::string text = organized->debugSummary();
  EXPECT_NE(text.find("16 pts"), std::string::npos) << text;
  EXPECT_NE(text.find("organized 4x4"), std::string::npos) << text;
}

TEST_F(NodesTest, ComputeAndUpdateDataLogsCloudSummary) {
  tg::clearLogRecords();
  tg::SetLogConfig(tg::LogConfig{tg::LogLevel::Info, tg::kAllLogCategories});

  pg::VoxelGridNode grid;
  ASSERT_TRUE(SetProp(grid, "leaf_x", 2.0));
  ASSERT_TRUE(SetProp(grid, "leaf_y", 2.0));
  ASSERT_TRUE(SetProp(grid, "leaf_z", 2.0));
  const auto input = MakeXyzData();
  grid.setInPortData(0, input);
  grid.computeAndUpdateData();

  std::vector<tg::LogRecord> records;
  tg::copyLogRecordsSince(0, records);
  tg::SetLogConfig(tg::LogConfig{});
  tg::clearLogRecords();

  EXPECT_NE(
      FindLogRecord(records, tg::LogCategory::Data, "property 'leaf_x'="),
      nullptr);
  const tg::LogRecord* output =
      FindLogRecord(records, tg::LogCategory::Data, "out[0]");
  ASSERT_NE(output, nullptr);
  EXPECT_NE(output->text.find("PointXYZ("), std::string::npos) << output->text;
  EXPECT_NE(FindLogRecord(records, tg::LogCategory::Timing, "took"), nullptr);
}

TEST_F(NodesTest, IcpRecoversTranslation) {
  const auto ids =
      pg::AlgorithmRegistry::Instance().TypesFor("icp_point_to_point");
  ASSERT_EQ(ids.size(), 3u);
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyz), ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyzi), ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyzRgb),
            ids.end());

  auto source_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (float x : {-0.2f, 0.2f}) {
    for (float y : {-0.2f, 0.2f}) {
      for (float z : {-0.2f, 0.2f}) {
        source_cloud->push_back(pcl::PointXYZ{x, y, z});
      }
    }
  }
  source_cloud->push_back(pcl::PointXYZ{0.01f, 0.02f, 0.03f});

  auto target_cloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*source_cloud);
  for (auto& point : target_cloud->points) {
    point.x += 0.05f;
    point.y += 0.02f;
  }

  const auto source = MakeCloudData(source_cloud);
  const auto target = MakeCloudData(target_cloud);

  pg::IcpPointToPointNode node;
  ASSERT_TRUE(SetProp(node, "max_correspondence_distance", 1.0));
  ASSERT_TRUE(SetProp(node, "max_iterations", 50));
  node.setInPortData(0, source);
  node.setInPortData(1, target);
  node.compute();

  const auto aligned =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(aligned, nullptr);
  EXPECT_EQ(aligned->type().id, pg::kPointTypeXyz);
  EXPECT_EQ(aligned->as<pcl::PointXYZ>()->size(), 9u);

  const auto transform =
      std::dynamic_pointer_cast<pg::TransformData>(node.getOutPortData(1));
  ASSERT_NE(transform, nullptr);
  EXPECT_NEAR(transform->matrix()(0, 3), 0.05, 0.02);
  EXPECT_NEAR(transform->matrix()(1, 3), 0.02, 0.02);
}

TEST_F(NodesTest, IcpValidatesInputs) {
  const auto source = MakeXyzData();

  pg::IcpPointToPointNode node;
  node.setInPortData(0, source);
  EXPECT_THROW(node.compute(), tg::NodeError);

  auto xyzi_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  xyzi_cloud->push_back(pcl::PointXYZI{0.0f, 0.0f, 0.0f, 1.0f});
  const auto xyzi_target = std::make_shared<pg::CloudData>(
      xyzi_cloud, pg::PointTypeRegistry::Instance().Find(pg::kPointTypeXyzi));
  node.setInPortData(1, xyzi_target);
  EXPECT_THROW(node.compute(), tg::NodeError);
}

TEST_F(NodesTest, NdtRecoversTranslation) {
  const auto ids = pg::AlgorithmRegistry::Instance().TypesFor("ndt");
  ASSERT_EQ(ids.size(), 3u);
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyz), ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyzi), ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), pg::kPointTypeXyzRgb),
            ids.end());

  auto source_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int iy = 0; iy < 10; ++iy) {
    for (int ix = 0; ix < 10; ++ix) {
      source_cloud->push_back(
          pcl::PointXYZ{0.2f * static_cast<float>(ix),
                        0.2f * static_cast<float>(iy), 0.0f});
    }
  }
  auto target_cloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*source_cloud);
  for (auto& point : target_cloud->points) {
    point.x += 1.0f;
  }

  const auto source = MakeCloudData(source_cloud);
  const auto target = MakeCloudData(target_cloud);

  pg::NdtNode node;
  ASSERT_TRUE(SetProp(node, "resolution", 1.0));
  ASSERT_TRUE(SetProp(node, "step_size", 0.1));
  ASSERT_TRUE(SetProp(node, "max_iterations", 30));
  node.setInPortData(0, source);
  node.setInPortData(1, target);
  node.compute();

  const auto transform =
      std::dynamic_pointer_cast<pg::TransformData>(node.getOutPortData(0));
  ASSERT_NE(transform, nullptr);
  EXPECT_NEAR(transform->matrix()(0, 3), 1.0, 0.25);
}

TEST_F(NodesTest, TransformPointCloudAppliesMatrix) {
  const auto cloud = MakeXyzData();
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix(0, 3) = 1.0f;
  matrix(1, 3) = 2.0f;
  matrix(2, 3) = 3.0f;
  const auto transform = std::make_shared<pg::TransformData>(matrix);

  pg::TransformPointCloudNode node;
  node.setInPortData(0, cloud);
  node.setInPortData(1, transform);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type().id, pg::kPointTypeXyz);
  ASSERT_EQ(output->as<pcl::PointXYZ>()->size(), 12u);
  EXPECT_FLOAT_EQ(output->as<pcl::PointXYZ>()->at(0).x, 1.0f);
  EXPECT_FLOAT_EQ(output->as<pcl::PointXYZ>()->at(0).y, 2.0f);
  EXPECT_FLOAT_EQ(output->as<pcl::PointXYZ>()->at(0).z, 3.0f);
}

TEST_F(NodesTest, TransformPointCloudValidatesTransform) {
  pg::TransformPointCloudNode node;
  node.setInPortData(0, MakeXyzData());
  EXPECT_THROW(node.compute(), tg::NodeError);
}

TEST(DoubleData, ExposesTypeAndValue) {
  pg::DoubleData data;
  EXPECT_EQ(data.type().id, "double");
  EXPECT_EQ(data.type().name, "Double");
  data.setValue(0.25);
  EXPECT_DOUBLE_EQ(data.value(), 0.25);
}

TEST_F(NodesTest, ConcatenateFieldsAddsNormals) {
  pg::ConcatenateFieldsNode node;
  const auto cloud = MakeXyzData();
  const auto normals = MakeNormalData(12);
  node.setInPortData(0, cloud);
  node.setInPortData(1, normals);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type().id, pg::kPointTypeXyzNormal);
  ASSERT_EQ(output->as<pcl::PointNormal>()->size(), 12u);
  EXPECT_FLOAT_EQ(output->as<pcl::PointNormal>()->at(0).normal_z, 1.0f);

  auto xyzi_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int i = 0; i < 12; ++i) {
    xyzi_cloud->push_back(
        pcl::PointXYZI{static_cast<float>(i % 2), static_cast<float>(i % 3),
                       static_cast<float>(i % 4), 1.0f});
  }
  const auto xyzi_data = std::make_shared<pg::CloudData>(
      xyzi_cloud, pg::PointTypeRegistry::Instance().Find(
                      pg::kPointTypeXyzi));
  pg::ConcatenateFieldsNode xyzi_node;
  ASSERT_TRUE(SetProp(
      xyzi_node, "point_type",
      AlgorithmTypeIndex("concatenate_fields", pg::kPointTypeXyzi)));
  xyzi_node.setInPortData(0, xyzi_data);
  xyzi_node.setInPortData(1, normals);
  xyzi_node.compute();
  const auto xyzi_output =
      std::dynamic_pointer_cast<pg::CloudData>(xyzi_node.getOutPortData(0));
  ASSERT_NE(xyzi_output, nullptr);
  EXPECT_EQ(xyzi_output->type().id, pg::kPointTypeXyziNormal);
}

TEST_F(NodesTest, MergeConcatenatesSameType) {
  pg::PointCloudMergeNode node;
  const auto first = MakeXyzData();
  const auto second = MakeXyzData();
  node.setInPortData(0, first);
  node.setInPortData(1, second);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type().id, pg::kPointTypeXyz);
  EXPECT_EQ(output->as<pcl::PointXYZ>()->size(), 24u);
}

TEST_F(NodesTest, MergeRejectsDifferentTypes) {
  auto xyzi_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  xyzi_cloud->push_back(pcl::PointXYZI{0.0f, 0.0f, 0.0f, 1.0f});
  const auto xyzi_data = std::make_shared<pg::CloudData>(
      xyzi_cloud, pg::PointTypeRegistry::Instance().Find(
                      pg::kPointTypeXyzi));

  pg::PointCloudMergeNode node;
  const auto first = MakeXyzData();
  node.setInPortData(0, first);
  node.setInPortData(1, xyzi_data);
  EXPECT_THROW(node.compute(), tg::NodeError);
}

TEST_F(NodesTest, CalcModelResoEstimatesSpacing) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int y = 0; y < 10; ++y) {
    for (int x = 0; x < 10; ++x) {
      cloud->push_back(pcl::PointXYZ{0.1f * static_cast<float>(x),
                                     0.1f * static_cast<float>(y), 0.0f});
    }
  }
  const auto input = MakeCloudData(cloud);

  pg::CalcModelResoNode node;
  ASSERT_TRUE(SetProp(node, "k", 10));
  node.setInPortData(0, input);
  node.compute();

  const auto value =
      std::dynamic_pointer_cast<pg::DoubleData>(node.getOutPortData(0));
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(value->type().id, "double");
  EXPECT_NEAR(value->value(), 0.1, 0.02);

  pg::CalcModelResoNode empty;
  EXPECT_THROW(empty.compute(), tg::NodeError);
}

TEST_F(NodesTest, SacIaAlignsPointClouds) {
  const auto source = MakeXyzData();
  const auto target = MakeXyzData();
  const auto source_features = MakeFpfhData(12);
  const auto target_features = MakeFpfhData(12);

  pg::SacIaNode node;
  ASSERT_TRUE(SetProp(node, "max_iterations", 3));
  node.setInPortData(0, source);
  node.setInPortData(1, source_features);
  node.setInPortData(2, target);
  node.setInPortData(3, target_features);
  node.compute();

  const auto aligned =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  const auto transform =
      std::dynamic_pointer_cast<pg::TransformData>(node.getOutPortData(1));
  ASSERT_NE(aligned, nullptr);
  ASSERT_NE(transform, nullptr);
  EXPECT_EQ(aligned->type().id, pg::kPointTypeXyz);
  EXPECT_EQ(aligned->as<pcl::PointXYZ>()->size(), 12u);
  EXPECT_TRUE(transform->matrix().allFinite());

  pg::SacIaNode missing;
  missing.setInPortData(0, source);
  missing.setInPortData(2, target);
  EXPECT_THROW(missing.compute(), tg::NodeError);
}

TEST_F(NodesTest, SiftKeypointFindsPoints) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int z = 0; z < 3; ++z) {
    for (int y = 0; y < 10; ++y) {
      for (int x = 0; x < 10; ++x) {
        pcl::PointXYZI point;
        point.x = 0.02f * static_cast<float>(x);
        point.y = 0.02f * static_cast<float>(y);
        point.z = 0.02f * static_cast<float>(z);
        point.intensity = static_cast<float>(z * 10 + x + y);
        cloud->push_back(point);
      }
    }
  }
  const auto input = std::make_shared<pg::CloudData>(
      cloud, pg::PointTypeRegistry::Instance().Find(pg::kPointTypeXyzi));

  pg::SiftKeypointNode node;
  node.setInPortData(0, input);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type().id, pg::kPointTypeXyz);
  EXPECT_LE(output->as<pcl::PointXYZ>()->size(), cloud->size());
}

TEST_F(NodesTest, IssKeypointFindsPoints) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int z = 0; z < 8; ++z) {
    for (int y = 0; y < 8; ++y) {
      for (int x = 0; x < 8; ++x) {
        cloud->push_back(pcl::PointXYZ{0.02f * static_cast<float>(x),
                                       0.02f * static_cast<float>(y),
                                       0.02f * static_cast<float>(z)});
      }
    }
  }
  const auto input = MakeCloudData(cloud);

  pg::IssKeypoint3DNode node;
  ASSERT_TRUE(SetProp(node, "salient_radius", 0.08));
  ASSERT_TRUE(SetProp(node, "non_max_radius", 0.04));
  ASSERT_TRUE(SetProp(node, "normal_radius", 0.08));
  ASSERT_TRUE(SetProp(node, "border_radius", 0.04));
  node.setInPortData(0, input);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type().id, pg::kPointTypeXyz);
  EXPECT_LE(output->as<pcl::PointXYZ>()->size(), cloud->size());
}

TEST_F(NodesTest, HarrisKeypointNeedsNormals) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int y = 0; y < 10; ++y) {
    for (int x = 0; x < 10; ++x) {
      cloud->push_back(pcl::PointXYZ{0.02f * static_cast<float>(x),
                                     0.02f * static_cast<float>(y),
                                     0.01f * static_cast<float>((x + y) % 3)});
    }
  }
  const auto input = MakeCloudData(cloud);
  const auto normals = MakeNormalData(cloud->size());

  pg::HarrisKeypoint3DNode node;
  ASSERT_TRUE(SetProp(node, "non_max_suppression", false));
  node.setInPortData(0, input);
  node.setInPortData(1, normals);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type().id, pg::kPointTypeXyzi);
  EXPECT_LE(output->as<pcl::PointXYZI>()->size(), cloud->size());

  pg::HarrisKeypoint3DNode missing;
  missing.setInPortData(0, input);
  EXPECT_THROW(missing.compute(), tg::NodeError);
}

TEST_F(NodesTest, NormalSpaceSamplingNeedsMatchingNormals) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int i = 0; i < 100; ++i) {
    cloud->push_back(pcl::PointXYZ{0.01f * static_cast<float>(i % 10),
                                   0.01f * static_cast<float>(i / 10),
                                   0.0f});
  }
  const auto input = MakeCloudData(cloud);
  const auto normals = MakeNormalData(cloud->size());

  pg::NormalSpaceSamplingNode node;
  ASSERT_TRUE(SetProp(node, "sample", 32));
  ASSERT_TRUE(SetProp(node, "bins_x", 4));
  ASSERT_TRUE(SetProp(node, "bins_y", 4));
  ASSERT_TRUE(SetProp(node, "bins_z", 4));
  node.setInPortData(0, input);
  node.setInPortData(1, normals);
  node.compute();

  const auto output =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type().id, pg::kPointTypeXyz);
  EXPECT_LE(output->as<pcl::PointXYZ>()->size(), 32u);
  EXPECT_GT(output->as<pcl::PointXYZ>()->size(), 0u);

  const auto wrong_normals = MakeNormalData(cloud->size() - 1);
  pg::NormalSpaceSamplingNode bad;
  bad.setInPortData(0, input);
  bad.setInPortData(1, wrong_normals);
  EXPECT_THROW(bad.compute(), tg::NodeError);
}

TEST_F(NodesTest, FpcsAlignsSmallCloud) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int y = 0; y < 6; ++y) {
    for (int x = 0; x < 6; ++x) {
      cloud->push_back(pcl::PointXYZ{0.05f * static_cast<float>(x),
                                     0.05f * static_cast<float>(y),
                                     0.01f * static_cast<float>((x * y) % 3)});
    }
  }
  const auto source = MakeCloudData(cloud);
  const auto target = MakeCloudData(cloud);

  pg::FpcsNode node;
  ASSERT_TRUE(SetProp(node, "ransac_iterations", 10));
  ASSERT_TRUE(SetProp(node, "max_runtime", 1));
  node.setInPortData(0, source);
  node.setInPortData(1, target);
  node.compute();

  const auto aligned =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  const auto transform =
      std::dynamic_pointer_cast<pg::TransformData>(node.getOutPortData(1));
  ASSERT_NE(aligned, nullptr);
  ASSERT_NE(transform, nullptr);
  EXPECT_EQ(aligned->type().id, pg::kPointTypeXyz);
  EXPECT_TRUE(transform->matrix().allFinite());

  pg::FpcsNode missing;
  missing.setInPortData(0, source);
  EXPECT_THROW(missing.compute(), tg::NodeError);

  pg::FpcsNode missing_normals;
  ASSERT_TRUE(SetProp(missing_normals, "use_normals", true));
  missing_normals.setInPortData(0, source);
  missing_normals.setInPortData(1, target);
  EXPECT_THROW(missing_normals.compute(), tg::NodeError);
}

TEST_F(NodesTest, IcpPointToPlaneAlignsPointNormals) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  for (int i = 0; i < 30; ++i) {
    const float angle = 0.37f * static_cast<float>(i);
    const float x = 0.3f * std::cos(angle);
    const float y = 0.3f * std::sin(angle);
    const float z = 0.02f * static_cast<float>(i % 7);
    pcl::PointNormal point;
    point.x = x;
    point.y = y;
    point.z = z;
    const float length = std::sqrt(x * x + y * y + (z + 0.1f) * (z + 0.1f));
    point.normal_x = x / length;
    point.normal_y = y / length;
    point.normal_z = (z + 0.1f) / length;
    point.curvature = 0.0f;
    cloud->push_back(point);
  }
  const auto input = std::make_shared<pg::CloudData>(
      cloud,
      pg::PointTypeRegistry::Instance().Find(pg::kPointTypeXyzNormal));

  pg::IcpPointToPlaneNode node;
  ASSERT_TRUE(SetProp(node, "max_iterations", 5));
  ASSERT_TRUE(SetProp(node, "max_correspondence_distance", 0.2));
  node.setInPortData(0, input);
  node.setInPortData(1, input);
  node.compute();

  const auto aligned =
      std::dynamic_pointer_cast<pg::CloudData>(node.getOutPortData(0));
  const auto transform =
      std::dynamic_pointer_cast<pg::TransformData>(node.getOutPortData(1));
  ASSERT_NE(aligned, nullptr);
  ASSERT_NE(transform, nullptr);
  EXPECT_EQ(aligned->type().id, pg::kPointTypeXyzNormal);
  EXPECT_EQ(aligned->as<pcl::PointNormal>()->size(), cloud->size());
  EXPECT_TRUE(transform->matrix().allFinite());
}

TEST_F(NodesTest, RegistersNodeTypes) {
  tg::Controller controller;
  pg::RegisterPclNodes(controller);
  EXPECT_TRUE(controller.hasNodeType("pcl.io.reader"));
  EXPECT_TRUE(controller.hasNodeType("pcl.io.writer"));
  EXPECT_TRUE(controller.hasNodeType("pcl.io.concatenate_fields"));
  EXPECT_TRUE(controller.hasNodeType("pcl.io.merge"));
  EXPECT_TRUE(controller.hasNodeType("pcl.filter.voxel_grid"));
  EXPECT_TRUE(controller.hasNodeType("pcl.filter.passthrough"));
  EXPECT_TRUE(controller.hasNodeType("pcl.filter.median_filter"));
  EXPECT_TRUE(controller.hasNodeType("pcl.filter.radius_outlier_removal"));
  EXPECT_TRUE(controller.hasNodeType("pcl.filter.crop_box"));
  EXPECT_TRUE(controller.hasNodeType("pcl.filter.approximate_voxel_grid"));
  EXPECT_TRUE(controller.hasNodeType("pcl.filter.uniform_sampling"));
  EXPECT_TRUE(controller.hasNodeType("pcl.filter.remove_nan"));
  EXPECT_TRUE(controller.hasNodeType("pcl.filter.bilateral_filter"));
  EXPECT_TRUE(controller.hasNodeType("pcl.filter.fast_bilateral_filter"));
  EXPECT_TRUE(controller.hasNodeType("pcl.filter.shadow_points"));
  EXPECT_TRUE(controller.hasNodeType("pcl.filter.covariance_sampling"));
  EXPECT_TRUE(controller.hasNodeType("pcl.features.normal_estimation"));
  EXPECT_TRUE(controller.hasNodeType("pcl.features.fpfh"));
  EXPECT_TRUE(controller.hasNodeType("pcl.tool.convert"));
  EXPECT_TRUE(controller.hasNodeType("pcl.registration.icp_point_to_point"));
  EXPECT_TRUE(controller.hasNodeType("pcl.registration.ndt"));
  EXPECT_TRUE(controller.hasNodeType("pcl.registration.sac_ia"));
  EXPECT_TRUE(controller.hasNodeType("pcl.tool.transform_point_cloud"));
  EXPECT_TRUE(controller.hasNodeType("pcl.tool.calc_model_reso"));

  EXPECT_TRUE(controller.hasNodeType("pcl.keypoints.sift"));
  EXPECT_TRUE(controller.hasNodeType("pcl.keypoints.iss_3d"));
  EXPECT_TRUE(controller.hasNodeType("pcl.keypoints.harris_3d"));
  EXPECT_TRUE(controller.hasNodeType("pcl.filter.normal_space_sampling"));
  EXPECT_TRUE(controller.hasNodeType("pcl.registration.fpcs"));
  EXPECT_TRUE(controller.hasNodeType("pcl.registration.icp_point_to_plane"));

  EXPECT_EQ(CategoryOf(controller, "pcl.io.reader"), pg::kCategoryIo);
  EXPECT_EQ(CategoryOf(controller, "pcl.io.writer"), pg::kCategoryIo);
  EXPECT_EQ(CategoryOf(controller, "pcl.io.concatenate_fields"),
            pg::kCategoryIo);
  EXPECT_EQ(CategoryOf(controller, "pcl.io.merge"), pg::kCategoryIo);

  EXPECT_EQ(CategoryOf(controller, "pcl.filter.voxel_grid"),
            pg::kCategoryFilterSampling);
  EXPECT_EQ(CategoryOf(controller, "pcl.filter.approximate_voxel_grid"),
            pg::kCategoryFilterSampling);
  EXPECT_EQ(CategoryOf(controller, "pcl.filter.uniform_sampling"),
            pg::kCategoryFilterSampling);
  EXPECT_EQ(CategoryOf(controller, "pcl.filter.covariance_sampling"),
            pg::kCategoryFilterSampling);

  EXPECT_EQ(CategoryOf(controller, "pcl.filter.median_filter"),
            pg::kCategoryFilterDenoising);
  EXPECT_EQ(CategoryOf(controller, "pcl.filter.radius_outlier_removal"),
            pg::kCategoryFilterDenoising);
  EXPECT_EQ(CategoryOf(controller, "pcl.filter.bilateral_filter"),
            pg::kCategoryFilterDenoising);
  EXPECT_EQ(CategoryOf(controller, "pcl.filter.fast_bilateral_filter"),
            pg::kCategoryFilterDenoising);
  EXPECT_EQ(CategoryOf(controller, "pcl.filter.shadow_points"),
            pg::kCategoryFilterDenoising);

  EXPECT_EQ(CategoryOf(controller, "pcl.filter.passthrough"),
            pg::kCategoryFilterRoi);
  EXPECT_EQ(CategoryOf(controller, "pcl.filter.crop_box"),
            pg::kCategoryFilterRoi);

  EXPECT_EQ(CategoryOf(controller, "pcl.filter.remove_nan"),
            pg::kCategoryFilterTool);

  EXPECT_EQ(CategoryOf(controller, "pcl.features.normal_estimation"),
            pg::kCategoryFeatures);
  EXPECT_EQ(CategoryOf(controller, "pcl.features.fpfh"),
            pg::kCategoryFeatures);

  EXPECT_EQ(CategoryOf(controller, "pcl.registration.icp_point_to_point"),
            pg::kCategoryRegistration);
  EXPECT_EQ(CategoryOf(controller, "pcl.registration.ndt"),
            pg::kCategoryRegistration);
  EXPECT_EQ(CategoryOf(controller, "pcl.registration.sac_ia"),
            pg::kCategoryRegistration);

  EXPECT_EQ(CategoryOf(controller, "pcl.tool.convert"), pg::kCategoryTool);
  EXPECT_EQ(CategoryOf(controller, "pcl.tool.transform_point_cloud"),
            pg::kCategoryTool);
  EXPECT_EQ(CategoryOf(controller, "pcl.tool.calc_model_reso"),
            pg::kCategoryTool);

  EXPECT_EQ(CategoryOf(controller, "pcl.keypoints.sift"),
            pg::kCategoryKeypoints);
  EXPECT_EQ(CategoryOf(controller, "pcl.keypoints.iss_3d"),
            pg::kCategoryKeypoints);
  EXPECT_EQ(CategoryOf(controller, "pcl.keypoints.harris_3d"),
            pg::kCategoryKeypoints);
  EXPECT_EQ(CategoryOf(controller, "pcl.filter.normal_space_sampling"),
            pg::kCategoryFilterSampling);
  EXPECT_EQ(CategoryOf(controller, "pcl.registration.fpcs"),
            pg::kCategoryRegistration);
  EXPECT_EQ(CategoryOf(controller, "pcl.registration.icp_point_to_plane"),
            pg::kCategoryRegistration);

  for (const auto& type : controller.nodeTypes()) {
    EXPECT_NE(type.category, "3D/Filter") << type.type_id;
  }
}

TEST_F(NodesTest, StrictTypeMatchingRejectsMismatchedEdge) {
  const auto path = dir_ / "cloud.pcd";
  ASSERT_TRUE(WriteXyzPcd(path));

  auto reader = std::make_shared<pg::PointCloudReaderNode>();
  ASSERT_TRUE(SetProp(*reader, "path", path.string()));
  reader->refreshPortTypes();

  auto grid = std::make_shared<pg::VoxelGridNode>();
  ASSERT_TRUE(SetProp(*grid, "point_type", 3));

  tg::Graph graph("strict");
  graph.addNode(reader);
  graph.addNode(grid);
  EXPECT_FALSE(graph.connect<pg::CloudData>(reader->outport.cloud,
                                            grid->inport.cloud));

  ASSERT_TRUE(SetProp(*grid, "point_type", 0));
  EXPECT_TRUE(graph.connect<pg::CloudData>(reader->outport.cloud,
                                           grid->inport.cloud));
}

TEST_F(NodesTest, RunsReaderGridWriterGraph) {
  const auto input_path = dir_ / "in.pcd";
  const auto output_path = dir_ / "out_graph.pcd";
  ASSERT_TRUE(WriteXyzPcd(input_path));

  auto reader = std::make_shared<pg::PointCloudReaderNode>();
  ASSERT_TRUE(SetProp(*reader, "path", input_path.string()));
  reader->refreshPortTypes();

  auto grid = std::make_shared<pg::VoxelGridNode>();
  ASSERT_TRUE(SetProp(*grid, "leaf_x", 2.0));
  ASSERT_TRUE(SetProp(*grid, "leaf_y", 2.0));
  ASSERT_TRUE(SetProp(*grid, "leaf_z", 2.0));

  auto writer = std::make_shared<pg::PointCloudWriterNode>();
  ASSERT_TRUE(SetProp(*writer, "path", output_path.string()));
  ASSERT_TRUE(SetProp(*writer, "input_type", 0));

  tg::Controller controller;
  auto graph = controller.createGraph("run");
  graph->addNode(reader);
  graph->addNode(grid);
  graph->addNode(writer);
  ASSERT_TRUE(graph->connect<pg::CloudData>(reader->outport.cloud,
                                            grid->inport.cloud));
  ASSERT_TRUE(graph->connect<pg::CloudData>(grid->outport.cloud,
                                            writer->inport.cloud));

  controller.run(*graph);
  EXPECT_TRUE(std::filesystem::exists(output_path));
}

}  // namespace
