#include <gtest/gtest.h>

#include <memory>

#include <pclgraph/algorithm_registry.hpp>
#include <pclgraph/point_type.hpp>
#include <taskgraph/Error.hpp>

namespace {

std::shared_ptr<pg::CloudData> Identity(const pg::CloudData& input,
                                        const void* /*params*/) {
  return std::make_shared<pg::CloudData>(input.cloud(), input.desc());
}

struct IdentityRegistration {
  IdentityRegistration() {
    PG_INSTANTIATE("identity", pg::kPointTypeXyz, &Identity);
  }
};

const IdentityRegistration kIdentityRegistration;

}  // namespace

TEST(PointTypeRegistry, RegistersBuiltinTypes) {
  auto& registry = pg::PointTypeRegistry::Instance();
  EXPECT_TRUE(registry.Contains(pg::kPointTypeXyz));
  EXPECT_TRUE(registry.Contains(pg::kPointTypeXyzi));
  EXPECT_TRUE(registry.Contains(pg::kPointTypeXyzL));
  EXPECT_TRUE(registry.Contains(pg::kPointTypeXyzRgb));
  EXPECT_TRUE(registry.Contains(pg::kPointTypeXyzRgbNormal));
  EXPECT_TRUE(registry.Contains(pg::kPointTypeXyzNormal));
  EXPECT_TRUE(registry.Contains(pg::kPointTypeXyziNormal));
  EXPECT_TRUE(registry.Contains(pg::kPointTypeSurfel));
  EXPECT_TRUE(registry.Contains(pg::kPointTypeNormal));
  EXPECT_TRUE(registry.Contains(pg::kPointTypeFpfhSignature33));
  EXPECT_EQ(registry.Find("missing"), nullptr);
  EXPECT_GE(registry.All().size(), 19u);

  const auto* fpfh = registry.Find(pg::kPointTypeFpfhSignature33);
  ASSERT_NE(fpfh, nullptr);
  EXPECT_EQ(fpfh->display_name, "FPFHSignature33");
}

TEST(PointTypeRegistry, DerivesFieldSets) {
  auto& registry = pg::PointTypeRegistry::Instance();

  const auto* xyz = registry.Find(pg::kPointTypeXyz);
  ASSERT_NE(xyz, nullptr);
  EXPECT_EQ(xyz->display_name, "PointXYZ");
  EXPECT_EQ(xyz->fields, pg::kFieldX | pg::kFieldY | pg::kFieldZ);

  const auto* xyzi = registry.Find(pg::kPointTypeXyzi);
  ASSERT_NE(xyzi, nullptr);
  EXPECT_EQ(xyzi->fields,
            pg::kFieldX | pg::kFieldY | pg::kFieldZ | pg::kFieldIntensity);

  const auto* rgb = registry.Find(pg::kPointTypeXyzRgb);
  ASSERT_NE(rgb, nullptr);
  EXPECT_EQ(rgb->fields,
            pg::kFieldX | pg::kFieldY | pg::kFieldZ | pg::kFieldRgb);

  const auto* normal = registry.Find(pg::kPointTypeXyzNormal);
  ASSERT_NE(normal, nullptr);
  EXPECT_EQ(normal->fields, pg::kFieldX | pg::kFieldY | pg::kFieldZ |
                                pg::kFieldNormalX | pg::kFieldNormalY |
                                pg::kFieldNormalZ | pg::kFieldCurvature);

  const auto* xyzi_normal = registry.Find(pg::kPointTypeXyziNormal);
  ASSERT_NE(xyzi_normal, nullptr);
  EXPECT_EQ(xyzi_normal->fields,
            pg::kFieldX | pg::kFieldY | pg::kFieldZ | pg::kFieldIntensity |
                pg::kFieldNormalX | pg::kFieldNormalY | pg::kFieldNormalZ |
                pg::kFieldCurvature);

  const auto* normal_only = registry.Find(pg::kPointTypeNormal);
  ASSERT_NE(normal_only, nullptr);
  EXPECT_EQ(normal_only->fields, pg::kFieldNormalX | pg::kFieldNormalY |
                                     pg::kFieldNormalZ | pg::kFieldCurvature);

  const auto* xyz_l = registry.Find(pg::kPointTypeXyzL);
  ASSERT_NE(xyz_l, nullptr);
  EXPECT_EQ(xyz_l->fields,
            pg::kFieldX | pg::kFieldY | pg::kFieldZ | pg::kFieldLabel);

  const auto* surfel = registry.Find(pg::kPointTypeSurfel);
  ASSERT_NE(surfel, nullptr);
  EXPECT_TRUE((surfel->fields & pg::kFieldRgba) != 0);
  EXPECT_TRUE((surfel->fields & pg::kFieldRadius) != 0);
  EXPECT_TRUE((surfel->fields & pg::kFieldConfidence) != 0);
}

TEST(CloudData, ExposesConcreteTypeAndTypedAccess) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->resize(2);

  pg::CloudData data(cloud, pg::PointTypeRegistry::Instance().Find(
                                pg::kPointTypeXyz));
  EXPECT_EQ(data.type().id, pg::kPointTypeXyz);
  EXPECT_EQ(data.type().name, "PointXYZ");

  const auto typed = data.as<pcl::PointXYZ>();
  ASSERT_NE(typed, nullptr);
  EXPECT_EQ(typed->size(), 2u);
  EXPECT_EQ(data.as<pcl::PointXYZI>(), nullptr);
}

TEST(CloudData, ConvertsThroughPclPointCloud2) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->push_back(pcl::PointXYZ{1.0f, 2.0f, 3.0f});
  cloud->push_back(pcl::PointXYZ{4.0f, 5.0f, 6.0f});

  const auto* desc =
      pg::PointTypeRegistry::Instance().Find(pg::kPointTypeXyz);
  ASSERT_NE(desc, nullptr);

  pcl::PCLPointCloud2 blob;
  desc->to_cloud2(cloud, blob);
  const auto round = desc->from_cloud2(blob);

  pg::CloudData data(round, desc);
  const auto typed = data.as<pcl::PointXYZ>();
  ASSERT_NE(typed, nullptr);
  ASSERT_EQ(typed->size(), 2u);
  EXPECT_FLOAT_EQ(typed->at(0).x, 1.0f);
  EXPECT_FLOAT_EQ(typed->at(1).z, 6.0f);
}

TEST(AlgorithmRegistry, DispatchesByConcreteType) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->resize(1);
  pg::CloudData data(cloud, pg::PointTypeRegistry::Instance().Find(
                                pg::kPointTypeXyz));

  const auto output = data.dispatch("identity");
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type().id, pg::kPointTypeXyz);
  EXPECT_EQ(output->as<pcl::PointXYZ>()->size(), 1u);

  EXPECT_THROW(data.dispatch("missing"), tg::NodeError);
}
