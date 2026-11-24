#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <taskgraph/NodeData.hpp>

namespace pg {

/** \brief 点云字段位集；只用于 Reader 推断与转换校验，不参与连线判断。 */
using PointFields = std::uint32_t;

inline constexpr PointFields kFieldX = 1u << 0;
inline constexpr PointFields kFieldY = 1u << 1;
inline constexpr PointFields kFieldZ = 1u << 2;
inline constexpr PointFields kFieldNormalX = 1u << 3;
inline constexpr PointFields kFieldNormalY = 1u << 4;
inline constexpr PointFields kFieldNormalZ = 1u << 5;
inline constexpr PointFields kFieldIntensity = 1u << 6;
inline constexpr PointFields kFieldRgb = 1u << 7;
inline constexpr PointFields kFieldRgba = 1u << 8;
inline constexpr PointFields kFieldCurvature = 1u << 9;
inline constexpr PointFields kFieldLabel = 1u << 10;
inline constexpr PointFields kFieldH = 1u << 11;
inline constexpr PointFields kFieldS = 1u << 12;
inline constexpr PointFields kFieldV = 1u << 13;
inline constexpr PointFields kFieldL = 1u << 14;
inline constexpr PointFields kFieldA = 1u << 15;
inline constexpr PointFields kFieldB = 1u << 16;
inline constexpr PointFields kFieldRange = 1u << 17;
inline constexpr PointFields kFieldVpX = 1u << 18;
inline constexpr PointFields kFieldVpY = 1u << 19;
inline constexpr PointFields kFieldVpZ = 1u << 20;
inline constexpr PointFields kFieldScale = 1u << 21;
inline constexpr PointFields kFieldStrength = 1u << 22;
inline constexpr PointFields kFieldRadius = 1u << 23;
inline constexpr PointFields kFieldConfidence = 1u << 24;
inline constexpr PointFields kFieldIntensityVariance = 1u << 25;

inline constexpr char kPointTypeXyz[] = "xyz";
inline constexpr char kPointTypeXyzi[] = "xyzi";
inline constexpr char kPointTypeXyzL[] = "xyzl";
inline constexpr char kPointTypeXyzRgba[] = "xyzrgba";
inline constexpr char kPointTypeXyzRgb[] = "xyzrgb";
inline constexpr char kPointTypeXyzRgbL[] = "xyzrgbl";
inline constexpr char kPointTypeXyzLab[] = "xyzlab";
inline constexpr char kPointTypeXyzHsv[] = "xyzhsv";
inline constexpr char kPointTypeInterest[] = "interest";
inline constexpr char kPointTypeXyzNormal[] = "xyznormal";
inline constexpr char kPointTypeXyzRgbNormal[] = "xyzrgbnormal";
inline constexpr char kPointTypeXyziNormal[] = "xyzinormal";
inline constexpr char kPointTypeXyzLNormal[] = "xyzl_normal";
inline constexpr char kPointTypeWithRange[] = "with_range";
inline constexpr char kPointTypeWithViewpoint[] = "with_viewpoint";
inline constexpr char kPointTypeWithScale[] = "with_scale";
inline constexpr char kPointTypeSurfel[] = "surfel";
inline constexpr char kPointTypeDem[] = "dem";
inline constexpr char kPointTypeNormal[] = "normal";
inline constexpr char kPointTypeFpfhSignature33[] = "fpfh_signature33";

/** \brief 点云形状摘要；只含元数据，不按点数分配。 */
struct PointCloudShape {
  std::size_t points = 0;
  std::size_t width = 0;
  std::size_t height = 0;
};

/** \brief 从类型擦除句柄读取形状；实现必须线程安全、无副作用。 */
using PointCloudShapeFn =
    PointCloudShape (*)(const std::shared_ptr<const void>&);

/** \brief 一个已注册点类型的运行时描述；地址在注册表中稳定。 */
struct PointTypeDesc {
  std::string id;
  std::string display_name;
  PointFields fields = 0;
  std::shared_ptr<const void> (*clone)(const std::shared_ptr<const void>&) =
      nullptr;
  void (*to_cloud2)(const std::shared_ptr<const void>&,
                    pcl::PCLPointCloud2&) = nullptr;
  std::shared_ptr<const void> (*from_cloud2)(const pcl::PCLPointCloud2&) =
      nullptr;
  PointCloudShapeFn summary = nullptr;
};

/** \brief 编译期点类型到运行时 id 的映射；未特化的类型 id 为空。 */
template <typename PointT>
inline constexpr const char* kPointTypeId = "";

template <>
inline constexpr const char* kPointTypeId<pcl::PointXYZ> = kPointTypeXyz;
template <>
inline constexpr const char* kPointTypeId<pcl::PointXYZI> = kPointTypeXyzi;
template <>
inline constexpr const char* kPointTypeId<pcl::PointXYZL> = kPointTypeXyzL;
template <>
inline constexpr const char* kPointTypeId<pcl::PointXYZRGBA> =
    kPointTypeXyzRgba;
template <>
inline constexpr const char* kPointTypeId<pcl::PointXYZRGB> = kPointTypeXyzRgb;
template <>
inline constexpr const char* kPointTypeId<pcl::PointXYZRGBL> =
    kPointTypeXyzRgbL;
template <>
inline constexpr const char* kPointTypeId<pcl::PointXYZLAB> =
    kPointTypeXyzLab;
template <>
inline constexpr const char* kPointTypeId<pcl::PointXYZHSV> =
    kPointTypeXyzHsv;
template <>
inline constexpr const char* kPointTypeId<pcl::InterestPoint> =
    kPointTypeInterest;
template <>
inline constexpr const char* kPointTypeId<pcl::PointNormal> =
    kPointTypeXyzNormal;
template <>
inline constexpr const char* kPointTypeId<pcl::PointXYZRGBNormal> =
    kPointTypeXyzRgbNormal;
template <>
inline constexpr const char* kPointTypeId<pcl::PointXYZINormal> =
    kPointTypeXyziNormal;
template <>
inline constexpr const char* kPointTypeId<pcl::PointXYZLNormal> =
    kPointTypeXyzLNormal;
template <>
inline constexpr const char* kPointTypeId<pcl::PointWithRange> =
    kPointTypeWithRange;
template <>
inline constexpr const char* kPointTypeId<pcl::PointWithViewpoint> =
    kPointTypeWithViewpoint;
template <>
inline constexpr const char* kPointTypeId<pcl::PointWithScale> =
    kPointTypeWithScale;
template <>
inline constexpr const char* kPointTypeId<pcl::PointSurfel> = kPointTypeSurfel;
template <>
inline constexpr const char* kPointTypeId<pcl::PointDEM> = kPointTypeDem;
template <>
inline constexpr const char* kPointTypeId<pcl::Normal> = kPointTypeNormal;
template <>
inline constexpr const char* kPointTypeId<pcl::FPFHSignature33> =
    kPointTypeFpfhSignature33;

/** \brief 编译期点类型到显示名的映射。 */
template <typename PointT>
inline constexpr const char* kPointTypeName = "";

template <>
inline constexpr const char* kPointTypeName<pcl::PointXYZ> = "PointXYZ";
template <>
inline constexpr const char* kPointTypeName<pcl::PointXYZI> = "PointXYZI";
template <>
inline constexpr const char* kPointTypeName<pcl::PointXYZL> = "PointXYZL";
template <>
inline constexpr const char* kPointTypeName<pcl::PointXYZRGBA> = "PointXYZRGBA";
template <>
inline constexpr const char* kPointTypeName<pcl::PointXYZRGB> = "PointXYZRGB";
template <>
inline constexpr const char* kPointTypeName<pcl::PointXYZRGBL> =
    "PointXYZRGBL";
template <>
inline constexpr const char* kPointTypeName<pcl::PointXYZLAB> = "PointXYZLAB";
template <>
inline constexpr const char* kPointTypeName<pcl::PointXYZHSV> = "PointXYZHSV";
template <>
inline constexpr const char* kPointTypeName<pcl::InterestPoint> =
    "InterestPoint";
template <>
inline constexpr const char* kPointTypeName<pcl::PointNormal> = "PointNormal";
template <>
inline constexpr const char* kPointTypeName<pcl::PointXYZRGBNormal> =
    "PointXYZRGBNormal";
template <>
inline constexpr const char* kPointTypeName<pcl::PointXYZINormal> =
    "PointXYZINormal";
template <>
inline constexpr const char* kPointTypeName<pcl::PointXYZLNormal> =
    "PointXYZLNormal";
template <>
inline constexpr const char* kPointTypeName<pcl::PointWithRange> =
    "PointWithRange";
template <>
inline constexpr const char* kPointTypeName<pcl::PointWithViewpoint> =
    "PointWithViewpoint";
template <>
inline constexpr const char* kPointTypeName<pcl::PointWithScale> =
    "PointWithScale";
template <>
inline constexpr const char* kPointTypeName<pcl::PointSurfel> = "PointSurfel";
template <>
inline constexpr const char* kPointTypeName<pcl::PointDEM> = "PointDEM";
template <>
inline constexpr const char* kPointTypeName<pcl::Normal> = "Normal";
template <>
inline constexpr const char* kPointTypeName<pcl::FPFHSignature33> =
    "FPFHSignature33";

/** \brief 由 pcl::traits::fieldList 派生字段位集。 */
template <typename PointT>
constexpr PointFields FieldsOf() {
  PointFields fields = 0;
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::x>::value) {
    fields |= kFieldX;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::y>::value) {
    fields |= kFieldY;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::z>::value) {
    fields |= kFieldZ;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::normal_x>::value) {
    fields |= kFieldNormalX;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::normal_y>::value) {
    fields |= kFieldNormalY;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::normal_z>::value) {
    fields |= kFieldNormalZ;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::intensity>::value) {
    fields |= kFieldIntensity;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::rgb>::value) {
    fields |= kFieldRgb;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::rgba>::value) {
    fields |= kFieldRgba;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::curvature>::value) {
    fields |= kFieldCurvature;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::label>::value) {
    fields |= kFieldLabel;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::h>::value) {
    fields |= kFieldH;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::s>::value) {
    fields |= kFieldS;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::v>::value) {
    fields |= kFieldV;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::L>::value) {
    fields |= kFieldL;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::a>::value) {
    fields |= kFieldA;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::b>::value) {
    fields |= kFieldB;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::range>::value) {
    fields |= kFieldRange;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::vp_x>::value) {
    fields |= kFieldVpX;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::vp_y>::value) {
    fields |= kFieldVpY;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::vp_z>::value) {
    fields |= kFieldVpZ;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::scale>::value) {
    fields |= kFieldScale;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::strength>::value) {
    fields |= kFieldStrength;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::radius>::value) {
    fields |= kFieldRadius;
  }
  if constexpr (pcl::traits::has_field<PointT, pcl::fields::confidence>::value) {
    fields |= kFieldConfidence;
  }
  if constexpr (pcl::traits::has_field<PointT,
                                       pcl::fields::intensity_variance>::value) {
    fields |= kFieldIntensityVariance;
  }
  return fields;
}

template <typename PointT>
std::shared_ptr<const void> CloneCloud(
    const std::shared_ptr<const void>& cloud) {
  return cloud;
}

template <typename PointT>
void ToCloud2(const std::shared_ptr<const void>& cloud,
              pcl::PCLPointCloud2& out) {
  const auto* typed =
      static_cast<const pcl::PointCloud<PointT>*>(cloud.get());
  pcl::toPCLPointCloud2(*typed, out);
}

template <typename PointT>
std::shared_ptr<const void> FromCloud2(const pcl::PCLPointCloud2& in) {
  auto cloud = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::fromPCLPointCloud2(in, *cloud);
  return cloud;
}

template <typename PointT>
PointCloudShape SummarizeCloud(const std::shared_ptr<const void>& cloud) {
  const auto* typed =
      static_cast<const pcl::PointCloud<PointT>*>(cloud.get());
  if (typed == nullptr) {
    return {};
  }
  return {static_cast<std::size_t>(typed->size()),
          static_cast<std::size_t>(typed->width),
          static_cast<std::size_t>(typed->height)};
}

template <typename PointT>
PointTypeDesc MakePointTypeDesc(std::string id, std::string display_name) {
  return PointTypeDesc{std::move(id), std::move(display_name),
                       FieldsOf<PointT>(), &CloneCloud<PointT>,
                       &ToCloud2<PointT>, &FromCloud2<PointT>,
                       &SummarizeCloud<PointT>};
}

/** \brief 运行时点类型注册表；首次访问时注册内置类型。 */
class PointTypeRegistry {
 public:
  static PointTypeRegistry& Instance();

  /** \brief 注册一个点类型；同 id 已存在时保留先注册的描述。 */
  void Register(PointTypeDesc desc);

  const PointTypeDesc* Find(std::string_view id) const;
  bool Contains(std::string_view id) const;
  const std::deque<PointTypeDesc>& All() const { return _types; }

 private:
  PointTypeRegistry();

  std::deque<PointTypeDesc> _types;
  std::unordered_map<std::string, std::size_t> _index;
};

/** \brief 图里传递的点云：类型擦除句柄 + 具体点类型描述符。 */
class CloudData final : public tg::NodeData {
 public:
  CloudData(std::shared_ptr<const void> cloud, const PointTypeDesc* desc)
      : _cloud(std::move(cloud)), _desc(desc) {}

  tg::NodeDataType type() const override {
    if (_desc == nullptr) {
      return {};
    }
    return tg::NodeDataType{_desc->id, _desc->display_name};
  }

  const PointTypeDesc* desc() const { return _desc; }
  const std::shared_ptr<const void>& cloud() const { return _cloud; }

  /** \brief 点数；无描述或缺形状函数时返回 0。 */
  std::size_t pointCount() const;

  /** \brief 日志用摘要：类型名、点数、organized 尺寸、字段。只读元数据。 */
  std::string debugSummary() const override;

  /** \brief 按具体点类型查分派表；未实例化时抛 tg::NodeError。 */
  std::shared_ptr<CloudData> dispatch(std::string_view algo_id,
                                      const void* params = nullptr) const;

  /** \brief 按编译期点类型取回云；类型不符时返回空。 */
  template <typename PointT>
  std::shared_ptr<const pcl::PointCloud<PointT>> as() const {
    if (_desc == nullptr || _desc->id != kPointTypeId<PointT>) {
      return nullptr;
    }
    return std::static_pointer_cast<const pcl::PointCloud<PointT>>(_cloud);
  }

 private:
  std::shared_ptr<const void> _cloud;
  const PointTypeDesc* _desc = nullptr;
};

}  // namespace pg
