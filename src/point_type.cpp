#include "pclgraph/point_type.hpp"

#include <cstddef>
#include <string>
#include <utility>

#include <pcl/impl/instantiate.hpp>

namespace pg {

namespace {

struct FieldGroup {
  PointFields mask = 0;
  bool all = false;
  const char* name = "";
};

// 摘要里的字段短名；顺序即输出顺序。
constexpr FieldGroup kFieldGroups[] = {
    {kFieldX | kFieldY | kFieldZ, true, "xyz"},
    {kFieldNormalX | kFieldNormalY | kFieldNormalZ, true, "normal"},
    {kFieldIntensity, false, "intensity"},
    {kFieldRgb | kFieldRgba, false, "rgb"},
    {kFieldCurvature, false, "curvature"},
    {kFieldLabel, false, "label"},
    {kFieldH | kFieldS | kFieldV, true, "hsv"},
    {kFieldL | kFieldA | kFieldB, true, "lab"},
    {kFieldRange, false, "range"},
    {kFieldVpX | kFieldVpY | kFieldVpZ, true, "viewpoint"},
    {kFieldScale, false, "scale"},
    {kFieldStrength, false, "strength"},
    {kFieldRadius, false, "radius"},
    {kFieldConfidence, false, "confidence"},
    {kFieldIntensityVariance, false, "intensity_variance"},
};

constexpr char kSeparator = '+';

std::string FieldsSummary(const PointTypeDesc& desc) {
  if (desc.id == kPointTypeFpfhSignature33) {
    // FPFH 没有 XYZ 字段位，用类型特判，避免摘要里字段为空。
    return "fpfh33";
  }
  std::string text;
  for (const FieldGroup& group : kFieldGroups) {
    const bool present = group.all ? (desc.fields & group.mask) == group.mask
                                   : (desc.fields & group.mask) != 0;
    if (!present) {
      continue;
    }
    if (!text.empty()) {
      text += kSeparator;
    }
    text += group.name;
  }
  return text.empty() ? std::string("none") : text;
}

constexpr char kOrganizedMarker[] = ", organized ";
constexpr char kUnorganizedMarker[] = ", unorganized";
constexpr char kGridSeparator = 'x';
constexpr char kFeaturesSuffix[] = " features";
constexpr char kPointsSuffix[] = " pts";

std::string ShapeSummary(const PointCloudShape& shape, bool features) {
  std::string text = std::to_string(shape.points);
  if (features) {
    text += kFeaturesSuffix;
    return text;
  }
  text += kPointsSuffix;
  if (shape.height > 1) {
    text += kOrganizedMarker;
    text += std::to_string(shape.width);
    text += kGridSeparator;
    text += std::to_string(shape.height);
  } else {
    text += kUnorganizedMarker;
  }
  return text;
}

}  // namespace

#define PCL_INSTANTIATE_PG_REGISTER_TYPE(T) \
  static_assert(kPointTypeId<T>[0] != '\0', "missing point type id"); \
  Register(MakePointTypeDesc<T>(kPointTypeId<T>, kPointTypeName<T>));

PointTypeRegistry& PointTypeRegistry::Instance() {
  static PointTypeRegistry registry;
  return registry;
}

PointTypeRegistry::PointTypeRegistry() {
  // 与安装的 PCL 预编译列表保持一致。
  PCL_INSTANTIATE(PG_REGISTER_TYPE, PCL_XYZ_POINT_TYPES)
  Register(MakePointTypeDesc<pcl::Normal>(kPointTypeNormal, "Normal"));
  Register(MakePointTypeDesc<pcl::FPFHSignature33>(
      kPointTypeFpfhSignature33, "FPFHSignature33"));
}

void PointTypeRegistry::Register(PointTypeDesc desc) {
  if (_index.find(desc.id) != _index.end()) {
    return;
  }
  _index.emplace(desc.id, _types.size());
  _types.push_back(std::move(desc));
}

const PointTypeDesc* PointTypeRegistry::Find(std::string_view id) const {
  const auto it = _index.find(std::string(id));
  if (it == _index.end()) {
    return nullptr;
  }
  return &_types[it->second];
}

bool PointTypeRegistry::Contains(std::string_view id) const {
  return Find(id) != nullptr;
}

std::size_t CloudData::pointCount() const {
  if (_cloud == nullptr || _desc == nullptr || _desc->summary == nullptr) {
    return 0;
  }
  return _desc->summary(_cloud).points;
}

std::string CloudData::debugSummary() const {
  constexpr char kUnknownType[] = "CloudData (unknown type)";
  constexpr char kUnnamedType[] = "PointCloud";
  constexpr char kFieldsLabel[] = ", fields=";
  if (_desc == nullptr) {
    return kUnknownType;
  }
  PointCloudShape shape;
  if (_cloud != nullptr && _desc->summary != nullptr) {
    shape = _desc->summary(_cloud);
  }
  std::string text =
      _desc->display_name.empty() ? kUnnamedType : _desc->display_name;
  text += '(';
  text += ShapeSummary(shape, _desc->id == kPointTypeFpfhSignature33);
  text += kFieldsLabel;
  text += FieldsSummary(*_desc);
  text += ')';
  return text;
}

}  // namespace pg
