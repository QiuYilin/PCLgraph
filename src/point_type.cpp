#include "pclgraph/point_type.hpp"

#include <utility>

#include <pcl/impl/instantiate.hpp>

namespace pg {

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

}  // namespace pg
