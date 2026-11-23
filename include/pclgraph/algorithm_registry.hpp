#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "pclgraph/point_type.hpp"

namespace pg {

/** \brief (算法 id, 点类型 id) -> 已实例化实现的分派表。 */
class AlgorithmRegistry {
 public:
  using Fn = std::shared_ptr<CloudData> (*)(const CloudData& input,
                                           const void* params);

  static AlgorithmRegistry& Instance();

  void Add(std::string_view algo_id, std::string_view type_id, Fn fn);
  Fn Find(std::string_view algo_id, std::string_view type_id) const;

  /** \brief 某算法已登记的点类型 id，按登记顺序。 */
  std::vector<std::string> TypesFor(std::string_view algo_id) const;

  /** \brief 按输入云的具体点类型查表并调用；未实例化时抛 tg::NodeError。 */
  std::shared_ptr<CloudData> Invoke(std::string_view algo_id,
                                    const CloudData& input,
                                    const void* params) const;

 private:
  AlgorithmRegistry() = default;

  std::unordered_map<std::string, std::unordered_map<std::string, Fn>> _ops;
  std::unordered_map<std::string, std::vector<std::string>> _types_by_algo;
};

}  // namespace pg

/** \brief 为某个算法 × 点类型登记实现。 */
#define PG_INSTANTIATE(algo_id, type_id, fn)                     \
  static_assert((type_id)[0] != '\0', "missing point type id");   \
  ::pg::AlgorithmRegistry::Instance().Add((algo_id), (type_id), (fn))
