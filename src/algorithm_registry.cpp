#include "pclgraph/algorithm_registry.hpp"

#include <algorithm>
#include <string>

#include <taskgraph/Error.hpp>

namespace pg {

AlgorithmRegistry& AlgorithmRegistry::Instance() {
  static AlgorithmRegistry registry;
  return registry;
}

void AlgorithmRegistry::Add(std::string_view algo_id, std::string_view type_id,
                            Fn fn) {
  const std::string algo(algo_id);
  _ops[algo][std::string(type_id)] = fn;
  auto& types = _types_by_algo[algo];
  const std::string type(type_id);
  if (std::find(types.begin(), types.end(), type) == types.end()) {
    types.push_back(type);
  }
}

std::vector<std::string> AlgorithmRegistry::TypesFor(
    std::string_view algo_id) const {
  const auto it = _types_by_algo.find(std::string(algo_id));
  if (it == _types_by_algo.end()) {
    return {};
  }
  return it->second;
}

AlgorithmRegistry::Fn AlgorithmRegistry::Find(std::string_view algo_id,
                                              std::string_view type_id) const {
  const auto algo = _ops.find(std::string(algo_id));
  if (algo == _ops.end()) {
    return nullptr;
  }
  const auto type = algo->second.find(std::string(type_id));
  if (type == algo->second.end()) {
    return nullptr;
  }
  return type->second;
}

std::shared_ptr<CloudData> AlgorithmRegistry::Invoke(
    std::string_view algo_id, const CloudData& input,
    const void* params) const {
  if (input.desc() == nullptr) {
    throw tg::NodeError("point cloud has no type descriptor");
  }
  const Fn fn = Find(algo_id, input.desc()->id);
  if (fn == nullptr) {
    throw tg::NodeError("algorithm '" + std::string(algo_id) +
                        "' is not instantiated for point type '" +
                        input.desc()->id + "'");
  }
  return fn(input, params);
}

std::shared_ptr<CloudData> CloudData::dispatch(std::string_view algo_id,
                                               const void* params) const {
  return AlgorithmRegistry::Instance().Invoke(algo_id, *this, params);
}

}  // namespace pg
