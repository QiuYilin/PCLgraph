#pragma once

// PCLgraph 公共入口；Phase B 起在这里聚合点类型注册表与节点声明。
#include "pclgraph/algorithm_registry.hpp"
#include "pclgraph/double_data.hpp"
#include "pclgraph/nodes/nodes.hpp"
#include "pclgraph/nodes/register_nodes.hpp"
#include "pclgraph/point_type.hpp"
#include "pclgraph/transform_data.hpp"

namespace pg {

inline constexpr int kVersionMajor = 0;
inline constexpr int kVersionMinor = 1;
inline constexpr int kVersionPatch = 0;

}  // namespace pg
