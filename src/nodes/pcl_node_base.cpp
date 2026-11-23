#include "pclgraph/nodes/pcl_node_base.hpp"

#include <string_view>
#include <utility>

#include "pclgraph/algorithm_registry.hpp"

namespace pg {

std::vector<const PointTypeDesc*> PclNodeBase::AllTypes() {
  std::vector<const PointTypeDesc*> types;
  for (const auto& desc : PointTypeRegistry::Instance().All()) {
    types.push_back(&desc);
  }
  return types;
}

std::vector<const PointTypeDesc*> PclNodeBase::SupportedTypesFor(
    std::string_view algo_id) {
  std::vector<const PointTypeDesc*> types;
  auto& registry = PointTypeRegistry::Instance();
  for (const auto& id : AlgorithmRegistry::Instance().TypesFor(algo_id)) {
    if (const PointTypeDesc* desc = registry.Find(id)) {
      types.push_back(desc);
    }
  }
  return types;
}

tg::PropertyHints PclNodeBase::PointTypeHints(
    const std::vector<const PointTypeDesc*>& types, bool with_auto) {
  tg::PropertyHints hints;
  hints.editor = tg::PropertyEditor::Enum;
  hints.placement = tg::PropertyPlacement::Port;
  if (with_auto) {
    hints.choices.emplace_back("Auto");
  }
  for (const auto* type : types) {
    hints.choices.push_back(type->display_name);
  }
  return hints;
}

tg::PropertyHints PclNodeBase::EnumHints(
    std::initializer_list<const char*> choices) {
  tg::PropertyHints hints;
  hints.editor = tg::PropertyEditor::Enum;
  for (const char* choice : choices) {
    hints.choices.emplace_back(choice);
  }
  return hints;
}

tg::PropertyHints PclNodeBase::FileHints() {
  tg::PropertyHints hints;
  hints.editor = tg::PropertyEditor::File;
  return hints;
}

const PointTypeDesc* PclNodeBase::PointTypeAt(
    int index, const std::vector<const PointTypeDesc*>& types, bool with_auto) {
  const int offset = with_auto ? 1 : 0;
  const int type_index = index - offset;
  if (type_index < 0 ||
      static_cast<std::size_t>(type_index) >= types.size()) {
    return nullptr;
  }
  return types[static_cast<std::size_t>(type_index)];
}

tg::NodeDataType PclNodeBase::DataTypeOf(const PointTypeDesc* desc) {
  if (desc == nullptr) {
    return {};
  }
  return tg::NodeDataType{desc->id, desc->display_name};
}

}  // namespace pg
