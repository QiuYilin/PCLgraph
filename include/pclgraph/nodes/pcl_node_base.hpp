#pragma once

#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

#include <taskgraph/Node.hpp>

#include "pclgraph/point_type.hpp"

namespace pg {

/** \brief 点云节点公共基类：点云输入只作端口，不改成属性。 */
class PclNodeBase : public tg::Node {
 public:
  bool allowsPropertyPresentation(unsigned int) const override { return false; }

 protected:
  static std::vector<const PointTypeDesc*> AllTypes();
  /** \brief 某算法在 PCL 预编译列表里支持的点类型。 */
  static std::vector<const PointTypeDesc*> SupportedTypesFor(
      std::string_view algo_id);
  static tg::PropertyHints PointTypeHints(
      const std::vector<const PointTypeDesc*>& types, bool with_auto);
  static tg::PropertyHints EnumHints(std::initializer_list<const char*> choices);
  static tg::PropertyHints FileHints();

  /** \brief 按属性下标选具体点类型；with_auto 时 0 表示 Auto（返回空）。 */
  static const PointTypeDesc* PointTypeAt(
      int index, const std::vector<const PointTypeDesc*>& types, bool with_auto);

  static tg::NodeDataType DataTypeOf(const PointTypeDesc* desc);

  static std::shared_ptr<CloudData> Wrap(std::shared_ptr<const void> cloud,
                                         const PointTypeDesc* desc) {
    return std::make_shared<CloudData>(std::move(cloud), desc);
  }
};

}  // namespace pg
