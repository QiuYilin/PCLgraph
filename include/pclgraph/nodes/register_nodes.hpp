#pragma once

namespace tg {
class Controller;
}

namespace pg {

/** \brief 把 3D 节点种类登记到工程；category 供界面分组。 */
void RegisterPclNodes(tg::Controller& controller);

}  // namespace pg
