#include <iostream>
#include <memory>

#include <pclgraph/pclgraph.hpp>
#include <taskgraph/Graph.hpp>
#include <taskgraph/Log.hpp>

namespace {

class SmokeNode : public tg::Node {
 public:
  SmokeNode() { setTypeId("SmokeNode"); }

  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int) override {
    return nullptr;
  }

 protected:
  void onSetInPortData(unsigned int, std::shared_ptr<tg::NodeData>) override {}
};

}  // namespace

int main() {
  std::cout << "pclgraph " << pg::kVersionMajor << '.' << pg::kVersionMinor << '.'
            << pg::kVersionPatch << '\n';
  tg::Graph graph("pclgraph-smoke");
  auto node = std::make_shared<SmokeNode>();
  graph.addNode(node);
  tg::logInfo("{}", graph.dump());
  std::cout << graph.dump() << '\n';
  return 0;
}
