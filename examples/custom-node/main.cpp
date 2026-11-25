#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <system_error>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pclgraph/pclgraph.hpp>
#include <taskgraph/Controller.hpp>
#include <taskgraph/Error.hpp>

namespace custom_node_example {

class TranslateCloudNode final : public pg::PclNodeBase {
 public:
  static constexpr const char* kTypeId = "custom.translate";
  static constexpr unsigned int kInIndex = 0;
  static constexpr unsigned int kOutIndex = 0;

  struct InPort {
    tg::In<pg::CloudData> cloud;
  };
  struct OutPort {
    tg::Out<pg::CloudData> cloud;
  };

  TranslateCloudNode()
      : inport{tg::In<pg::CloudData>{this, kInIndex}},
        outport{tg::Out<pg::CloudData>{this, kOutIndex}} {
    setTypeId(kTypeId);
    tg::PropertyHints hints;
    hints.editor = tg::PropertyEditor::Double;
    hints.min = -1.0;
    hints.max = 1.0;
    hints.step = 0.01;
    hints.decimals = 3;
    _offset = addProperty("offset", 0.1, hints);
  }

  unsigned int nFixedPorts(tg::PortType port_type) const override {
    return port_type == tg::PortType::In || port_type == tg::PortType::Out
               ? 1u
               : 0u;
  }

  tg::NodeDataType fixedDataType(tg::PortType port_type,
                                 unsigned int index) const override {
    if ((port_type != tg::PortType::In && port_type != tg::PortType::Out) ||
        index != kInIndex) {
      return {};
    }
    return DataTypeOf(
        pg::PointTypeRegistry::Instance().Find(pg::kPointTypeId<pcl::PointXYZ>));
  }

  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override {
    if ((port_type == tg::PortType::In || port_type == tg::PortType::Out) &&
        index == kInIndex) {
      return "cloud";
    }
    return "";
  }

  const char* fixedPortCaption(tg::PortType port_type,
                               unsigned int index) const override {
    return fixedPortId(port_type, index);
  }

  void compute() override {
    const auto input = std::dynamic_pointer_cast<pg::CloudData>(_in.lock());
    if (input == nullptr) {
      throw tg::NodeError("Translate: missing input");
    }
    const auto cloud = input->as<pcl::PointXYZ>();
    if (cloud == nullptr) {
      throw tg::NodeError("Translate: input is not PointXYZ");
    }
    auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);
    const double offset = std::get<double>(_offset->value);
    for (auto& point : *output) {
      point.x += static_cast<float>(offset);
    }
    _out = Wrap(std::shared_ptr<const void>(std::move(output)), input->desc());
  }

  void onSetInPortData(unsigned int index,
                       std::shared_ptr<tg::NodeData> data) override {
    if (index == kInIndex) {
      _in = std::move(data);
    }
  }

  std::shared_ptr<tg::NodeData> getOutPortData(unsigned int index) override {
    return index == kOutIndex ? _out : nullptr;
  }

  void setOffset(double offset) { _offset->value = offset; }

  InPort inport;
  OutPort outport;

 private:
  std::shared_ptr<tg::Node::Property> _offset;
  std::weak_ptr<tg::NodeData> _in;
  std::shared_ptr<tg::NodeData> _out;
};

}  // namespace custom_node_example

int main() {
  using std::filesystem::path;

  const path dir =
      std::filesystem::temp_directory_path() / "pclgraph-custom-node-example";
  std::error_code error;
  std::filesystem::create_directories(dir, error);
  const path input_path = dir / "points.pcd";
  const path output_path = dir / "translated.pcd";

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int i = 0; i < 12; ++i) {
    cloud->push_back(pcl::PointXYZ{static_cast<float>(i % 2),
                                   static_cast<float>(i % 3),
                                   static_cast<float>(i % 4)});
  }
  pcl::PCDWriter writer;
  if (writer.write(input_path.string(), *cloud, false) != 0) {
    std::cerr << "failed to write " << input_path << '\n';
    return EXIT_FAILURE;
  }

  tg::Controller project;
  pg::RegisterPclNodes(project);
  project.registerNodeType<custom_node_example::TranslateCloudNode>(
      "Translate", "3D/Tool");

  auto graph = project.createGraph();
  auto reader = graph->createNode<pg::PointCloudReaderNode>();
  auto translate =
      graph->createNode<custom_node_example::TranslateCloudNode>();
  auto sink = graph->createNode<pg::PointCloudWriterNode>();

  reader->setPath(input_path.string());
  translate->setOffset(0.25);
  sink->setPath(output_path.string());

  if (!graph->connect<pg::CloudData>(reader->outport.cloud,
                                     translate->inport.cloud)) {
    std::cerr << "failed to connect reader -> translate\n";
    return EXIT_FAILURE;
  }
  if (!graph->connect<pg::CloudData>(translate->outport.cloud,
                                     sink->inport.cloud)) {
    std::cerr << "failed to connect translate -> writer\n";
    return EXIT_FAILURE;
  }

  if (const auto run_error = project.tryRun(*graph)) {
    std::cerr << "run failed: " << *run_error << '\n';
    return EXIT_FAILURE;
  }

  const auto translated = translate->outport.cloud.nodeData();
  const auto translated_cloud =
      translated != nullptr ? translated->as<pcl::PointXYZ>() : nullptr;
  const std::size_t output_size =
      translated_cloud != nullptr ? translated_cloud->size() : 0;

  std::cout << "input:  " << input_path.string() << " (" << cloud->size()
            << " points)\n";
  std::cout << "custom: " << output_size << " points\n";
  std::cout << "output: " << output_path.string()
            << " (exists=" << std::filesystem::exists(output_path) << ")\n";
  return EXIT_SUCCESS;
}
