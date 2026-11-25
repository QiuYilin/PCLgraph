# Custom nodes

PCLgraph nodes are ordinary taskgraph nodes that consume and produce `pg::CloudData`. The public extension point is `pg::PclNodeBase` in `include/pclgraph/nodes/pcl_node_base.hpp`; the helpers `DataTypeOf()` and `Wrap()` keep the point-type descriptor attached to the data.

PCLgraph does not install an SDK yet, so custom nodes are built in-tree: add the class to `include/pclgraph/nodes/nodes.hpp` and a source file under `src/nodes/`, or define it in your own target that links `pclgraph::Nodes`. `examples/custom-node/main.cpp` is a runnable version of the node below.

## Minimal node

This node takes one `PointXYZ` cloud, adds an `offset` property to every x coordinate, and exposes the result on one output port:

```cpp
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
    return DataTypeOf(pg::PointTypeRegistry::Instance().Find(
        pg::kPointTypeId<pcl::PointXYZ>));
  }

  const char* fixedPortId(tg::PortType port_type,
                          unsigned int index) const override {
    return (port_type == tg::PortType::In || port_type == tg::PortType::Out) &&
                   index == kInIndex
               ? "cloud"
               : "";
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
```

## Hooks

| Hook | Purpose |
| --- | --- |
| `nFixedPorts` | Number of fixed input/output ports. Do not call `inputPortCount()` or `outputPortCount()` from here. |
| `fixedDataType` | The `tg::NodeDataType` of a port. Use `DataTypeOf(desc)` so the id and display name come from the point-type registry. |
| `fixedPortId` / `fixedPortCaption` | Stable port id and the label drawn on the canvas. |
| `onSetInPortData` | Cache the upstream data. Keep a `weak_ptr` and lock it in `compute()`. |
| `getOutPortData` | Return the data computed by the last run. |
| `compute` | Read properties, validate the input point type, write `_out`. |
| `portTypeProperty` | Optional. Bind a type-selector property to a port so the canvas draws a type dropdown. |

Point-type helpers on `pg::PclNodeBase`:

- `AllTypes()` — every registered point type.
- `SupportedTypesFor(algo_id)` — the types registered for an algorithm.
- `PointTypeHints(types, with_auto)` — enum hints placed on the port.
- `PointTypeAt(index, types, with_auto)` — resolve the selected property index.
- `DataTypeOf(desc)` / `Wrap(cloud, desc)` — build port metadata and a `pg::CloudData`.

## Properties

Use `addProperty(name, initial, hints)` and keep the returned `std::shared_ptr<tg::Node::Property>`. Values are `std::variant<int, double, std::string, bool>`; read them with `std::get<double>(property->value)` and friends. If a property changes the port type, call `refreshPortTypes()` after updating it.

Type selectors belong on the port, not in the property table:

```cpp
_types = SupportedTypesFor("my_algo");
_point_type = addProperty("point_type", 0, PointTypeHints(_types, false));
_point_type_index = static_cast<int>(getProperties().size()) - 1;
```

Return `_point_type_index` from `portTypeProperty()` for the port that uses it.

## Registration

Register built-in PCL nodes first, then any custom types:

```cpp
tg::Controller project;
pg::RegisterPclNodes(project);
project.registerNodeType<TranslateCloudNode>("Translate", "3D/Tool");
```

`registerNodeType` takes the caption shown in the palette and the `3D/<group>` category used for grouping. The type id comes from the node's `kTypeId`.

## Multiple point types

For a node that supports several point types, keep one implementation per `(algorithm id, point type id)` in `pg::AlgorithmRegistry` and dispatch at run time:

```cpp
template <typename PointT>
std::shared_ptr<pg::CloudData> MyOp(const pg::CloudData& input,
                                    const void* params) {
  const auto cloud = input.as<PointT>();
  if (cloud == nullptr) {
    throw tg::NodeError("MyOp: input point type mismatch");
  }
  // ... run the PCL algorithm and return CloudData with input.desc() ...
}

#define MY_INSTANTIATE(T) \
  PG_INSTANTIATE("my_algo", pg::kPointTypeId<T>, &MyOp<T>);

struct MyRegistrations {
  MyRegistrations() { PCL_INSTANTIATE(MY_INSTANTIATE, PCL_XYZ_POINT_TYPES) }
};

const MyRegistrations kMyRegistrations;
```

Then call `input->dispatch("my_algo", &params)` in `compute()`. `SupportedTypesFor("my_algo")` returns the registered types, which is what the type-selector hints use.

## Testing

Add a case to `tests/nodes_test.cpp`: construct the node, feed a `pg::CloudData` through `setInPortData()`, call `compute()`, and inspect `getOutPortData()`. Run `ctest --test-dir build -C Release --output-on-failure`; `examples/custom-node` is a runnable end-to-end check.
