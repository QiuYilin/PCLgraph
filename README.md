# PCLgraph

PCLgraph is a 3D vision dataflow framework built on the [Point Cloud Library (PCL)](https://pointclouds.org/), with [taskgraph](https://github.com/QiuYilin/taskgraph) as the graph runtime and ImGui workbench. Every PCL algorithm is a typed node; point clouds travel between nodes as `pg::CloudData`; a graph is a first-order, single-assignment, acyclic term (taskgraph semantics), and one `run` computes its value.

The repository is named PCLgraph; the CMake project is `pclgraph` (targets `pclgraph::Core` / `pclgraph::Nodes`), and the C++ namespace is `pg`. Longer-term goals are in [docs/roadmap.md](docs/roadmap.md).

## Why PCLgraph

PCL is a mature, widely used point-cloud library with a long history of contributed algorithms. In recent years its release cadence has been conservative, PCL 2.0 has not shipped, and much new work lives in separate research code rather than in the library.

PCL also relies heavily on templates: the point type is usually a compile-time choice. PCLgraph keeps the PCL instantiations inside the nodes and moves that choice to the graph edge, where a port type selector picks the concrete type. Trying an algorithm, wiring a pipeline, and comparing results no longer require writing a new template stack for every point type.

PCLgraph targets two groups: people learning point-cloud processing, who can build and inspect a pipeline visually, and researchers validating algorithms, who can drop a new method into a graph and compare it with existing nodes. Once built, changing the graph topology, parameters, or a supported point type does not require recompiling, so new combinations can be validated by editing the graph. It makes PCL algorithms easier to use, combine, and reuse.

## Modules

| Path | Target | Contents |
| --- | --- | --- |
| `include/pclgraph/` | — | Public headers: point types, algorithm registry, `CloudData` / `TransformData` / `DoubleData`, node declarations. |
| `src/` | `pclgraph::Core` | Point-type registry and algorithm dispatch; depends on `taskgraph::Core` and `pcl_common`. |
| `src/nodes/` | `pclgraph::Nodes` | IO / Filter / Keypoints / Features / Registration / Tool nodes and `RegisterPclNodes`. |
| `src/smoke.cpp` | `pclgraph_smoke` | Minimal consumer linking `pclgraph::Nodes`. |
| `examples/core/` | — | `pclgraph-core-example`: builds Reader → VoxelGrid → Writer in code. |
| `examples/custom-node/` | — | `pclgraph-custom-node-example`: defines, registers, and runs a custom PCL node. |
| `examples/ui/` | — | `pclgraph-ui-example`: taskgraph ImGui workbench plus the Point Cloud monitor. |
| `tests/` | `pclgraph_tests` | gtest suites for core and nodes. |

Layering: `pclgraph::Nodes` → `pclgraph::Core` → `taskgraph::Core`. The UI example additionally links `taskgraph::Ui` and `imgui-vtk`.

# Dependencies

C++17. Prefer [vcpkg](https://github.com/microsoft/vcpkg) for dependencies; custom ports come from the personal registry [QiuYilin/vcpkg-registry](https://github.com/QiuYilin/vcpkg-registry). taskgraph comes from the registry as `taskgraph[ui-imgui] >= 0.3.0` (ImGui frontend). taskflow must be **3.11.0** (the last release that builds as C++17; the builtin registry already defaults to 4.x). PCL comes from the vcpkg `pcl` port (Boost is transitive), and tests use `gtest`. The UI's 3D point-cloud view uses `imgui-vtk` from the registry, which brings VTK 9, gl3w, and glfw3 transitively.

Dependencies can be installed in classic mode or manifest mode.

## Classic mode

Copy the `registries` entry from [`.github/manifest/vcpkg-configuration.json`](.github/manifest/vcpkg-configuration.json) into `vcpkg-configuration.json` at the vcpkg root, add `default-registry` for your vcpkg checkout, and keep the personal registry's `baseline` from that file. `taskflow`, `imgui-vtk`, and `taskgraph` resolve from the personal registry; other ports stay on that checkout.

Base (core and its tests):

```powershell
C:\vcpkg\vcpkg.exe install taskflow spdlog nlohmann-json pcl gtest --triplet x64-windows
```

`PCLGRAPH_TASKGRAPH_UI=imgui` additionally needs:

```powershell
C:\vcpkg\vcpkg.exe install "imgui[docking-experimental,glfw-binding,opengl3-binding]" opengl imgui-node-editor imgui-vtk portable-file-dialogs "taskgraph[ui-imgui]" --triplet x64-windows
```

Classic mode installs packages into the vcpkg installed tree, and CMake finds them through the vcpkg toolchain. When switching between modes in the same build directory, delete it first: vcpkg does not allow changing `VCPKG_MANIFEST_MODE` inside one build tree.

## Manifest mode

Use [`.github/manifest/vcpkg.json`](.github/manifest/vcpkg.json) and [`.github/manifest/vcpkg-configuration.json`](.github/manifest/vcpkg-configuration.json) to install the same dependencies (including `taskgraph[ui-imgui]`) into the manifest tree:

```powershell
C:\vcpkg\vcpkg.exe install --x-manifest-root=.github/manifest --triplet x64-windows
```

Packages install into `.github/manifest/vcpkg_installed/` (ignored by Git); configure CMake with `-DVCPKG_MANIFEST_MODE=ON -DVCPKG_MANIFEST_DIR=.github/manifest`.

# Build

Configure with the vcpkg toolchain. The example below uses classic mode:

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows -DPCLGRAPH_TASKGRAPH_UI=imgui
cmake --build build --config Release
```

For manifest mode, add `-DVCPKG_MANIFEST_MODE=ON -DVCPKG_MANIFEST_DIR=.github/manifest` to the configure command. Executables land in `build/bin/<config>/`.

| Option | Values | Default | Meaning |
| --- | --- | --- | --- |
| `PCLGRAPH_BUILD_EXAMPLES` | `ON` / `OFF` | `ON` | Build `pclgraph-core-example` always, and `pclgraph-ui-example` when the UI is `imgui`. |
| `PCLGRAPH_TASKGRAPH_UI` | `imgui` / `qt` / `none` | `none` | Only `imgui` builds PCLgraph's UI example. |
| `PCLGRAPH_DEMO_PCD` | file path | empty | Local `chef.pcd` for the UI demo; empty downloads the PCL sample at build time. |
| `PCLGRAPH_BUILD_DOCS` | `ON` / `OFF` | `OFF` | Generate API docs with Doxygen (`doc`). |

PCLgraph currently has no install or export rules, so there is no `find_package(pclgraph)` yet; the examples link the in-tree targets.

## API docs

API docs are off by default. Configure with `-DPCLGRAPH_BUILD_DOCS=ON` (Doxygen required), then generate them with:

```powershell
cmake --build build --target doc
```

Output: `build/docs/html/index.html`. If CMake cannot find Doxygen, configure with `-DDOXYGEN_EXECUTABLE="C:/Program Files/doxygen/bin/doxygen.exe"` or add `C:\Program Files\doxygen\bin` to `PATH`.

# Tests

```powershell
ctest --test-dir build -C Release --output-on-failure
```

`pclgraph_tests` covers the point-type registry, `CloudData`, and algorithm dispatch, plus Reader inference, filters, `NormalEstimation` / `FPFH`, `ICP` / `NDT` / `SAC-IA` / `FPCS`, strict type matching, and the Reader → VoxelGrid → Writer graph. The UI example's `--check` loads and runs the demo project without opening a window, then verifies the keypoint count and the final nearest-neighbor alignment error.

The test `PATH` is pinned to `C:/vcpkg/installed/...`, so tests run with the classic-mode DLLs even when the build used manifest mode.

# Usage

## core

`examples/core/main.cpp` builds Reader → VoxelGrid → Writer and prints the point counts:

```cpp
tg::Controller project;
auto graph = project.createGraph();
auto reader = graph->createNode<pg::PointCloudReaderNode>();
auto grid = graph->createNode<pg::VoxelGridNode>();
auto sink = graph->createNode<pg::PointCloudWriterNode>();

reader->setPath("points.pcd");
grid->setLeafSize(2.0, 2.0, 2.0);
sink->setPath("downsampled.pcd");

graph->connect<pg::CloudData>(reader->outport.cloud, grid->inport.cloud);
graph->connect<pg::CloudData>(grid->outport.cloud, sink->inport.cloud);

if (const auto error = project.tryRun(*graph)) {
    std::cerr << "run failed: " << *error << '\n';
    return 1;
}
```

Template `createNode<T>()` instantiates the node directly, so `pg::RegisterPclNodes` is not required here. Registration is needed when a project is loaded from disk or when the workbench builds the node palette.

## core + ui

`examples/ui/main.cpp` registers PCL nodes, opens a project, and adds the point-cloud monitor to the taskgraph workbench:

```cpp
tg::Controller project;
pg::RegisterPclNodes(project);
project.openProject("demo-project");

Ui ui(project);
auto point_cloud = std::make_shared<PointCloudPanel>();
ui.AddMonitorView(point_cloud->MakeMonitorView());
ui.Run();
```

## Custom nodes

Derive from `pg::PclNodeBase`, declare `tg::In<pg::CloudData>` / `tg::Out<pg::CloudData>` ports, implement the port hooks and `compute()`, then register the type:

```cpp
class TranslateNode final : public pg::PclNodeBase {
 public:
  static constexpr const char* kTypeId = "custom.translate";
  struct InPort { tg::In<pg::CloudData> cloud; };
  struct OutPort { tg::Out<pg::CloudData> cloud; };

  TranslateNode()
      : inport{tg::In<pg::CloudData>{this, 0}},
        outport{tg::Out<pg::CloudData>{this, 0}} {
    setTypeId(kTypeId);
  }

  unsigned int nFixedPorts(tg::PortType) const override;
  tg::NodeDataType fixedDataType(tg::PortType, unsigned int) const override;
  const char* fixedPortId(tg::PortType, unsigned int) const override;
  const char* fixedPortCaption(tg::PortType, unsigned int) const override;
  void compute() override;
  // onSetInPortData / getOutPortData ...
  InPort inport;
  OutPort outport;
};

project.registerNodeType<TranslateNode>("Translate", "3D/Tool");
```

See [docs/custom-nodes.md](docs/custom-nodes.md) for the full node, multi-point-type dispatch, and tests; `examples/custom-node` is the runnable version.

# Nodes

Point clouds travel through the graph as `pg::CloudData` (a type-erased handle plus a concrete point-type descriptor). Ports match point-type ids strictly; use the Convert node to change type, and register everything with `pg::RegisterPclNodes(tg::Controller&)`.

Supported XYZ types follow PCL's `PCL_XYZ_POINT_TYPES`, plus `pcl::Normal`; dropdowns are generated from PCL's precompiled instantiations. `point_type` on Reader defaults to `Auto`; Writer needs an `input_type` matching its upstream. Type selectors sit on the port, not in the property table.

See [docs/nodes.md](docs/nodes.md) for the full node list and per-node type constraints.

# Examples

- `pclgraph-core-example`: builds Reader → VoxelGrid → Writer in code and prints the point counts before and after downsampling.
- `pclgraph-ui-example`: opens `examples/ui/demo-project` in the taskgraph ImGui workbench and runs a full registration pipeline (VoxelGrid, ISS3D, NormalEstimation, FPFH, SAC-IA, ICP) ending in a Writer. `--check` runs it without a window and validates the keypoint count and alignment error.

See [docs/ui.md](docs/ui.md) for the UI.
