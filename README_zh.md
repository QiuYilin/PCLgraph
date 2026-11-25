# PCLgraph

PCLgraph 是基于 [PCL](https://pointclouds.org/) 的三维视觉数据流框架，图运行时和 ImGui 工作台由 [taskgraph](https://github.com/QiuYilin/taskgraph) 提供。每个 PCL 算法是一个带类型的节点，点云以 `pg::CloudData` 在节点之间传递；图是一阶、单赋值、无环项（taskgraph 语义），一次 `run` 算出它的值。

仓库名为 PCLgraph；CMake 工程名为 `pclgraph`（目标 `pclgraph::Core` / `pclgraph::Nodes`），C++ 命名空间用 `pg`。长期目标见 [docs/roadmap.md](docs/roadmap.md)。

## 为什么做 PCLgraph

PCL 是一个成熟、使用广泛且历史很长的点云库，很多算法最初都来自研究者的贡献。近几年它的演进节奏相对保守，PCL 2.0 尚未落地，不少新工作也停留在独立的研究代码里。

PCL 同时大量依赖模板：点类型通常是编译期选择。PCLgraph 把 PCL 的模板实例化收进节点内部，把点类型的选择放到图的端口上，用端口类型选择器决定具体类型。这样试算法、拼流程、做对比时，不必为每种点类型重新写一套模板代码。

PCLgraph 面向两类人：刚接触点云处理、希望可视化地搭流程和看结果的人；以及需要验证算法、把新方法接进图里和现有节点对比的研究者。库构建好之后，调整图的连接、参数或已支持的点类型不需要重新编译，新的组合可以直接在图上验证。让 PCL 算法更容易使用、组合和复用。

## 模块

| 路径 | 目标 | 内容 |
| --- | --- | --- |
| `include/pclgraph/` | — | 公共头：点类型、算法注册表、`CloudData` / `TransformData` / `DoubleData`、节点声明。 |
| `src/` | `pclgraph::Core` | 点类型注册表与算法分派；依赖 `taskgraph::Core` 和 `pcl_common`。 |
| `src/nodes/` | `pclgraph::Nodes` | IO / Filter / Keypoints / Features / Registration / Tool 节点与 `RegisterPclNodes`。 |
| `src/smoke.cpp` | `pclgraph_smoke` | 链接 `pclgraph::Nodes` 的最小消费者。 |
| `examples/core/` | — | `pclgraph-core-example`：代码搭 Reader → VoxelGrid → Writer。 |
| `examples/custom-node/` | — | `pclgraph-custom-node-example`：定义、注册并运行一个自定义 PCL 节点。 |
| `examples/ui/` | — | `pclgraph-ui-example`：taskgraph ImGui 工作台 + Point Cloud 监视器。 |
| `tests/` | `pclgraph_tests` | core 与 nodes 的 gtest。 |

依赖分层：`pclgraph::Nodes` → `pclgraph::Core` → `taskgraph::Core`；UI 示例另链 `taskgraph::Ui` 与 `imgui-vtk`。

# 依赖

C++17。推荐用 [vcpkg](https://github.com/microsoft/vcpkg) 安装依赖；自定义端口来自个人注册表 [QiuYilin/vcpkg-registry](https://github.com/QiuYilin/vcpkg-registry)。taskgraph 取注册表里的 `taskgraph[ui-imgui] >= 0.3.0`（ImGui 前端）；taskflow 须为 **3.11.0**（最后一个可按 C++17 编译的版本，builtin 默认已是 4.x）。PCL 用 vcpkg 的 `pcl` port（Boost 为其传递依赖），测试用 `gtest`。UI 的三维点云视图用注册表里的 `imgui-vtk`，由它传递引入 VTK 9、gl3w 和 glfw3。

依赖可以按全局模式或清单模式安装。

## 全局模式

把 [`.github/manifest/vcpkg-configuration.json`](.github/manifest/vcpkg-configuration.json) 里的 `registries` 放进 vcpkg 根目录的 `vcpkg-configuration.json`，补上你这份 vcpkg 检出的 `default-registry`，个人注册表的 `baseline` 以该文件为准。`taskflow`、`imgui-vtk`、`taskgraph` 从个人注册表解析，其余端口仍来自这份检出。

基础（core 与 core 测试）：

```powershell
C:\vcpkg\vcpkg.exe install taskflow spdlog nlohmann-json pcl gtest --triplet x64-windows
```

`PCLGRAPH_TASKGRAPH_UI=imgui` 另需：

```powershell
C:\vcpkg\vcpkg.exe install "imgui[docking-experimental,glfw-binding,opengl3-binding]" opengl imgui-node-editor imgui-vtk portable-file-dialogs "taskgraph[ui-imgui]" --triplet x64-windows
```

全局模式把依赖装进 vcpkg 的 installed 目录，CMake 通过 vcpkg 工具链查找。切换模式时如果复用同一个构建目录，先删掉再重建：vcpkg 不允许在同一个构建目录里改这个开关。

## 清单模式

使用 [`.github/manifest/vcpkg.json`](.github/manifest/vcpkg.json) 和 [`.github/manifest/vcpkg-configuration.json`](.github/manifest/vcpkg-configuration.json)，把同样的依赖（含 `taskgraph[ui-imgui]`）装到清单目录：

```powershell
C:\vcpkg\vcpkg.exe install --x-manifest-root=.github/manifest --triplet x64-windows
```

依赖装到 `.github/manifest/vcpkg_installed/`（已忽略）；配置 CMake 时加 `-DVCPKG_MANIFEST_MODE=ON -DVCPKG_MANIFEST_DIR=.github/manifest`。

# 构建

用 vcpkg 工具链配置。下面以全局模式为例：

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows -DPCLGRAPH_TASKGRAPH_UI=imgui
cmake --build build --config Release
```

清单模式在配置命令里加 `-DVCPKG_MANIFEST_MODE=ON -DVCPKG_MANIFEST_DIR=.github/manifest`。可执行文件在 `build/bin/<config>/`。

| 选项 | 取值 | 默认 | 含义 |
| --- | --- | --- | --- |
| `PCLGRAPH_BUILD_EXAMPLES` | `ON` / `OFF` | `ON` | 始终构建 `pclgraph-core-example`；UI 为 `imgui` 时再构建 `pclgraph-ui-example`。 |
| `PCLGRAPH_TASKGRAPH_UI` | `imgui` / `qt` / `none` | `none` | 只有 `imgui` 会构建 PCLgraph 的 UI 示例。 |
| `PCLGRAPH_DEMO_PCD` | 文件路径 | 空 | 本地 `chef.pcd`；为空时在构建时下载 PCL 样例。 |
| `PCLGRAPH_BUILD_DOCS` | `ON` / `OFF` | `OFF` | 用 Doxygen 生成 API 文档（`doc`）。 |

PCLgraph 目前没有 install/export 规则，所以还没有 `find_package(pclgraph)`；示例直接链接树内目标。

## API 文档

API 文档默认关闭。配置时加 `-DPCLGRAPH_BUILD_DOCS=ON`（需要 Doxygen），再生成：

```powershell
cmake --build build --target doc
```

输出在 `build/docs/html/index.html`。如果 CMake 找不到 Doxygen，配置时加 `-DDOXYGEN_EXECUTABLE="C:/Program Files/doxygen/bin/doxygen.exe"`，或把 `C:\Program Files\doxygen\bin` 加进 `PATH`。

# 测试

```powershell
ctest --test-dir build -C Release --output-on-failure
```

`pclgraph_tests` 覆盖点类型注册表、`CloudData`、算法分派，以及 Reader 推断、滤波、`NormalEstimation` / `FPFH`、`ICP` / `NDT` / `SAC-IA` / `FPCS`、类型严格匹配、Reader → VoxelGrid → Writer 整图。UI 示例的 `--check` 不开窗口验证 demo 工程、关键点数量和最终最近邻对齐误差。

测试的 `PATH` 固定指向 `C:/vcpkg/installed/...`，所以即使用清单模式构建，测试运行时仍用全局 DLL。

# 用法

## core

`examples/core/main.cpp` 代码搭 Reader → VoxelGrid → Writer，并打印点数：

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

模板 `createNode<T>()` 直接实例化节点，这里不需要 `pg::RegisterPclNodes`；从磁盘载入工程、或工作台生成节点 palette 时才需要注册。

## core + ui

`examples/ui/main.cpp` 注册 PCL 节点、打开工程，并把点云监视器挂到 taskgraph 工作台：

```cpp
tg::Controller project;
pg::RegisterPclNodes(project);
project.openProject("demo-project");

Ui ui(project);
auto point_cloud = std::make_shared<PointCloudPanel>();
ui.AddMonitorView(point_cloud->MakeMonitorView());
ui.Run();
```

## 自定义节点

继承 `pg::PclNodeBase`，声明 `tg::In<pg::CloudData>` / `tg::Out<pg::CloudData>` 端口，实现端口 hook 和 `compute()`，再注册：

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

完整节点、多类型分派和测试见 [docs/custom-nodes.md](docs/custom-nodes.md)；可运行版本在 `examples/custom-node`。

# 节点

点云在图中以 `pg::CloudData` 传递（类型擦除句柄 + 具体点类型描述符）。端口按点类型 id 严格匹配，跨类型用 Convert 节点；节点通过 `pg::RegisterPclNodes(tg::Controller&)` 登记。

支持的 XYZ 类型跟随 PCL 的 `PCL_XYZ_POINT_TYPES`，另加 `pcl::Normal`；下拉项按 PCL 预编译的实例化生成。Reader 的 `point_type` 默认 `Auto`，Writer 需要与上游一致的 `input_type`；类型选择器在端口上，不进属性表。

完整节点列表和每个节点的类型限制见 [docs/nodes.md](docs/nodes.md)。

# 示例

- `pclgraph-core-example`：代码搭 Reader → VoxelGrid → Writer，打印下采样前后的点数。
- `pclgraph-ui-example`：在 taskgraph ImGui 工作台里打开 `examples/ui/demo-project`，跑一条完整配准流程（VoxelGrid、ISS3D、NormalEstimation、FPFH、SAC-IA、ICP）并写到 Writer。`--check` 不开窗口跑通并检查关键点数量和配准误差。

UI 说明见 [docs/ui.md](docs/ui.md)。
