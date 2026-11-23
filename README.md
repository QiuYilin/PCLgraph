# PCLgraph

一个基于点云库PCL的三维视觉数据流编程框架

仓库名为 PCLgraph；CMake 包名与 vcpkg 包名用 `pclgraph`，C++ 命名空间用 `pg`。

# 依赖

C++17。依赖用 [vcpkg](https://github.com/microsoft/vcpkg) 管理，自定义端口来自个人注册表 [QiuYilin/vcpkg-registry](https://github.com/QiuYilin/vcpkg-registry)。开发期 taskgraph 直接链接本地源码 `C:\Users\qiuyi\program\taskgraph`（0.3.0 基础层，尚未发布），不经过 vcpkg；taskflow 须为 **3.11.0**（最后一个可按 C++17 编译的版本，builtin 默认已是 4.x）。PCL 用 vcpkg 的 `pcl` port（Boost 为其传递依赖），测试用 `gtest`。

## 全局模式

把 [`.github/manifest/vcpkg-configuration.json`](.github/manifest/vcpkg-configuration.json) 里的 `registries` 放进 vcpkg 根目录的 `vcpkg-configuration.json`，并补上 `default-registry`，把它的 `baseline` 写成这份 vcpkg 检出的提交。`taskflow` 从个人注册表解析，其余端口仍来自这份检出；taskgraph 不由 vcpkg 安装。

```powershell
vcpkg install taskflow spdlog nlohmann-json pcl gtest
```

装进 `C:\vcpkg\installed`。

本地 taskgraph 由 CMake 的 `PCLGRAPH_TASKGRAPH_DIR` 指向源码目录链接；本机路径写在已忽略的 `CMakeUserPresets.json` 里，使用预设 `vs-2026-amd64-local`。该预设关闭 `VCPKG_MANIFEST_MODE`，直接使用全局已装依赖。

## 清单模式

使用 [`.github/manifest/vcpkg.json`](.github/manifest/vcpkg.json) 和 [`.github/manifest/vcpkg-configuration.json`](.github/manifest/vcpkg-configuration.json)；这两份也是将来 CI 复用的文件，装 `taskflow`、`spdlog`、`nlohmann-json`、`pcl`、`gtest`，taskgraph 仍由源码链接。在仓库根目录执行：

```powershell
C:\vcpkg\vcpkg.exe install --x-manifest-root=.github/manifest --triplet x64-windows
```

依赖装到 `.github/manifest/vcpkg_installed/`（已忽略）。

# 节点

点云在图中传递 `pg::CloudData`（类型擦除句柄 + 具体点类型描述符）；端口按点类型 id 严格匹配，跨类型必须经过转换节点。节点通过 `pg::RegisterPclNodes(tg::Controller&)` 登记。

| 类型 id | 名称 | 分类 |
| --- | --- | --- |
| `pcl.io.reader` | PointCloud Reader | 3D/IO |
| `pcl.io.writer` | PointCloud Writer | 3D/IO |
| `pcl.io.concatenate_fields` | Concatenate Fields | 3D/IO |
| `pcl.io.merge` | Merge | 3D/IO |
| `pcl.filter.voxel_grid` | VoxelGrid | 3D/Filter-sampling |
| `pcl.filter.passthrough` | PassThrough | 3D/Filter-ROI |
| `pcl.filter.median_filter` | MedianFilter3d | 3D/Filter-Denoising |
| `pcl.filter.radius_outlier_removal` | RadiusOutlierRemoval | 3D/Filter-Denoising |
| `pcl.filter.crop_box` | CropBox | 3D/Filter-ROI |
| `pcl.filter.approximate_voxel_grid` | ApproximateVoxelGrid | 3D/Filter-sampling |
| `pcl.filter.uniform_sampling` | UniformSampling | 3D/Filter-sampling |
| `pcl.filter.remove_nan` | RemoveNan | 3D/Filter-Tool |
| `pcl.filter.bilateral_filter` | BilateralFilter | 3D/Filter-Denoising |
| `pcl.filter.fast_bilateral_filter` | FastBilateralFilter | 3D/Filter-Denoising |
| `pcl.filter.shadow_points` | ShadowPoints | 3D/Filter-Denoising |
| `pcl.filter.covariance_sampling` | CovarianceSampling | 3D/Filter-sampling |
| `pcl.filter.normal_space_sampling` | NormalSpaceSampling | 3D/Filter-sampling |
| `pcl.keypoints.sift` | SIFT | 3D/Keypoints |
| `pcl.keypoints.iss_3d` | ISS3D | 3D/Keypoints |
| `pcl.keypoints.harris_3d` | Harris3D | 3D/Keypoints |
| `pcl.features.normal_estimation` | NormalEstimation | 3D/Features |
| `pcl.features.fpfh` | FPFH | 3D/Features |
| `pcl.registration.icp_point_to_point` | ICP Point-to-Point | 3D/Registration |
| `pcl.registration.ndt` | NDT | 3D/Registration |
| `pcl.registration.sac_ia` | SAC-IA | 3D/Registration |
| `pcl.registration.icp_point_to_plane` | ICP Point-to-Plane | 3D/Registration |
| `pcl.registration.fpcs` | FPCS | 3D/Registration |
| `pcl.tool.convert` | PointCloud Convert | 3D/Tool |
| `pcl.tool.transform_point_cloud` | Transform PointCloud | 3D/Tool |
| `pcl.tool.calc_model_reso` | Calc Model Reso | 3D/Tool |

分类名是 `3D/<组>` 形式的二级路径；taskgraph 当前把 `category` 当单个字符串分组，不展开两级树。

Reader 的 `point_type` 默认 `Auto`，按文件头字段推断；
Writer 需要选一个与上游一致的 `input_type`。类型选择器（`point_type` / `input_type` / `output_type`）直接显示在端口类型名处，不进属性表；属性表只放路径、格式、滤波参数等。`MedianFilter3d` 要求 organized 点云。

点类型不再手写：支持的 XYZ 类型跟随安装的 PCL `PCL_XYZ_POINT_TYPES`（当前 PCL 1.15.1 为 18 种），另加 `pcl::Normal`；filter 的类型下拉按 PCL 预编译的实例化列表生成。`BilateralFilter` 只有 `PointXYZI` / `PointXYZINormal`，`FastBilateralFilter` 只有 `PointXYZ` / `PointXYZRGB` / `PointXYZRGBA`，且要求 organized。`ShadowPoints` / `CovarianceSampling` 需要第二个 `Normal` 输入；`CovarianceSampling` 的主点云只支持 PCL 预编译的 5 种。`FPFH` 同样需要 `Normal`，主点云只支持 PCL 核心 5 种，输出 `FPFHSignature33`。`ICP Point-to-Point` 输出对齐点云和 `Transform`，只支持 PCL 预编译的 `PointXYZ / PointXYZI / PointXYZRGB` 同类型配对。
`NDT` 输出 `Transform`，类型范围同 Point-to-Point；`SAC-IA` 需要源/目标点云及各自的 `FPFH` 特征（4 个输入），类型范围同 FPFH，输出对齐点云和 `Transform`；`Transform PointCloud` 用 `Transform` 变换任意已注册的 XYZ 点云。`Concatenate Fields` 把主点云与 `Normal` 合并，支持 `PointXYZ / PointXYZI / PointXYZRGB / PointXYZL` 四组输出；`Merge` 拼接两个同类型点云。`Calc Model Reso` 输入固定 `PointXYZ`，输出 `Double` 平均最近邻距离（原插件输出的是平方值）。
`SIFT` 固定 `PointXYZI → PointXYZ`；`ISS3D` 输入/输出 `PointXYZ`；`Harris3D` 需要 `PointXYZ + Normal`，输出 `PointXYZI`。`NormalSpaceSampling` 需要 `Normal`，支持 `PointXYZ / PointXYZI`。`FPCS` 支持 `PointXYZ / PointXYZI`，可选两路法线，输出对齐点云和 `Transform`；`ICP Point-to-Plane` 只支持 `PointNormal / PointXYZINormal`。

# 示例

- `pclgraph-core-example`：代码搭 Reader→VoxelGrid→Writer，在临时目录生成点云并打印下采样前后的点数。
- `pclgraph-ui-example`：taskgraph ImGui 工作台，加载 `examples/ui/demo-project`；`--check` 可在不开窗口的情况下验证工程能载入并跑通。

本地预设 `vs-2026-amd64-local` 已设 `PCLGRAPH_TASKGRAPH_UI=imgui`，会同时构建两个示例。

# 预期目标

1.得益于PCL算法的结构化设计，将每个算法作为数据流一个执行单元

2.核心思想是图拓扑排序+数据流传递+线程池,类似于有数据流机制的PCL特化的taskflow/CGraph

3.提供开发过程中的可视化辅助，可能使用imgui，类似于Halcon的HDEVELOP，帮助三维视觉开发者快速生成dll

# 预期用法

```
pg::Graph A;
pg::Executor;
pg::VoxGridNode a;
a.property[0] = 0;
pg::ResistrationNode b;
b.property[1] = 1;
A.add(a);
A.add(b);
if(A.connect<tg::CloudData>(a.outport.cloud,b.inport.cloud)){
    Executor.run(A);
}
```
