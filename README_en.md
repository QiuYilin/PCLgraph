# PCLgraph

A 3D vision dataflow programming framework built on the Point Cloud Library (PCL).

The repository is named PCLgraph; the CMake package and vcpkg port are `pclgraph`, and the C++ namespace is `pg`.

# Dependencies

C++17. Dependencies are managed with [vcpkg](https://github.com/microsoft/vcpkg), and custom ports come from the personal registry [QiuYilin/vcpkg-registry](https://github.com/QiuYilin/vcpkg-registry). During development taskgraph links against the local source tree `C:\Users\qiuyi\program\taskgraph` (0.3.0 foundation, not released yet) instead of vcpkg; taskflow must be **3.11.0** (the last release that builds as C++17; the builtin registry already defaults to 4.x). PCL comes from the vcpkg `pcl` port (Boost is transitive), and tests use `gtest`. The UI's 3D point-cloud view uses `imgui-vtk` from the registry, which brings VTK 9, gl3w, and glfw3 transitively.

## Classic mode

Copy the `registries` entry from [`.github/manifest/vcpkg-configuration.json`](.github/manifest/vcpkg-configuration.json) into `vcpkg-configuration.json` at the vcpkg root, add `default-registry`, and set its `baseline` to the commit of that vcpkg checkout. `taskflow` resolves from the personal registry; other ports stay on that checkout, and taskgraph is not installed by vcpkg.

```powershell
vcpkg install taskflow spdlog nlohmann-json pcl gtest imgui-vtk
```

Packages install into `C:\vcpkg\installed`.

CMake links the local taskgraph through `PCLGRAPH_TASKGRAPH_DIR`; the machine-specific path lives in the ignored `CMakeUserPresets.json` and is used by the `vs-2026-amd64-local` preset. That preset disables `VCPKG_MANIFEST_MODE` and uses the globally installed packages.

## Manifest mode

Use [`.github/manifest/vcpkg.json`](.github/manifest/vcpkg.json) and [`.github/manifest/vcpkg-configuration.json`](.github/manifest/vcpkg-configuration.json); these are also the files CI will reuse, installing `taskflow`, `spdlog`, `nlohmann-json`, `pcl`, `gtest`, and `imgui-vtk` while taskgraph still comes from source. From the repository root:

```powershell
C:\vcpkg\vcpkg.exe install --x-manifest-root=.github/manifest --triplet x64-windows
```

Packages install into `.github/manifest/vcpkg_installed/` (ignored by Git).

# Nodes

Point clouds travel through the graph as `pg::CloudData` (a type-erased handle plus a concrete point-type descriptor); ports match point-type ids strictly, so changing type requires the convert node. Register everything with `pg::RegisterPclNodes(tg::Controller&)`.

| Type id | Caption | Category |
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

Category names are two-level `3D/<group>` paths; taskgraph groups nodes by the single `category` string and does not render a two-level tree.

Reader defaults to `point_type = Auto` and infers from the file header;
 Writer needs an `input_type` matching its upstream. Type selectors (`point_type` / `input_type` / `output_type`) appear in place of the port type name, not in the property table; the table keeps paths, formats, filter parameters, and so on. `MedianFilter3d` requires an organized point cloud.

Point types are no longer hand-written: the supported XYZ types follow the installed PCL `PCL_XYZ_POINT_TYPES` (18 types in PCL 1.15.1), plus `pcl::Normal`; filter type dropdowns are generated from PCL's precompiled instantiations. `BilateralFilter` supports only `PointXYZI` / `PointXYZINormal`, and `FastBilateralFilter` only `PointXYZ` / `PointXYZRGB` / `PointXYZRGBA`; the fast variant requires an organized cloud. `ShadowPoints` and `CovarianceSampling` take a second `Normal` input; `CovarianceSampling` supports only PCL's 5 precompiled main point types. `FPFH` also takes a `Normal` input, supports only PCL's 5 core main types, and outputs `FPFHSignature33`. `ICP Point-to-Point` outputs an aligned cloud and a `Transform`, and supports only PCL's precompiled `PointXYZ / PointXYZI / PointXYZRGB` same-type pairs.
 `NDT` outputs a `Transform` with the same type range as Point-to-Point; `SAC-IA` takes source/target clouds plus their `FPFH` features (4 inputs), supports the FPFH type range, and outputs an aligned cloud and a `Transform`; `Transform PointCloud` applies a `Transform` to any registered XYZ cloud. `Concatenate Fields` combines a main cloud with a `Normal` cloud for `PointXYZ / PointXYZI / PointXYZRGB / PointXYZL`; `Merge` concatenates two same-type clouds. `Calc Model Reso` takes a fixed `PointXYZ` cloud and outputs a `Double` mean nearest-neighbor distance (the original plugin outputs the squared value).
`SIFT` is fixed `PointXYZI → PointXYZ`; `ISS3D` takes and outputs `PointXYZ`; `Harris3D` takes `PointXYZ + Normal` and outputs `PointXYZI`. `NormalSpaceSampling` requires `Normal` and supports `PointXYZ / PointXYZI`. `FPCS` supports `PointXYZ / PointXYZI`, accepts optional normals, and outputs an aligned cloud plus a `Transform`; `ICP Point-to-Plane` supports only `PointNormal / PointXYZINormal`.

# Examples

- `pclgraph-core-example`: builds Reader → VoxelGrid → Writer in code, writes a temporary cloud, and prints the point counts before and after downsampling.
- `pclgraph-ui-example`: taskgraph ImGui workbench that opens `examples/ui/demo-project`; the demo graph is a full registration pipeline `Reader → VoxelGrid → ISS3D → NormalEstimation → Convert → FPFH → SAC-IA → Transform → ICP → Writer`; `--check` validates that the project loads and runs without opening a window, and also checks the keypoint count and final nearest-neighbor alignment error.

The sample cloud is not committed: CMake downloads PCL 1.15.1's `chef.pcd` (pinned SHA256) into the build tree by default, or use `-DPCLGRAPH_DEMO_PCD=<local chef.pcd>`. At startup the example normalizes the sample and writes `source.pcd` / `target.pcd` next to the executable; the target is a known rigid transform of the source, used to verify the registration error. This is a keypoint variant of PCL's official `template_alignment`: the official sample does not use ISS keypoints and computes FPFH on the downsampled cloud directly.

**Point Cloud** is an independent floating window, not part of the Output panel: it opens automatically on first run, can be moved or docked, and reopens from the title-bar **View → Point Cloud** menu after closing. Its open/closed state is stored in the taskgraph session. Select a node with a cloud output and press **Run Graph**: the window follows the canvas selection by default; clear **Follow Selection** or pick an entry from the dropdown to pin one node output for use as a resident view. VTK requires OpenGL 3.2, so the taskgraph ImGui window context has been raised to 3.2 core. The view depends on the new `Ui::SelectedNode()` in the local taskgraph source; the registry taskgraph can only ship it after 0.3.0.

The local `vs-2026-amd64-local` preset sets `PCLGRAPH_TASKGRAPH_UI=imgui`, so both examples are built.

# Goals

1. Thanks to the structured design of PCL algorithms, treat each algorithm as one execution unit in the dataflow.

2. Core idea: graph topological sorting + dataflow transfer + thread pool, similar to a PCL-specialized taskflow/CGraph with dataflow semantics.

3. Provide visual assistance during development, possibly with ImGui, similar to Halcon's HDEVELOP, to help 3D vision developers generate DLLs quickly.

# Intended usage

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
