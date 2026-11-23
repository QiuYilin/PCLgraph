# PCLgraph UI demo

这个工程演示完整的点云配准链路，供 `pclgraph-ui-example` 的 Point Cloud 视图逐级查看：

```text
Source Reader -> VoxelGrid -> ISS3D -> NormalEstimation -> Convert(PointNormal -> Normal) -> FPFH ┐
                                                                                                  ├-> SAC-IA -> Transform -> ICP -> Writer
Target Reader -> VoxelGrid -> ISS3D -> NormalEstimation -> Convert(PointNormal -> Normal) -> FPFH ┘
```

## 点云来源

- 原始样例：PCL 1.15.1 的 `doc/tutorials/content/sources/alignment_prerejective/chef.pcd`，5092 点，含法线字段；加载时只取 XYZ。
- 仓库不提交 PCD。CMake 默认下载到构建目录：
  - URL：`https://raw.githubusercontent.com/PointCloudLibrary/pcl/pcl-1.15.1/doc/tutorials/content/sources/alignment_prerejective/chef.pcd`
  - SHA256：`a4924fc43c78c07d3fba6a0b7226ccf5c2134ea0a2e3e50d5e65b40c2f09fdfc`
  - 离线覆盖：`-DPCLGRAPH_DEMO_PCD=<本地 chef.pcd>`
  - 下载失败时先设置 `HTTP_PROXY` / `HTTPS_PROXY` 重新 configure，仍失败再用上面的本地覆盖。
- 程序启动时（GUI 和 `--check` 都一样）把样例归一化：
  - 去中心；
  - 最大边长缩放到 1.0；
  - 写 exe 旁的 `source.pcd`。
- `target.pcd` 由 source 按已知刚体变换生成：

```text
target = T(0.10, -0.06, 0.08) * Rz(11°) * Ry(4°) * source
```

`source.pcd`、`target.pcd`、`aligned.pcd` 只出现在构建输出/exe 旁，源码树会被 `.gitignore` 忽略。

## 关键参数

| 阶段 | 参数 |
| --- | --- |
| VoxelGrid | leaf `0.02 / 0.02 / 0.02`，min_points `1` |
| ISS3D | salient `0.10`，non-max `0.05`，normal `0.10`，border `0.05`，gamma `0.975`，min_neighbors `5` |
| NormalEstimation | `k_search=20`，`radius_search=0` |
| Convert | `input_type=9`（xyznormal），`output_type=18`（normal） |
| FPFH | `k=10`，bins `11/11/11` |
| SAC-IA | max_iterations `1000`，min_sample_distance `0.05`，nr_samples `3`，k_correspondences `5`，TruncatedError `100` |
| ICP | max_iterations `100`，max_correspondence_distance `0.05`，transformation_epsilon `1e-8`，euclidean_fitness_epsilon `1e-6` |

`NormalEstimationNode` 输出 `PointNormal`，而 FPFH 的法线输入只接受 `Normal`，所以每路需要一个 `Convert`。这是 PCLgraph 节点类型模型的差异；PCL 官方 C++ 示例直接使用 `pcl::Normal`。

## 验证

`pclgraph-ui-example --check` 会：

1. 准备 source/target；
2. 运行整张图；
3. 检查两路 ISS3D 关键点数都大于 FPFH 的 `k`；
4. 读 `aligned.pcd` 和 `target.pcd`，用 KD-tree 计算平均最近邻距离；
5. 误差必须不超过 `0.02`，否则以非零退出。

当前样例的参考输出：Source VoxelGrid 3133 点、Target VoxelGrid 3123 点、关键点 67/63、对齐平均误差约 `0.003`。

## 与官方样例的关系

这是 PCL 官方 `template_alignment`（SAC-IA + FPFH）的**关键点变体**：

- 官方样例：VoxelGrid → NormalEstimation → FPFH → SAC-IA，不使用关键点。
- 本 demo：在 VoxelGrid 之后加 ISS3D，只在关键点上算法线和 FPFH，再送 SAC-IA；最后用 ICP 精配准。
- 官方 `alignment_prerejective` 使用 `SampleConsensusPrerejective`，PCLgraph 当前没有对应节点；本 demo 不复刻它。
