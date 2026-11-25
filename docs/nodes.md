# Nodes

PCLgraph registers its nodes with `pg::RegisterPclNodes(tg::Controller&)`. Point clouds travel through the graph as `pg::CloudData` (a type-erased handle plus a concrete point-type descriptor); ports match point-type ids strictly, so changing type requires the Convert node.

## Node list

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

Category names are single `3D/<group>` strings; taskgraph groups nodes by that string and does not render a two-level tree.

## Point types

- Supported XYZ types follow the installed PCL's `PCL_XYZ_POINT_TYPES`, plus `pcl::Normal`.
- Filter type dropdowns are generated from PCL's precompiled instantiations.
- Reader `point_type` defaults to `Auto` and infers from the file header.
- Writer `input_type` must match its upstream.
- Type selectors (`point_type` / `input_type` / `output_type`) appear on the port, not in the property table.

## Type constraints

### IO

- `Concatenate Fields` takes a main cloud plus a `Normal` cloud and outputs `PointXYZ / PointXYZI / PointXYZRGB / PointXYZL` combinations.
- `Merge` concatenates two clouds of the same type.
- `Reader` and `Writer` follow the rules in [Point types](#point-types).

### Filters

- `MedianFilter3d` requires an organized cloud.
- `BilateralFilter` supports only `PointXYZI / PointXYZINormal`.
- `FastBilateralFilter` supports only `PointXYZ / PointXYZRGB / PointXYZRGBA`, and requires an organized cloud.
- `ShadowPoints` and `CovarianceSampling` take a second `Normal` input.
- `CovarianceSampling` supports only PCL's 5 precompiled main point types.
- `NormalSpaceSampling` requires `Normal` and supports `PointXYZ / PointXYZI`.
- Other filters use PCL's precompiled instantiations.

### Keypoints

- `SIFT` is fixed `PointXYZI → PointXYZ`.
- `ISS3D` takes and outputs `PointXYZ`.
- `Harris3D` takes `PointXYZ + Normal` and outputs `PointXYZI`.

### Features

- `NormalEstimation` uses PCL's precompiled type pairs.
- `FPFH` takes a `Normal` input, supports PCL's 5 core main types, and outputs `FPFHSignature33`.

### Registration

- `ICP Point-to-Point` outputs an aligned cloud and a `Transform`; it supports only PCL's precompiled `PointXYZ / PointXYZI / PointXYZRGB` same-type pairs.
- `NDT` outputs a `Transform` with the same type range as Point-to-Point.
- `SAC-IA` takes source/target clouds plus their `FPFH` features (4 inputs), supports the FPFH type range, and outputs an aligned cloud and a `Transform`.
- `ICP Point-to-Plane` supports only `PointNormal / PointXYZINormal`.
- `FPCS` supports `PointXYZ / PointXYZI`, accepts optional normals, and outputs an aligned cloud and a `Transform`.

### Tools

- `Convert` changes the point type.
- `Transform PointCloud` applies a `Transform` to any registered XYZ cloud.
- `Calc Model Reso` takes a fixed `PointXYZ` cloud and outputs a `Double` mean nearest-neighbor distance (the original plugin outputs the squared value).

Each node's port type selector is the final authority for its supported set.

For the extension API, see [custom-nodes.md](custom-nodes.md).
