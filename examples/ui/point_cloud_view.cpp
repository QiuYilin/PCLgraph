#include "point_cloud_view.hpp"

#include <GL/gl3w.h>
#include <imgui-vtk/VtkViewer.h>
#include <imgui_internal.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <string>

#include <pcl/PCLPointCloud2.h>

#include <vtkElevationFilter.h>
#include <vtkFloatArray.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVertexGlyphFilter.h>

#include <ThemeStyle.hpp>
#include <UiWidgets.hpp>
#include <WorkbenchParts.hpp>

#include "pclgraph/point_type.hpp"

namespace {

constexpr const char* kMonitorViewId = "point-cloud";
constexpr const char* kMonitorViewTitle = "Point Cloud";
constexpr bool kMonitorOpenOnFirstRun = false;

constexpr const char* kFitLabel = "Fit";
constexpr const char* kMoreActionsLabel = "...";
constexpr const char* kViewOptionsTooltip = "View Options";
constexpr const char* kViewOptionsPopupId = "##PointCloudViewOptions";
constexpr const char* kPointSizeLabel = "Point Size";
constexpr const char* kBuildFailedHint = "Failed to build the point cloud.";
constexpr const char* kLoaderFailedHint =
    "OpenGL loader (gl3w) failed; the VTK view cannot start.";

constexpr float kPointSizeSliderWidthEpx = 140.0f;
constexpr float kDefaultPointSize = 2.0f;
constexpr float kMinPointSize = 1.0f;
constexpr float kMaxPointSize = 10.0f;
constexpr float kMinViewportHeightEpx = 48.0f;
constexpr const char* kPointSizeFormat = "%.1f";

constexpr WidgetMetrics kToolbarTextMetrics{kOutputControlHeightRatio, 0.0f,
                                             false};

constexpr double kViewerBackground = 0.18;
constexpr double kViewerBackgroundAlpha = 1.0;
constexpr double kElevationRangeMin = 0.0;
constexpr double kElevationRangeMax = 1.0;

constexpr unsigned int kByteMask = 0xFFu;
constexpr unsigned int kRgbRedShift = 16u;
constexpr unsigned int kRgbGreenShift = 8u;
constexpr unsigned int kRgbBlueShift = 0u;
constexpr const char* kFallbackInstanceId = "point-cloud";

// draw 发生在该实例的 ImGui::Begin 里，窗口名是 "Point Cloud###monitor.point-cloud.N"。
std::string CurrentMonitorInstanceKey() {
  const ImGuiWindow* window = ImGui::GetCurrentWindowRead();
  if (window == nullptr || window->Name == nullptr || window->Name[0] == '\0') {
    return kFallbackInstanceId;
  }
  const std::string name(window->Name);
  const std::string marker(kWindowIdSeparator);
  const std::size_t separator = name.rfind(marker);
  if (separator == std::string::npos ||
      separator + marker.size() >= name.size()) {
    return name;
  }
  return name.substr(separator + marker.size());
}

bool HasXyzFields(const pg::PointTypeDesc& desc) {
  const pg::PointFields xyz =
      pg::kFieldX | pg::kFieldY | pg::kFieldZ;
  return (desc.fields & xyz) == xyz;
}

bool AcceptsPointCloud(const tg::NodeData& data) {
  const auto* cloud = dynamic_cast<const pg::CloudData*>(&data);
  return cloud != nullptr && cloud->desc() != nullptr &&
         cloud->cloud() != nullptr && HasXyzFields(*cloud->desc());
}

const pcl::PCLPointField* FindField(const pcl::PCLPointCloud2& blob,
                                    const char* name) {
  for (const pcl::PCLPointField& field : blob.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

bool ReadFieldFloat(const pcl::PCLPointCloud2& blob, std::size_t point_index,
                    const pcl::PCLPointField& field, float& value) {
  const std::size_t offset =
      point_index * static_cast<std::size_t>(blob.point_step) + field.offset;
  if (offset + sizeof(float) > blob.data.size()) {
    return false;
  }
  std::memcpy(&value, blob.data.data() + offset, sizeof(float));
  return true;
}

bool ReadFieldUint32(const pcl::PCLPointCloud2& blob, std::size_t point_index,
                     const pcl::PCLPointField& field, std::uint32_t& value) {
  const std::size_t offset =
      point_index * static_cast<std::size_t>(blob.point_step) + field.offset;
  if (offset + sizeof(std::uint32_t) > blob.data.size()) {
    return false;
  }
  std::memcpy(&value, blob.data.data() + offset, sizeof(std::uint32_t));
  return true;
}

bool IsFloatField(const pcl::PCLPointField* field) {
  return field != nullptr && field->datatype == pcl::PCLPointField::FLOAT32;
}

}  // namespace

struct PointCloudPanel::PointCloudInstance {
  std::unique_ptr<VtkViewer> viewer;
  std::string viewer_error;
  bool viewer_failed = false;
  const tg::NodeData* shown_data = nullptr;
  std::shared_ptr<const void> shown_cloud;
  vtkSmartPointer<vtkActor> actor;
  std::string build_error;
  float point_size = kDefaultPointSize;
};

PointCloudActorResult BuildPointCloudActor(const pg::CloudData& cloud) {
  PointCloudActorResult result;
  const pg::PointTypeDesc* desc = cloud.desc();
  if (desc == nullptr || cloud.cloud() == nullptr) {
    result.message = "Cloud data is empty.";
    return result;
  }
  if (!HasXyzFields(*desc) || desc->to_cloud2 == nullptr) {
    result.message = "Point type has no x/y/z fields.";
    return result;
  }

  pcl::PCLPointCloud2 blob;
  desc->to_cloud2(cloud.cloud(), blob);
  const pcl::PCLPointField* x_field = FindField(blob, "x");
  const pcl::PCLPointField* y_field = FindField(blob, "y");
  const pcl::PCLPointField* z_field = FindField(blob, "z");
  if (!IsFloatField(x_field) || !IsFloatField(y_field) ||
      !IsFloatField(z_field)) {
    result.message = "Point cloud x/y/z fields are not float32.";
    return result;
  }

  const std::size_t point_count =
      static_cast<std::size_t>(blob.width) * blob.height;
  if (point_count == 0) {
    result.message = "Point cloud has no points.";
    return result;
  }

  const pcl::PCLPointField* color_field = FindField(blob, "rgb");
  if (color_field == nullptr) {
    color_field = FindField(blob, "rgba");
  }
  const bool has_color = IsFloatField(color_field);
  const pcl::PCLPointField* intensity_field = FindField(blob, "intensity");
  const bool has_intensity = !has_color && IsFloatField(intensity_field);

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  vtkNew<vtkUnsignedCharArray> colors;
  colors->SetName("Colors");
  colors->SetNumberOfComponents(3);
  vtkNew<vtkFloatArray> intensity;
  intensity->SetName("Intensity");
  intensity->SetNumberOfComponents(1);

  for (std::size_t point = 0; point < point_count; ++point) {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    if (!ReadFieldFloat(blob, point, *x_field, x) ||
        !ReadFieldFloat(blob, point, *y_field, y) ||
        !ReadFieldFloat(blob, point, *z_field, z)) {
      continue;
    }
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    points->InsertNextPoint(x, y, z);
    if (has_color) {
      std::uint32_t packed = 0;
      ReadFieldUint32(blob, point, *color_field, packed);
      const double red =
          static_cast<double>((packed >> kRgbRedShift) & kByteMask);
      const double green =
          static_cast<double>((packed >> kRgbGreenShift) & kByteMask);
      const double blue =
          static_cast<double>((packed >> kRgbBlueShift) & kByteMask);
      colors->InsertNextTuple3(red, green, blue);
    } else if (has_intensity) {
      float value = 0.0f;
      if (ReadFieldFloat(blob, point, *intensity_field, value) &&
          std::isfinite(value)) {
        intensity->InsertNextValue(value);
      } else {
        intensity->InsertNextValue(0.0f);
      }
    }
  }

  if (points->GetNumberOfPoints() == 0) {
    result.message = "Point cloud has no finite x/y/z points.";
    return result;
  }

  vtkNew<vtkPolyData> polydata;
  polydata->SetPoints(points);
  if (has_color) {
    polydata->GetPointData()->SetScalars(colors);
  } else if (has_intensity) {
    polydata->GetPointData()->SetScalars(intensity);
  }

  vtkNew<vtkVertexGlyphFilter> glyph;
  if (!has_color && !has_intensity) {
    const double* bounds = polydata->GetBounds();
    vtkNew<vtkElevationFilter> elevation;
    elevation->SetInputData(polydata);
    elevation->SetLowPoint(kElevationRangeMin, kElevationRangeMin, bounds[4]);
    elevation->SetHighPoint(kElevationRangeMin, kElevationRangeMin, bounds[5]);
    elevation->SetScalarRange(kElevationRangeMin, kElevationRangeMax);
    glyph->SetInputConnection(elevation->GetOutputPort());
  } else {
    glyph->SetInputData(polydata);
  }
  glyph->Update();

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(glyph->GetOutputPort());
  mapper->ScalarVisibilityOn();
  if (has_color) {
    mapper->SetColorModeToDirectScalars();
  } else if (has_intensity) {
    mapper->SetScalarModeToUsePointFieldData();
    mapper->SelectColorArray("Intensity");
    mapper->SetUseLookupTableScalarRange(true);
    mapper->SetScalarRange(intensity->GetRange());
  } else {
    mapper->SetScalarModeToUsePointData();
    mapper->SetScalarRange(kElevationRangeMin, kElevationRangeMax);
  }

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetPointSize(kDefaultPointSize);

  result.actor = actor.GetPointer();
  result.point_count = static_cast<std::size_t>(points->GetNumberOfPoints());
  return result;
}

PointCloudPanel::PointCloudPanel() = default;

PointCloudPanel::~PointCloudPanel() = default;

MonitorView PointCloudPanel::MakeMonitorView() {
  MonitorView view;
  view.id = kMonitorViewId;
  view.title = kMonitorViewTitle;
  view.open_on_first_run = kMonitorOpenOnFirstRun;
  view.accepts = &AcceptsPointCloud;
  view.draw = [self = shared_from_this()](const tg::NodeData& data,
                                          float width, float height) {
    self->DrawContent(data, width, height);
  };
  view.status = [self = shared_from_this()](const tg::NodeData* data) {
    return self->StatusText(data);
  };
  return view;
}

PointCloudPanel::PointCloudInstance& PointCloudPanel::CurrentInstance() {
  const std::string key = CurrentMonitorInstanceKey();
  std::unique_ptr<PointCloudInstance>& slot = _instances[key];
  if (slot == nullptr) {
    slot = std::make_unique<PointCloudInstance>();
  }
  return *slot;
}

void PointCloudPanel::DrawContent(const tg::NodeData& data, float width,
                                  float height) {
  if (width <= 0.0f || height <= 0.0f) {
    return;
  }
  PointCloudInstance& instance = CurrentInstance();
  EnsureViewer(instance);
  UpdateActor(instance, data);
  if (instance.viewer == nullptr) {
    const std::string& error = !_gl_loader_error.empty() ? _gl_loader_error
                                                         : instance.viewer_error;
    if (!error.empty()) {
      ImGui::TextWrapped("%s", error.c_str());
    }
    return;
  }
  // 视口贴在内容区顶部：imgui-vtk 用子窗口内容起点换算鼠标坐标。
  const float ui_scale = ImGui::GetStyle().FontScaleMain;
  const float options_height =
      kOutputToolbarHeightEpx * ui_scale + ImGui::GetStyle().ItemSpacing.y;
  const float viewport_height =
      (std::max)(height - options_height, kMinViewportHeightEpx);
  instance.viewer->render(ImVec2(width, viewport_height));
  if (!instance.build_error.empty()) {
    ImGui::TextDisabled("%s", instance.build_error.c_str());
    return;
  }
  DrawOptions(instance);
}

std::string PointCloudPanel::StatusText(const tg::NodeData* data) const {
  const auto* cloud = dynamic_cast<const pg::CloudData*>(data);
  if (cloud == nullptr || cloud->desc() == nullptr) {
    return {};
  }
  std::string text = cloud->desc()->display_name;
  text += " / ";
  text += std::to_string(cloud->pointCount());
  text += " points";
  return text;
}

void PointCloudPanel::DrawOptions(PointCloudInstance& instance) {
  const float ui_scale = ImGui::GetStyle().FontScaleMain;
  // 和 Output 相同的 22×s 紧凑行；来源选择由宿主画，这里只留 Fit 与 "..."。
  PushToolbarStyle();
  if (TextAction(kFitLabel, kToolbarTextMetrics)) {
    instance.viewer->getRenderer()->ResetCamera();
  }

  ImGui::SameLine();
  if (TextAction(kMoreActionsLabel, kToolbarTextMetrics, false,
                 kViewOptionsTooltip)) {
    ImGui::OpenPopup(kViewOptionsPopupId);
  }
  if (ImGui::BeginPopup(kViewOptionsPopupId)) {
    ImGui::SetNextItemWidth(kPointSizeSliderWidthEpx * ui_scale);
    if (ImGui::SliderFloat(kPointSizeLabel, &instance.point_size, kMinPointSize,
                           kMaxPointSize, kPointSizeFormat)) {
      if (instance.actor != nullptr) {
        instance.actor->GetProperty()->SetPointSize(instance.point_size);
      }
    }
    ImGui::EndPopup();
  }
  PopToolbarStyle();
}

void PointCloudPanel::EnsureViewer(PointCloudInstance& instance) {
  if (instance.viewer != nullptr || instance.viewer_failed ||
      _gl_loader_failed) {
    return;
  }
  if (!_gl_loader_ready) {
    if (gl3wInit() != 0) {
      _gl_loader_failed = true;
      _gl_loader_error = kLoaderFailedHint;
      return;
    }
    _gl_loader_ready = true;
  }
  try {
    instance.viewer = std::make_unique<VtkViewer>();
    instance.viewer->getRenderer()->SetBackground(
        kViewerBackground, kViewerBackground, kViewerBackground);
    instance.viewer->getRenderer()->SetBackgroundAlpha(kViewerBackgroundAlpha);
  } catch (const std::exception& error) {
    instance.viewer_failed = true;
    instance.viewer_error = error.what();
  }
}

void PointCloudPanel::UpdateActor(PointCloudInstance& instance,
                                  const tg::NodeData& data) {
  const auto* cloud = dynamic_cast<const pg::CloudData*>(&data);
  std::shared_ptr<const void> cloud_data =
      cloud != nullptr ? cloud->cloud() : nullptr;
  if (&data == instance.shown_data &&
      cloud_data.get() == instance.shown_cloud.get()) {
    return;
  }
  ClearActor(instance);
  instance.shown_data = &data;
  instance.shown_cloud = std::move(cloud_data);
  if (cloud == nullptr || instance.shown_cloud == nullptr ||
      instance.viewer == nullptr) {
    return;
  }
  PointCloudActorResult result = BuildPointCloudActor(*cloud);
  if (result.actor == nullptr) {
    instance.build_error =
        result.message.empty() ? kBuildFailedHint : result.message;
    return;
  }
  instance.actor = result.actor;
  instance.actor->GetProperty()->SetPointSize(instance.point_size);
  instance.viewer->addActor(instance.actor);
}

void PointCloudPanel::ClearActor(PointCloudInstance& instance) {
  if (instance.actor != nullptr && instance.viewer != nullptr) {
    instance.viewer->removeActor(instance.actor);
  }
  instance.actor = nullptr;
  instance.shown_data = nullptr;
  instance.shown_cloud.reset();
  instance.build_error.clear();
}
