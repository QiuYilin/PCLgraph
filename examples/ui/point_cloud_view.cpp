#include "point_cloud_view.hpp"

#include <GL/gl3w.h>
#include <imgui-vtk/VtkViewer.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <string>
#include <utility>

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

#include <WorkbenchParts.hpp>
#include <taskgraph/Controller.hpp>
#include <taskgraph/Graph.hpp>

#include "pclgraph/point_type.hpp"

namespace {

constexpr const char* kViewWindowId = "point-cloud";
constexpr const char* kViewWindowTitle = "Point Cloud";
constexpr bool kViewWindowOpenOnFirstRun = true;

constexpr const char* kFollowSelectionLabel = "Follow Selection";
constexpr const char* kSourceComboId = "##PointCloudSource";
constexpr const char* kFitLabel = "Fit";
constexpr const char* kPointSizeLabel = "Point Size";
constexpr const char* kNoPointCloudHint =
    "Run the graph to compute a point cloud.";
constexpr const char* kNoSelectedNodeHint = "No node selected.";
constexpr const char* kNoPinnedNodeHint = "No node pinned.";
constexpr const char* kSelectedNodeNoCloudHint =
    "Selected node has no point cloud output.";
constexpr const char* kPinnedNodeNoCloudHint =
    "Pinned node has no point cloud output.";
constexpr const char* kBuildFailedHint = "Failed to build the point cloud.";
constexpr const char* kLoaderFailedHint =
    "OpenGL loader (gl3w) failed; the VTK view cannot start.";
constexpr const char* kFallbackPortCaption = "cloud";

constexpr float kSourceComboWidthEpx = 360.0f;
constexpr float kPointSizeSliderWidthEpx = 140.0f;
constexpr float kDefaultPointSize = 2.0f;
constexpr float kMinPointSize = 1.0f;
constexpr float kMaxPointSize = 10.0f;
constexpr float kMinViewportHeightEpx = 48.0f;
constexpr const char* kPointSizeFormat = "%.1f";

constexpr double kViewerBackground = 0.18;
constexpr double kViewerBackgroundAlpha = 1.0;
constexpr double kElevationRangeMin = 0.0;
constexpr double kElevationRangeMax = 1.0;

constexpr unsigned int kByteMask = 0xFFu;
constexpr unsigned int kRgbRedShift = 16u;
constexpr unsigned int kRgbGreenShift = 8u;
constexpr unsigned int kRgbBlueShift = 0u;

bool HasXyzFields(const pg::PointTypeDesc& desc) {
  const pg::PointFields xyz =
      pg::kFieldX | pg::kFieldY | pg::kFieldZ;
  return (desc.fields & xyz) == xyz;
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

PointCloudPanel::PointCloudPanel(tg::Controller& controller,
                                 SelectionProvider selected_node)
    : _controller(controller),
      _selected_node(std::move(selected_node)),
      _point_size(kDefaultPointSize) {}

PointCloudPanel::~PointCloudPanel() = default;

ViewWindow PointCloudPanel::MakeViewWindow() {
  ViewWindow view;
  view.id = kViewWindowId;
  view.title = kViewWindowTitle;
  view.open_on_first_run = kViewWindowOpenOnFirstRun;
  view.draw = [self = shared_from_this()] { self->Draw(); };
  return view;
}

void PointCloudPanel::Draw() {
  EnsureViewer();
  CollectOutputs();
  const OutputEntry* entry = ActiveOutput();
  if (_viewer != nullptr) {
    UpdateActor(entry);
    // 视图放在窗口内容最上方：VtkViewer 用内容起点换算鼠标坐标，
    // 工具栏放在下面时旋转/缩放的落点才对得上。
    const float footer_height =
        ImGui::GetTextLineHeightWithSpacing() +
        ImGui::GetFrameHeightWithSpacing() +
        ImGui::GetStyle().ItemSpacing.y;
    const ImVec2 available = ImGui::GetContentRegionAvail();
    const float viewport_height =
        (std::max)(available.y - footer_height, kMinViewportHeightEpx);
    _viewer->render(ImVec2(available.x, viewport_height));
  }
  ImGui::TextWrapped("%s", _status.c_str());
  DrawSourceControls();
  if (_viewer == nullptr && !_viewer_error.empty()) {
    ImGui::TextWrapped("%s", _viewer_error.c_str());
  }
}

void PointCloudPanel::CollectOutputs() {
  _entries.clear();
  _pinned_node_exists = false;
  for (const std::shared_ptr<tg::Graph>& graph : _controller.getGraphs()) {
    if (graph == nullptr) {
      continue;
    }
    for (const std::shared_ptr<tg::Node>& node : graph->getNodes()) {
      if (node == nullptr) {
        continue;
      }
      if (node.get() == _pinned_node) {
        _pinned_node_exists = true;
      }
      for (unsigned int port = 0; port < node->outputPortCount(); ++port) {
        std::shared_ptr<pg::CloudData> data =
            std::dynamic_pointer_cast<pg::CloudData>(
                node->getOutPortData(port));
        if (data == nullptr || data->desc() == nullptr ||
            data->cloud() == nullptr || !HasXyzFields(*data->desc())) {
          continue;
        }
        const char* caption =
            node->fixedPortCaption(tg::PortType::Out, port);
        std::string label = graph->getName();
        label += " / ";
        label += node->getName();
        label += " / ";
        label += (caption != nullptr && caption[0] != '\0')
                     ? caption
                     : kFallbackPortCaption;
        OutputEntry entry;
        entry.graph = graph;
        entry.node = node.get();
        entry.port = port;
        entry.data = std::move(data);
        entry.label = std::move(label);
        _entries.push_back(std::move(entry));
      }
    }
  }
  if (!_follow_selection && _pinned_node != nullptr && !_pinned_node_exists) {
    _pinned_node = nullptr;
    _pinned_port = 0;
  }
}

const PointCloudPanel::OutputEntry* PointCloudPanel::ActiveOutput() const {
  tg::Node* target = nullptr;
  bool exact_port = false;
  if (_follow_selection) {
    target = _selected_node ? _selected_node() : nullptr;
  } else {
    target = _pinned_node;
    exact_port = true;
  }
  if (target == nullptr) {
    return nullptr;
  }
  for (const OutputEntry& entry : _entries) {
    if (entry.node != target) {
      continue;
    }
    if (exact_port && entry.port != _pinned_port) {
      continue;
    }
    return &entry;
  }
  return nullptr;
}

void PointCloudPanel::DrawSourceControls() {
  bool follow = _follow_selection;
  if (ImGui::Checkbox(kFollowSelectionLabel, &follow)) {
    if (!follow) {
      if (const OutputEntry* current = ActiveOutput()) {
        _pinned_node = current->node;
        _pinned_port = current->port;
      } else {
        _pinned_node = nullptr;
        _pinned_port = 0;
      }
    } else {
      _pinned_node = nullptr;
      _pinned_port = 0;
    }
    _follow_selection = follow;
  }
  ImGui::SameLine();

  const OutputEntry* current = ActiveOutput();
  tg::Node* selected =
      _follow_selection && _selected_node ? _selected_node() : nullptr;
  const char* preview = nullptr;
  if (current != nullptr) {
    preview = current->label.c_str();
  } else if (_follow_selection) {
    preview = selected != nullptr ? kSelectedNodeNoCloudHint
                                  : kNoSelectedNodeHint;
  } else {
    preview = _pinned_node != nullptr ? kPinnedNodeNoCloudHint
                                      : kNoPinnedNodeHint;
  }

  const float width =
      (std::min)(ImGui::GetContentRegionAvail().x,
                 kSourceComboWidthEpx * ImGui::GetStyle().FontScaleMain);
  ImGui::SetNextItemWidth(width);
  if (ImGui::BeginCombo(kSourceComboId, preview)) {
    if (_entries.empty()) {
      ImGui::TextDisabled("%s", kNoPointCloudHint);
    }
    for (std::size_t index = 0; index < _entries.size(); ++index) {
      const OutputEntry& entry = _entries[index];
      const bool selected_entry =
          current != nullptr && current->node == entry.node &&
          current->port == entry.port;
      ImGui::PushID(static_cast<int>(index));
      if (ImGui::Selectable(entry.label.c_str(), selected_entry)) {
        _follow_selection = false;
        _pinned_node = entry.node;
        _pinned_port = entry.port;
      }
      ImGui::PopID();
    }
    ImGui::EndCombo();
  }

  ImGui::SameLine();
  if (ImGui::Button(kFitLabel) && _viewer != nullptr) {
    _viewer->getRenderer()->ResetCamera();
  }
  ImGui::SameLine();
  ImGui::SetNextItemWidth(kPointSizeSliderWidthEpx *
                          ImGui::GetStyle().FontScaleMain);
  if (ImGui::SliderFloat(kPointSizeLabel, &_point_size, kMinPointSize,
                         kMaxPointSize, kPointSizeFormat)) {
    if (_actor != nullptr) {
      _actor->GetProperty()->SetPointSize(_point_size);
    }
  }
}

void PointCloudPanel::EnsureViewer() {
  if (_viewer != nullptr || _viewer_failed) {
    return;
  }
  if (!_gl_loader_ready) {
    if (gl3wInit() != 0) {
      _viewer_failed = true;
      _viewer_error = kLoaderFailedHint;
      _status = kLoaderFailedHint;
      return;
    }
    _gl_loader_ready = true;
  }
  try {
    _viewer = std::make_unique<VtkViewer>();
    _viewer->getRenderer()->SetBackground(kViewerBackground, kViewerBackground,
                                          kViewerBackground);
    _viewer->getRenderer()->SetBackgroundAlpha(kViewerBackgroundAlpha);
  } catch (const std::exception& error) {
    _viewer_failed = true;
    _viewer_error = error.what();
    _status = _viewer_error;
  }
}

void PointCloudPanel::UpdateActor(const OutputEntry* entry) {
  if (entry == nullptr) {
    ClearActor();
    if (_entries.empty()) {
      _status = kNoPointCloudHint;
    } else if (_follow_selection) {
      _status = (_selected_node && _selected_node() != nullptr)
                    ? kSelectedNodeNoCloudHint
                    : kNoSelectedNodeHint;
    } else {
      _status = _pinned_node != nullptr ? kPinnedNodeNoCloudHint
                                        : kNoPinnedNodeHint;
    }
    return;
  }
  if (_shown_data == entry->data) {
    return;
  }
  ClearActor();
  _shown_data = entry->data;
  PointCloudActorResult result = BuildPointCloudActor(*entry->data);
  if (result.actor == nullptr || _viewer == nullptr) {
    _status = result.message.empty() ? kBuildFailedHint : result.message;
    return;
  }
  _actor = result.actor;
  _actor->GetProperty()->SetPointSize(_point_size);
  _viewer->addActor(_actor);
  std::string status = entry->label;
  status += " / ";
  status += entry->data->desc()->display_name;
  status += " / ";
  status += std::to_string(result.point_count);
  status += " points";
  _status = std::move(status);
}

void PointCloudPanel::ClearActor() {
  if (_actor != nullptr && _viewer != nullptr) {
    _viewer->removeActor(_actor);
  }
  _actor = nullptr;
  _shown_data.reset();
}
