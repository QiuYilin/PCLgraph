#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <vtkActor.h>
#include <vtkSmartPointer.h>

namespace tg {
class NodeData;
}  // namespace tg

namespace pg {
class CloudData;
}  // namespace pg

struct MonitorView;
class VtkViewer;

/** \brief 点云转 VTK 的结果：actor、有效点数与失败说明。 */
struct PointCloudActorResult {
  vtkSmartPointer<vtkActor> actor;
  std::size_t point_count = 0;
  std::string message;
};

/** \brief 把带 XYZ 的 pg::CloudData 转成 VTK actor；不触碰 OpenGL。 */
PointCloudActorResult BuildPointCloudActor(const pg::CloudData& cloud);

/** \brief 点云监视器：VTK 内容 + 点云选项；来源选择由 taskgraph 管。 */
class PointCloudPanel
    : public std::enable_shared_from_this<PointCloudPanel> {
 public:
  PointCloudPanel();
  ~PointCloudPanel();

  PointCloudPanel(const PointCloudPanel&) = delete;
  PointCloudPanel& operator=(const PointCloudPanel&) = delete;

  MonitorView MakeMonitorView();

 private:
  void DrawContent(const tg::NodeData& data, float width, float height);
  std::string StatusText(const tg::NodeData* data) const;
  void DrawOptions();
  void EnsureViewer();
  void UpdateActor(const tg::NodeData& data);
  void ClearActor();

  std::unique_ptr<VtkViewer> _viewer;
  std::string _viewer_error;
  bool _gl_loader_ready = false;
  bool _viewer_failed = false;

  const tg::NodeData* _shown_data = nullptr;
  std::shared_ptr<const void> _shown_cloud;
  vtkSmartPointer<vtkActor> _actor;
  std::string _build_error;
  float _point_size;
};
