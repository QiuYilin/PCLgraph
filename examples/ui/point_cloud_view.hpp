#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>

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

/** \brief 点云监视器：每个 Monitor 窗口一份 VTK；来源选择由 taskgraph 管。 */
class PointCloudPanel
    : public std::enable_shared_from_this<PointCloudPanel> {
 public:
  PointCloudPanel();
  ~PointCloudPanel();

  PointCloudPanel(const PointCloudPanel&) = delete;
  PointCloudPanel& operator=(const PointCloudPanel&) = delete;

  MonitorView MakeMonitorView();

 private:
  struct PointCloudInstance;

  void DrawContent(const tg::NodeData& data, float width, float height);
  std::string StatusText(const tg::NodeData* data) const;
  PointCloudInstance& CurrentInstance();
  void DrawOptions(PointCloudInstance& instance);
  void EnsureViewer(PointCloudInstance& instance);
  void UpdateActor(PointCloudInstance& instance, const tg::NodeData& data);
  void ClearActor(PointCloudInstance& instance);

  // 关掉的窗口没有回调，停靠后未激活的页签也不会每帧 draw，所以不按帧回收。
  std::unordered_map<std::string, std::unique_ptr<PointCloudInstance>>
      _instances;
  bool _gl_loader_ready = false;
  bool _gl_loader_failed = false;
  std::string _gl_loader_error;
};
