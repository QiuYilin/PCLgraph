#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <vtkActor.h>
#include <vtkSmartPointer.h>

namespace tg {
class Controller;
class Graph;
class Node;
}  // namespace tg

namespace pg {
class CloudData;
}  // namespace pg

struct ViewWindow;
class VtkViewer;

/** \brief 点云转 VTK 的结果：actor、有效点数与失败说明。 */
struct PointCloudActorResult {
  vtkSmartPointer<vtkActor> actor;
  std::size_t point_count = 0;
  std::string message;
};

/** \brief 把带 XYZ 的 pg::CloudData 转成 VTK actor；不触碰 OpenGL。 */
PointCloudActorResult BuildPointCloudActor(const pg::CloudData& cloud);

/** \brief 工作台 Point Cloud 面板：默认跟随选中节点，也可锁定到某个输出。 */
class PointCloudPanel
    : public std::enable_shared_from_this<PointCloudPanel> {
 public:
  using SelectionProvider = std::function<tg::Node*()>;

  PointCloudPanel(tg::Controller& controller, SelectionProvider selected_node);
  ~PointCloudPanel();

  PointCloudPanel(const PointCloudPanel&) = delete;
  PointCloudPanel& operator=(const PointCloudPanel&) = delete;

  ViewWindow MakeViewWindow();
  void Draw();

 private:
  struct OutputEntry {
    std::shared_ptr<tg::Graph> graph;
    tg::Node* node = nullptr;
    unsigned int port = 0;
    std::shared_ptr<pg::CloudData> data;
    std::string label;
  };

  void CollectOutputs();
  const OutputEntry* ActiveOutput() const;
  void EnsureViewer();
  void DrawSourceControls();
  void UpdateActor(const OutputEntry* entry);
  void ClearActor();

  tg::Controller& _controller;
  SelectionProvider _selected_node;
  std::vector<OutputEntry> _entries;

  std::unique_ptr<VtkViewer> _viewer;
  std::string _viewer_error;
  bool _gl_loader_ready = false;
  bool _viewer_failed = false;

  bool _follow_selection = true;
  tg::Node* _pinned_node = nullptr;
  unsigned int _pinned_port = 0;
  bool _pinned_node_exists = false;

  std::shared_ptr<pg::CloudData> _shown_data;
  vtkSmartPointer<vtkActor> _actor;
  float _point_size;
  std::string _status;
};
