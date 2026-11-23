#include <pclgraph/pclgraph.hpp>
#include <taskgraph/taskgraph.hpp>
#include <Ui.hpp>

#include <filesystem>
#include <iostream>
#include <string>
#include <system_error>

namespace {

// 可执行文件所在目录（argv[0] 解析）；解析失败则退回当前目录。
std::filesystem::path ExecutableDirectory(const char* argv0) {
  std::error_code error;
  std::filesystem::path invoked(argv0 != nullptr ? argv0 : "");
  if (invoked.empty()) {
    return std::filesystem::current_path(error);
  }
  const std::filesystem::path absolute =
      std::filesystem::absolute(invoked, error);
  if (error) {
    return std::filesystem::current_path(error);
  }
  return absolute.parent_path();
}

}  // namespace

int main(int argc, char** argv) {
  tg::Controller project;
  pg::RegisterPclNodes(project);

  const char* argv0 = argc > 0 ? argv[0] : nullptr;
  const std::filesystem::path demo_root =
      ExecutableDirectory(argv0) / "demo-project";
  if (!project.openProject(demo_root)) {
    std::cerr << "Failed to open demo project: " << demo_root << '\n';
    return 1;
  }

  // --check：不开窗口，只验证 demo 工程能载入并跑通。
  if (argc > 1 && std::string(argv[1]) == "--check") {
    std::cout << "project: " << project.projectName() << '\n';
    for (const auto& graph : project.getGraphs()) {
      std::cout << "graph: " << graph->getName() << '\n';
      if (const auto error = project.tryRun(*graph)) {
        std::cerr << "run failed: " << *error << '\n';
        return 1;
      }
    }
    const auto output = demo_root / "out.pcd";
    std::cout << "output: " << output.string()
              << " (exists=" << std::filesystem::exists(output) << ")\n";
    return std::filesystem::exists(output) ? 0 : 1;
  }

  Ui ui(project);
  ui.Run();
  return 0;
}
