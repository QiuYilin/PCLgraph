# PCLgraph 项目规则

## taskgraph 改动边界

- PCLgraph 会话（含子任务）对本地 taskgraph checkout
  `C:\Users\qiuyi\program\taskgraph` **只读**。可以读取源码、参与 PCLgraph 构建；
  不得编辑、格式化、改配置、执行 `git add`/`commit`，或做任何会改变其工作树的操作。
- PCLgraph 任务需要 taskgraph 改动时，不要在 PCLgraph 会话里直接改。
  先说明需要改什么、为什么、影响哪些 PCLgraph 功能，然后提醒用户：
  **这块要切换到 taskgraph 项目、交接后执行**。
- PCLgraph 侧可以先完成不依赖该 taskgraph 改动的部分；依赖该改动的功能保持阻塞并写明，
  不要复制 taskgraph 源码或打临时补丁绕过。

## taskgraph 功能不在本项目实现

- 工作台与画布能力属于 taskgraph：标题栏、活动栏、主侧栏、辅助侧栏、面板、状态栏、命令面板；
  画布交互、节点与端口绘制、类型名浮层、连线；会话与工程读写、节点注册、执行与数据流语义；
  日志视图与日志配置。PCLgraph 不复制、不重写，也不打临时补丁绕过。
- PCLgraph 只实现 PCL 领域内容：点类型与字段、PCL 节点算法与参数、`CloudData` /
  `TransformData` 等数据摘要、演示图与示例结果。
- 需要的新能力落在 taskgraph 范围时，按「taskgraph 改动边界」交接：说明缺什么、为什么、
  影响哪些 PCLgraph 功能，切到 taskgraph 项目实现；PCLgraph 侧相关功能保持阻塞，
  不用本地实现顶上。
- 拿不准时按分层判断：与 PCL 无关、且任何 taskgraph 使用方都需要的能力，属于 taskgraph。
