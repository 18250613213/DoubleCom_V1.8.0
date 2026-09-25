# V1.9.1 UI and Direction Controls Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 完成字体放大、串口实时数据独立标签、四方向独立控制按钮，并发布 V1.9.1 单文件程序。

**Architecture:** 保持现有 PyQt5 单窗口架构，只重组既有控件父布局；方向统计对象增加显式终止状态，主窗口使用统一控制方法和按钮状态刷新函数。业务统计、报告生成和三路独立 ENU 参考逻辑保持不变。

**Tech Stack:** Python 3、PyQt5、PyQtGraph、unittest、PyInstaller、Git

**Spec:** `optimized_v1.9.0/docs/superpowers/specs/2026-09-17-ui-direction-controls-v1.9.1.md`

## Global Constraints

- 仅修改 `optimized_v1.9.0`，不修改根目录原版源代码。
- 三路 ENU 不得共享参考点。
- 暂停/继续保留累计统计和累计时长；重新测试清空全部方向数据。
- 终止保留统计和快照，且重新测试前不可继续。
- 发布版本为 1.9.1，并生成可运行的单文件 EXE。

---

### Task 1: 为界面布局和方向状态写失败测试

**Files:**
- Create: `optimized_v1.9.0/tests/test_ui_direction_controls.py`
- Modify: `optimized_v1.9.0/tests/test_optimized_core.py`

**Interfaces:**
- Consumes: `NMEADataAnalyzer`、`DirectionStatistics`
- Produces: 独立数据流标签、放大字体、四按钮及终止状态的回归约束

- [ ] **Step 1: 写真实窗口控件测试**

  实例化离屏 Qt 窗口，验证“串口实时数据流”标签包含 `data_preview`；验证操作区控件字体不小于 14px；验证每个方向存在四个独立按钮及其初始可用状态。

- [ ] **Step 2: 写统计终止状态测试**

  启动、暂停、继续和终止一个 `DirectionStatistics`，验证累计时长和样本保留，且终止后不能再次启动。

- [ ] **Step 3: 运行测试并确认因缺少新界面与终止状态而失败**

  Run: `python -m unittest tests.test_ui_direction_controls tests.test_optimized_core -v`

### Task 2: 实现界面重组和显式方向控制状态机

**Files:**
- Modify: `optimized_v1.9.0/main.py`
- Modify: `optimized_v1.9.0/src/positioning/direction_statistics.py`

**Interfaces:**
- Consumes: Task 1 的行为测试
- Produces: `DirectionStatistics.terminate()`、`is_terminated`、每方向 `start/pause/resume/terminate` 按钮处理器

- [ ] **Step 1: 添加统计对象终止状态**

  `terminate()` 结算时长并锁定对象；`start()` 在终止后返回失败；`reset()` 清除终止状态。

- [ ] **Step 2: 将信息流控件迁移到独立主标签**

  新建“串口实时数据流”页面，复用原有控件和信号，不复制数据管线。

- [ ] **Step 3: 放大全局操作区字体与尺寸**

  标题 16px、分区及交互控件至少 14px，同时调整最小高度和内边距。

- [ ] **Step 4: 创建每方向四按钮并连接独立操作**

  统一状态刷新按钮可用性；启动其他方向时只暂停同串口当前活动方向；终止保存 ENU 快照并锁定该方向。

- [ ] **Step 5: 运行目标测试并确认通过**

  Run: `python -m unittest tests.test_ui_direction_controls tests.test_optimized_core -v`

### Task 3: 回归验证、版本更新与发布

**Files:**
- Modify: `optimized_v1.9.0/main.py`
- Modify: `optimized_v1.9.0/DoubleCom_Optimized.spec`
- Modify: `optimized_v1.9.0/README.md`
- Modify: `optimized_v1.9.0/release/SHA256SUMS.txt`
- Replace: `optimized_v1.9.0/release/DoubleCom_Optimized_V1.9.0.exe` with `DoubleCom_Optimized_V1.9.1.exe`

**Interfaces:**
- Consumes: 完成的 V1.9.1 源码和测试
- Produces: 经冒烟验证并带 SHA-256 的 V1.9.1 单文件程序

- [ ] **Step 1: 更新版本、构建名称与 README**

  将应用版本和输出名称更新为 1.9.1，并记录新界面和方向控制语义。

- [ ] **Step 2: 运行完整源码验证**

  Run: `python -m compileall -q main.py selftest_cli.py src tests`

  Run: `python -m unittest discover -s tests -v`

  Run: `python selftest_cli.py --minutes 5 --speed 100`

- [ ] **Step 3: 提交源码变更**

  Commit: `feat(v1.9.1): 优化界面并拆分方向测试控制`

- [ ] **Step 4: 构建、冒烟运行并计算哈希**

  Run: `python -m PyInstaller --clean --noconfirm --workpath .build --distpath release DoubleCom_Optimized.spec`

  启动 EXE 后验证进程存活，再终止所有路径完全匹配的冒烟进程；计算 SHA-256 并更新校验文件。

- [ ] **Step 5: 提交发布物并创建标签**

  Commit: `build(v1.9.1): 发布优化版单文件EXE`

  Tag: `v1.9.1-optimized`

