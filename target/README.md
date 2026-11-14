# B 题 · 两轮自平衡小车 —— 赛题解析与翻译

> **2026 年大学生电子设计竞赛 · 重庆大学校级比赛**
> 原始文件：[`2026年大学生电子设计竞赛重庆大学校级比赛-B题.pdf`](./2026年大学生电子设计竞赛重庆大学校级比赛-B题.pdf)
>
> 本文只做两件事：**整理赛题原文**、**逐条解析并翻译题意**。不含方案论证、器件选型、控制策略或实现建议。
> 如本文与 PDF 原文有任何出入，以 PDF 原文为准。

## 目录

- [1. 题目信息](#1-题目信息)
- [2. 题目原文](#2-题目原文)
  - [2.1 任务](#21-任务)
  - [2.2 要求](#22-要求)
  - [2.3 设计报告](#23-设计报告)
  - [2.4 说明](#24-说明)
- [3. 场地与路径](#3-场地与路径)
- [4. 要求逐条释义](#4-要求逐条释义)
- [5. 评分构成](#5-评分构成)
- [6. 说明与限制释义](#6-说明与限制释义)
- [7. 术语解释](#7-术语解释)
- [8. 原文未明确的事项](#8-原文未明确的事项)
- [9. 英文翻译（参考）](#9-英文翻译参考)
- [附录 原始资料](#附录-原始资料)

---

## 1. 题目信息

| 项目 | 内容 |
| --- | --- |
| 竞赛 | 2026 年大学生电子设计竞赛 重庆大学校级比赛 |
| 题号 | **B 题** |
| 题目 | **两轮自平衡小车** |
| 任务对象 | 采用 MCU 控制的两轮自平衡自动行驶小车 |
| 场地 | 面积不小于 220cm × 120cm，两段对称的黑色半圆弧，半径 40cm，线宽约 1.8cm |
| 主控 | 组委会发放的 **STM32F407ZGT6** 开发板，作为唯一主控 |
| 明令禁止 | 小车上搭载图像传感器 / 摄像头 |
| 现场要求 | 6 项，共 100 分（10 + 20 + 20 + 20 + 20 + 10） |
| 设计报告 | 20 分 |
| 器材 | 单片机、底盘（仅含电机、车轮、亚克力板）均由组委会提供借用，赛后归还 |

---

## 2. 题目原文

> 以下为 PDF 原文的完整转录（仅调整了少量排版，文字未改动）。

### 2.1 任务

设计一个采用 MCU 控制的两轮自平衡自动行驶小车，能在指定路径上自动行驶，行驶场地示意如图 1 所示。场地面积不小于 220cm×120cm。图中两个对称半圆弧线的半径为 40cm，弧线为黑色，线宽 1.8cm 左右，弧线的四个顶点分别定义为 A、B、C 和 D 点。建议场地采用白色哑光喷绘布制作。场地除两个半圆弧外，不得添加任何标记。

**图 1　小车行驶场地示意图**（见 PDF 第 1 页）

### 2.2 要求

1. 将小车放置在起跑线，一键启动后在不用外力搀扶情况下能够立刻实现稳定站立 3 秒，前后移动幅度不超过 15cm。　　（10 分）
2. 将小车放在 A 点，小车能自动行驶到 B 点停车，停车时有声光提示。　　（20 分）
3. 将小车放在位置 A 点，小车自动行驶到 B 点后，沿半弧线行驶到 C 点，再由 C 点自动行驶到 D 点，最后沿半弧线行驶到 A 点停车，每经过一个点，声光提示一次。完成一圈用时不大于 45 秒。　　（20 分）
4. 将小车放在位置 A 点，小车能自动行驶到 C 点后，沿半弧线行驶到 B 点，再由 B 点自动行驶到 D 点，最后沿半弧线行驶到 A 点停车。每经过一个点，声光提示一次。完成一圈用时不大于 60 秒。　　（20 分）
5. 按要求 4 的路径自动行驶 4 圈停车，用时越少越好。　　（20 分）
6. 采用组委会发放的 STM32F407ZGT6 开发板作为唯一主控控制小车的状态。　　（10 分）

### 2.3 设计报告

| 项　目 | 主要内容 | 满分 |
| --- | --- | --- |
| 方案论证 | 比较与选择，方案描述 | 3 |
| 理论分析与计算 | 系统相关参数设计 | 5 |
| 电路与程序设计 | 系统组成，原理框图与各部分的电路图，系统软件与流程图 | 5 |
| 测试方案与测试结果 | 测试结果完整性，测试结果分析 | 5 |
| 设计报告结构及规范性 | 摘要，正文结构规范，图表的完整与准确性 | 2 |
| **总　分** |  | **20** |

### 2.4 说明

1. 小车上不得搭载图像传感器/摄像头。
2. 单片机由组委会提供借用，比赛完成后归还。
3. 小车底盘由组委会提供借用，底盘仅包含如下图所示的电机、车轮和亚克力板。比赛完成后归还。

**附图**（见 PDF 第 2 页）：两轮底盘实物照片，含两个车轮、两台电机、亚克力板与铜柱。

---

## 3. 场地与路径

### 3.1 题给参数

| 项目 | 题给内容 |
| --- | --- |
| 场地面积 | 不小于 **220cm × 120cm** |
| 圆弧 | 两段**对称**的半圆弧线 |
| 圆弧半径 | **40cm** |
| 弧线颜色 | 黑色 |
| 弧线线宽 | **1.8cm 左右** |
| 场地表面 | **建议**采用白色哑光喷绘布制作 |
| 标记限制 | 场地除两个半圆弧外，**不得添加任何标记** |
| 顶点定义 | 弧线的四个顶点分别定义为 **A、B、C、D** 点 |

### 3.2 图示与点位

按图 1 的标注（场地宽 220cm、高 120cm，A–B 间距 100cm，弧半径 40cm，圆心到右边缘、到下边缘均为 60cm）可确定各点的相对位置。以场地左上角为原点，`x` 向右、`y` 向下，单位 cm：

```
          x=60                            x=160
           A ┈┈┈┈┈ 100 cm ┈┈┈┈┈ B              ← 上端点 (y=20)
         ╱                        ╲
  x=20  │       r = 40 cm          │  x=200      y=60（两圆心连线）
         ╲                        ╱
           D ┈┈┈┈┈ 100 cm ┈┈┈┈┈ C              ← 下端点 (y=100)
        （左半圆）              （右半圆）

场地：220cm × 120cm，四周留边 20cm（最左 x=20、最右 x=200）。
图中虚线 A–B、C–D 仅表示几何位置，实际场地上只有两段黑色半圆弧。
```

| 点 | 含义 | 距场地边缘 | 坐标 (x, y) |
| --- | --- | --- | --- |
| A | 左半圆上端点 | 左 60cm、上 20cm | (60, 20) |
| B | 右半圆上端点 | 左 160cm、上 20cm | (160, 20) |
| C | 右半圆下端点 | 左 160cm、下 20cm | (160, 100) |
| D | 左半圆下端点 | 左 60cm、下 20cm | (60, 100) |

| 弧线 | 圆心 | 半径 | 连接的点 | 途经最远点 |
| --- | --- | --- | --- | --- |
| 左半圆弧 | (60, 60) | 40cm | A ↔ D | (20, 60)，距左边缘 20cm |
| 右半圆弧 | (160, 60) | 40cm | B ↔ C | (200, 60)，距右边缘 20cm |

### 3.3 各要求对应的路径

| 要求 | 起点 | 路径顺序 | 终点 | 时限 |
| --- | --- | --- | --- | --- |
| 2 | A | A → B | B（停车，声光提示） | 无 |
| 3 | A | A → B →（右半弧）→ C → D →（左半弧）→ A | A（停车） | ≤ 45 秒 |
| 4 | A | A → C →（右半弧）→ B → D →（左半弧）→ A | A（停车） | ≤ 60 秒 |
| 5 | A | 按第 4 条路径行驶 4 圈 | A（停车） | 越短越好 |

补充说明（据原文可得）：

- 场地只有两段半圆弧，因此 A–B、C–D、A–C、B–D 这些路段在场地中**没有引导线**。
- 第 3 条中，B→C 沿**右半圆弧**、D→A 沿**左半圆弧**；第 4 条中，C→B 沿**右半圆弧**、D→A 沿**左半圆弧**。
- 第 3 条与第 4 条的区别：第 3 条经 B 到 C（右弧从上到下），第 4 条经 C 到 B（右弧从下到上），且第 4 条的 A→C、B→D 为斜向路段。

### 3.4 按题给尺寸的换算

以下仅为尺寸与时限的数学换算，按路径中心线计算，不含起停、对准等额外行程：

| 项目 | 计算 | 结果 |
| --- | --- | --- |
| 单段半圆弧长 | π × 40 | **125.7 cm** |
| A–B、C–D 的直线距离 | 图注 100cm | **100 cm** |
| A–C、B–D 的直线距离 | √(100² + 80²) | **128.1 cm** |
| 第 3 条单圈路径长度 | 100 + 125.7 + 100 + 125.7 | **451.3 cm** |
| 第 3 条折算平均速度 | 451.3 ÷ 45 | **≥ 10.03 cm/s** |
| 第 4 条单圈路径长度 | 128.1 + 125.7 + 128.1 + 125.7 | **507.5 cm** |
| 第 4 条折算平均速度 | 507.5 ÷ 60 | **≥ 8.46 cm/s** |
| 第 5 条四圈路径长度 | 4 × 507.5 | **2029.8 cm（约 20.3m）** |

---

## 4. 要求逐条释义

| # | 原文对象 | 字面释义 | 判分关注点（按原文） |
| --- | --- | --- | --- |
| 1 | 起跑线、一键启动、稳定站立 3 秒、前后幅度 ≤15cm | 把车放在起跑线上，只做一次启动操作；启动后在不借助外力（没有人或物搀扶）的情况下立刻稳定站立 3 秒，期间车体前后方向的移动幅度不超过 15cm | 是否一键启动；是否无需外力；能否稳定站立 3 秒；前后位移是否 ≤15cm |
| 2 | A 点 → B 点、停车、声光提示 | 从 A 点出发，自动行驶到 B 点后停下，停车的同时发出声音和灯光提示 | 是否到达并停在 B 点；停车时是否有声、有光 |
| 3 | A→B→C→D→A、每点提示、≤45 秒 | 从 A 出发到 B，再沿半圆弧到 C，再从 C 到 D，最后沿半圆弧回到 A 停车；每经过一个点都要声光提示一次；从出发到停回 A 点不超过 45 秒 | 路径顺序是否正确；经过的点是否提示；是否停在 A 点；整圈是否 ≤45 秒 |
| 4 | A→C→B→D→A、每点提示、≤60 秒 | 从 A 出发到 C，再沿半圆弧到 B，再从 B 到 D，最后沿半圆弧回到 A 停车；每经过一个点都要声光提示一次；整圈不超过 60 秒 | 路径顺序是否正确；经过的点是否提示；是否停在 A 点；整圈是否 ≤60 秒 |
| 5 | 第 4 条路径 × 4 圈、用时越少越好 | 按第 4 条的路径连续行驶 4 圈后停车；在满足要求的前提下用时越短得分越高 | 是否完成 4 圈；是否停车；总用时 |
| 6 | STM32F407ZGT6、唯一主控 | 小车状态的控制器必须是组委会发放的 STM32F407ZGT6 开发板，且该板是唯一主控 | 是否使用指定开发板；是否作为唯一主控 |

---

## 5. 评分构成

### 5.1 现场要求（满分 100 分）

| # | 要求摘要 | 分值 |
| --- | --- | --- |
| 1 | 一键启动，无外力搀扶稳定站立 3 秒，前后移动 ≤15cm | 10 |
| 2 | A → B 停车，声光提示 | 20 |
| 3 | A→B→C→D→A，每点声光提示，≤45 秒 | 20 |
| 4 | A→C→B→D→A，每点声光提示，≤60 秒 | 20 |
| 5 | 第 4 条路径行驶 4 圈，用时越少越好 | 20 |
| 6 | STM32F407ZGT6 开发板作为唯一主控 | 10 |
| **合计** |  | **100** |

### 5.2 设计报告（满分 20 分）

见 [2.3 设计报告](#23-设计报告)。

---

## 6. 说明与限制释义

| 原文条款 | 释义 | 约束对象 |
| --- | --- | --- |
| 小车上不得搭载图像传感器/摄像头 | 车上不得安装摄像头，也不得安装任何图像传感器 | 车载传感器 |
| 单片机由组委会提供借用，比赛完成后归还 | 主控板由组委会统一发放，不是自备器材；比赛结束后归还 | 主控 |
| 小车底盘由组委会提供借用，底盘仅包含如下图所示的电机、车轮和亚克力板。比赛完成后归还 | 底盘由组委会统一发放，**只包含**电机、车轮、亚克力板；比赛结束后归还 | 车体 |
| 场地除两个半圆弧外，不得添加任何标记（任务段） | 场地上除原有的两段半圆弧外，不能再增加任何标记 | 场地 |

补充：任务段中"场地面积不小于 220cm×120cm"是下限；"建议场地采用白色哑光喷绘布制作"是建议性表述。

---

## 7. 术语解释

| 术语 | 出处 | 按原文的含义 |
| --- | --- | --- |
| 两轮自平衡 | 标题、任务 | 车体只有左右两个车轮（两轮着地），不依靠第三个支撑点即可保持直立 |
| 起跑线 | 要求 1 | 小车的出发位置；图 1 未标注其具体位置 |
| 一键启动 | 要求 1 | 通过一次启动操作使小车开始工作 |
| 不用外力搀扶 | 要求 1 | 启动及站立过程中，没有人或物体扶住小车 |
| 前后移动幅度 | 要求 1 | 车体沿前后方向产生的最大位移量 |
| 声光提示 | 要求 2、3、4 | 同时给出声音提示和灯光提示 |
| 每经过一个点 | 要求 3、4 | 每经过 A、B、C、D 中的一个点，就提示一次 |
| 完成一圈 | 要求 3、4 | 从 A 点出发，按该条路径行驶并回到 A 点 |
| 唯一主控 | 要求 6 | 小车状态只由该 STM32F407ZGT6 开发板控制 |
| 半弧线 / 顶点 | 任务、要求 3、4 | 半圆弧线；"顶点"指半圆弧的两个端点 |

---

## 8. 原文未明确的事项

以下内容原文没有给出明确说明，仅按字面整理，供阅读题目时注意；具体以组委会解释或现场裁判要求为准。

| # | 事项 | 原文情况 | 需要确认的内容 |
| --- | --- | --- | --- |
| 1 | 起跑线的位置 | 要求 1 只写"起跑线"，图 1 未标注 | 起跑线在场地中的具体位置 |
| 2 | 停车精度 | 要求 2、3、4 只写"停车" | 停车位置允许的误差范围 |
| 3 | A 点是否计入提示 | 要求 3、4 写"每经过一个点，声光提示一次"，而起点与终点均为 A | A 点（起点/终点）是否也要提示 |
| 4 | 中途是否允许停车 | 要求 3、4 只在 A 点要求停车；要求 5 写"4 圈停车" | 在 B、C、D 处是否允许停车 |
| 5 | "唯一主控"的边界 | 要求 6 写"作为唯一主控" | 是否允许其他辅助处理单元或带处理功能的传感器 |
| 6 | 底盘的具体配置 | 说明 3 写"仅包含电机、车轮和亚克力板" | 电机参数、轮径、轮距、是否带编码器 |
| 7 | 场地材质 | 任务写"**建议**场地采用白色哑光喷绘布制作" | 是否为强制要求 |
| 8 | 车体尺寸与重量 | 原文未提及 | 是否有尺寸、重量限制 |
| 9 | 电源 | 原文未提及，底盘清单中不含电池 | 电池是否由参赛队自备 |

---

## 9. 英文翻译（参考）

> 以下为题目原文的英文翻译，仅作理解参考；如与中文原文有歧义，以中文原文为准。

### 9.1 Task

Design a two-wheel self-balancing autonomous car controlled by an MCU that can drive automatically along a designated path. The driving field is shown schematically in Figure 1. The field area shall be no less than 220 cm × 120 cm. The two symmetric semicircular arcs in the figure have a radius of 40 cm; the arcs are black with a line width of about 1.8 cm; the four endpoints of the arcs are defined as points A, B, C and D, respectively. It is recommended that the field be made of white matte printed cloth. No markings other than the two semicircular arcs shall be added to the field.

### 9.2 Requirements

1. Place the car on the starting line. After a one-key start, it shall immediately stand stably for 3 seconds without external support, and its fore-and-aft movement shall not exceed 15 cm. (10 points)
2. Place the car at point A. The car shall automatically drive to point B and stop, with an audible and visual indication when it stops. (20 points)
3. Place the car at point A. The car shall automatically drive to point B, then follow the semicircular arc to point C, then automatically drive from point C to point D, and finally follow the semicircular arc to point A and stop. Each time it passes a point, it shall give an audible and visual indication once. One lap shall take no more than 45 seconds. (20 points)
4. Place the car at point A. The car shall automatically drive to point C, then follow the semicircular arc to point B, then automatically drive from point B to point D, and finally follow the semicircular arc to point A and stop. Each time it passes a point, it shall give an audible and visual indication once. One lap shall take no more than 60 seconds. (20 points)
5. Automatically drive 4 laps along the path of Requirement 4 and stop; the shorter the time, the better. (20 points)
6. Use the STM32F407ZGT6 development board issued by the organizing committee as the sole main controller to control the state of the car. (10 points)

### 9.3 Design Report

| Item | Main content | Full marks |
| --- | --- | --- |
| Scheme demonstration | Comparison and selection; scheme description | 3 |
| Theoretical analysis and calculation | Design of system-related parameters | 5 |
| Circuit and program design | System composition; block diagram and circuit diagrams of each part; system software and flowcharts | 5 |
| Test plan and test results | Completeness of test results; analysis of test results | 5 |
| Structure and standardization of the design report | Abstract; standardized structure; completeness and accuracy of figures and tables | 2 |
| **Total** |  | **20** |

### 9.4 Notes

1. The car shall not be equipped with an image sensor or a camera.
2. The microcontroller is provided by the organizing committee on loan and shall be returned after the competition.
3. The car chassis is provided by the organizing committee on loan. The chassis includes only the motors, wheels and acrylic plates shown in the figure below, and shall be returned after the competition.

### 9.5 Field dimensions

- Field: no less than 220 cm × 120 cm.
- Two symmetric semicircular arcs: radius 40 cm, black, line width about 1.8 cm.
- Straight distance between the two upper endpoints A and B: 100 cm; the arc centers lie on the horizontal centerline, 60 cm from the top and bottom edges, and the right arc center is 60 cm from the right edge.
- Arc endpoints: A (left arc, upper end), B (right arc, upper end), C (right arc, lower end), D (left arc, lower end).

---

## 附录 原始资料

| 文件 / 内容 | 说明 |
| --- | --- |
| [`2026年大学生电子设计竞赛重庆大学校级比赛-B题.pdf`](./2026年大学生电子设计竞赛重庆大学校级比赛-B题.pdf) | 题目原文，共 2 页 |
| 第 1 页 · 图 1 | 小车行驶场地示意图（220cm × 120cm、两段半径 40cm 的半圆弧、A/B/C/D 点） |
| 第 2 页 · 附图 | 组委会提供的两轮底盘实物照片（电机、车轮、亚克力板） |

> 第 3.4 节的长度与速度是由题给尺寸换算出的一次量，未计入圆弧线宽（按路径中心线计），也不代表任何实现方案；若与原文冲突，以 PDF 原文为准。

