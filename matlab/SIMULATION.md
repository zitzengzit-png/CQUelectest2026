# 平衡车控制仿真文档（MATLAB）

> 适用固件：`prj/elet2026`（TIM2 @4199、三环 250/50/10 Hz）
> 仿真入口：`matlab/balance_model.m`；自动调参：`matlab/autotune_pid.m`
> 本文所有数字均为本机实跑输出（MATLAB R2025b，12 V，总延迟 10 ms）

---

## 1. 目的与范围

| 做什么 | 不做什么 |
|---|---|
| 复现固件的三环控制结构（含采样率、零阶保持、延迟、量化、饱和、摩擦） | 不替代实车整定（符号、死区、延迟仍需实测） |
| 用解析设计给出增益的**量级**，再用时域仿真验证 | 不含巡线/路径状态机（见 `tasks/CONTROL_TASK.md`） |
| 多工况**自动调参**，输出可直接抄进 `robot_config.h` 的 `#define` | 不含轮地打滑、偏航-平衡耦合、地面不平度 |
| 生成可视化（频闪图、对比图、动画 GIF） | 不做代码生成 / HIL |

**唯一参数来源原则**：模型通过 `read_robot_config.m` 直接解析
`prj/elet2026/tasks/robot_config.h`，**不在 MATLAB 里另存一份增益**。改固件宏后
`balance_model` 立刻反映，避免"文档一套、代码一套"。

---

## 2. 文件清单与运行方式

| 文件 | 作用 |
|---|---|
| `read_robot_config.m` | 解析 `robot_config.h` 的数值型 `#define` |
| `balance_model.m` | 建模 / 线性设计 / 时域验证 / 敏感性分析；`balance_model('api')` 返回内部函数句柄 |
| `autotune_pid.m` | 自动调参（4 个后端：pattern / patternsearch / fmincon / fminsearch） |
| `vis_balance.m` | 位置阶跃频闪图 + 速度阶跃动画 |
| `vis_stand.m` | 要求 1（原地站立）激励—回位图 |
| `vis_attitude.m` | 姿态环单独 vs HOLD 的扰动对比 |
| `vis_tuned.m` | 用自动调参结果重跑并出对比图 / 动画 |
| `bm_draw_car.m` | 共享的侧视画车函数 |

```matlab
cd matlab
balance_model          % ~30 s，打印 12 节表格
autotune_pid           % ~4 s，写 logs/tuned_pid.txt
vis_balance; vis_stand; vis_attitude; vis_tuned
```

> **批处理注意**：`-batch` 下**不要调用 `usejava('desktop')`**——R2025b 在
> `-batch` 里调 `usejava` 会偶发内部断言崩溃
> （`Failed to bind constant term '[M-function usejava]'`）。
> `balance_model.m` 已改用 `batchStartupOptionUsed`。

---

## 3. 参数来源（可追溯）

| 类别 | 参数 | 值 | 来源 |
|---|---|---|---|
| DATASHEET | 空载转速 / 最大扭矩 / 减速比 | 170 rpm / 3.3 kg·cm / 1:35 | 迈辰微 GA25-370BMQ V1.0 |
| DATASHEET | 额定电压 / 堵转电流 / 质量 | 12 V / 1.8 A / 90 g | 同上 |
| MEASURED | 轮径 / 轮距 | 76 mm / 170 mm | 用户实测 |
| MEASURED | 重心高度（轴上方） | 40 mm | 用户实测 |
| MEASURED | 光电排距轴 / 排间距 / 探头间距 | 50 / 100 / 11.5 mm | 用户实测 |
| MEASURED | 编码器 | 385 线/轮圈 ×4 = 1540 count/rev | 用户确认（已含减速比） |
| MEASURED | 自由倒下时间（5°→20°） | 200 ms → `Jp = 0.0031 kg·m²` | 用户实测（反推） |
| CODE | PWM 满量程 / 三环频率 | 4199 / 250-50-10 Hz | `robot_config.h` |
| ESTIMATE | 车体质量 / 轮质量 | 0.62 / 0.10 kg | 按零件估算（未称重） |
| ESTIMATE | 死区 | 170 count（≈4%） | 常规值，**待实测** |

---

## 4. 被控对象模型

### 4.1 两自由度（轮轴位移 x、机身俯仰 θ）

```
M·ẍ + m·l·θ̈ = τ/R
(J + m·l²)·θ̈ − m·g·l·θ = −m·l·ẍ
```

消去 ẍ 得

```
θ̈ = ω0²·θ − A·u ,   ω0² = m·g·l / I_eff ,   A = (m·l / (M·R·I_eff))·Kt
I_eff = J + m·l² − (m·l)²/M ,   M = m + m_w + J_w/R²
```

线性 PD 后的闭环：`ωn² = A·Kp − ω0²`，`2ζωn = A·Kd`。
用"满占空比轮加速度 a_max"改写后（**不含质量/惯量**，便于现场标定）：

```
Kp[PWM/rad] = (k²+1)·u_max·g / a_max ,   Kd = 2ζk·u_max·g / (ω0·a_max)
```

### 4.2 电机模型

```
τ = Kt·u − (τ_stall/v_nl)·v − τ_f·tanh(v/v_eps)      （Kt = τ_stall / u_max）
```

- 反电动势（droop）项使"纯 PD 无前馈"的闭环出现 **+4.49 /s 的右半平面极点**
  → 必须加速度前馈 `k_ff = (τ_stall/v_nl)/Kt = 6207 count/(m/s)`；
- 摩擦项幅值取与实测死区等效的力矩（170 count × Kt ≈ 0.013 N·m），
  `v_eps = 10 mm/s`（平滑静摩擦）。

### 4.3 12 V 下的数值

```
v_nl = 0.676 m/s     τ_stall = 0.324 N·m/轮    a_max(不打滑) = 22.1 m/s²
M = 0.770 kg         I_eff = 0.00230 kg·m²     l_eff = 0.093 m
ω0 = 10.28 rad/s (1.64 Hz)                     自由倒下（5°→20°）= 201 ms
```

---

## 5. 控制器结构（与固件逐项对应）

```
目标位置 --10 Hz P--> 目标速度 --50 Hz P--> 目标倾角 --250 Hz PD--> 共模 PWM
                                                        ^
                                        速度前馈 k_ff*v |
     +- 延迟补偿：pitch_used = pitch + pitch_rate*10 ms
     +- 摩擦前馈：u += 170*tanh(u/20)

目标 yaw --50 Hz P--> 目标 yaw rate --250 Hz PI--> 差模 PWM --> 左右轮
```

| 环 | 频率 | 形式 | 固件宏 | 当前值 |
|---|---|---|---|---|
| 位置外环 | 10 Hz | P（限 0.5 m/s） | `PID_POS_KP_DISTANCE` | **1.44** |
| 速度中环 | 50 Hz | P（限 ±10°） | `PID_SPEED_KP` | 12.0 |
| 姿态内环 | 250 Hz | PD | `PID_ATT_KP_PITCH` / `KD` | **149.0 / 10.30** |
| 姿态前馈 | 250 Hz | `k_ff*v` | `PID_ATT_FF_SPEED` | 6207 |
| 延迟补偿 | 250 Hz | `θ+θ̇T` | `PID_DELAY_COMP_S` | 0.010 s |
| 摩擦前馈 | 250 Hz | `170*tanh(u/20)` | `MOTOR_DEADBAND_PWM` / `_EPS` | 170 / 20 |
| yaw 角度 | 50 Hz | P | `PID_YAW_ANGLE_KP` | 3.0 |
| yaw rate | 250 Hz | PI（抗饱和） | `PID_YAW_KP/KI` | 18.5 / 230 |
| 差模限幅 | — | `|diff| <= 2100` | `PID_ATT_MAX_DIFF_PWM` | 2100 |

**共模/差模合成**：`L = common - diff`、`R = common + diff`，各自限到 ±4199。
`|common| + |diff| > 4199` 时超出量由共模与差模各承担一半，所以差模上限取 2100
给平衡留头部空间。

**姿态环不加积分**：位置/速度环已提供等效积分作用，给倾角环加 I 会在模式切换时
windup。依据见 `[2b]`（无速度前馈时闭环出现 +4.49 /s 右半平面极点）与 `[10c]`
（坡上静差必须靠**速度环** PI 消除）。

---

## 6. 采样、延迟、量化建模（与固件一致）

| 项 | 固件实现 | 仿真实现 |
|---|---|---|
| 内环节拍 | 4 ms（250 Hz，IMU DMA 完成唤醒） | `n_in = 4`，控制量**零阶保持** |
| 中环/编码器 | 20 ms（50 Hz） | 20 ms 滑动窗口，**结束于 2 ms 前** |
| 外环 | 100 ms（10 Hz） | 100 ms |
| 总反馈延迟 | 10 ms = IMU 内部 2 + 传输/运算 6 + 编码器 2 | 环形缓冲真延迟，可选 `θ+θ̇T` 预测 |
| 编码器量化 | 1540 count/rev、76 mm 轮 → **0.1550 mm/count** | 速度按 50 Hz 量化，LSB = **7.75 mm/s** |
| PWM | 4199 计数，`compare = 0` 为短路刹车 | 限幅 + 死区（170）+ 摩擦前馈 |
| 传感器噪声 | IMU 实测噪声 | 0.15° / 1°/s rms（仅验证时开启） |

---

## 7. 仿真工况与判据

| 工况 | 场景 | 指标 |
|---|---|---|
| `stand` | 5° 初始倾角释放 → `POSITION(0,0)` | 最大倾角 / 最大位移 / 稳定时间 / 终值 |
| `step` | 0.30 m 位置阶跃 | 上升时间 / 超调 / 稳态误差 |
| `speed` | 0 → 0.40 m/s 速度阶跃 | 超调 / 上升 / 终速 |
| `push` | 1.5 N 水平力 0.15 s | 最大倾角 / 最大位移 / 回位 |
| `bias` | 0.4 N 恒定（≈3° 坡/风） | 站位偏移 |
| `yaw` | 45°/s 阶跃 | 上升 / 超调 / 稳态 |

**判据**：`ζ ∈ [0.6, 0.8]`、`PM ≥ 45°`、`PM@10 ms ≥ 40°`、弧线外轮 < 85% 空载、
要求 1 的 `|x| ≤ 15 cm`。

---


## 8. 结果

### 8.1 线性设计（解析值 + 精确延迟裕度）

解析设计（k=2、ζ=0.7、`Jp=0.0031`）：`Kp = 162.5 PWM/deg`、`Kd = 8.85 PWM/(deg/s)`。

```
闭环极点   -14.39 ± 14.69j  ->  ωn = 20.56 rad/s (3.27 Hz),  ζ = 0.700
增益穿越   30.19 rad/s (4.81 Hz)     相位裕度 58.7°
精确延迟裕度 33.9 ms                 PM@10 ms = 41.4°, PM@20 ms = 24.1°
PD 饱和角  4199/Kp ≈ 26°（Kp=162.5）/ 28°（Kp=149）
```

**最大稳定 Kp vs 延迟**（Kd 固定，二分求解精确相位条件）：

| 延迟 | 0 ms | 5 ms | 10 ms | 20 ms | 30 ms | 50 ms |
|---|---|---|---|---|---|---|
| Kp 上限 [PWM/deg] | ∞ | 1684 | 797 | 351 | 199 | 65 |

→ **10 ms 下 Kp 有 ~5 倍余量；20 ms 只剩 2.4 倍**。这就是"延迟必须实测"的量化依据。

**电池电压敏感性**（同一 `Jp`）：

| 电池 | 7.4 V (2S) | 8.4 V | 11.1 V (3S) | 12 V |
|---|---|---|---|---|
| 需要的 Kp | 263.5 | 232.2 | 175.7 | 162.5 |
| 需要的 Kd | 14.35 | 12.64 | 9.57 | 8.85 |

→ **若实际是 2S，当前 149/10.30 会偏软约 1.7 倍**；这是实车整定前必须确认的第一件事。

### 8.2 时域（当前固件参数 149 / 10.30 / 12 / 1.44）

```
释放 5°（POSITION 0,0）
  0 ms  : 峰值 5.00°, |x| 3.3 cm, x@3s 0.2 cm, 峰值 PWM 1109
  5 ms  : 峰值 5.00°, |x| 3.2 cm, x@3s 0.2 cm, 峰值 PWM 1102
  10 ms : 峰值 5.00°, |x| 3.1 cm, x@3s 0.1 cm, 峰值 PWM 1122
  10 ms + 预测 : 峰值 5.00°, |x| 3.1 cm
0.30 m/s 推一下
  0 ms  : |x| 峰值 8.3 cm, 4 s 回 -0.1 cm, 峰值 PWM 2417
  10 ms : |x| 峰值 7.9 cm, 4 s 回 -0.1 cm, 峰值 PWM 2510
0.30 m 位置阶跃（10 ms）
  kp_speed = 8   : 上升 0.82 s, 超调 12.4%, 误差 -0.42 cm
  kp_speed = 12  : 上升 0.79 s, 超调  2.4%, 误差 -0.05 cm   ← 当前值
  kp_speed = 19.5: 上升 0.38 s, 超调 711%  → 发散（当前 kp/kd 下已失稳）
0 → 0.40 m/s 速度阶跃
  kp_speed = 8   : 上升 0.74 s, 超调 3.3%, 终速 0.393 m/s
  kp_speed = 19.5: 上升 0.06 s, 超调 113%, 终速 0.618 m/s  → 不可用
```

**结论**：`kp_speed = 12` 是唯一兼顾"不过冲 + 不失稳"的点；把它加到 19.5 会
同时破坏位置环和速度环（说明速度环带宽已被内环相位限制住，这与 §8.1 的 PM 结论一致）。

### 8.3 摩擦 / 死区补偿（`[5b]`，0.30 m 阶跃）

| 对象/补偿 | 最大倾角 | pitch p-p（3~4 s） | 速度 p-p | 输出总变差 | 终值误差 |
|---|---|---|---|---|---|
| 死区模型，不补偿 | 9.29° | 1.95° | 59 mm/s | 3060 | +1.23 cm |
| 摩擦模型，不补偿 | 8.96° | 2.37° | 170 mm/s | 1908 | **-6.43 cm** |
| 摩擦 + 硬 sign | 9.00° | 1.02° | 37 mm/s | 5298 | +0.07 cm |
| **摩擦 + tanh(20)** | 8.99° | **0.91°** | **34 mm/s** | **4698** | **-0.05 cm** |

**结论**：不补偿时摩擦会让位置保持留下 **-6.43 cm** 静差；加 tanh 前馈后
降到 **-0.05 cm**，同时把残余摆动（p-p 0.91°）和抖动（总变差 4698）都压到最低。
`eps` 扫描（5/10/20/40）显示 eps 越大越平滑，但小指令补偿越弱，取 20 为折中。

### 8.4 坡 / 风：积分应该加在哪一环（`[10b]`、`[10c]`）

恒定 0.4 N（≈3° 地面）下，纯 P 位置环**没有平衡点**（不倾斜→加速；要倾斜→必须
有速度误差），车会一直爬。两种候选（**均为设计研究，固件尚未实现**）：

| 方案 | 坡上偏移 | 0.30 m 阶跃超调 | 结论 |
|---|---|---|---|
| 位置环加 I（ki=0.2/0.5） | 10.2 / 6.2 cm | **22% / 45%** | 否决 |
| 速度环加 I（ki=4，12 s） | **-0.7 cm** | 1.6% | 采用（待实现） |

当前固件（`kp_pos=1.44`，无积分）在 0.4 N 下偏移 **20 cm**（`kp_pos=0.7` 时是 37 cm），
—— `kp_pos` 只能按 1/kp 等比例缩小，**要消零必须加积分**，且应加在速度环。

### 8.5 要求 1：原地站立（`[10]`，3 s 窗口，限 15 cm）

| 激励 | 最大倾角 | 最大位移 | x@3 s | 回到 2 cm 内 | 判定 |
|---|---|---|---|---|---|
| 3° 释放 | 3.00° | 1.9 cm | 0.2 cm | 0.00 s | PASS |
| 5° 释放 | 5.00° | 3.1 cm | 0.1 cm | 0.59 s | PASS |
| 0.20 m/s 初速 | 4.42° | 5.4 cm | 0.0 cm | 0.98 s | PASS |
| 1.5 N / 0.15 s | 9.92° | **13.8 cm** | 0.1 cm | 1.40 s | PASS（余量 1.2 cm） |
| 3.0 N / 0.15 s | — | — | — | — | FAIL（物理上限，会倒） |

要求 1 是"**无外力**站立"，对应前两行（1.9 / 3.1 cm），余量极大；后两行只是
抗扰余量的参考。注意 1.5 N 的位移已接近 15 cm，现场若有人碰车会有风险。

---


### 8.6 自动调参结果

`autotune_pid` 在 5 工况加权代价上做坐标模式搜索（107 次评估、约 4 s）：

| | 调参前（原固件） | **调参后（现固件）** |
|---|---|---|
| `PID_ATT_KP_PITCH` | 162.5 | **149.0** |
| `PID_ATT_KD_PITCH` | 8.66 | **10.30** |
| `PID_SPEED_KP` | 12.0 | 12.0（已最优，未动） |
| `PID_POS_KP_DISTANCE` | 0.70 | **1.44** |
| 代价 J | 41.58 | **10.15**（-76%） |
| PM @10 ms | 40.7° | **47.4°** |
| 延迟裕度 | 33.9 ms | 34.8 ms |

带噪声验证（0.15° / 1°/s rms）：

| 工况 | 指标 | 调参前 | 调参后 |
|---|---|---|---|
| step | 上升时间 | 2.55 s | **0.75 s** |
| step | 超调 | 0.0 % | 3.2 % |
| step | 稳态误差 @4 s | -1.55 cm | **0.14 cm** |
| speed | 超调 | 6.8 % | **2.7 %** |
| speed | 终速 | 0.394 m/s | 0.404 m/s |
| stand | 最大位移 | 2.81 cm | 3.12 cm |
| push | 最大位移 | 12.84 cm | 13.74 cm |
| push | 回位 @4 s | 1.20 cm | **0.10 cm** |

**取舍**：调参把"快"放在"稳"之前——阶跃时峰值倾角由 4.2° 升到 **8.0°**
（更用力加速），超调 3.2%（0.30 m → 0.9 cm）。若实车觉得过冲，把
`kp_pos` 降到 1.2（上升 1.06 s、超调 1.1%），或把 `W.step` 里的超调权重调大重跑。

**四个优化后端对比**（同一代价、同一起点 J=41.58、预算 300）：

| method | 工具箱 | J | 评估 | 用时 | kp / kd / kps / kpp |
|---|---|---|---|---|---|
| `pattern`（默认） | 无 | **10.15** | 107 | 3.9 s | 149.0 / 10.30 / 12.0 / 1.44 |
| `patternsearch` | Global Optimization | 10.30 | 300 | 7.8 s | 162.5 / 9.24 / 12.0 / 1.40 |
| `fminsearch` | 无（基座） | 29.10 | 300 | 11.4 s | 235.4 / 9.12 / 11.4 / 0.71 |
| `fmincon`（sqp） | Optimization | 37.73 | 166 | 7.9 s | 未离开起点 |

`fmincon` 失败的原因：代价函数**分段不可导**（倾倒判 100、单工况封顶 25、
PWM 饱和加罚、250/50/10 Hz 采样切换），有限差分梯度无意义 → 立即在起点满足
一阶条件退出。`fminsearch` 在 4 维 log 空间探测效率低。两种 derivative-free
结果（10.15 / 10.30）都是合法解，差 1.5%，可任选。

---

## 9. 自动调参系统

### 9.1 结构

```
balance_model('api')  ->  struct('params', @params, 'derive', @derive,
                                 'sim', @sim_balance, 'delay_margin', @delay_margin, ...)
autotune_pid          ->  复用同一套 params/derive/sim（不复制仿真代码）
```

### 9.2 代价函数

```
J = 1.2*J_stand + 1.5*J_step + 1.0*J_speed + 1.2*J_push + 0.5*J_bias
```

每个 `J_case` 是"指标/目标值"的平方和（例如 step 用超调/4%、上升/0.7 s、
稳态误差/0.8 cm），并加惩罚：倾倒 +100、PWM > 97% 饱和 +3、单工况封顶 25。
调参时**关闭传感器噪声**保证代价可重复；最终对比再开噪声。

### 9.3 用法

```matlab
autotune_pid                      % 默认：pattern, 200 evals / 420 s
autotune_pid('patternsearch')     % 用 Global Optimization Toolbox
autotune_pid('fmincon', 400, 600)
autotune_pid(400, 600)            % 兼容旧用法（budget, max_time）
```

输出 `matlab/logs/tuned_pid.txt`（可直接抄进 `robot_config.h`）：

```
% autotune_pid (pattern): balance_model, 12 V, Td=10 ms, 107 evals, J 41.5835 -> 10.1479
#define PID_ATT_KP_PITCH          149.01f
#define PID_ATT_KD_PITCH          10.30f
#define PID_SPEED_KP              12.00f
#define PID_POS_KP_DISTANCE       1.44f
% baseline: 162.50 / 8.66 / 12.00 / 0.70
```

### 9.4 改偏好

只改 `autotune_pid.m` 开头的权重，例如"不许超调"：

```matlab
W.step  = 2.5;    % 提高 step 权重
W.push  = 1.0;
```
并把 `case_cost` 里 step 的超调目标从 `overshoot/4` 改成 `overshoot/2`。

---

## 10. 可视化产物

| 脚本 | 输出（`matlab/logs/`，git 忽略） | 内容 |
|---|---|---|
| `vis_balance` | `balance_strobe.png` / `balance_anim.gif` | 位置阶跃频闪、速度阶跃动画 |
| `vis_stand` | `stand_still.png` | 要求 1：激励—回站立，含 ±15 cm 带 |
| `vis_attitude` | `attitude_dist.png` | 姿态环单独 vs HOLD 的扰动对比 |
| `vis_tuned` | `tuned_strobe.png` / `tuned_compare.png` / `tuned_anim.gif` | 最优参数 vs 固件原值的并排对比 + 动画 |

`vis_tuned` **读取 `tuned_pid.txt`**（不写死参数），所以改完调参结果重跑即可。

---

## 11. 局限与假设（重要，实车前必读）

| # | 假设 | 影响 | 怎么消除 |
|---|---|---|---|
| 1 | **符号链正确**（pitch / pitch_rate / 电机 / 编码器 / yaw rate 全为 +1） | 任一项反 → 一松手就倒 | 手扶 5 项验证（见下） |
| 2 | 电池 **12 V** | 2S（7.4 V）时需要 Kp≈250 / Kd≈14.3，现参数偏软 1.7× | 看电池标签 |
| 3 | 总延迟 **10 ms** | 实际 20 ms 时 PM 只有 24°，需实测 | 用陀螺/融合角互相关测 |
| 4 | 死区 **170** | 实际 300 → 补偿不足、位置抖 | 架空测"刚起转的 PWM" |
| 5 | 质量/惯量（0.62+0.10 kg、`Jp=0.0031`） | 只影响 `ω0`，进而影响 Kd 的合理值 | 称重 + 自由倒下测时间 |
| 6 | 无打滑、无偏航-平衡耦合、地面水平 | 实车大角度转弯会有耦合 | 需要更高保真就得 3D/多体模型 |
| 7 | 不含巡线/路径状态机 | 要求 2~5 的时序验证不在此模型内 | 见 `CONTROL_TASK.md` |

**符号链 5 项验证（实车 30 秒，不用改代码）**：

| # | 动作 | 期望 | 不符则改 |
|---|---|---|---|
| 1 | 手压车头 10° | `current_pitch` 单调变化 | `IMU_PITCH_SIGN` |
| 2 | 同 1，看角速度 | `pitch_rate` 与 pitch **同号**，停住归 0 | `IMU_PITCH_RATE_SIGN` / `_AXIS` |
| 3 | `ATTITUDE(0,0)` 手扶下压 | **两轮向前**滚 | 电机线 / `SETPWM` |
| 4 | 手推车前进 | `current_speed > 0`、距离增大 | `ENCODER_*_SIGN` |
| 5 | 手转车体逆时针 | `current_yaw_rate > 0` | `IMU_YAW_RATE_SIGN` |

---

## 12. 复现清单

```powershell
# 1) 主模型（打印 12 节表格 + 2 张图，约 30 s）
matlab -batch "cd('matlab'); balance_model"

# 2) 自动调参（约 4 s，写 logs/tuned_pid.txt）
matlab -batch "cd('matlab'); autotune_pid"

# 3) 用最优参数重跑并出对比图/动画
matlab -batch "cd('matlab'); vis_tuned"

# 4) 其余可视化
matlab -batch "cd('matlab'); vis_balance; vis_stand; vis_attitude"
```

前置条件：MATLAB R2019b+（用到 `batchStartupOptionUsed`、`exportgraphics`）；
`pattern` 后端**不需要任何工具箱**。

---

*文档随代码同步维护：改 `robot_config.h` 的增益后，重跑 `balance_model` 即可得到本文对应表格的新数值。*

