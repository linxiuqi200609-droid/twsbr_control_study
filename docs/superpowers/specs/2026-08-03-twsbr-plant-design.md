# 两轮自平衡车 MATLAB/Simulink 模型设计

## 1. 目标

在 `D:\Research\srtp` 中建立两轮自平衡车纵向等效倒立摆模型，同时交付 MATLAB 数值模型与由 MATLAB 脚本自动生成的 Simulink 模型。两种实现共用同一组物理参数和动力学方程，为后续基础 PID 与串级 PID 控制器提供统一被控对象。

本阶段只完成平衡车被控对象、线性化、开环仿真和一致性验证，不实现 PID、串级 PID、转向、偏航或侧倾控制。

## 2. 建模范围与约定

模型使用纵向四状态等效轮式倒立摆：

\[
\mathbf{x}=\begin{bmatrix}x&\dot{x}&\theta&\dot{\theta}\end{bmatrix}^{\mathrm T}
\]

- `x`：轮轴水平位移，单位 m。
- `x_dot`：轮轴水平速度，单位 m/s。
- `theta`：车体相对竖直向上的倾角，单位 rad；`theta = 0` 表示直立平衡。
- `theta_dot`：车体角速度，单位 rad/s。
- `u`：归一化电机控制命令；被控对象通过 `motor_force_gain` 将其换算为水平等效驱动力。
- `force_disturbance`：施加在轮轴方向的外力扰动，单位 N。
- `torque_disturbance`：施加在车体上的外力矩扰动，单位 N·m。

正倾角表示车体质心向正 `x` 方向偏移。正控制命令产生正方向水平驱动力；根据动力学耦合关系，它在直立点附近产生负方向角加速度。

## 3. 默认物理参数

第一版沿用样板工程的参数，所有值集中存放，后续实车测量后可以直接替换：

| 参数 | MATLAB 字段 | 默认值 | 单位 | 含义 |
|---|---|---:|---|---|
| 车体质量 | `body_mass` | 1.20 | kg | 车身及质心以上等效质量 |
| 轮组等效质量 | `wheel_mass_equiv` | 0.30 | kg | 左右轮及平移部分的等效质量 |
| 质心高度 | `com_length` | 0.18 | m | 轮轴到车体质心距离 |
| 车体转动惯量 | `body_inertia` | 0.025 | kg·m² | 绕轮轴的等效转动惯量 |
| 轮半径 | `wheel_radius` | 0.05 | m | 为后续电机模型和实车换算预留 |
| 平移黏性阻尼 | `viscous_damping` | 0.05 | N·s/m | 轮轴方向等效阻尼 |
| 重力加速度 | `gravity` | 9.81 | m/s² | 重力常数 |
| 电机力增益 | `motor_force_gain` | 12.0 | N | `u = 1` 时的等效水平驱动力 |
| 控制命令上限 | `u_max` | 1.0 | 1 | 后续执行器统一限幅 |
| 倾倒阈值 | `theta_fail_deg` | 30.0 | deg | 实验成功/失败判定阈值 |
| 位移边界 | `x_limit` | 5.0 | m | 实验越界判定阈值 |

参数函数必须检查质量、长度、惯量、重力和电机增益为正数，阻尼不小于零，避免无效参数进入模型。

## 4. 非线性动力学

定义：

\[
a=M+m,\qquad d=I+ml^2,\qquad c=ml\cos\theta
\]

其中 `m` 为车体质量，`M` 为轮组等效质量，`l` 为质心高度，`I` 为车体转动惯量。等效驱动力和方程右端为：

\[
F=k_u u+F_d
\]

\[
r_x=F-b\dot{x}+ml\dot{\theta}^2\sin\theta
\]

\[
r_\theta=mgl\sin\theta+\tau_d
\]

令质量矩阵行列式：

\[
\Delta=ad-c^2
\]

则加速度为：

\[
\ddot{x}=\frac{d r_x-c r_\theta}{\Delta},\qquad
\ddot{\theta}=\frac{-c r_x+a r_\theta}{\Delta}
\]

状态导数输出顺序固定为：

\[
\dot{\mathbf{x}}=\begin{bmatrix}\dot{x}&\ddot{x}&\dot{\theta}&\ddot{\theta}\end{bmatrix}^{\mathrm T}
\]

被控对象内部不对 `u` 限幅；执行器限幅由仿真入口或后续控制系统统一实施，从而保证不同控制器使用完全相同的限幅规则。

## 5. 线性模型

在直立平衡点 `x = 0`、`x_dot = 0`、`theta = 0`、`theta_dot = 0`、`u = 0` 附近解析线性化，得到：

\[
\dot{\mathbf{x}}=A\mathbf{x}+Bu
\]

MATLAB 函数返回 `A`、`B`、`C = eye(4)` 和 `D = zeros(4,1)`。另写中心差分数值线性化函数，对非线性状态方程计算雅可比矩阵，用于验证解析矩阵。中心差分默认步长为 `1e-6`。

线性模型必须验证可控性矩阵 `ctrb(A,B)` 的秩为 4。若用户缺少 Control System Toolbox，测试使用手工拼接 `[B, A*B, A^2*B, A^3*B]`，不依赖 `ctrb`。

## 6. MATLAB 文件结构

```text
D:\Research\srtp\
├─ twsbr_params.m
├─ twsbr_dynamics.m
├─ twsbr_linear_model.m
├─ twsbr_numerical_linearize.m
├─ simulate_open_loop.m
├─ build_twsbr_simulink.m
├─ run_project.m
├─ README.md
├─ tests\
│  └─ test_twsbr_model.m
├─ results\
└─ twsbr_plant.slx
```

各文件职责：

- `twsbr_params.m`：返回默认参数结构并进行参数检查。
- `twsbr_dynamics.m`：计算连续非线性状态导数，支持控制输入、水平外力和车体扰动力矩。
- `twsbr_linear_model.m`：返回直立平衡点附近的解析状态空间矩阵。
- `twsbr_numerical_linearize.m`：以中心差分计算 `A`、`B`，验证解析推导。
- `simulate_open_loop.m`：使用 MATLAB ODE 求解器执行零状态平衡与初始倾角倾倒实验，输出结果结构和曲线。
- `build_twsbr_simulink.m`：使用 Simulink API 自动创建并保存 `twsbr_plant.slx`，避免依赖手工拖拽。
- `run_project.m`：一键运行测试、生成 Simulink 模型、执行 MATLAB 和 Simulink 验证并保存结果。
- `test_twsbr_model.m`：MATLAB Unit Test 自动化测试。
- `README.md`：说明模型假设、文件用途、运行方式和后续 PID 接口。

## 7. Simulink 结构

`twsbr_plant.slx` 由脚本生成，顶层保持接口清晰：

```text
u -------------------------┐
force_disturbance ---------┼--> Nonlinear Plant --> state [x; x_dot; theta; theta_dot]
torque_disturbance --------┘
```

`Nonlinear Plant` 子系统包含：

- 三个输入端口：`u`、`force_disturbance`、`torque_disturbance`；
- 一个四维连续积分器，初值由工作区变量 `x0` 提供；
- 一个 MATLAB Function 模块，执行与 `twsbr_dynamics.m` 相同的动力学方程；
- 一个四维状态输出端口；
- 状态拆分与记录模块，记录 `x`、`x_dot`、`theta`、`theta_dot` 和 `u`。

顶层测试工况使用常值零控制输入和可配置扰动，仿真采用连续求解器。脚本设置模型参数和初值，不依赖 Base Workspace 中未声明的变量。

## 8. 开环实验

MATLAB 与 Simulink 都执行两类基础工况：

1. 平衡点实验：初始状态全零、控制和扰动全零，整个仿真中状态应保持为零。
2. 倾倒实验：初始倾角为 `3 deg`，其他状态为零，控制和扰动全零；倾角绝对值应随时间增长，证明开环直立点不稳定。

默认仿真时间为 2 s。结果保存到 `results`，至少包括 MAT 数据和倾角、角速度、位移、速度四联图。绘图时倾角和角速度分别转换为 deg 和 deg/s，内部计算仍使用 SI 单位。

## 9. 测试与验收

自动测试覆盖：

- 默认参数完整且物理上有效。
- 零状态、零输入、零扰动的状态导数为零，绝对误差不超过 `1e-12`。
- 正初始倾角在零输入下产生正角加速度。
- 正控制输入在直立平衡点产生负角加速度。
- 解析线性模型与中心差分线性化一致，绝对误差不超过 `1e-7`、相对误差不超过 `1e-6`。
- 线性系统可控性矩阵秩为 4。
- 非法参数会触发明确错误。
- Simulink 模型可以成功更新和运行。
- MATLAB 与 Simulink 在相同初始状态和零输入下的状态轨迹最大绝对差不超过 `1e-4`。

最终交付成立的条件是：所有 MATLAB 测试通过；`twsbr_plant.slx` 实际生成并能运行；`run_project.m` 从干净 MATLAB 会话可一键复现数据与图；结果目录包含开环实验数据和图像。

## 10. 后续控制器接口

后续 PID 和串级 PID 统一读取状态向量：姿态内环使用 `theta`、`theta_dot`，速度或位置外环使用 `x_dot` 或 `x`。控制器只输出归一化命令 `u0`，公共执行器模块执行 `u = min(max(u0,-u_max),u_max)` 后输入被控对象。模型参数、扰动端口和评价阈值保持不变，保证各控制算法的对比条件一致。

