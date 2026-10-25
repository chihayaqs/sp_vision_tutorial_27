# 27算法组导航方向招新大作业

## 概要

本项目实现迷宫场景的一次目标自主导航：机器人从固定起点 `(0.900, 0.900)` 出发，经 A* 规划与碰撞约束平滑、速度参考生成和约束 MPC 跟踪，到达固定终点 `(14.100, 14.100)` 并停稳。

## 环境依赖

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-humble-pluginlib ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-std-srvs \
  ros-humble-rviz2 ros-humble-behaviortree-cpp-v3 \
  qtbase5-dev libopencv-dev libyaml-cpp-dev libeigen3-dev \
  python3-opencv python3-pygame python3-numpy python3-pil \
  gnome-terminal
```

## 编译运行

在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
bash bash/sim_nav_all_start.sh
```

脚本会在新终端启动静态 TF、导航节点、RViz 和机器人仿真。进入 RViz 的 **2D Goal Pose** 模式，在固定终点 `(14.100, 14.100)` 点击一次。

## 详细报告

### 一、导航参数配置

首先需要统一地图、路径、里程计、控制指令及坐标系，形成完整的导航闭环。相关配置集中在 `src/sp_nav_bringup/config/nav_params.yaml`。

#### 1.1 地图与通行安全

地图节点 `esdf_map_publisher` 读取 `sp_nav_bringup/map/maze_map.yaml`，根据障碍物距离生成 `/global_costmap`。规划时不仅要考虑路径是否穿墙，还要为具有实际尺寸的机器人留下通行余量。

- `robot_radius = 0.25 m`，`margin = 0.15 m`：二者之和为 `0.40 m`。距离障碍物不足这一数值的位置被赋予致命代价，覆盖边长 `0.50 m` 的方形底盘约 `0.354 m` 的角点半径并留出约 `0.046 m` 余量。
- `d_safe = 0.70 m`，`w = 12.0`：在禁行区外、距离障碍物不足 `0.70 m` 的区域继续施加软代价，使规划结果在可行时更倾向于走廊中间，而不是紧贴墙面。
- `lethal_cost = 100`，`cost_weight = 3.0`：A* 将代价达到 `100` 的栅格视为不可通行，并在路程代价中引入栅格代价权重，在路线长度与离墙距离之间进行权衡。
安全余量内的区域直接禁止通过，其余邻墙区域则允许搜索，但会增加路径代价。

#### 1.2 话题、插件与坐标系

| 配置项 | 当前值 | 作用 |
| --- | --- | --- |
| `planner_server.costmap_topic` | `/global_costmap` | A* 搜索所用代价地图 |
| `planner_server.path_topic` | `/global_path` | 发布规划路径，供控制器订阅 |
| `controller_server.odom_topic` | `/Odometry` | 提供机器人实测速度 |
| `controller_server.cmd_vel_topic` | `/sentry/cmd_vel` | 向仿真底盘发布运动指令 |
| `controller_server.control_frequency` | `50.0 Hz` | 每`0.02s`更新一次控制指令 |
| `planner_server.plugin_type` | `sp_global_planner/AStarPlanner` | 加载全局规划插件 |
| `controller_server.plugin_type` | `MpcController` | 加载实现的MPC插件 |

全局目标和路径使用`map`坐标系，里程计的参考系为`lidar_odom`，底盘指令则采用`base_link`。`controller_server.cpp` 通过TF获取机器人在地图中的位姿，并将实测速度变换到地图系；MPC在统一坐标系内计算，输出时再将平移指令旋转回 `base_link`，防止机器人姿态变化造成运动方向错误。

MPC插件还通过 `sp_controller_server/plugin_description.xml`注册，并在对应的 `CMakeLists.txt`、`package.xml` 中配置插件库与 Eigen3 依赖，确保 `pluginlib`能实际加载。

#### 1.3 一次目标的导航流程

`click_nav.xml` 收到 `/goal_pose` 后发起一次导航 Action。`default_nav_with_fallback.xml` 以 20 Hz 更新实时位姿，并每秒重新规划剩余路径，使控制器能从当前偏差处继续跟踪；规划失败时先停控再重试。到达判定满足后关闭控制输出并停车。

### 二、A* 路径规划与平滑

#### 2.1 保证搜索路径能够通行

全局规划沿用原工程的八邻域 A*，以移动距离和代价地图中的栅格代价共同计算搜索成本。由于对角移动可能从障碍物墙角间穿过，在扩展斜向相邻栅格时，额外检查对应的两个正交相邻栅格；只要其中一个不可通行，就拒绝这次斜向移动。
搜索结束后，将栅格路径首尾替换为输入的实际起点、终点坐标，使规划终点与后续的到达判断一致。

#### 2.2 平滑折角并检查碰撞

A* 输出的栅格路径会频繁改变方向，不适合直接作为连续控制器的跟踪参考。`sp_global_planner/plugins/astar_planner.cpp` 中的 `smoothPath()` 对中间点迭代调整：一项将平滑点拉向原始路径，避免偏移过远；另一项根据前后相邻点减小局部折角。起点和终点保持固定。

- `smoothing_data_weight = 0.015`：保留对原路径的约束，抑制过大的形变。
- `smoothing_smooth_weight = 0.30`：增强相邻路径点之间的平滑作用。
- `smoothing_iterations = 150`：重复局部调整，让折角逐步减小。
每次更新前，`isSegmentFree()` 都会检查新点与前后相邻点连接的线段，采样间隔为地图分辨率的四分之一，即约`0.0125 m`，只要线段经过不可通行栅格，就拒绝这次调整，这样平滑受到障碍物约束，不会为了减小折角直接穿墙。
最后通过 `path_spacing = 0.05 m` 按弧长重新采样，保留精确终点，并以相邻点连线方向设置路径朝向。均匀的点距也便于下游计算弧长、曲率和速度参考。

### 三、MPC 跟踪控制器实现
MPC 的核心位于 `src/sp_controller_server/plugins/`。实现分为路径参考、底盘预测模型、带约束优化和实时输出四个环节，并通过 `MpcController` 接入原工程的 `ControllerPlugin` 接口。

#### 3.1 通过三个插件接口串接处理流程

- `configure()`：读取控制周期、动力学参数、预测步数、速度与加速度限制、目标函数权重，预先建立预测矩阵及二次规划求解器。
- `setPlan(path)`：接收 `/global_path`，交给 `TrajectoryReference` 转换为包含路径位置和目标速度的参考数据。
- `computeVelocityCommands(pose, velocity)`：根据当前位姿、实测速度和未来参考轨迹进行滚动优化，返回 `TwistStamped`。控制器服务器再将其中的速度指令发布到 `/sentry/cmd_vel`。
三个接口分别负责初始化、更新参考、求解。

#### 3.2 将几何路径转换为速度参考

`trajectory_reference.hpp` 中的 `TrajectoryReference` 首先计算路径累计弧长，然后根据直线段、转弯和终点停车的不同要求生成速度上界，将未来各时刻应该以什么速度到达传递给MPC。
- `max_speed = 1.6 m/s`：参考轨迹的巡航速度上限。
- `max_acceleration = 1.0 m/s²`：限制沿路径的加速和减速，使相邻参考速度不会突变。
- `lateral_acceleration = 0.8 m/s²`：约束转弯时的横向加速度。曲率越大，允许速度越低，使用关系 $v \leq \sqrt{a_{lat}/|\kappa|}$ 计算弯道速度上界。

曲率通过路径当前位置前后约`0.18 m`的采样点估算。将终点速度设为`0`后，沿路径反向应用 $v_i^2 \leq v_{i+1}^2 + 2a\Delta s$，限制最晚开始减速的时刻；再正向限制加速能力，使全程速度变化满足同一组约束。

每次控制求解前，`preview()` 将机器人当前位置投影到当前路径最近的线段；如果机器人偏离路径，参考仍指向当前位置附近的路径，而不会保持在墙另一侧的旧进度。随后按实测切向速度和加减速限制向前推进，返回未来各步的 `(x, y, vx, vy)`。

#### 3.3 建立考虑延迟和惯性的预测模型

`predictive_model.hpp` 中的 `DelayedVelocityModel` 分别为地图系的 x、y 方向建立离散模型，每个方向保存位置、实际速度、滤波速度以及尚未生效的历史指令队列。

- `control_frequency = 50 Hz`，对应 `dt = 0.02s`：预测模型与控制循环使用相同时间步长。
- `command_delay = 0.1667 s`：指令按 `ceil(delay / dt)` 离散为`9`级延迟队列，历史命令依次向前传递。
- `filter_time_constant = 0.08 s`：用一阶环节近似指令进入速度系统前的滤波过程。
- `velocity_time_constant = 0.2923 s`：用另一一阶环节描述实际速度跟随命令的惯性。
- `observer_gain = 1.0`：每轮根据实测速度与模型预测速度的差修正滤波状态，同时保留延迟队列里的旧指令。
离散状态采用 $x_{k+1}=Ax_k+Bu_k$ 表示。`buildPrediction()` 将其展开为多个控制步，分别预先计算当前状态和未来新指令对位置、速度、加速度的影响矩阵。这样每轮只需更新实测状态及参考值。

#### 3.4 设置预测范围与优化目标

MPC同时考虑路径位置误差、目标速度误差和输入大小，既要贴近轨迹，也要避免过大的速度命令。预测末端另设附加权重，使优化器在本轮计算中提前考虑后续转弯和减速。
- `prediction_steps = 80`：以 `0.02s`为步长预测未来 `1.6s`，为延迟响应和提前制动提供预测范围。
- `move_steps = 4`：每4个预测步共用一组控制指令。每个方向只有20个待优化值，x、y 两个方向合计 `40` 个变量；状态预测仍保持 `0.02 s`的分辨率。
- `position_weight = 30.0`，`velocity_weight = 2.0`：以位置跟踪为主，同时要求运动速度与参考速度一致。
- `input_weight = 0.12`：惩罚过大的指令，避免只为缩小位置误差而给出过于激进的控制。
- `terminal_position_weight = 12.0`，`terminal_velocity_weight = 2.0`：进一步强调预测末端的位置和速度状态。

#### 3.5 加入运动约束并求解二次规划

最小化跟踪误差得到的指令不一定符合底盘能力，因此还需要约束指令和预测运动的合成大小。

- `command_limit = 2.0 m/s`：限制输出的二维平移指令。
- `actual_speed_limit = 2.0 m/s`：限制模型预测的实际合速度。
- `actual_acceleration_limit = 2.0 m/s²`：限制模型预测的合加速度。
代码通过内接正八边形对速度、加速度的圆形模长约束进行保守线性近似，写成 `lower ≤ A·u ≤ upper`。同时，考虑到新指令在延迟结束前尚不能影响底盘，实际速度与加速度约束从新指令能够生效的预测步开始施加。
`constrained_qp.hpp` 中的 `ConstrainedQp` 先检查无约束最优解是否已经满足限制；若不满足，则用 ADMM迭代求解。`qp_rho = 0.03`是求解器的惩罚参数，`qp_iterations = 800`是迭代上限。固定矩阵的分解在初始化阶段缓存，控制循环中复用，并以上一轮结果作为本轮的初值，减少重复计算。

#### 3.6 按实测状态滚动更新控制指令

`computeVelocityCommands()` 每轮根据 TF 位姿和 `/Odometry` 更新地图系状态，生成未来参考，求解下一段控制序列，但只执行第一组`(u_x, u_y)`。下一周期重新读取反馈，再计算新的控制序列，持续修正预测和实际运动之间的偏差。
MPC内部的 x、y 指令位于 `map`坐标系；输出前利用当前偏航角将其旋转为 `base_link` 坐标系，再作为 `TwistStamped` 返回。

### 四、到达判定与 RViz 展示

#### 4.1 同时检查终点误差和实际速度

为了防止机器人高速经过目标附近时就被判定完成，`sp_nav_bt/plugins/nav_state_track.cpp` 从 TF 获取地图系下的当前位置，并订阅 `/Odometry` 获取实际平面速度，要求同时满足：
- `goal_tolerance = 0.04 m`：当前位置与目标位置的平面距离小于 `0.04 m`。
- `goal_speed_tolerance = 0.06 m/s`：实际平面合速度小于 `0.06 m/s`。
两项同时满足时，`NavStateTrack` 才返回 `SUCCESS`。这一判定与速度参考末端设为零相互配合，主要依靠零速参考与MPC反馈减速让机器人实现到达并基本停稳。

#### 4.2 RViz 展示

`src/sp_nav_bringup/rviz/rviz.rviz` 使用 `map` 作为固定坐标系，显示 `/global_costmap`、`/global_path` 及 TF 中的机器人位姿；通过 **2D Goal Pose** 向 `/goal_pose` 发布指定终点。由此可直接观察规划路径是否避障、机器人是否沿路运动、是否在目标附近停稳。
