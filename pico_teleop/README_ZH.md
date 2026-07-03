# PICO / OpenXR 双臂遥操作

该目录用于使用 PICO4 Ultra Enterprise 或其他 OpenXR 头显/手柄控制两台 UFACTORY xArm。

输入端支持两种 backend：

- `udp`：复用 GSPlayground `XRStreamer` Unity 工程的数据协议。Unity 通过 OpenXR 读取 HMD、左手柄、右手柄的 6D 位姿和按键状态，并通过 UDP 发送到 Python。
- `xrobotoolkit`：通过 XRoboToolkit PICO App + Ubuntu PC Service 读取 PICO 输入，适合只用一台 Ubuntu 电脑同时接收 PICO 和控制 xArm 的部署。

## 数据流

1. PICO 输入 backend 输出统一的 `XRFrame`，顺序为 HMD、左手柄、右手柄。
2. 坐标转换为 `x=front, y=left, z=up`，位置单位为米，四元数格式为 `xyzw`。
3. `uf_robot_pico_teleop_dual.py` 将左右手柄相对校准时刻的位姿变化映射到左右 xArm 末端。
4. 通过 `UFRobot.send_action()` 发送 `[x, y, z, rx, ry, rz, gripper_norm]`。

## 运行

只跑仿真/可视化模式：

```bash
pip install -r requirements-sim.txt
```

真实机械臂模式：

```bash
pip install -r requirements.txt
```

建议使用 Python 3.10 环境。如果只跑 `--sim true --sim-viewer console`，实际只需要 `numpy` 和 `pyyaml`；如果使用默认 `--sim-viewer plot`，还需要 matplotlib；真实机械臂模式需要 xArm Python SDK。

## 网络拓扑

一台 Ubuntu 电脑可以同时接 PICO 和两台 xArm，但需要让它们网络互通。推荐使用一个独立路由器/AP：

```text
PICO 4 Ultra Enterprise -- 5 GHz Wi-Fi -- 路由器/AP
Ubuntu PC              -- 有线网口  -- 路由器/AP/交换机
左 xArm 控制箱          -- 有线网口  -- 路由器/AP/交换机
右 xArm 控制箱          -- 有线网口  -- 路由器/AP/交换机
```

路由器不需要联网，只需要提供同一个局域网。也可以用 Ubuntu 热点或多网卡直连，但 XRoboToolkit PC Service 的发现、PICO 到 PC 的连接、xArm 静态 IP 会更难排查；首次 bring-up 建议用路由器/AP + 交换机。

如果没有路由器，也可以让 Ubuntu 开 Wi-Fi 热点给 PICO：

```bash
nmcli dev status
nmcli device wifi hotspot ifname <wifi_iface> ssid xarm-pico password "your-password"
```

其中 `<wifi_iface>` 通常是 `wlan0`、`wlp2s0`、`wlo1` 之类。热点启动后，PICO 连接 `xarm-pico`；xArm 控制箱仍建议通过 Ubuntu 的有线网口或一个小交换机连接。此时 Ubuntu 需要同时有：

- Wi-Fi 热点网段：给 PICO / XRoboToolkit 使用。
- 有线网段：给两台 xArm 使用。

如果 XRoboToolkit App 自动发现不到 PC Service，可以在 PICO App 里手动输入 Ubuntu 热点侧 IP。常见热点 IP 是 `10.42.0.1`，以 `ip addr` 实际显示为准。

### GSPlayground / UDP backend

该模式适合复用现有 `PICO -> Windows Business Streaming/Unity XRStreamer -> UDP -> Ubuntu` 链路：

```bash
cd ufactory_teleop/pico_teleop
python uf_robot_pico_teleop_dual.py --config config/xarm6_pico_teleop_dual.yaml --sim true --sim-viewer console
```

### XRoboToolkit backend

该模式用于一台 Ubuntu 电脑完成 PICO 输入和 xArm 控制：

1. 在 PICO 头显上安装并启动 XRoboToolkit PICO App。
2. 在 Ubuntu 上安装并启动 XRoboToolkit PC Service。
3. 安装 `xrobotoolkit_sdk` Python binding，建议使用 conda 环境，并参考 XRoboToolkit-PC-Service-Pybind：
   ```bash
   git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git
   cd XRoboToolkit-PC-Service-Pybind
   bash setup_ubuntu.sh
   ```
4. 如果使用 USB-C 线直连 PICO，而不是 Wi-Fi/路由器，在 Ubuntu 上建立 ADB reverse：
   ```bash
   adb devices
   adb reverse --remove-all
   adb reverse tcp:63901 tcp:63901
   adb reverse --list
   ```
   然后在 PICO 的 XRoboToolkit App 里把 PC Service IP 填为 `127.0.0.1`。
5. 先检查 XRoboToolkit 原始输入流：
   ```bash
   cd ufactory_teleop/pico_teleop
   python inspect_xrobotoolkit_stream.py --hz 5
   ```
   该脚本会同时打印 SDK 原始 OpenXR 位置、转换后的机器人坐标系位置、相对启动时刻的位移、grip/trigger/A/B/X/Y 按键。
6. 再跑 sim：
   ```bash
   cd ufactory_teleop/pico_teleop
   python uf_robot_pico_teleop_dual.py --config config/xarm7_xrobotoolkit_teleop_dual.yaml --sim true --sim-viewer console
   ```

`--sim` 是必填安全参数；`--sim true` 表示仿真模式，`--sim false` 表示真实机械臂模式。仿真模式默认打开 matplotlib 3D 视图，显示左右目标 TCP、轨迹和末端坐标轴。如果当前环境没有图形界面或没有安装 matplotlib，用 `--sim-viewer console`。console 模式会打印每只手柄在机器人坐标系下的位置、grip/trigger 状态、以及生成的目标 TCP 位姿。

确认方向、左右手柄、grip/trigger 和重校准按钮符合预期后，再分阶段连接真实机械臂。先确认
`config/xarm7_xrobotoolkit_teleop_dual.yaml` 里的 `L.RobotConfig.robot_ip` 和
`R.RobotConfig.robot_ip` 分别对应两台控制箱，然后运行无运动连接检查：

```bash
cd pico_teleop
python check_xarm_connection.py --config config/xarm7_xrobotoolkit_teleop_dual.yaml
```

这个脚本只打开 xArm SDK 连接并读取 state、mode、错误/警告码、TCP 位姿和关节角；不会
`motion_enable`、不会初始化夹爪、也不会发送运动指令。

第一次真实 teleop 建议只测试单臂。另一台机械臂可以断开网线或不通电。以左臂为例，先只检查左臂：

```bash
python check_xarm_connection.py --config config/xarm7_xrobotoolkit_teleop_dual.yaml --arm left
```

然后空载、急停可触达，并跳过启动移动和夹爪初始化：

```bash
python uf_robot_pico_teleop_dual.py \
  --config config/xarm7_xrobotoolkit_teleop_dual.yaml \
  --sim false \
  --arm left \
  --no-start-move \
  --no-gripper-init
```

当前 xArm7 XRoboToolkit 配置把 xArm Studio 的 `Initial Position` 记录为全零 `start_joints`，对应 TCP 大约是 `[205.05, -0.055, 119.202, -3.1415, 0.0001, 0.0]`。如果只想测试这个关节空间初始位姿移动，不启动 XR teleop，用：

```bash
python uf_robot_pico_teleop_dual.py \
  --config config/xarm7_xrobotoolkit_teleop_dual.yaml \
  --sim false \
  --arm left \
  --initial-position-only \
  --start-joint-speed 0.25 \
  --start-joint-acc 0.5 \
  --no-gripper-init
```

如果希望进入 teleop 前先把选中的真实机械臂移动到这个关节空间初始位姿，用：

```bash
python uf_robot_pico_teleop_dual.py \
  --config config/xarm7_xrobotoolkit_teleop_dual.yaml \
  --sim false \
  --arm left \
  --start-at-initial \
  --no-gripper-init
```

如果希望进入 teleop 前先把选中的真实机械臂柔和地移动到配置里的 Cartesian `robot_base_pose`，用：

```bash
python uf_robot_pico_teleop_dual.py \
  --config config/xarm7_xrobotoolkit_teleop_dual.yaml \
  --sim false \
  --arm left \
  --start-at-base \
  --no-gripper-init
```

`--start-at-base` 会跳过旧的 `start_joints` 启动移动，只把 `--arm` 选中的机械臂用 `reset_speed` / `reset_acc` 移动到 `TeleoperatorConfig.robot_base_pose`，然后再建立 teleop 参考。

如果只想测试回到原始位姿的速度/加速度，不启动 XR teleop，用：

```bash
python uf_robot_pico_teleop_dual.py \
  --config config/xarm7_xrobotoolkit_teleop_dual.yaml \
  --sim false \
  --arm left \
  --reset-to-base-only \
  --reset-speed 20 \
  --reset-acc 80 \
  --no-gripper-init
```

当前 xArm7 XRoboToolkit 配置默认 `reset_speed: 30`、`reset_acc: 100`，适合先做柔和测试。如果仍然觉得突然，就继续降低；确认路径安全后再逐步调大。

启动后流程：

1. 终端按 Enter 后，脚本会先等待有效 XR 数据，再连接 `--arm` 选择的 xArm。
2. 如果没有 `--no-start-move`，真实机械臂模式会移动到 `start_joints` / `start_tcp_pose`；加上 `--no-start-move` 时连接后保持当前位姿。
3. PICO 输入 backend 开始提供 XR 数据。
4. 默认按住左右手柄 `grip` 才控制对应机械臂。`grip` 同时是 deadman 和离合：松开会暂停输出；下一次重新按住时，会先用当前手柄位姿和当前 TCP 位姿重新建立参考，再开始发送运动，避免松开期间手柄移动导致重新按住时跳变。
5. 默认 `trigger` 控制对应夹爪，`0` 为打开，`1` 为闭合；这条夹爪通道和运动 deadman 解耦，松开 `grip` 时仍会响应 trigger。
6. 默认按右手柄 `A` 会用 `reset_speed` / `reset_acc` 把 `--arm` 选择的真实机械臂移动到配置里的 `robot_base_pose`，然后重新校准对应手柄零点；示例默认位姿为 `[350, 0, 250, 3.141593, 0, -1.570796]`，TCP/tool 的 Z 轴竖直向下；按右手柄 `B` 会对 `--arm` 选择的机械臂发送软件急停并退出脚本。

## 配置重点

- `XRConfig.backend`：`udp` 使用 GSPlayground UDP 包；`xrobotoolkit` 使用 XRoboToolkit SDK。
- `XRConfig.udp_host` / `udp_port`：`udp` backend 下 Python 监听的 UDP 地址和端口。
- `XRConfig.xrobotoolkit_poll_hz`：`xrobotoolkit` backend 轮询 SDK 的频率。
- `XRConfig.deadman_threshold`：grip 超过该阈值才控制机械臂，默认配置为 `0.5`。
- `XRConfig.use_deadman`：是否启用 grip deadman。
- `XRConfig.stop_button`：PICO 软件停止按钮，默认 `right_b`。
- `XRConfig.stop_robot_on_stop_button`：为 `true` 时，按 stop button 会对选中的真实 xArm 调用 SDK `emergency_stop()`，然后退出 teleop；这不能替代物理急停按钮。
- `TeleoperatorConfig.controller`：`left` 或 `right`，决定使用哪个 PICO 手柄。
- `controller_to_robot_eef`：手柄坐标系到机器人末端坐标系的固定变换，通常需要现场微调。
- `robot_base_pose`：校准时对应的机器人末端基准位姿；示例配置让末端 Z 轴竖直向下。
- `position_scale`：手柄平移缩放系数。
- `action_scale`：最终机器人平移目标的缩放系数，围绕校准时的机器人基准位姿缩放。比如 `action_scale: 0.2` 表示手柄移动 50 mm 时，机器人 TCP 大约只移动 10 mm。首次真机测试建议保持较小值，确认方向后再逐步调大。
- `position_delta_frame`：手柄平移增量的解释方式。`head_yaw` 表示每次建立参考时记录头显 yaw，并把“当时头朝向的前方”作为机器人 `+x`，左方作为机器人 `+y`，上方作为机器人 `+z`；这是当前 xArm7 配置的默认值。`world` 表示按机器人世界坐标系 `x=front, y=left, z=up` 直接叠加；`controller` 表示旧逻辑，平移会跟随校准时手柄局部坐标轴。
- `use_current_robot_pose_as_base`：为 `true` 时，启动/自动建立参考时使用机器人当前 TCP 位姿作为基准；右手柄 `A` 的显式重校准会覆盖为配置里的默认 `robot_base_pose`。
- `GripperBridge.publish_redis`：可选打开 GSPlayground 兼容 Redis 字段，发布 `{redis_key}:gripper:action` 和 `{redis_key}:gripper:closure`。
- `GripperBridge.driver`：默认 `none`；设为 `changingtek` 时会尝试调用 `gs_env.real.changingtek.gripper.Gripper`，用左右 trigger 直接驱动 ChangingTek 夹爪。
- `--sim-viewer`：`mujoco` 显示两个 xArm7 模型并分别用 pose IK 跟随左右目标 TCP 的位置和姿态，`plot` 显示 3D 目标 TCP，`console` 打印目标 pose，`none` 只运行映射逻辑。
- `--arm`：`left` / `right` / `both`，选择实际连接和控制哪只机械臂；单臂真机首测建议 `--arm left` 或 `--arm right`。
- `--no-start-move`：真实机械臂模式下只连接和初始化，不移动到 `start_joints` / `start_tcp_pose`，适合第一次连真机。
- `--initial-position-only`：真实机械臂模式下只把选中的机械臂移动到 `RobotConfig.start_joints` / `start_tcp_pose`，然后退出，不启动 XR teleop。
- `--start-at-initial`：真实机械臂模式下连接后，将选中的机械臂移动到 `RobotConfig.start_joints` / `start_tcp_pose` 再开始 teleop。
- `--start-at-base`：真实机械臂模式下连接后，将选中的机械臂柔和移动到 `robot_base_pose` 再开始 teleop；会自动跳过 `start_joints` 启动移动。
- `--reset-to-base-only`：真实机械臂模式下只把选中的机械臂移动到 `robot_base_pose`，然后退出，不启动 XR teleop。
- `--reset-speed` / `--reset-acc`：命令行覆盖 `RobotConfig.reset_speed` / `reset_acc`，方便现场测试柔和程度。
- `--start-joint-speed` / `--start-joint-acc`：命令行覆盖 `RobotConfig.start_joint_speed` / `start_joint_acc`，方便现场测试 xArm Studio 初始位姿移动的柔和程度。
- `--no-gripper-init`：真实机械臂模式下不在启动时打开/初始化夹爪。

## 安全注意

- 首次调试请降低 `robot_speed` 和 `robot_acc`。
- 默认示例配置已把 `robot_speed` 降到 `100`、`robot_acc` 降到 `500`，确认安全后再逐步提高。
- 确认左右 `robot_ip`、`controller`、`controller_to_robot_eef` 没有填反。
- 真实 xArm7 优先使用 `config/xarm7_xrobotoolkit_teleop_dual.yaml`；xArm6 示例仅保留作参考。
- xArm7 示例没有配置 `start_tcp_pose`，连接后只移动到 7 轴 `start_joints`，进入控制时用当前 TCP 作为 base。
- 如果没有夹爪，把 `gripper_type` 设为 `0`。
- 保持急停可触达。
- 先不装夹爪或空载测试坐标方向，再启用真实任务。
