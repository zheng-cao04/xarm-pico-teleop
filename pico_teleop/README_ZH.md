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

建议使用 Python 3.10 环境。如果只跑 `--sim --sim-viewer console`，实际只需要 `numpy` 和 `pyyaml`；如果使用默认 `--sim-viewer plot`，还需要 matplotlib；真实机械臂模式需要 xArm Python SDK。

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
python uf_robot_pico_teleop_dual.py --config config/xarm6_pico_teleop_dual.yaml --sim --sim-viewer console
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
4. 先跑 sim：
   ```bash
   cd ufactory_teleop/pico_teleop
   python uf_robot_pico_teleop_dual.py --config config/xarm7_xrobotoolkit_teleop_dual.yaml --sim --sim-viewer console
   ```

`--sim` 默认打开 matplotlib 3D 视图，显示左右目标 TCP、轨迹和末端坐标轴。如果当前环境没有图形界面或没有安装 matplotlib，用 `--sim-viewer console`。

确认方向、左右手柄、grip/trigger 和重校准按钮符合预期后，再运行真实机械臂模式：

```bash
cd ufactory_teleop/pico_teleop
python uf_robot_pico_teleop_dual.py --config config/xarm7_xrobotoolkit_teleop_dual.yaml
```

启动后流程：

1. 真实机械臂模式会连接并初始化两台 xArm，机械臂会移动到 `start_joints` / `start_tcp_pose`；`--sim` 模式不会连接 xArm。
2. PICO 输入 backend 开始提供 XR 数据。
3. 终端按 Enter 后进入控制循环。
4. 默认按住左右手柄 `grip` 才控制对应机械臂，松开会暂停并清除该臂手柄零点。
5. 默认 `trigger` 控制对应夹爪，`0` 为打开，`1` 为闭合。
6. 默认按右手柄 `A` 重新校准左右手柄零点，按右手柄 `B` 停止脚本。

## 配置重点

- `XRConfig.backend`：`udp` 使用 GSPlayground UDP 包；`xrobotoolkit` 使用 XRoboToolkit SDK。
- `XRConfig.udp_host` / `udp_port`：`udp` backend 下 Python 监听的 UDP 地址和端口。
- `XRConfig.xrobotoolkit_poll_hz`：`xrobotoolkit` backend 轮询 SDK 的频率。
- `XRConfig.deadman_threshold`：grip 超过该阈值才控制机械臂，默认配置为 `0.5`。
- `XRConfig.use_deadman`：是否启用 grip deadman。
- `TeleoperatorConfig.controller`：`left` 或 `right`，决定使用哪个 PICO 手柄。
- `controller_to_robot_eef`：手柄坐标系到机器人末端坐标系的固定变换，通常需要现场微调。
- `robot_base_pose`：校准时对应的机器人末端基准位姿。
- `position_scale`：手柄平移缩放系数。
- `use_current_robot_pose_as_base`：为 `true` 时，每次校准使用机器人当前 TCP 位姿作为基准。
- `--sim-viewer`：`plot` 显示 3D 目标 TCP，`console` 打印目标 pose，`none` 只运行映射逻辑。

## 安全注意

- 首次调试请降低 `robot_speed` 和 `robot_acc`。
- 默认示例配置已把 `robot_speed` 降到 `100`、`robot_acc` 降到 `500`，确认安全后再逐步提高。
- 确认左右 `robot_ip`、`controller`、`controller_to_robot_eef` 没有填反。
- 真实 xArm7 优先使用 `config/xarm7_xrobotoolkit_teleop_dual.yaml`；xArm6 示例仅保留作参考。
- xArm7 示例没有配置 `start_tcp_pose`，连接后只移动到 7 轴 `start_joints`，进入控制时用当前 TCP 作为 base。
- 如果没有夹爪，把 `gripper_type` 设为 `0`。
- 保持急停可触达。
- 先不装夹爪或空载测试坐标方向，再启用真实任务。
