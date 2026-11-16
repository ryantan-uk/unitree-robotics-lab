# 环境说明

## 学生环境：Windows + VMware Workstation + Ubuntu 22.04

1. **建虚拟机**：Ubuntu 22.04 Desktop（x86_64），建议 4 核 / 8 GB 内存 / 40 GB 磁盘。
2. **网络选桥接模式**（不是 NAT）：虚拟机设置 → 网络适配器 → 桥接模式。
   然后在 VMware 的"虚拟网络编辑器"里把 `VMnet0` 桥接到**连机器人的那块物理有线网卡**（不要选"自动"，否则可能桥到 Wi-Fi 上）。
   原因：DDS 靠组播发现对方，NAT 下虚拟机和机器人不在同一个二层网络里。
3. **装课程**：
   ```bash
   sudo apt update && sudo apt install -y git
   git clone <本仓库> ~/unitree-robotics-lab
   cd ~/unitree-robotics-lab && bash scripts/install.sh
   ```
   脚本会自动判断用官方源还是清华镜像（连不上 pypi.org 时用镜像；也可 `UNITREE_LAB_MIRROR=cn bash scripts/install.sh` 强制）。
   可重复执行，中途断网了重跑即可。
4. **每开一个终端**：`source ~/unitree-robotics-lab/scripts/env.sh`。想省事可以把这行加进 `~/.bashrc`。
5. **连真机**：网线连机器人；在 Ubuntu 的网络设置里给有线网卡配一个与机器人同网段的静态地址；`python3 scripts/connect.py`。

安装脚本做了什么：

| 步骤 | 内容 |
|---|---|
| 1 | 基础工具（git、编译器、pip） |
| 2 | ROS 2 Humble（`ros-base`）+ CycloneDDS RMW + robot_state_publisher + Nav2 + slam_toolbox + RViz2 |
| 3 | Python 依赖：`cyclonedds==0.10.2`、Jupyter。**不用 pip 装 numpy**（ROS 2 Humble 的二进制包按系统的 numpy 1.21 编译） |
| 4 | 固定版本获取 `unitree_sdk2_python`、`unitree_ros2`，以及 `unitree_ros` 里的两份 URDF（放在 `~/unitree_lab_deps/`） |
| 5 | 在 `~/unitree_lab_ws/` 里 `colcon build --symlink-install`：Unitree 消息 + 本仓库 `src/` 下的三个包 |
| 6–7 | 写入路径配置；自检 |

仓库只包含源码，不包含任何编译产物；`src/` 下的包是通过符号链接进入工作空间的，改了 Python 节点立即生效，改了 C++ 或接口定义后需要：

```bash
cd ~/unitree_lab_ws && colcon build --symlink-install --packages-select unitree_lab_cpp
```

## 开发环境：Mac + OrbStack

为了和学生环境一致，用 x86_64 的 Ubuntu 22.04：

```bash
orb create --arch amd64 ubuntu:jammy unitree-lab
orb -m unitree-lab bash -c 'git clone <仓库路径> ~/unitree-robotics-lab && cd ~/unitree-robotics-lab && bash scripts/install.sh'
```

注意：

- 仓库放在虚拟机自己的家目录下，不要放在路径含空格的挂载目录里（colcon 不支持）。
- Apple Silicon 上 amd64 是转译运行，慢 2–3 倍：安装约 30–40 分钟，Python 控制循环达不到标称频率（11、14 节的讲解里用到了这一点）。
- 至少留 5 GB 磁盘。
- OrbStack 连不了真机，也不方便开 RViz2。这两类内容在这里只验证到"代码能跑、数据能出"，其余见 `real_robot_checklist.md`。

## 离线模式的实现细节（排错时有用）

- 没运行过 `scripts/connect.py` 时，`env.sh` 把 `CYCLONEDDS_URI` 指向 `config/cyclonedds_offline.xml`：只用回环网卡 `lo`。
- 回环不支持组播，所以这份配置关掉组播、改用"向 localhost 的一串端口单播"来发现，并把同机参与者上限从默认的 10 提到 120（Nav2 一启动就超过 10 个）。
- `common.connect()` 在离线时启动 `ros2 bag play --loop data/bags/<robot>_demo`；SDK 侧用同一份配置初始化（`common/connect.py` 的 `init_sdk`）。
- 上一个 Notebook 没清干净的后台进程，会在下一次 `connect()` 时被清掉；也可以手动 `pkill -f "ros2 bag play"`。
- 若 `ros2 topic list` 的结果看起来是旧的：`ros2 daemon stop` 后重试（守护进程会记住它启动时的 DDS 配置）；课程里的命令都带了 `--no-daemon`。

## 维护者

- Notebook 的源文件在 `tools/nbsrc/*.py`（纯文本，便于评审），`python3 tools/build_notebooks.py` 生成 `notebooks/*.ipynb`。请改源文件，不要直接改 `.ipynb`。
- 样本数据由 `python3 -m tools.datagen.make_data` 重新生成（`--sim-only` 只重做不经过 DDS 的两份 npz）。生成器里的机器人替身只是为了造数据，不属于课程内容。
