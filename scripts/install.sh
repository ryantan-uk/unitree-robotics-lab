#!/usr/bin/env bash
# unitree-robotics-lab 一键安装脚本
# 用法:  bash scripts/install.sh
# 目标机: 全新的 Ubuntu 22.04 (x86_64)，VMware 或 OrbStack 均可。可重复执行。
#
# 可选环境变量:
#   UNITREE_LAB_MIRROR=auto|cn|official   软件源 (默认 auto: 连不上 pypi.org 就用清华源)
#   UNITREE_LAB_DEPS=~/unitree_lab_deps   第三方源码 (Unitree 官方仓库) 放哪
#   UNITREE_LAB_WS=~/unitree_lab_ws       colcon 工作空间放哪
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPS="${UNITREE_LAB_DEPS:-$HOME/unitree_lab_deps}"
WS="${UNITREE_LAB_WS:-$HOME/unitree_lab_ws}"
MIRROR="${UNITREE_LAB_MIRROR:-auto}"

# 课程内容核对所依据的上游版本 (见 docs/source_audit.md)。固定版本，保证人人一致。
SDK2PY_SHA=814556d15970dd2ecf1c9984e845ca02ab07e206   # unitree_sdk2_python
ROS2_SHA=668d1ec5a05d1c38d3306bdca7d59f2ba3581a88     # unitree_ros2
UNITREE_ROS_SHA="${UNITREE_ROS_SHA:-master}"          # unitree_ros (只取 URDF)

step() { printf '\n\033[1;34m==> [%s] %s\033[0m\n' "$1" "$2"; }
retry() { local n=0; until "$@"; do n=$((n+1)); [ "$n" -ge 4 ] && return 1; echo "  重试 $n/3: $*"; sleep 5; done; }

# ---------------------------------------------------------------- 0. 检查
step 0/7 "检查系统"
. /etc/os-release
if [ "${VERSION_ID:-}" != "22.04" ]; then
  echo "本课程只支持 Ubuntu 22.04 (当前: ${PRETTY_NAME:-unknown})"; exit 1
fi
echo "系统: $PRETTY_NAME / $(uname -m)   仓库: $REPO"
case "$REPO" in *" "*) echo "仓库路径里有空格，colcon 会出错。请把仓库放到不含空格的路径 (例如 ~/unitree-robotics-lab)"; exit 1;; esac

if [ "$MIRROR" = auto ]; then
  if curl -fsS -m 8 -o /dev/null https://pypi.org/simple/pip/ 2>/dev/null; then MIRROR=official; else MIRROR=cn; fi
fi
echo "软件源: $MIRROR"
if [ "$MIRROR" = cn ]; then
  ROS_APT=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
  PIP_ARGS=(-i https://pypi.tuna.tsinghua.edu.cn/simple)
else
  ROS_APT=http://packages.ros.org/ros2/ubuntu
  PIP_ARGS=()
fi

# ---------------------------------------------------------------- 1. 基础工具
step 1/7 "安装基础工具"
export DEBIAN_FRONTEND=noninteractive
retry sudo apt-get update -qq
retry sudo apt-get install -y -qq --no-install-recommends \
  ca-certificates curl gnupg lsb-release git build-essential cmake \
  python3-pip python3-dev iproute2 iputils-ping locales
sudo locale-gen en_US.UTF-8 >/dev/null

# ---------------------------------------------------------------- 2. ROS 2 Humble
step 2/7 "安装 ROS 2 Humble"
KEY=/usr/share/keyrings/ros-archive-keyring.gpg
if [ ! -s "$KEY" ]; then
  retry sudo curl -fsSL -m 60 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$KEY" \
    || retry sudo curl -fsSL -m 60 https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key -o "$KEY"
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=$KEY] $ROS_APT jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
retry sudo apt-get update -qq
# 分两批装并及时清缓存，磁盘紧张的虚拟机也装得下
retry sudo apt-get install -y -qq --no-install-recommends \
  ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-xacro \
  ros-humble-tf2-tools ros-humble-tf2-ros ros-humble-sensor-msgs-py \
  ros-humble-example-interfaces ros-humble-rosbag2-storage-default-plugins \
  python3-colcon-common-extensions python3-matplotlib python3-pytest
sudo apt-get clean
retry sudo apt-get install -y -qq --no-install-recommends \
  ros-humble-navigation2 ros-humble-nav2-simple-commander ros-humble-slam-toolbox ros-humble-rviz2
sudo apt-get clean

# ---------------------------------------------------------------- 3. Python 依赖
step 3/7 "安装 Python 依赖 (Jupyter, CycloneDDS)"
# 注意: 不用 pip 装 numpy —— ROS 2 Humble 的二进制包按系统自带的 numpy 1.21 编译，
# 升到 numpy 2 会让 rclpy 的消息转换出错。
retry python3 -m pip install --user -q "${PIP_ARGS[@]}" \
  "cyclonedds==0.10.2" "notebook>=7,<8" "nbconvert>=7" "ipykernel>=6" "nbformat>=5" \
  "matplotlib-inline==0.1.7"   # 0.2 起要求更新的 matplotlib，与系统自带的 3.5 不兼容
export PATH="$HOME/.local/bin:$PATH"

# ---------------------------------------------------------------- 4. Unitree 官方源码
step 4/7 "获取 Unitree 官方源码 (固定版本)"
mkdir -p "$DEPS"
fetch_at() {  # fetch_at <dir> <url> <sha-or-branch>
  local dir="$1" url="$2" ref="$3"
  if [ ! -d "$dir/.git" ]; then git init -q "$dir"; git -C "$dir" remote add origin "$url"; fi
  retry git -C "$dir" fetch -q --depth 1 origin "$ref"
  git -C "$dir" checkout -q FETCH_HEAD
  echo "  $(basename "$dir") @ $(git -C "$dir" rev-parse --short HEAD)"
}
fetch_at "$DEPS/unitree_sdk2_python" https://github.com/unitreerobotics/unitree_sdk2_python.git "$SDK2PY_SHA"
fetch_at "$DEPS/unitree_ros2"        https://github.com/unitreerobotics/unitree_ros2.git        "$ROS2_SHA"
# URDF: 只稀疏检出 go2 / g1 两个 description 目录
if [ ! -d "$DEPS/unitree_ros/.git" ]; then
  git init -q "$DEPS/unitree_ros"
  git -C "$DEPS/unitree_ros" remote add origin https://github.com/unitreerobotics/unitree_ros.git
  git -C "$DEPS/unitree_ros" config core.sparseCheckout true
  git -C "$DEPS/unitree_ros" config remote.origin.promisor true
  git -C "$DEPS/unitree_ros" config remote.origin.partialclonefilter blob:none
  printf 'robots/go2_description/\nrobots/g1_description/\n' > "$DEPS/unitree_ros/.git/info/sparse-checkout"
fi
if retry git -C "$DEPS/unitree_ros" fetch -q --depth 1 --filter=blob:none origin "$UNITREE_ROS_SHA" \
   && retry git -C "$DEPS/unitree_ros" checkout -q FETCH_HEAD; then
  echo "  unitree_ros (URDF) @ $(git -C "$DEPS/unitree_ros" rev-parse --short HEAD)"
else
  echo "  [警告] unitree_ros (URDF) 获取失败: 只影响第 16 节 (需要官方 URDF)。网络恢复后重跑本脚本即可补上。"
fi

# unitree_sdk2py 不走 pip: 它声明依赖 opencv-python (会强行升级 numpy)，而本课程不用它的视频接口
# 用 .pth 文件把两个源码目录加入 Python 路径 (改了源码立即生效，等价于可编辑安装；
# Ubuntu 22.04 自带的 pip 22.0 做 `pip install --user -e` 会失败，所以不用它)
USER_SITE="$(python3 -c 'import site; print(site.getusersitepackages())')"
mkdir -p "$USER_SITE"
printf '%s\n%s\n' "$DEPS/unitree_sdk2_python" "$REPO" > "$USER_SITE/unitree_lab.pth"

# ---------------------------------------------------------------- 5. colcon 工作空间
step 5/7 "编译 colcon 工作空间 (Unitree 消息 + 课程节点 + C++ 示例)"
mkdir -p "$WS/src"
for p in unitree_go unitree_hg unitree_api; do
  ln -sfn "$DEPS/unitree_ros2/cyclonedds_ws/src/unitree/$p" "$WS/src/$p"
done
for p in "$REPO"/src/*/; do
  [ -f "$p/package.xml" ] && ln -sfn "${p%/}" "$WS/src/$(basename "$p")"
done
set +u; source /opt/ros/humble/setup.bash; set -u
# --symlink-install: 改了 src/ 里的 Python 节点立即生效，不用重新编译
( cd "$WS" && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    --event-handlers console_cohesion+ status- 2>&1 | grep -vE "^\s*$" | tail -n 25 )
[ -f "$WS/install/setup.bash" ] || { echo "colcon 编译失败"; exit 1; }

# ---------------------------------------------------------------- 6. 记录路径
step 6/7 "写入环境配置"
cat > "$REPO/config/local_paths.env" <<EOF
# 由 scripts/install.sh 生成，勿提交
export UNITREE_LAB_DEPS="$DEPS"
export UNITREE_LAB_WS="$WS"
EOF

# ---------------------------------------------------------------- 7. 自检
step 7/7 "自检"
set +u; source "$REPO/scripts/env.sh"; set -u
python3 - <<'EOF'
import rclpy, numpy, matplotlib, cyclonedds
from unitree_go.msg import LowState                       # ROS 2 消息 (colcon 编译产物)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_  # SDK2 Python 的 IDL
import common
print("  rclpy OK | numpy", numpy.__version__, "| cyclonedds-python OK | unitree_go/unitree_sdk2py/common OK")
EOF
echo
echo "安装完成。之后每开一个新终端先执行:"
echo "    source $REPO/scripts/env.sh"
echo "然后:  jupyter notebook notebooks/      (连真机前先跑 python3 scripts/connect.py)"
