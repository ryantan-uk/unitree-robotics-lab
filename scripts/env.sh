#!/usr/bin/env bash
# 每个新终端执行一次:  source scripts/env.sh
# 作用: 载入 ROS 2 Humble、课程工作空间，并决定 DDS 走哪块网卡。
#   - 没跑过 scripts/connect.py  -> 离线模式: 只走本机回环 lo，数据来自回放 data/bags/
#   - 跑过 scripts/connect.py    -> 真机模式: 走你选的网卡 (config/cyclonedds.env)
_LAB_REPO="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
export UNITREE_LAB_REPO="$_LAB_REPO"

[ -f "$_LAB_REPO/config/local_paths.env" ] && . "$_LAB_REPO/config/local_paths.env"
: "${UNITREE_LAB_WS:=$HOME/unitree_lab_ws}"
: "${UNITREE_LAB_DEPS:=$HOME/unitree_lab_deps}"
export UNITREE_LAB_WS UNITREE_LAB_DEPS

. /opt/ros/humble/setup.bash
[ -f "$UNITREE_LAB_WS/install/setup.bash" ] && . "$UNITREE_LAB_WS/install/setup.bash"

export PATH="$HOME/.local/bin:$PATH"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # Unitree 机器人内部用 CycloneDDS，两边必须一致
export ROS_DOMAIN_ID=0                          # Unitree 默认 domain 0
unset ROS_LOCALHOST_ONLY

if [ -f "$_LAB_REPO/config/cyclonedds.env" ]; then
  . "$_LAB_REPO/config/cyclonedds.env"         # 真机: 由 connect.py 生成
else
  export UNITREE_LAB_IFACE=lo                   # 离线: 回环
  export CYCLONEDDS_URI="file://$_LAB_REPO/config/cyclonedds_offline.xml"
fi
echo "[unitree-lab] 网卡=$UNITREE_LAB_IFACE  RMW=$RMW_IMPLEMENTATION  (切回离线: python3 scripts/connect.py --offline)"
unset _LAB_REPO
