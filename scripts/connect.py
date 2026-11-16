#!/usr/bin/env python3
"""连接机器人，一步到位:  选网卡 -> 生成 cyclonedds.xml -> 设好环境变量 -> 确认收到 Topic

    python3 scripts/connect.py              交互选择网卡 (连真机)
    python3 scripts/connect.py --iface eth0 直接指定网卡
    python3 scripts/connect.py --offline    切回离线 (回环 + 回放 data/ 里的 Rosbag)
    python3 scripts/connect.py --check      不改配置，只检查当前能不能收到 Topic

跑完后:  source scripts/env.sh   (让当前终端用上新配置)

Unitree 机器人的有线网口默认在 192.168.123.0/24 网段 (Go2 机载 192.168.123.161，
G1 机载 192.168.123.161/164；以官方文档为准)，你的网卡需要有一个同网段的地址。
本脚本到 "确认收到 Topic" 为止，不展开排障。
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
CONFIG = REPO / "config"
XML = CONFIG / "cyclonedds.xml"
ENV = CONFIG / "cyclonedds.env"
ROBOT_NET = "192.168.123."

XML_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<!-- 由 scripts/connect.py 生成。DDS 只走网卡 {iface}。 -->
<CycloneDDS>
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="{iface}" priority="default" multicast="default"/>
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
"""


def list_ifaces() -> list[dict]:
    out = subprocess.run(["ip", "-j", "-4", "addr"], capture_output=True, text=True, check=True).stdout
    rows = []
    for item in json.loads(out):
        addrs = [a["local"] for a in item.get("addr_info", [])]
        rows.append({"name": item["ifname"], "addrs": addrs, "up": "UP" in item.get("flags", []),
                     "robot_net": any(a.startswith(ROBOT_NET) for a in addrs)})
    return rows


def choose_iface(rows) -> str:
    print("本机网卡:")
    cands = [r for r in rows if r["name"] != "lo"]
    for i, r in enumerate(cands):
        mark = "  <- 与机器人同网段" if r["robot_net"] else ""
        print(f"  [{i}] {r['name']:<12} {', '.join(r['addrs']) or '(无 IPv4)':<22} {'UP' if r['up'] else 'DOWN'}{mark}")
    default = next((i for i, r in enumerate(cands) if r["robot_net"]), 0)
    if not sys.stdin.isatty():
        return cands[default]["name"]
    ans = input(f"选哪一块连机器人? [默认 {default}] ").strip()
    return cands[int(ans) if ans else default]["name"]


def write_config(iface: str):
    XML.write_text(XML_TEMPLATE.format(iface=iface))
    ENV.write_text(f'# 由 scripts/connect.py 生成，勿提交\nexport UNITREE_LAB_IFACE="{iface}"\n'
                   f'export CYCLONEDDS_URI="file://{XML}"\n')
    print(f"已生成 {XML.relative_to(REPO)} 和 {ENV.relative_to(REPO)}")


def check(iface: str, uri: str, seconds: float = 6.0) -> bool:
    """用 ROS 2 看一眼网络上有没有机器人的 Topic，并真的收一条 /lowstate。"""
    env = {**os.environ, "CYCLONEDDS_URI": uri, "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp", "ROS_DOMAIN_ID": "0"}
    code = r"""
import sys, time, rclpy
from rclpy.node import Node
rclpy.init()
n = Node("unitree_lab_connect_check")
t_end = time.monotonic() + float(sys.argv[1])
got = {}
subs = []
while time.monotonic() < t_end and not got:
    rclpy.spin_once(n, timeout_sec=0.2)
    if not subs:
        for name, types in n.get_topic_names_and_types():
            if name == "/lowstate" and n.count_publishers(name) > 0:
                pkg, _, cls = types[0].split("/")
                mod = __import__(pkg + ".msg", fromlist=[cls])
                subs.append(n.create_subscription(getattr(mod, cls), name, lambda m: got.setdefault("msg", m), 10))
topics = sorted(t for t, _ in n.get_topic_names_and_types() if n.count_publishers(t) > 0)
print("TOPICS", len(topics), " ".join(topics[:12]))
if got:
    m = got["msg"]
    robot = "G1 (unitree_hg)" if type(m).__module__.startswith("unitree_hg") else "Go2 (unitree_go)"
    print("LOWSTATE", robot, "tick=%d" % m.tick, "rpy=[%.3f %.3f %.3f]" % tuple(m.imu_state.rpy))
"""
    out = subprocess.run([sys.executable, "-c", code, str(seconds)], capture_output=True, text=True, env=env, timeout=seconds + 30)
    topics = [l for l in out.stdout.splitlines() if l.startswith("TOPICS")]
    low = [l for l in out.stdout.splitlines() if l.startswith("LOWSTATE")]
    print(f"网卡 {iface}: " + (topics[0][7:] if topics else "没有发现任何 Topic"))
    if low:
        print("  收到 /lowstate ->", low[0][9:])
        print("连接成功。")
        return True
    if iface == "lo":
        print("  离线模式下没有数据是正常的: 各个 Notebook 会自己回放 data/bags/ 里的 Rosbag。")
        print("  想现在就看: 另开终端运行  ros2 bag play --loop data/bags/go2_demo  再执行本检查。")
    else:
        print("  没有收到 /lowstate。先确认: 网线已接、机器人已开机、本网卡有 192.168.123.x 的地址。")
        if out.stderr.strip():
            print(out.stderr.strip().splitlines()[-1])
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", help="直接指定网卡名")
    ap.add_argument("--offline", action="store_true", help="切回离线模式")
    ap.add_argument("--check", action="store_true", help="只检查，不改配置")
    ap.add_argument("--seconds", type=float, default=6.0)
    args = ap.parse_args()

    if "rmw_cyclonedds_cpp" not in os.environ.get("RMW_IMPLEMENTATION", ""):
        print("请先执行:  source scripts/env.sh"); return 2

    if args.offline:
        XML.unlink(missing_ok=True); ENV.unlink(missing_ok=True)
        print("已切回离线模式 (回环 lo)。请重新执行:  source scripts/env.sh")
        return 0

    if args.check:
        iface = os.environ.get("UNITREE_LAB_IFACE", "lo")
        return 0 if check(iface, os.environ["CYCLONEDDS_URI"], args.seconds) or iface == "lo" else 1

    rows = list_ifaces()
    iface = args.iface or choose_iface(rows)
    if iface not in [r["name"] for r in rows]:
        print(f"没有名为 {iface} 的网卡"); return 2
    if iface == "lo":
        print("lo 是回环网卡，连不到真机。离线模式请用 --offline"); return 2
    if not next(r for r in rows if r["name"] == iface)["robot_net"]:
        print(f"[注意] {iface} 没有 {ROBOT_NET}x 的地址，多半收不到机器人数据。")
    write_config(iface)
    ok = check(iface, f"file://{XML}", args.seconds)
    print("\n下一步:  source scripts/env.sh   然后把 Notebook 第一格的 LIVE 改成 True")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
