"""连接: 每个 Notebook 第一格都是

    from common import connect
    lab = connect("go2", live=LIVE)

LIVE=True  (真机): DDS 走 scripts/connect.py 选好的网卡。
LIVE=False (默认, 没有真机时): DDS 只走本机回环 lo，并循环回放 data/bags/ 里的一段 Rosbag，
           于是订阅代码照常能收到数据。回放的数据是**合成的** (见 data/README.md)。
           离线时没有机器人在听，所以运动指令一律只打印不下发 (common/safety.py)。
"""
from __future__ import annotations

import atexit
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

REPO = Path(os.environ.get("UNITREE_LAB_REPO", Path(__file__).resolve().parent.parent))
CONFIG = REPO / "config"
DATA = REPO / "data"
OFFLINE_XML = CONFIG / "cyclonedds_offline.xml"
LIVE_ENV = CONFIG / "cyclonedds.env"
_PIDFILE = Path("/tmp/unitree_lab_children.pids")

_sdk_iface: str | None = None       # 本进程里 SDK 已经在哪块网卡上初始化过


def _read_live_env() -> dict:
    env = {}
    if LIVE_ENV.exists():
        for line in LIVE_ENV.read_text().splitlines():
            line = line.strip()
            if line.startswith("export ") and "=" in line:
                k, v = line[len("export "):].split("=", 1)
                env[k] = v.strip().strip('"').strip("'")
    return env


def _kill_stale():
    """杀掉上一个 Notebook 没清干净的子进程 (bag 回放、后台节点)。"""
    if not _PIDFILE.exists():
        return
    for pid in _PIDFILE.read_text().split():
        try:
            os.killpg(int(pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError, ValueError):
            pass
    _PIDFILE.unlink(missing_ok=True)
    time.sleep(0.3)


def init_sdk(iface: str):
    """初始化 SDK2 的 DDS 通道 (整个进程只能做一次)。

    真机: 就是官方的 ChannelFactoryInitialize(0, 网卡名)。
    离线: 多做一件事 —— 把 SDK 内置的 DDS 配置换成 config/cyclonedds_offline.xml。
          回环网卡不支持组播，且 SDK 内置配置在同一台机器上最多容纳 10 个 DDS 参与者，
          离线时所有进程都挤在本机，不够用。这只影响回环上的发现，不改变任何 topic 和消息。
    """
    global _sdk_iface
    if _sdk_iface == iface:
        return
    if _sdk_iface is not None:
        raise RuntimeError(f"SDK 已在网卡 {_sdk_iface} 上初始化，不能在同一个进程里换到 {iface}。请重启内核。")
    import unitree_sdk2py.core.channel as ch
    if iface == "lo":
        import re
        # cyclonedds-python 只接受 ASCII 的配置串，所以去掉 XML 里的中文注释
        ch.ChannelConfigHasInterface = re.sub(r"<!--.*?-->", "", OFFLINE_XML.read_text(), flags=re.S)
    ch.ChannelFactoryInitialize(0, iface)
    _sdk_iface = iface


class Background:
    """后台子进程，退出 with 块或内核退出时连同子孙一起杀掉。"""

    def __init__(self, cmd, log: str | None = None, env: dict | None = None, quiet: bool = True):
        self.cmd = cmd if isinstance(cmd, str) else " ".join(cmd)
        self.log_path = Path(log) if log else Path(f"/tmp/unitree_lab_{int(time.time() * 1000) % 10**8}.log")
        self._f = open(self.log_path, "w")
        self.proc = subprocess.Popen(["bash", "-c", self.cmd], stdout=self._f, stderr=subprocess.STDOUT,
                                     start_new_session=True, env={**os.environ, **(env or {})})
        with open(_PIDFILE, "a") as f:
            f.write(f"{self.proc.pid}\n")
        atexit.register(self.stop)
        if not quiet:
            print(f"[后台] {self.cmd}  (日志 {self.log_path})")

    def alive(self) -> bool:
        return self.proc.poll() is None

    def output(self, tail: int | None = None) -> str:
        self._f.flush()
        text = self.log_path.read_text(errors="replace")
        return "\n".join(text.splitlines()[-tail:]) if tail else text

    def wait_for(self, text: str, timeout: float = 30.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if text in self.output():
                return True
            if not self.alive():
                break
            time.sleep(0.1)
        return text in self.output()

    def kill(self):
        """立刻强杀 (不等待)。用来模拟 "数据源突然消失"。"""
        if self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def stop(self, sig=signal.SIGINT, grace: float = 3.0):
        if self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, sig)
                self.proc.wait(grace)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait(2)
            except ProcessLookupError:
                pass
        if not self._f.closed:
            self._f.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.stop()


@dataclass
class Lab:
    robot: str
    live: bool
    iface: str
    bag: str | None = None
    procs: list = field(default_factory=list)

    def close(self):
        for p in self.procs:
            p.stop()
        self.procs.clear()

    def cut(self):
        """模拟数据突然中断 (离线: 强杀回放进程)。真机上请直接拔网线。"""
        for p in self.procs:
            p.kill()

    def __repr__(self):
        where = "真机" if self.live else (f"离线, 回放 data/bags/{self.bag} (合成数据)" if self.bag else "离线, 无数据源")
        return f"<Lab {self.robot} | {where} | 网卡 {self.iface}>"


def connect(robot: str = "go2", live: bool = False, bag: str | None = "", sdk: bool = True,
            loop: bool = True, quiet: bool = False) -> Lab:
    """建立到机器人的连接；没有真机时改为回放一段 Rosbag。

    bag: 离线时回放 data/bags/ 下的哪一段。"" = 默认的 <robot>_demo；None = 不回放。
    sdk: 是否同时初始化 SDK2 的 DDS 通道 (只用 ROS 2 的章节可以关掉)。
    """
    if robot not in ("go2", "g1"):
        raise ValueError("robot 只能是 'go2' 或 'g1'")
    _kill_stale()

    if live:
        env = _read_live_env()
        if "UNITREE_LAB_IFACE" not in env:
            raise RuntimeError("LIVE=True 需要先在终端运行:  python3 scripts/connect.py   (选网卡并生成配置)")
        iface = env["UNITREE_LAB_IFACE"]
        os.environ["CYCLONEDDS_URI"] = env["CYCLONEDDS_URI"]
        bag = None
    else:
        iface = "lo"
        os.environ["CYCLONEDDS_URI"] = f"file://{OFFLINE_XML}"
        bag = f"{robot}_demo" if bag == "" else bag
    os.environ["UNITREE_LAB_IFACE"] = iface
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    os.environ["ROS_DOMAIN_ID"] = "0"

    lab = Lab(robot=robot, live=live, iface=iface, bag=bag)
    if sdk:
        init_sdk(iface)
    if bag:
        lab.procs.append(play_bag(bag, loop=loop))
        time.sleep(2.0)
    if not quiet:
        print(lab)
        if not live:
            print("  连真机: 终端运行 python3 scripts/connect.py，再把本页的 LIVE 改成 True。")
    return lab


def play_bag(name: str, loop: bool = False, rate: float = 1.0, extra: str = "") -> Background:
    """后台回放 data/bags/<name>。返回 Background，可 .stop()。"""
    path = Path(name) if Path(name).exists() else DATA / "bags" / name
    if not path.exists():
        raise FileNotFoundError(f"找不到 {path}")
    cmd = f"exec ros2 bag play {path} {'--loop ' if loop else ''}--rate {rate} {extra}"
    return Background(cmd, log=f"/tmp/unitree_lab_bag_{path.name}.log")
