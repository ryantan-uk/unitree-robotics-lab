"""运动闸门: 运动指令默认只打印不下发。

规则只有一条: **没有显式确认，就不会有任何运动指令离开你的电脑。**

    gate = MotionGate(lab)
    gate.call(client.StandUp)            # 只打印 "[未下发] StandUp()"
    gate.confirm("我已清场并手握急停")      # 显式确认 (必须一字不差；离线时确认无效)
    gate.call(client.StandUp)            # 真的执行，返回 API 的返回值
"""
from __future__ import annotations

CONFIRM_PHRASE = "我已清场并手握急停"

CHECKLIST = """上真机前逐条确认:
  1. 机器人周围 2 米内没有人和障碍物，地面平整不滑
  2. 遥控器在手边，知道怎么让机器人立刻进入阻尼/急停 (组合键以官方手册为准，先在手册上查好)
  3. G1 做低层或上肢实验时已挂好保护吊架
  4. 电量充足，没有过温告警
  5. 你清楚接下来这条指令会让机器人做什么"""


class MotionGate:
    def __init__(self, lab=None, live: bool | None = None, verbose: bool = True):
        self.live = bool(lab.live) if live is None else bool(live)
        self.armed = False
        self.verbose = verbose
        self.log: list[tuple[str, bool]] = []     # (指令描述, 是否真的下发)
        self._warned = False

    # -- 确认 ---------------------------------------------------------------
    def confirm(self, phrase: str) -> bool:
        if not self.live:
            print("[闸门] 现在是离线模式 (LIVE=False)，没有机器人，确认无效；指令仍然只打印。")
            return False
        if phrase != CONFIRM_PHRASE:
            print(f'[闸门] 确认语不对，仍然不下发。需要一字不差: gate.confirm("{CONFIRM_PHRASE}")')
            return False
        self.armed = True
        print("[闸门] 已确认 —— 之后的运动指令会真的下发到机器人。")
        print(CHECKLIST)
        return True

    def disarm(self):
        self.armed = False
        print("[闸门] 已关闭，之后的运动指令只打印不下发。")

    @property
    def will_send(self) -> bool:
        """只有 "连着真机" 且 "确认过" 才下发。"""
        return self.live and self.armed

    @property
    def why_not(self) -> str:
        return "离线" if not self.live else "未确认"

    # -- 执行 ---------------------------------------------------------------
    def call(self, fn, *args, **kwargs):
        """受闸门保护地调用一个运动 API。未下发时返回 None。"""
        parts = [repr(a) for a in args] + [f"{k}={v!r}" for k, v in kwargs.items()]
        desc = f"{getattr(fn, '__name__', str(fn))}({', '.join(parts)})"
        send = self.will_send
        self.log.append((desc, send))
        if not send:
            if self.verbose:
                print(f"[未下发·{self.why_not}] {desc}")
            return None
        if self.verbose:
            print(f"[已下发] {desc}")
        return fn(*args, **kwargs)

    def allow_write(self, what: str = "LowCmd") -> bool:
        """给循环里的高频发布用: 返回这一拍能不能写。只在第一次被拦时提示。"""
        if self.will_send:
            return True
        if not self._warned and self.verbose:
            print(f"[未下发·{self.why_not}] {what} 持续被闸门拦截")
            self._warned = True
        return False
