"""用纯 ROS 2 调 Unitree 的 RPC 服务 —— 不经过 SDK。

SDK2 的 SportClient / LocoClient / MotionSwitcherClient 底下都是同一件事:
    往  rt/api/<service>/request   发一条 unitree_api/Request   (parameter 是 JSON 字符串)
    从  rt/api/<service>/response  等一条 identity.id 相同的 unitree_api/Response
(出处: unitree_sdk2py/rpc/client_stub.py, core/channel_name.py; unitree_ros2 的示例也是这么做的)

在 ROS 2 里这两条 topic 叫 /api/<service>/request 和 /api/<service>/response。
"""
from __future__ import annotations

import json
import threading
import time

from unitree_api.msg import Request, Response


class RosRpcClient:
    def __init__(self, node, service: str):
        self.node, self.service = node, service
        self._pub = node.create_publisher(Request, f"/api/{service}/request", 10)
        self._sub = node.create_subscription(Response, f"/api/{service}/response", self._on_response, 10)
        self._pending: dict[int, list] = {}
        self._lock = threading.Lock()

    def _on_response(self, msg: Response):
        with self._lock:
            slot = self._pending.get(msg.header.identity.id)
        if slot is not None:
            slot[0] = msg
            slot[1].set()

    def ready(self) -> bool:
        return self._pub.get_subscription_count() > 0

    def make_request(self, api_id: int, params: dict | None = None, noreply: bool = False) -> Request:
        req = Request()
        req.header.identity.id = time.monotonic_ns()
        req.header.identity.api_id = int(api_id)
        req.header.policy.noreply = noreply
        req.parameter = json.dumps(params or {})
        return req

    def send(self, api_id: int, params: dict | None = None):
        """只发不等 (用于高频的 Move)。"""
        self._pub.publish(self.make_request(api_id, params, noreply=True))

    def call(self, api_id: int, params: dict | None = None, timeout: float = 2.0):
        """发请求并阻塞等应答。返回 (code, data字符串)；超时返回 (3104, None)。

        注意: 需要节点正在别的线程里 spin (否则收不到应答)，见 04 节的讨论。
        """
        req = self.make_request(api_id, params)
        slot = [None, threading.Event()]
        with self._lock:
            self._pending[req.header.identity.id] = slot
        self._pub.publish(req)
        ok = slot[1].wait(timeout)
        with self._lock:
            self._pending.pop(req.header.identity.id, None)
        if not ok:
            return 3104, None          # RPC_ERR_CLIENT_API_TIMEOUT
        return slot[0].header.status.code, slot[0].data
