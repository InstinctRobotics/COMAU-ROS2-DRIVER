"""
Disconnect safety, characterisation on the Comau controller simulator (outside this repo).

The simulator (~/Videos/comau_sim, used here as a library) replays the CURRENT PDL behaviour of the controller:
a MOVEFLY stream is queued with ADVANCE and nothing cancels it when the motion socket (port 1101) closes.
These tests stream MOVEFLY steps at 100 Hz like the driver, close the socket abruptly, and measure what the
robot does afterwards:

  * test_queue_runs_on_after_abrupt_close: the problem (robot keeps moving after the socket is gone). This test
    documents the simulator, NOT the new motion_server.pdl (the simulator does not run PDL): when the simulator
    is updated to the new motion_server behaviour this test must be inverted.
  * test_cancel_before_close_*: what the new PDL (and the driver's cancel on deactivation) achieve: the
    robot_server 'X' command ahead of the close empties the queue, the robot brakes from where it is instead of
    running on to the last streamed goal. The braking distance itself (v^2 / 2a) is physics and stays.

Both print the measured numbers (run with -s): they are quoted in docs/disconnect-safety.md.
The tests are skipped when the simulator folder or numpy is missing. They run in real time (~4 s each) on
127.0.0.1 with ports chosen by the OS; nothing touches a real controller.
"""
import asyncio
import math
import os
import socket
import struct
import sys
import time

import pytest

SIM_DIR = os.environ.get("COMAU_SIM_DIR", os.path.expanduser("~/Videos/comau_sim"))
if not os.path.isdir(os.path.join(SIM_DIR, "comau_sim")):
    pytest.skip(f"simulator not found in {SIM_DIR}", allow_module_level=True)
pytest.importorskip("numpy")
sys.path.insert(0, SIM_DIR)

from comau_sim import config  # noqa: E402
from comau_sim.alarm_manager import AlarmManager  # noqa: E402
from comau_sim.movefly_model import _tcp_mm  # noqa: E402
from comau_sim.robot_state import RobotState  # noqa: E402
from comau_sim.ros2_pdl_servers import ComauRos2PdlBridge  # noqa: E402

STEP_S = 0.010          # the driver streams a step per 10 ms control cycle
RATES_DEG_S = (60.0, 120.0)   # joint 1 sweep rates: TCP ~0.5 m/s and ~1.0 m/s at the home radius
LIN_VEL_M_S = 1.25      # cap sent with every step (driver default fly_lin_velocity)
FLY_DIST_MM = 100.0     # driver default fly_dist


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _movefly(msg_id: int, q_deg) -> bytes:
    return (struct.pack("<Ic", msg_id, b"F") + struct.pack("<2f", LIN_VEL_M_S, FLY_DIST_MM)
            + struct.pack("<6f", *q_deg))


async def _scenario(cancel_before_close: bool, rate_deg_s: float) -> dict:
    stream_s = 1.5 if rate_deg_s <= 60.0 else 1.1   # J1 must stay inside its +-170 deg limit
    config.ROS2_MOTION_PORT, config.ROS2_STATE_PORT, config.ROS2_ROBOT_PORT = (_free_port() for _ in range(3))
    state = RobotState()
    bridge = ComauRos2PdlBridge(state, AlarmManager())
    await bridge.start("127.0.0.1")
    model = bridge._model
    try:
        await asyncio.sleep(0.2)
        _, motion_w = await asyncio.open_connection("127.0.0.1", config.ROS2_MOTION_PORT)
        _, robot_w = await asyncio.open_connection("127.0.0.1", config.ROS2_ROBOT_PORT)
        q0 = list(state.joint_pos)

        # Stream like the driver: one step per 10 ms along a joint-1 sweep
        t0 = time.monotonic()
        msg_id, q = 0, list(q0)
        while (t := time.monotonic() - t0) < stream_s:
            q = list(q0)
            q[0] += rate_deg_s * t
            msg_id += 1
            motion_w.write(_movefly(msg_id, q))
            await motion_w.drain()
            await asyncio.sleep(STEP_S)

        # The ROS side goes away
        tcp_close = _tcp_mm(model.q)
        v_close = model.v
        queue_at_close = len(model._queue) + (1 if model._seg is not None else 0)
        lead_mm = math.dist(_tcp_mm(q), tcp_close)   # last streamed target vs where the robot is
        t_close = time.monotonic()
        if cancel_before_close:
            robot_w.write(struct.pack("<Ic", msg_id + 1, b"X"))   # robot_server CANCEL
            await robot_w.drain()
        motion_w.transport.abort()                                # abrupt close of the motion socket

        # What does the robot do now?
        travelled, t_stop, last_pos = 0.0, None, tcp_close
        v_after_50ms = None
        while time.monotonic() - t_close < 3.0:
            await asyncio.sleep(0.004)
            pos = _tcp_mm(model.q)
            travelled += math.dist(pos, last_pos)
            last_pos = pos
            if v_after_50ms is None and time.monotonic() - t_close >= 0.05:
                v_after_50ms = model.v
            if t_stop is None and model.idle:
                t_stop = time.monotonic() - t_close
                break
        robot_w.transport.abort()
        return {"v_close_mm_s": v_close, "queued_moves_at_close": queue_at_close, "lead_mm": lead_mm,
                "travelled_after_close_mm": travelled, "stop_after_s": t_stop,
                "v_50ms_after_close_mm_s": v_after_50ms, "idle_at_end": model.idle}
    finally:
        await bridge.stop()


@pytest.mark.parametrize("rate", RATES_DEG_S)
def test_queue_runs_on_after_abrupt_close(rate):
    r = asyncio.run(_scenario(cancel_before_close=False, rate_deg_s=rate))
    print(f"\n[no cancel, J1 {rate:g} deg/s] ", {k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
    # The problem: the socket is gone, the robot is still moving on the queued steps...
    assert r["v_close_mm_s"] > 100.0, "the stream must have the robot moving at the time of the close"
    assert r["v_50ms_after_close_mm_s"] is not None and r["v_50ms_after_close_mm_s"] > 50.0
    # ...all the way to the last streamed goal, which is ahead of the robot by the cruise lag
    assert r["travelled_after_close_mm"] > 0.8 * r["lead_mm"]
    # it then ends by itself (the queue is bounded: no runaway)
    assert r["stop_after_s"] is not None and r["stop_after_s"] < 2.0


@pytest.mark.parametrize("rate", RATES_DEG_S)
def test_cancel_before_close_empties_the_queue(rate):
    r = asyncio.run(_scenario(cancel_before_close=True, rate_deg_s=rate))
    print(f"\n[cancel first, J1 {rate:g} deg/s] ", {k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
    assert r["v_close_mm_s"] > 100.0
    assert r["stop_after_s"] is not None and r["stop_after_s"] < 2.0
    # braking from where it is: never beyond the last streamed goal
    assert r["travelled_after_close_mm"] < r["lead_mm"] + 5.0
