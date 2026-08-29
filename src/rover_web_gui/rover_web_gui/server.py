"""FastAPI application for the rover web GUI.

Serves the dashboard (`/`) and a standalone microscope view
(`/microscope`, meant to be opened in its own browser tab per the
project brief), a single bidirectional WebSocket carrying telemetry
down and control commands up, MJPEG endpoints for the two cameras,
and two POST endpoints for the microscope's snapshot/recording
services.

All ROS 2 I/O is delegated to :class:`rover_web_gui.ros_bridge.RosBridge`,
which owns its own executor thread - this module never touches rclpy
directly except through that bridge, and always hands blocking bridge
calls off to a thread pool via ``run_in_executor`` so the asyncio event
loop is never stalled.
"""

from __future__ import annotations

import asyncio
import json
import logging
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .ros_bridge import RosBridge

_STATIC_DIR = Path(__file__).parent / "static"

app = FastAPI(title="Mars Rover Ground Control")
app.mount("/static", StaticFiles(directory=str(_STATIC_DIR)), name="static")

bridge: Optional[RosBridge] = None


def set_bridge(new_bridge: RosBridge) -> None:
    global bridge
    bridge = new_bridge


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(str(_STATIC_DIR / "index.html"))


@app.get("/microscope")
async def microscope_page() -> FileResponse:
    return FileResponse(str(_STATIC_DIR / "microscope.html"))


@app.get("/api/config")
async def get_config() -> Response:
    """Drive sensitivity (deadzone, max linear/angular speed) for the
    frontend's virtual joystick, plus the mast's transport-position
    preset - all fetched once on page load rather than hardcoded in
    JS, so neither can drift from what other nodes actually use (see
    ros_bridge.RosBridge.get_static_config).
    """
    if bridge is None:
        return Response(status_code=503, content="ROS bridge not ready")
    loop = asyncio.get_event_loop()
    config = await loop.run_in_executor(None, bridge.get_static_config)
    return Response(content=json.dumps(config), media_type="application/json")


# ------------------------------------------------------------- WebSocket ---
@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    loop = asyncio.get_event_loop()
    sender_task = asyncio.create_task(_telemetry_sender(ws))
    try:
        while True:
            raw = await ws.receive_text()
            try:
                message = json.loads(raw)
            except json.JSONDecodeError:
                continue
            await loop.run_in_executor(None, _handle_control_message, message)
    except WebSocketDisconnect:
        pass
    finally:
        sender_task.cancel()


async def _telemetry_sender(ws: WebSocket, rate_hz: float = 10.0) -> None:
    period = 1.0 / rate_hz
    loop = asyncio.get_event_loop()
    try:
        while True:
            if bridge is not None:
                snapshot = await loop.run_in_executor(None, bridge.get_snapshot)
                # FIXED: this used to have no try/except of its own, so any
                # one board's own snapshot data failing to json.dumps() (a
                # real, previously-hit example: a fixed-size ROS array field
                # read out as numpy.int32/numpy.bool_ elements rather than
                # plain Python ones, which json.dumps() cannot serialize at
                # all) raised uncaught here - which didn't just skip that
                # one board, it killed this ENTIRE task silently (an
                # unhandled exception in an asyncio task doesn't crash the
                # process, it just stops running, logged only as "Task
                # exception was never retrieved" server-side, nothing the
                # browser would ever see). Every board's own telemetry
                # shares this one task and one combined snapshot, so one
                # board's own bad field took every board down at once, not
                # just its own - board-agnostic on purpose, not specific to
                # whichever board actually happened to have the problem.
                # WebSocketDisconnect/CancelledError still propagate to the
                # outer try/except below, unchanged - only genuinely
                # unexpected failures are caught and logged here, so the
                # loop can keep serving every other board's own good data
                # on the next tick instead of dying entirely.
                try:
                    await ws.send_text(json.dumps({"type": "telemetry", "data": snapshot}))
                except (WebSocketDisconnect, asyncio.CancelledError):
                    raise
                except Exception:
                    logging.exception("telemetry snapshot failed to send - skipping this tick, not the whole stream")
            await asyncio.sleep(period)
    except (WebSocketDisconnect, asyncio.CancelledError):
        pass


def _handle_control_message(message: dict) -> None:
    if bridge is None:
        return
    kind = message.get("type")
    payload = message.get("data", {})

    if kind == "drive":
        bridge.send_drive(payload.get("linear_x", 0.0), payload.get("angular_z", 0.0))
    elif kind == "point_turn":
        bridge.send_drive(0.0, payload.get("angular_z", 0.0))
    elif kind == "drive_mode":
        bridge.send_drive_mode(payload.get("mode", 0))
    elif kind == "arm":
        targets = payload.get("joint_target_steps", [0, 0, 0, 0, 0])
        bridge.send_arm(targets, payload.get("enable", True))
    elif kind == "mast":
        bridge.send_mast(
            payload.get("head_yaw_decideg", 0),
            payload.get("head_pitch_decideg", 0),
            payload.get("lift_mode", 0),
            payload.get("driver_enable", False),
        )
    elif kind == "antenna":
        bridge.send_antenna(
            payload.get("azimuth_decideg", 0),
            payload.get("elevation_decideg", 0),
            payload.get("driver_enable", False),
        )
    elif kind == "microscope":
        bridge.send_microscope(
            payload.get("focus_target_steps", 0),
            payload.get("led_pwm", 0),
            payload.get("cover_open", False),
            payload.get("driver_enable", False),
        )


# ---------------------------------------------------------- video streams ---
_MJPEG_BOUNDARY = "frame"


async def _mjpeg_stream(get_frame_fn, rate_hz: float = 12.0):
    period = 1.0 / rate_hz
    while True:
        frame = get_frame_fn()
        if frame is not None:
            yield (
                f"--{_MJPEG_BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(frame)}\r\n\r\n"
            ).encode("ascii") + frame + b"\r\n"
        await asyncio.sleep(period)


@app.get("/video/microscope")
async def video_microscope() -> StreamingResponse:
    def get_frame():
        return bridge.get_latest_microscope_jpeg() if bridge else None

    return StreamingResponse(
        _mjpeg_stream(get_frame),
        media_type=f"multipart/x-mixed-replace; boundary={_MJPEG_BOUNDARY}",
    )


@app.get("/video/main")
async def video_main() -> StreamingResponse:
    def get_frame():
        return bridge.get_latest_main_camera_jpeg() if bridge else None

    return StreamingResponse(
        _mjpeg_stream(get_frame),
        media_type=f"multipart/x-mixed-replace; boundary={_MJPEG_BOUNDARY}",
    )


# --------------------------------------------------- microscope endpoints ---
@app.post("/api/microscope/snapshot")
async def take_snapshot() -> Response:
    if bridge is None:
        return Response(status_code=503, content="ROS bridge not ready")
    loop = asyncio.get_event_loop()
    result = await loop.run_in_executor(None, bridge.call_snapshot)
    return Response(content=json.dumps(result), media_type="application/json")


@app.post("/api/microscope/recording/toggle")
async def toggle_recording() -> Response:
    if bridge is None:
        return Response(status_code=503, content="ROS bridge not ready")
    loop = asyncio.get_event_loop()
    result = await loop.run_in_executor(None, bridge.call_toggle_recording)
    return Response(content=json.dumps(result), media_type="application/json")


# --------------------------------------------------------- arm endpoints ---
@app.post("/api/arm/home/all")
async def home_all_joints() -> Response:
    if bridge is None:
        return Response(status_code=503, content="ROS bridge not ready")
    loop = asyncio.get_event_loop()
    result = await loop.run_in_executor(None, bridge.call_home_joint, -1)
    return Response(content=json.dumps(result), media_type="application/json")


@app.post("/api/arm/home/{joint_index}")
async def home_one_joint(joint_index: int) -> Response:
    if bridge is None:
        return Response(status_code=503, content="ROS bridge not ready")
    if joint_index < 0 or joint_index > 4:
        return Response(
            status_code=400,
            content=json.dumps({"accepted": False, "message": "joint_index must be 0-4"}),
            media_type="application/json",
        )
    loop = asyncio.get_event_loop()
    result = await loop.run_in_executor(None, bridge.call_home_joint, joint_index)
    return Response(content=json.dumps(result), media_type="application/json")


@app.post("/api/arm/preset/initial")
async def arm_preset_initial() -> Response:
    # UPGRADED: this endpoint used to send a hardcoded [0,0,0,0,0] via
    # a regular ArmCommand - a "go to zero" move, not actually "go to
    # a deliberately-chosen initial pose" (the two only coincided
    # because the preset itself was, at the time, also a placeholder
    # all-zero value). Now calls the firmware's own PRESET_INITIAL via
    # the new arm_preset service instead, so this button tracks
    # whatever arm_mega2.ino's own kInitialPoseSteps actually is, not
    # a value duplicated here. Same URL, same button in the web GUI -
    # only what happens underneath changed.
    if bridge is None:
        return Response(status_code=503, content="ROS bridge not ready")
    loop = asyncio.get_event_loop()
    result = await loop.run_in_executor(None, bridge.call_arm_preset, 0)
    return Response(content=json.dumps(result), media_type="application/json")


@app.post("/api/arm/preset/transport")
async def arm_preset_transport() -> Response:
    if bridge is None:
        return Response(status_code=503, content="ROS bridge not ready")
    loop = asyncio.get_event_loop()
    result = await loop.run_in_executor(None, bridge.call_arm_preset, 1)
    return Response(content=json.dumps(result), media_type="application/json")


@app.post("/api/arm/preset/service")
async def arm_preset_service() -> Response:
    if bridge is None:
        return Response(status_code=503, content="ROS bridge not ready")
    loop = asyncio.get_event_loop()
    result = await loop.run_in_executor(None, bridge.call_arm_preset, 2)
    return Response(content=json.dumps(result), media_type="application/json")


@app.post("/api/arm/estop/engage")
async def arm_estop_engage() -> Response:
    if bridge is None:
        return Response(status_code=503, content="ROS bridge not ready")
    loop = asyncio.get_event_loop()
    result = await loop.run_in_executor(None, bridge.call_emergency_stop, True)
    return Response(content=json.dumps(result), media_type="application/json")


@app.post("/api/arm/estop/clear")
async def arm_estop_clear() -> Response:
    if bridge is None:
        return Response(status_code=503, content="ROS bridge not ready")
    loop = asyncio.get_event_loop()
    result = await loop.run_in_executor(None, bridge.call_emergency_stop, False)
    return Response(content=json.dumps(result), media_type="application/json")
