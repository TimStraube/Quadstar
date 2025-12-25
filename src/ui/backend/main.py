import sys
import os
import asyncio
# Ensure the repository `src` directory is on sys.path so packages like
# `controller` and `simulation` can be imported as packages.
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, Response
import threading
import time
from fastapi.middleware.cors import CORSMiddleware
try:
    from util.joystick import Joystick
except ImportError:
    Joystick = None  # Fallback, damit das Backend startet


app = FastAPI()

# CORS für das Frontend erlauben
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Oder ["http://localhost:8100"]
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

joystick_instance = None
joystick_thread = None
joystick_data = {
    "thrust": 0,
    "roll": 0,
    "pitch": 0,
    "connected": False
}

# Hilfsfunktion, um die Werte regelmäßig zu aktualisieren
def poll_joystick():
    global joystick_instance, joystick_data
    while joystick_instance and joystick_instance.active:
        # Die Werte aus dem Joystick holen
        joystick_data["thrust"] = getattr(joystick_instance, "axis_thrust", 0)
        joystick_data["roll"] = getattr(joystick_instance, "roll", 0)
        joystick_data["pitch"] = getattr(joystick_instance, "pitch", 0)
        joystick_data["connected"] = joystick_instance.connected
        time.sleep(0.05)  # 20 Hz

@app.post("/connectJoystick")
def connect_joystick():
    global joystick_instance, joystick_thread
    if joystick_instance is None or not joystick_instance.active:
        joystick_instance = Joystick(mode={"operation"})
        joystick_thread = threading.Thread(target=joystick_instance.start)
        joystick_thread.daemon = True
        joystick_thread.start()
        # Polling-Thread starten
        poll_thread = threading.Thread(target=poll_joystick)
        poll_thread.daemon = True
        poll_thread.start()
        return JSONResponse(content={"status": "connected"})
    else:
        return JSONResponse(content={"status": "already connected"})

@app.post("/disconnectJoystick")
def disconnect_joystick():
    global joystick_instance
    if joystick_instance:
        joystick_instance.active = False
        joystick_instance = None
        return JSONResponse(content={"status": "disconnected"})
    return JSONResponse(content={"status": "not connected"})

@app.get("/joystickValues")
def get_joystick_values():
    return joystick_data

@app.websocket("/ws/joystick")
async def websocket_joystick(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(joystick_data)
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        pass

@app.post("/")
async def simulation_data():
    # Proxy to the simulation Testbench so frontend receives real simulated data
    try:
        # lazy import to avoid circular issues during startup
        from simulation.test import Testbench
    except Exception as e:
        # If the Testbench cannot be imported, return an error to the frontend
        return JSONResponse(status_code=500, content={
            "error": "failed to import Testbench",
            "detail": str(e)
        })

    # Create a global testbench instance if not present
    global _testbench_instance
    try:
        _testbench_instance
    except NameError:
        _testbench_instance = Testbench()

    # Run one simulation step and map values to frontend expected keys
    try:
        res = _testbench_instance.simulationStep()
        # Testbench.simulationStep returns (t, north, east, down, yaw, pitch, roll)
        t = res[0]
        north = res[1]
        east = res[2]
        down = res[3]
        # Map to roll, pitch, yaw expected by frontend
        yaw = res[4]
        pitch = res[5]
        roll = res[6]

        # include the current waypoint index so the frontend can highlight it
        current_idx = None
        try:
            current_idx = int(_testbench_instance.current_wp_index)
        except Exception:
            current_idx = None
        return JSONResponse(content={
            "t": t,
            "north": north,
            "east": east,
            "down": down,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "current_wp_index": current_idx
        })
        
    except Exception as e:
        # Return a 500 error instead of a zero-value fallback so the frontend
        # can surface the problem to the user and avoid silently masking bugs.
        return JSONResponse(status_code=500, content={
            "error": "simulation step failed",
            "detail": str(e)
        })


@app.options("/")
async def options_root():
    # Explicit OPTIONS handler as a fallback in case middleware doesn't catch preflight
    headers = {
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Methods": "POST, OPTIONS",
        "Access-Control-Allow-Headers": "Content-Type",
    }
    return Response(status_code=204, headers=headers)


@app.post("/waypoints")
async def set_waypoints(payload: dict):
    """Accepts JSON payload {"waypoints": [{"x":..,"y":..,"z":..}, ...]} and stores
    them in the running Testbench instance so the simulation will follow them.
    """
    try:
        wp_list = payload.get("waypoints", [])
    except Exception:
        return JSONResponse(status_code=400, content={"error": "invalid payload"})

    # ensure testbench instance exists
    global _testbench_instance
    try:
        _testbench_instance
    except NameError:
        try:
            from simulation.test import Testbench
            _testbench_instance = Testbench()
        except Exception as e:
            return JSONResponse(status_code=500, content={"error": "failed to create testbench", "detail": str(e)})

    # map payload into internal waypoint representation
    newwps = []
    for w in wp_list:
        try:
            x = float(w.get('x'))
            y = float(w.get('y'))
            z = float(w.get('z'))
            # Frontend sends z as altitude (up). Store altitude as-is in backend.
            # Testbench will convert altitude -> down when computing errors.
            newwps.append((x, y, z))
        except Exception:
            continue
    _testbench_instance.waypoints = newwps
    _testbench_instance.current_wp_index = 0
    # optional tolerance parameter (meters) from frontend
    try:
        tol = float(payload.get('tolerance'))
        # clamp to reasonable range 0..2
        tol = max(0.0, min(2.0, tol))
        _testbench_instance.wp_tolerance = tol
    except Exception:
        # if not provided, keep existing tolerance or default
        try:
            _testbench_instance.wp_tolerance = float(getattr(_testbench_instance, 'wp_tolerance', 0.2))
        except Exception:
            _testbench_instance.wp_tolerance = 0.2
    return JSONResponse(content={"status": "ok", "count": len(newwps)})


@app.options("/waypoints")
async def options_waypoints():
    headers = {
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Methods": "POST, OPTIONS",
        "Access-Control-Allow-Headers": "Content-Type",
    }
    return Response(status_code=204, headers=headers)


@app.post("/pid")
async def set_pid(payload: dict):
    """Accepts JSON payload to update PID parameters on the running Testbench.controller.
    Example payload keys (all optional):
      - vel_P_gain, vel_I_gain, vel_D_gain : list[3] or single number
      - attitute_p_gain : list[3] or single number
      - rate_P_gain, rate_D_gain : list[3] or single number
    The endpoint will create the Testbench instance if missing.
    """
    # Ensure testbench exists
    global _testbench_instance
    try:
        _testbench_instance
    except NameError:
        try:
            from simulation.test import Testbench
            _testbench_instance = Testbench()
        except Exception as e:
            return JSONResponse(status_code=500, content={"error": "failed to create testbench", "detail": str(e)})

    controller = getattr(_testbench_instance, 'controller', None)
    if controller is None:
        return JSONResponse(status_code=500, content={"error": "no controller present on testbench"})

    def apply_param(key, attr_name):
        if key not in payload:
            return
        val = payload.get(key)
        try:
            # If a single number provided, expand to 3
            if isinstance(val, (int, float)):
                arr = [float(val)] * 3
            else:
                arr = [float(x) for x in val]
            if len(arr) == 1:
                arr = arr * 3
            # sanitize: prevent zero or negative gains which can freeze sim
            min_gain = 1e-3
            arr = [max(float(x), min_gain) for x in arr]
            # set attribute if exists
            if hasattr(controller, attr_name):
                setattr(controller, attr_name, np.array(arr))
        except Exception:
            pass

    import numpy as np
    apply_param('vel_P_gain', 'vel_P_gain')
    apply_param('vel_I_gain', 'vel_I_gain')
    apply_param('vel_D_gain', 'vel_D_gain')
    apply_param('attitute_p_gain', 'attitute_p_gain')
    apply_param('rate_P_gain', 'rate_P_gain')
    apply_param('rate_D_gain', 'rate_D_gain')

    # Return current parameters for UI to show
    def to_list(x):
        try:
            return [float(v) for v in list(x)]
        except Exception:
            return None

    return JSONResponse(content={
        'vel_P_gain': to_list(getattr(controller, 'vel_P_gain', None)),
        'vel_I_gain': to_list(getattr(controller, 'vel_I_gain', None)),
        'vel_D_gain': to_list(getattr(controller, 'vel_D_gain', None)),
        'attitute_p_gain': to_list(getattr(controller, 'attitute_p_gain', None)),
        'rate_P_gain': to_list(getattr(controller, 'rate_P_gain', None)),
        'rate_D_gain': to_list(getattr(controller, 'rate_D_gain', None)),
    })


@app.options("/pid")
async def options_pid():
    headers = {
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Methods": "POST, OPTIONS",
        "Access-Control-Allow-Headers": "Content-Type",
    }
    return Response(status_code=204, headers=headers)


@app.get("/debug")
async def debug_state():
    """Return internal state useful for debugging the simulation & controller.
    This is a development-only endpoint.
    """
    try:
        from simulation.test import Testbench
    except Exception as e:
        return JSONResponse(status_code=500, content={"error": "failed to import Testbench", "detail": str(e)})

    global _testbench_instance
    try:
        _testbench_instance
    except NameError:
        _testbench_instance = Testbench()

    tb = _testbench_instance
    ctrl = getattr(tb, 'controller', None)
    quad = getattr(tb, 'quad', None)

    def to_list(x):
        try:
            return [float(v) for v in list(x)]
        except Exception:
            try:
                return list(map(float, x))
            except Exception:
                return None

    out = {
        'velocity_set': to_list(getattr(tb, 'velocity_set', [])),
        'wp_tolerance': float(getattr(tb, 'wp_tolerance', 0.2)),
        'current_wp_index': int(getattr(tb, 'current_wp_index', 0)),
    }
    if ctrl is not None:
        out.update({
            'controller.vel_P_gain': to_list(getattr(ctrl, 'vel_P_gain', [])),
            'controller.vel_I_gain': to_list(getattr(ctrl, 'vel_I_gain', [])),
            'controller.vel_D_gain': to_list(getattr(ctrl, 'vel_D_gain', [])),
            'controller.rate_set': to_list(getattr(ctrl, 'rate_set', [])),
            'controller.rate_sp': to_list(getattr(ctrl, 'rate_sp', [])),
            'controller.motor_commands': to_list(getattr(ctrl, 'motor_commands', [])),
            'controller.thrust_set': to_list(getattr(ctrl, 'thrust_set', [])),
        })
    if quad is not None:
        out.update({
            'quad.state': to_list(getattr(quad, 'state', [])),
            'quad.wMotor': to_list(getattr(quad, 'wMotor', [])),
            'quad.thr': to_list(getattr(quad, 'thr', [])),
        })

    return JSONResponse(content=out)

