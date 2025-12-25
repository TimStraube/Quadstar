import sys
import os
import asyncio
# Ensure the repository `src` directory is on sys.path so packages like
# `controller` and `simulation` can be imported as packages.
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
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

        return JSONResponse(content={
            "t": t,
            "north": north,
            "east": east,
            "down": down,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw
        })
    except Exception as e:
        # Return a 500 error instead of a zero-value fallback so the frontend
        # can surface the problem to the user and avoid silently masking bugs.
        return JSONResponse(status_code=500, content={
            "error": "simulation step failed",
            "detail": str(e)
        })

