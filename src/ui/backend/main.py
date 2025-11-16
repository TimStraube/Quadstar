import sys
import sys
import os
import asyncio
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../controller')))
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
    allow_origins=["http://localhost:8100"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"]
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
