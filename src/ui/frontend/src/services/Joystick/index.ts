// src/ui/frontend/src/services/Joystick/index.ts

export type JoystickValues = {
  thrust: number;
  roll: number;
  pitch: number;
  connected: boolean;
};

const API_URL = "http://localhost:8000";

export async function connectJoystick() {
  const res = await fetch(`${API_URL}/connectJoystick`, { method: "POST" });
  return res.json();
}

export async function disconnectJoystick() {
  const res = await fetch(`${API_URL}/disconnectJoystick`, { method: "POST" });
  return res.json();
}

export async function getJoystickValues(): Promise<JoystickValues> {
  const res = await fetch(`${API_URL}/joystickValues`);
  return res.json();
}

export function listenJoystickValues(onUpdate: (values: JoystickValues) => void) {
  const ws = new WebSocket(`${API_URL.replace('http', 'ws')}/ws/joystick`);
  ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    onUpdate(data);
  };
  return ws;
}
