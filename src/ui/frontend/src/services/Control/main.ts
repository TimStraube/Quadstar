// src/ui/frontend/src/services/Info/main.ts

export type MotorInfo = {
  speed: number;
  temperature: number;
};

export type StepMotorInfo = {
  position: number;
  temperature: number;
};

export type InfoValues = {
  motors: [MotorInfo, MotorInfo, MotorInfo, MotorInfo];
  stepMotors: [StepMotorInfo, StepMotorInfo];
  battery: number;
};

const API_URL = "http://localhost:8000";

export async function getInfoValues(): Promise<InfoValues> {
  const res = await fetch(`${API_URL}/infoValues`);
  return res.json();
}

export function listenInfoValues(onUpdate: (values: InfoValues) => void) {
  const ws = new WebSocket(`${API_URL.replace('http', 'ws')}/ws/info`);
  ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    onUpdate(data);
  };
  return ws;
}
