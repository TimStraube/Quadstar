// src/ui/frontend/src/services/Info/index.ts

export type Quaternion = {
  w: number;
  x: number;
  y: number;
  z: number;
};

export type InfoValues = {
  quaternion: Quaternion;
  speed: number;
};

const API_URL = "http://localhost:5000";

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
