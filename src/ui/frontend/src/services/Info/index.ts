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

const API_URL = "http://localhost:5001";

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

// Fetch current PID/controller gains from backend
export async function getPidValues(): Promise<any> {
  try {
    const res = await fetch(`${API_URL}/pid`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({})
    });
    if (!res.ok) throw new Error(`pid fetch failed ${res.status}`);
    return res.json();
  } catch (e) {
    console.warn('getPidValues error', e);
    return null;
  }
}

// Send PID/controller gains to backend and return canonical values
export async function setPidValues(payload: any): Promise<any> {
  try {
    const res = await fetch(`${API_URL}/pid`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    });
    if (!res.ok) throw new Error(`setPidValues failed ${res.status}`);
    return res.json();
  } catch (e) {
    console.warn('setPidValues error', e);
    return null;
  }
}

// Send waypoints list + tolerance to backend
export async function postWaypoints(payload: { waypoints: Array<{x:number,y:number,z:number}>, tolerance: number }): Promise<any> {
  try {
    const res = await fetch(`${API_URL}/waypoints`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    });
    if (!res.ok) throw new Error(`postWaypoints failed ${res.status}`);
    return res.json();
  } catch (e) {
    console.warn('postWaypoints error', e);
    return null;
  }
}
