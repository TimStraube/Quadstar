import React from 'react';
import { IonButton } from '@ionic/react';

type Waypoint = { x: number; y: number; z: number };

interface Props {
  waypoints: Waypoint[];
  setWaypoints: (wps: Waypoint[]) => void
  wpTolerance: number;
  setWpTolerance: (v: number) => void;
  openPanel: string | null;
  setOpenPanel: (s: string | null) => void;
  activeWaypoint: number | null;
  setActiveWaypoint: (i: number | null) => void;
  singlePos: { north: number; east: number; alt: number };
  setSinglePos: (p: { north: number; east: number; alt: number }) => void;
  simulationObjects: React.MutableRefObject<any>;
}

const WaypointsPanel: React.FC<Props> = ({ waypoints, setWaypoints, wpTolerance, setWpTolerance, openPanel, setOpenPanel, activeWaypoint, setActiveWaypoint, singlePos, setSinglePos, simulationObjects }) => {
  return (
    <div className="panel-root">
      <div style={{display:'flex', alignItems:'center', justifyContent:'space-between', marginBottom:6, cursor:'pointer'}} onClick={() => setOpenPanel(op => op === 'waypoints' ? null : 'waypoints')}>
        <div style={{display:'flex', gap:8, alignItems:'center'}}>
          <div style={{fontWeight:700}}>WAYPOINTS</div>
        </div>
        <div style={{display: 'flex', gap: 6, alignItems: 'center'}}>
          <div className="panel-toggle" aria-hidden>{openPanel === 'waypoints' ? '▾' : '▸'}</div>
        </div>
      </div>
      {openPanel === 'waypoints' && <>
        {waypoints.map((wp, i) => (
          <div key={i} style={{display: 'flex', gap: 6, marginBottom: 6, alignItems: 'center'}}>
            <input type="number" step="0.1" placeholder="north" value={typeof wp.x === 'number' ? wp.x.toFixed(2) : wp.x} onChange={(e)=>{
              const v = e.target.value === '' ? 0 : Number(e.target.value);
              const copy = [...waypoints]; copy[i] = {x:v,y:copy[i].y,z:copy[i].z}; setWaypoints(copy);
            }} className="transparent-input" style={{width: '32%'}} />
            <input type="number" step="0.1" placeholder="east" value={typeof wp.y === 'number' ? wp.y.toFixed(2) : wp.y} onChange={(e)=>{
              const v = e.target.value === '' ? 0 : Number(e.target.value);
              const copy = [...waypoints]; copy[i] = {x:copy[i].x,y:v,z:copy[i].z}; setWaypoints(copy);
            }} className="transparent-input" style={{width: '32%'}} />
            <input type="number" min={0} max={3} step="0.1" placeholder="alt" value={typeof wp.z === 'number' ? wp.z.toFixed(2) : wp.z} onChange={(e)=>{
              const raw = e.target.value;
              const num = raw === '' ? 0 : Number(raw);
              let v = Number.isNaN(num) ? 0 : Math.max(0, num);
              v = Math.min(3, v);
              const copy = [...waypoints]; copy[i] = {x:copy[i].x,y:copy[i].y,z:v}; setWaypoints(copy);
            }} className="transparent-input" style={{width: '32%'}} />
          </div>
        ))}
        <div style={{display:'flex', gap:6, marginTop:6}}>
          <IonButton onClick={() => {
            const newwps = Array.from({length:5}, () => ({x: (Math.random() * 6) - 3, y: (Math.random() * 6) - 3, z: (Math.random() * 3)}));
            setWaypoints(newwps);
          }}>
            Regenerate Waypoints
          </IonButton>
        </div>
      </>}

      {/* Single-position setter (toggleable) */}
      {/* Position setter moved to its own panel (PositionPanel) */}
    </div>
  );
};

export default WaypointsPanel;
