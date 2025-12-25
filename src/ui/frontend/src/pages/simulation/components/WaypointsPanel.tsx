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
    <div style={{
      position: 'fixed',
      top: 12,
      left: 12,
      zIndex: 250,
      width: 320,
      maxWidth: '90vw',
      background: 'rgba(0,0,0,0.5)',
      color: 'white',
      padding: 8,
      borderRadius: 8,
      pointerEvents: 'auto'
    }}>
      <div style={{display:'flex', alignItems:'center', justifyContent:'space-between', marginBottom:6}}>
        <div style={{display:'flex', gap:8, alignItems:'center'}}>
          <div style={{fontWeight:700}}>Waypoints (north, east, alt)</div>
        </div>
        <div>
          <button onClick={() => setOpenPanel(op => op === 'waypoints' ? null : 'waypoints')} style={{background: 'transparent', border: 'none', color: 'white', cursor: 'pointer'}}>{openPanel === 'waypoints' ? '▾' : '▸'}</button>
        </div>
      </div>
      {openPanel === 'waypoints' && <>
        {waypoints.map((wp, i) => (
          <div key={i} style={{display: 'flex', gap: 6, marginBottom: 6, alignItems: 'center'}}>
            <input type="number" step="0.1" placeholder="north" value={typeof wp.x === 'number' ? wp.x.toFixed(2) : wp.x} onChange={(e)=>{
              const v = e.target.value === '' ? 0 : Number(e.target.value);
              const copy = [...waypoints]; copy[i] = {x:v,y:copy[i].y,z:copy[i].z}; setWaypoints(copy);
            }} style={{width: '32%', background: '#ffffff', color: '#000000', padding: '4px', borderRadius: 4, border: '1px solid rgba(0,0,0,0.2)'}} />
            <input type="number" step="0.1" placeholder="east" value={typeof wp.y === 'number' ? wp.y.toFixed(2) : wp.y} onChange={(e)=>{
              const v = e.target.value === '' ? 0 : Number(e.target.value);
              const copy = [...waypoints]; copy[i] = {x:copy[i].x,y:v,z:copy[i].z}; setWaypoints(copy);
            }} style={{width: '32%', background: '#ffffff', color: '#000000', padding: '4px', borderRadius: 4, border: '1px solid rgba(0,0,0,0.2)'}} />
            <input type="number" min={0} max={3} step="0.1" placeholder="alt" value={typeof wp.z === 'number' ? wp.z.toFixed(2) : wp.z} onChange={(e)=>{
              const raw = e.target.value;
              const num = raw === '' ? 0 : Number(raw);
              let v = Number.isNaN(num) ? 0 : Math.max(0, num);
              v = Math.min(3, v);
              const copy = [...waypoints]; copy[i] = {x:copy[i].x,y:copy[i].y,z:v}; setWaypoints(copy);
            }} style={{width: '32%', background: '#ffffff', color: '#000000', padding: '4px', borderRadius: 4, border: '1px solid rgba(0,0,0,0.2)'}} />
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

      {/* Single-position setter (immer sichtbar) */}
      <div style={{marginTop:8, borderTop: '1px solid rgba(255,255,255,0.08)', paddingTop:8}}>
        <div style={{fontWeight:600, fontSize:13, marginBottom:6}}>Setze Quad-Position (einmalig)</div>
        <div style={{display:'flex', gap:6, marginBottom:6}}>
          <input type="number" step="0.1" placeholder="north" value={singlePos.north} onChange={(e)=>{ const v = e.target.value === '' ? 0 : Number(e.target.value); setSinglePos(p=>({...p, north:v})); }} style={{width:'33%', background: '#ffffff', color: '#000000', padding: '6px', borderRadius:4, border:'1px solid rgba(0,0,0,0.2)'}} />
          <input type="number" step="0.1" placeholder="east" value={singlePos.east} onChange={(e)=>{ const v = e.target.value === '' ? 0 : Number(e.target.value); setSinglePos(p=>({...p, east:v})); }} style={{width:'33%', background: '#ffffff', color: '#000000', padding: '6px', borderRadius:4, border:'1px solid rgba(0,0,0,0.2)'}} />
          <input type="number" min={0} step="0.1" placeholder="alt" value={singlePos.alt} onChange={(e)=>{ const raw = e.target.value; const num = raw === '' ? 0 : Number(raw); const v = Number.isNaN(num) ? 0 : Math.max(0, num); setSinglePos(p=>({...p, alt:v})); }} style={{width:'33%', background: '#ffffff', color: '#000000', padding: '6px', borderRadius:4, border:'1px solid rgba(0,0,0,0.2)'}} />
        </div>
        <div style={{display:'flex', gap:6}}>
          <IonButton onClick={() => {
            // Send to backend
            const payload = { north: Number(singlePos.north), east: Number(singlePos.east), alt: Number(singlePos.alt) };
            fetch('http://localhost:5001/set_position', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(payload) })
              .then(r => r.json()).then(j => { console.log('set_position response', j); })
              .catch(e => console.error('set_position error', e));
            // Also update local model for immediate feedback if available
            try {
              const objs = simulationObjects.current;
              if (objs) {
                const alt = Number(singlePos.alt);
                if (objs.model) {
                  objs.model.position.set(-Number(singlePos.east), alt, Number(singlePos.north));
                } else if (objs.cube1) {
                  objs.cube1.position.set(-Number(singlePos.east), alt, Number(singlePos.north));
                  objs.cube2.position.set(-Number(singlePos.east), alt, Number(singlePos.north));
                  objs.cube3.position.set(-Number(singlePos.east), alt, Number(singlePos.north));
                }
                try { objs.renderer.render(objs.scene, objs.camera); } catch(e){}
              }
            } catch(e){}
          }}>Set Quad Position</IonButton>
        </div>
      </div>
    </div>
  );
};

export default WaypointsPanel;
