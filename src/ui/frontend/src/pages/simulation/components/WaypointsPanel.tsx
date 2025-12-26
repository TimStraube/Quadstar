import React, { useEffect, useState } from 'react';
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
  // local string state for waypoint inputs so users can clear without becoming 0
  const [localWps, setLocalWps] = useState<string[][]>(waypoints.map(w => [w.x.toFixed(2), w.y.toFixed(2), w.z.toFixed(2)]));

  useEffect(() => {
    setLocalWps(waypoints.map(w => [w.x.toFixed(2), w.y.toFixed(2), w.z.toFixed(2)]));
  }, [waypoints, openPanel]);

  const commitField = (index: number, axis: 0 | 1 | 2) => {
    const raw = (localWps[index] && localWps[index][axis]) ? localWps[index][axis] : '';
    const trimmed = raw.trim();
    if (trimmed === '') {
      // don't overwrite if empty; reset local to formatted current wp value
      setLocalWps(l => { const c = [...l]; c[index] = c[index].map((s, k) => String([waypoints[index].x, waypoints[index].y, waypoints[index].z][k].toFixed(2))); return c; });
      return;
    }

    const parsed = Number(trimmed);
    if (!isFinite(parsed)) {
      // invalid number: revert to formatted current value
      setLocalWps(l => { const c = [...l]; c[index] = c[index].map((s, k) => String([waypoints[index].x, waypoints[index].y, waypoints[index].z][k].toFixed(2))); return c; });
      return;
    }

    const copy = [...waypoints];
    if (axis === 0) copy[index] = { x: parseFloat(parsed.toFixed(2)), y: copy[index].y, z: copy[index].z };
    if (axis === 1) copy[index] = { x: copy[index].x, y: parseFloat(parsed.toFixed(2)), z: copy[index].z };
    if (axis === 2) {
      let v = Math.max(0, parsed);
      v = Math.min(3, v);
      copy[index] = { x: copy[index].x, y: copy[index].y, z: parseFloat(v.toFixed(2)) };
    }
    setWaypoints(copy);
    // reflect formatted values in local state
    setLocalWps(l => { const c = [...l]; c[index] = c[index].map((s, k) => String([copy[index].x, copy[index].y, copy[index].z][k].toFixed(2))); return c; });
  };

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

      {openPanel === 'waypoints' && (
        <>
          {waypoints.map((wp, i) => (
            <div key={i} style={{display: 'flex', gap: 6, marginBottom: 6, alignItems: 'center'}}>
              <input
                type="number"
                step="0.1"
                placeholder="north"
                value={localWps[i] ? localWps[i][0] : String(wp.x.toFixed(2))}
                onChange={(e) => {
                  const raw = e.target.value;
                  setLocalWps(l => { const c = [...l]; if (!c[i]) c[i] = [String(wp.x), String(wp.y), String(wp.z)]; c[i][0] = raw; return c; });
                }}
                onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitField(i, 0); } }}
                onBlur={() => commitField(i, 0)}
                className="transparent-input"
                style={{width: '32%'}}
              />
              <input
                type="number"
                step="0.1"
                placeholder="east"
                value={localWps[i] ? localWps[i][1] : String(wp.y.toFixed(2))}
                onChange={(e) => {
                  const raw = e.target.value;
                  setLocalWps(l => { const c = [...l]; if (!c[i]) c[i] = [String(wp.x), String(wp.y), String(wp.z)]; c[i][1] = raw; return c; });
                }}
                onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitField(i, 1); } }}
                onBlur={() => commitField(i, 1)}
                className="transparent-input"
                style={{width: '32%'}}
              />
              <input
                type="number"
                min={0}
                max={3}
                step="0.1"
                placeholder="alt"
                value={localWps[i] ? localWps[i][2] : String(wp.z.toFixed(2))}
                onChange={(e) => {
                  const raw = e.target.value;
                  setLocalWps(l => { const c = [...l]; if (!c[i]) c[i] = [String(wp.x), String(wp.y), String(wp.z)]; c[i][2] = raw; return c; });
                }}
                onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitField(i, 2); } }}
                onBlur={() => commitField(i, 2)}
                className="transparent-input"
                style={{width: '32%'}}
              />
              <button title="Remove waypoint" onClick={() => {
                const copy = [...waypoints];
                copy.splice(i, 1);
                setWaypoints(copy);
                setLocalWps(copy.map(w => [w.x.toFixed(2), w.y.toFixed(2), w.z.toFixed(2)]));
              }} style={{background: 'transparent', border: 'none', color: '#c00', fontSize: 18, lineHeight: '18px', padding: '2px 6px', cursor: 'pointer'}}>✖</button>
            </div>
          ))}

          <div style={{display:'flex', gap:6, marginTop:6}}>
            <IonButton onClick={() => {
              const newwps = Array.from({length:5}, () => {
                const rx = (Math.random() * 6) - 3;
                const ry = (Math.random() * 6) - 3;
                const rz = (Math.random() * 3);
                return { x: parseFloat(rx.toFixed(2)), y: parseFloat(ry.toFixed(2)), z: parseFloat(rz.toFixed(2)) };
              });
              setWaypoints(newwps);
              // also update local immediately so UI reflects two-decimal values
              setLocalWps(newwps.map(w => [w.x.toFixed(2), w.y.toFixed(2), w.z.toFixed(2)]));
            }}>
              Regenerate Waypoints
            </IonButton>
            <IonButton onClick={() => {
              // add one waypoint (copy last if available)
              const copy = [...waypoints];
              const last = copy.length > 0 ? copy[copy.length - 1] : { x: 0, y: 0, z: 0 };
              const nw = { x: parseFloat(last.x.toFixed(2)), y: parseFloat(last.y.toFixed(2)), z: parseFloat(last.z.toFixed(2)) };
              copy.push(nw);
              setWaypoints(copy);
              setLocalWps(copy.map(w => [w.x.toFixed(2), w.y.toFixed(2), w.z.toFixed(2)]));
            }}>
              Add Waypoint
            </IonButton>
          </div>
        </>
      )}
    </div>
  );
};

export default WaypointsPanel;
