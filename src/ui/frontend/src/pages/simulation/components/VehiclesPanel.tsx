import React, { useEffect, useState } from 'react';
import { IonButton } from '@ionic/react';

type Vehicle = { name: string; x: number; y: number; z: number };

interface Props {
  vehicles: Vehicle[];
  setVehicles: (vs: Vehicle[]) => void;
  openPanel: string | null;
  setOpenPanel: (s: string | null) => void;
}

const VehiclesPanel: React.FC<Props> = ({ vehicles, setVehicles, openPanel, setOpenPanel }) => {
  const [localVehicles, setLocalVehicles] = useState<string[][]>(vehicles.map(v => [v.name || 'veh', v.x.toFixed(2), v.y.toFixed(2), v.z.toFixed(2)]));

  useEffect(() => {
    setLocalVehicles(vehicles.map(v => [v.name || 'veh', v.x.toFixed(2), v.y.toFixed(2), v.z.toFixed(2)]));
  }, [vehicles, openPanel]);

  const commitField = (index: number, axis: 0 | 1 | 2 | 3) => {
    const raw = (localVehicles[index] && localVehicles[index][axis]) ? localVehicles[index][axis] : '';
    const trimmed = raw.trim();
    if (trimmed === '') {
      setLocalVehicles(l => {
        const c = [...l];
        c[index] = c[index].map((s, k) => {
          const arr = [vehicles[index].name, vehicles[index].x, vehicles[index].y, vehicles[index].z];
          const val = arr[k];
          return String(typeof val === 'number' ? (val as number).toFixed(2) : val);
        });
        return c;
      });
      return;
    }
    if (axis === 0) {
      const copy = [...vehicles];
      copy[index] = { name: trimmed, x: copy[index].x, y: copy[index].y, z: copy[index].z };
      setVehicles(copy);
      setLocalVehicles(l => { const c = [...l]; c[index][0] = trimmed; return c; });
      return;
    }
    const parsed = Number(trimmed);
    if (!isFinite(parsed)) {
      setLocalVehicles(l => {
        const c = [...l];
        c[index] = c[index].map((s, k) => {
          const arr = [vehicles[index].name, vehicles[index].x, vehicles[index].y, vehicles[index].z];
          const val = arr[k];
          return String(typeof val === 'number' ? (val as number).toFixed(2) : val);
        });
        return c;
      });
      return;
    }
    const copy = [...vehicles];
    if (axis === 1) copy[index] = { name: copy[index].name, x: parseFloat(parsed.toFixed(2)), y: copy[index].y, z: copy[index].z };
    if (axis === 2) copy[index] = { name: copy[index].name, x: copy[index].x, y: parseFloat(parsed.toFixed(2)), z: copy[index].z };
    if (axis === 3) {
      let v = Math.max(0, parsed);
      v = Math.min(100, v);
      copy[index] = { name: copy[index].name, x: copy[index].x, y: copy[index].y, z: parseFloat(v.toFixed(2)) };
    }
    setVehicles(copy);
    setLocalVehicles(l => {
      const c = [...l];
      c[index] = c[index].map((s, k) => {
        const arr = [copy[index].name, copy[index].x, copy[index].y, copy[index].z];
        const val = arr[k];
        return String(typeof val === 'number' ? (val as number).toFixed(2) : val);
      });
      return c;
    });
  };

  return (
    <div className="panel-root">
      <div style={{display:'flex', alignItems:'center', justifyContent:'space-between', marginBottom:6, cursor:'pointer'}} onClick={() => setOpenPanel(openPanel === 'vehicles' ? null : 'vehicles')}>
        <div style={{display:'flex', gap:8, alignItems:'center'}}>
          <div style={{fontWeight:700}}>VEHICLES</div>
        </div>
        <div style={{display: 'flex', gap: 6, alignItems: 'center'}}>
          <div className="panel-toggle" aria-hidden>{openPanel === 'vehicles' ? '▾' : '▸'}</div>
        </div>
      </div>

      {openPanel === 'vehicles' && (
        <>
          {vehicles.map((v, i) => (
            <div key={i} style={{display: 'flex', gap: 6, marginBottom: 6, alignItems: 'center'}}>
              <input
                type="text"
                placeholder="name"
                value={localVehicles[i] ? localVehicles[i][0] : (v.name || '')}
                onChange={(e) => { const raw = e.target.value; setLocalVehicles(l => { const c = [...l]; if (!c[i]) c[i] = [v.name, String(v.x), String(v.y), String(v.z)]; c[i][0] = raw; return c; }); }}
                onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitField(i, 0); } }}
                onBlur={() => commitField(i, 0)}
                className="transparent-input"
                style={{width: '28%'}}
              />
              <input
                type="number"
                step="0.1"
                placeholder="north"
                value={localVehicles[i] ? localVehicles[i][1] : String(v.x.toFixed(2))}
                onChange={(e) => { const raw = e.target.value; setLocalVehicles(l => { const c = [...l]; if (!c[i]) c[i] = [v.name, String(v.x), String(v.y), String(v.z)]; c[i][1] = raw; return c; }); }}
                onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitField(i, 1); } }}
                onBlur={() => commitField(i, 1)}
                className="transparent-input"
                style={{width: '22%'}}
              />
              <input
                type="number"
                step="0.1"
                placeholder="east"
                value={localVehicles[i] ? localVehicles[i][2] : String(v.y.toFixed(2))}
                onChange={(e) => { const raw = e.target.value; setLocalVehicles(l => { const c = [...l]; if (!c[i]) c[i] = [v.name, String(v.x), String(v.y), String(v.z)]; c[i][2] = raw; return c; }); }}
                onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitField(i, 2); } }}
                onBlur={() => commitField(i, 2)}
                className="transparent-input"
                style={{width: '22%'}}
              />
              <input
                type="number"
                min={0}
                max={100}
                step="0.1"
                placeholder="alt"
                value={localVehicles[i] ? localVehicles[i][3] : String(v.z.toFixed(2))}
                onChange={(e) => { const raw = e.target.value; setLocalVehicles(l => { const c = [...l]; if (!c[i]) c[i] = [v.name, String(v.x), String(v.y), String(v.z)]; c[i][3] = raw; return c; }); }}
                onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitField(i, 3); } }}
                onBlur={() => commitField(i, 3)}
                className="transparent-input"
                style={{width: '22%'}}
              />
              <button title="Remove vehicle" onClick={() => {
                const copy = [...vehicles];
                copy.splice(i, 1);
                setVehicles(copy);
                setLocalVehicles(copy.map(w => [w.name || '', w.x.toFixed(2), w.y.toFixed(2), w.z.toFixed(2)]));
              }} style={{background: 'transparent', border: 'none', color: '#c00', fontSize: 18, lineHeight: '18px', padding: '2px 6px', cursor: 'pointer'}}>✖</button>
            </div>
          ))}

          <div style={{display:'flex', gap:6, marginTop:6}}>
            <IonButton onClick={() => {
              const copy = [...vehicles];
              const last = copy.length > 0 ? copy[copy.length - 1] : { name: 'veh', x: 0, y: 0, z: 0 };
              const nw = { name: last.name || 'veh', x: parseFloat(last.x.toFixed(2)), y: parseFloat(last.y.toFixed(2)), z: parseFloat(last.z.toFixed(2)) };
              copy.push(nw);
              setVehicles(copy);
              setLocalVehicles(copy.map(w => [w.name || '', w.x.toFixed(2), w.y.toFixed(2), w.z.toFixed(2)]));
            }}>
              Add Vehicle
            </IonButton>
          </div>
        </>
      )}
    </div>
  );
};

export default VehiclesPanel;
