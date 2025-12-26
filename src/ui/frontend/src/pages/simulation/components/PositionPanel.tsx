import React from 'react';
import { IonButton } from '@ionic/react';

type Props = {
  singlePos: { north: number; east: number; alt: number };
  setSinglePos: (p: { north: number; east: number; alt: number }) => void;
  simulationObjects: React.MutableRefObject<any>;
  openPanel: string | null;
  setOpenPanel: (s: string | null) => void;
};

const PositionPanel: React.FC<Props> = ({ singlePos, setSinglePos, simulationObjects, openPanel, setOpenPanel }) => {
  const isOpen = openPanel === 'position';

  const onSetPosition = () => {
    const payload = { north: Number(singlePos.north), east: Number(singlePos.east), alt: Number(singlePos.alt) };
    fetch('http://localhost:5001/set_position', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) })
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
        try { objs.renderer.render(objs.scene, objs.camera); } catch (e) { }
      }
    } catch (e) { }
  };

  return (
    <div className="panel-root">
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
        <div style={{ fontWeight: 700 }}>Set Position</div>
        <button className="panel-toggle" onClick={() => setOpenPanel(op => op === 'position' ? null : 'position')}>{isOpen ? '▾' : '▸'}</button>
      </div>

      {isOpen && (
        <div style={{ marginTop: 8 }}>
          <div style={{ display: 'flex', gap: 6, marginBottom: 6 }}>
            <input type="number" step="0.1" placeholder="north" value={singlePos.north} onChange={(e) => { const v = e.target.value === '' ? 0 : Number(e.target.value); setSinglePos(p => ({ ...p, north: v })); }} className="transparent-input" style={{ width: '33%' }} />
            <input type="number" step="0.1" placeholder="east" value={singlePos.east} onChange={(e) => { const v = e.target.value === '' ? 0 : Number(e.target.value); setSinglePos(p => ({ ...p, east: v })); }} className="transparent-input" style={{ width: '33%' }} />
            <input type="number" min={0} step="0.1" placeholder="alt" value={singlePos.alt} onChange={(e) => { const raw = e.target.value; const num = raw === '' ? 0 : Number(raw); const v = Number.isNaN(num) ? 0 : Math.max(0, num); setSinglePos(p => ({ ...p, alt: v })); }} className="transparent-input" style={{ width: '33%' }} />
          </div>
          <div style={{ display: 'flex', gap: 6 }}>
            <IonButton onClick={onSetPosition}>Set Quad Position</IonButton>
          </div>
        </div>
      )}
    </div>
  );
};

export default PositionPanel;
