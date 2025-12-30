import React, { useEffect, useState } from 'react';
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

  // local string state so inputs can be cleared by the user (empty string allowed)
  const [local, setLocal] = useState<{ north: string; east: string; alt: string }>({ north: String(singlePos.north), east: String(singlePos.east), alt: String(singlePos.alt) });

  // sync prop -> local when panel opens or singlePos changes
  useEffect(() => {
    setLocal({ north: String(singlePos.north), east: String(singlePos.east), alt: String(singlePos.alt) });
  }, [singlePos, isOpen]);

  // send the given position (or current singlePos if none provided) to backend and update the scene
  const onSetPosition = (posOverride?: { north: number; east: number; alt: number }) => {
    const pos = posOverride ?? singlePos;
    const payload = { north: Number(pos.north), east: Number(pos.east), alt: Number(pos.alt) };
    fetch('http://localhost:5001/set_position', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) })
      .then(r => r.json()).then(j => { console.log('set_position response', j); })
      .catch(e => console.error('set_position error', e));

    // Also update local model for immediate feedback if available
    try {
      const objs = simulationObjects.current;
      if (objs) {
        const alt = Number(pos.alt);
        if (objs.model) {
          objs.model.position.set(-Number(pos.east), alt, Number(pos.north));
        } else if (objs.cube1) {
          objs.cube1.position.set(-Number(pos.east), alt, Number(pos.north));
          objs.cube2.position.set(-Number(pos.east), alt, Number(pos.north));
          objs.cube3.position.set(-Number(pos.east), alt, Number(pos.north));
        }
        try { objs.renderer.render(objs.scene, objs.camera); } catch (e) { }
      }
    } catch (e) { }
  };

  // commit local inputs to singlePos but only for fields that are non-empty
  const commitLocal = () => {
    const prev = singlePos;
    const newPos = { ...prev };
    const nRaw = local.north.trim();
    const eRaw = local.east.trim();
    const aRaw = local.alt.trim();

    if (nRaw !== '') {
      const nNum = Number(nRaw);
      if (!Number.isNaN(nNum)) newPos.north = nNum;
    }
    if (eRaw !== '') {
      const eNum = Number(eRaw);
      if (!Number.isNaN(eNum)) newPos.east = eNum;
    }
    if (aRaw !== '') {
      const aNum = Number(aRaw);
      if (!Number.isNaN(aNum)) newPos.alt = Math.max(0, aNum);
    }

    setSinglePos(newPos);
    return newPos;
  };

  // commit current inputs and send to backend (used for Enter)
  const commitAndSend = () => {
    const newPos = commitLocal();
    onSetPosition(newPos);
  };

  return (
    <div className="panel-root">
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', cursor: 'pointer' }} onClick={() => setOpenPanel(openPanel === 'position' ? null : 'position')}>
        <div style={{ fontWeight: 700 }}>POSITION</div>
        <div className="panel-toggle" aria-hidden>{isOpen ? '▾' : '▸'}</div>
      </div>

      {isOpen && (
        <div style={{ marginTop: 8 }}>
          <div style={{ display: 'flex', gap: 6, marginBottom: 6 }}>
            <input
              type="number"
              step="0.1"
              placeholder="north"
              value={local.north}
              onChange={(e) => setLocal(l => ({ ...l, north: e.target.value }))}
              onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitAndSend(); } }}
              onBlur={() => {
                const raw = local.north.trim();
                if (raw !== '') {
                  const num = Number(raw);
                  if (!Number.isNaN(num)) setSinglePos({ ...singlePos, north: num });
                }
                setLocal(l => ({ ...l, north: raw }));
              }}
              className="transparent-input"
              style={{ width: '33%' }}
            />
            <input
              type="number"
              step="0.1"
              placeholder="east"
              value={local.east}
              onChange={(e) => setLocal(l => ({ ...l, east: e.target.value }))}
              onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitAndSend(); } }}
              onBlur={() => {
                const raw = local.east.trim();
                if (raw !== '') {
                  const num = Number(raw);
                  if (!Number.isNaN(num)) setSinglePos({ ...singlePos, east: num });
                }
                setLocal(l => ({ ...l, east: raw }));
              }}
              className="transparent-input"
              style={{ width: '33%' }}
            />
            <input
              type="number"
              min={0}
              step="0.1"
              placeholder="alt"
              value={local.alt}
              onChange={(e) => setLocal(l => ({ ...l, alt: e.target.value }))}
              onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitAndSend(); } }}
              onBlur={() => {
                const raw = local.alt.trim();
                if (raw !== '') {
                  const num = Number(raw);
                  if (!Number.isNaN(num)) setSinglePos({ ...singlePos, alt: Math.max(0, num) });
                }
                setLocal(l => ({ ...l, alt: raw }));
              }}
              className="transparent-input"
              style={{ width: '33%' }}
            />
          </div>
          <div style={{ display: 'flex', gap: 6 }}>
            <IonButton onClick={() => onSetPosition && onSetPosition()}>Set Quad Position</IonButton>
          </div>
        </div>
      )}
    </div>
  );
};

export default PositionPanel;
