import React, { useEffect, useState } from 'react';
import { IonButton } from '@ionic/react';
import { postQuadParams } from '../../../services/Info';

type Props = {
  mass: number;
  setMass: (v: number) => void;
  inertia: number[]; // [Ixx, Iyy, Izz]
  setInertia: (v: number[]) => void;
  armLength: number;
  setArmLength: (v: number) => void;
  openPanel: string | null;
  setOpenPanel: (s: string | null) => void;
  onApplyScene?: () => void;
};

const ParamsPanel: React.FC<Props> = ({ mass, setMass, inertia, setInertia, armLength, setArmLength, openPanel, setOpenPanel, onApplyScene }) => {
  const isOpen = openPanel === 'params';

  // local string state so inputs can be cleared without immediately becoming 0
  const [localMass, setLocalMass] = useState<string>(String(mass));
  const [localInertia, setLocalInertia] = useState<string[]>([String(inertia[0]), String(inertia[1]), String(inertia[2])]);
  const [localArm, setLocalArm] = useState<string>(String(armLength));

  // sync prop -> local when panel opens or props change
  useEffect(() => {
    setLocalMass(String(mass));
    setLocalInertia([String(inertia[0]), String(inertia[1]), String(inertia[2])]);
    setLocalArm(String(armLength));
  }, [mass, inertia, armLength, isOpen]);

  // commit only non-empty fields from local to props
  const commitLocal = () => {
    if (localMass.trim() !== '') {
      const m = Number(localMass);
      if (!Number.isNaN(m)) setMass(m);
    }

    const newInertia = [...inertia];
    localInertia.forEach((raw, idx) => {
      if (raw.trim() !== '') {
        const v = Number(raw);
        if (!Number.isNaN(v)) newInertia[idx] = v;
      }
    });
    setInertia(newInertia);

    if (localArm.trim() !== '') {
      const a = Number(localArm);
      if (!Number.isNaN(a)) setArmLength(a);
    }
  };

  const applyBackend = async () => {
    const payload = { mass, inertia, arm_length: armLength };
    try {
      const res = await postQuadParams(payload);
      console.log('postQuadParams result', res);
    } catch (e) {
      console.error('failed to post quad params', e);
    }
  };

  return (
    <div className="panel-root">
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: 8, cursor: 'pointer' }} onClick={() => setOpenPanel(op => op === 'params' ? null : 'params')}>
        <div style={{ fontWeight: 700 }}>PARAMETERS</div>
        <div className="panel-toggle" aria-hidden>{isOpen ? '▾' : '▸'}</div>
      </div>

      {isOpen && (
        <div style={{ marginTop: 8, display: 'flex', flexDirection: 'column', gap: 8 }}>
          <label style={{ fontSize: 13 }}>Mass (kg)</label>
          <input
            type="number"
            step={0.01}
            placeholder="mass"
            value={localMass}
            onChange={(e) => setLocalMass(e.target.value)}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitLocal(); } }}
            onBlur={() => { if (localMass.trim() !== '') { const m = Number(localMass); if (!Number.isNaN(m)) setMass(m); } setLocalMass(localMass.trim()); }}
            className="transparent-input"
          />

          <div>
            <label style={{ fontSize: 13 }}>Inertia (kg·m²)</label>
            <div style={{ display: 'flex', gap: 6, marginTop: 6 }}>
              <input
                title="Ixx"
                type="number"
                step={0.0001}
                placeholder="Ixx"
                value={localInertia[0]}
                onChange={(e) => setLocalInertia(li => { const copy = [...li]; copy[0] = e.target.value; return copy; })}
                onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitLocal(); } }}
                onBlur={() => { const raw = localInertia[0].trim(); if (raw !== '') { const v = Number(raw); if (!Number.isNaN(v)) setInertia([v, inertia[1], inertia[2]]); } setLocalInertia(li => { const c = [...li]; c[0] = raw; return c; }); }}
                className="transparent-input"
                style={{ width: 80 }}
              />
              <input
                title="Iyy"
                type="number"
                step={0.0001}
                placeholder="Iyy"
                value={localInertia[1]}
                onChange={(e) => setLocalInertia(li => { const copy = [...li]; copy[1] = e.target.value; return copy; })}
                onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitLocal(); } }}
                onBlur={() => { const raw = localInertia[1].trim(); if (raw !== '') { const v = Number(raw); if (!Number.isNaN(v)) setInertia([inertia[0], v, inertia[2]]); } setLocalInertia(li => { const c = [...li]; c[1] = raw; return c; }); }}
                className="transparent-input"
                style={{ width: 80 }}
              />
              <input
                title="Izz"
                type="number"
                step={0.0001}
                placeholder="Izz"
                value={localInertia[2]}
                onChange={(e) => setLocalInertia(li => { const copy = [...li]; copy[2] = e.target.value; return copy; })}
                onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitLocal(); } }}
                onBlur={() => { const raw = localInertia[2].trim(); if (raw !== '') { const v = Number(raw); if (!Number.isNaN(v)) setInertia([inertia[0], inertia[1], v]); } setLocalInertia(li => { const c = [...li]; c[2] = raw; return c; }); }}
                className="transparent-input"
                style={{ width: 80 }}
              />
            </div>
          </div>

          <label style={{ fontSize: 13 }}>Arm length (m)</label>
          <input
            type="number"
            step={0.001}
            placeholder="arm length"
            value={localArm}
            onChange={(e) => setLocalArm(e.target.value)}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitLocal(); } }}
            onBlur={() => { if (localArm.trim() !== '') { const a = Number(localArm); if (!Number.isNaN(a)) setArmLength(a); } setLocalArm(localArm.trim()); }}
            className="transparent-input"
          />

          <div style={{ display: 'flex', gap: 8, justifyContent: 'flex-end', marginTop: 6 }}>
            <IonButton size="small" onClick={() => { setOpenPanel(null); }}>Done</IonButton>
            <IonButton size="small" onClick={() => { if (onApplyScene) onApplyScene(); }}>
              Apply to Scene
            </IonButton>
            <IonButton size="small" onClick={applyBackend} color="tertiary">Apply to Backend</IonButton>
          </div>
        </div>
      )}
    </div>
  );
};

export default ParamsPanel;
