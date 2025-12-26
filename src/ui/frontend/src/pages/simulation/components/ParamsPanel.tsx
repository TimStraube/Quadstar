import React from 'react';
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
          <input type="number" step={0.01} value={String(mass)} onChange={(e) => setMass(Number(e.target.value || 0))} className="transparent-input" />

          <div>
            <label style={{ fontSize: 13 }}>Inertia (kg·m²)</label>
            <div style={{ display: 'flex', gap: 6, marginTop: 6 }}>
              <input title="Ixx" type="number" step={0.0001} value={String(inertia[0])} onChange={(e) => { const v = Number(e.target.value || 0); setInertia([v, inertia[1], inertia[2]]); }} className="transparent-input" style={{ width: 80 }} />
              <input title="Iyy" type="number" step={0.0001} value={String(inertia[1])} onChange={(e) => { const v = Number(e.target.value || 0); setInertia([inertia[0], v, inertia[2]]); }} className="transparent-input" style={{ width: 80 }} />
              <input title="Izz" type="number" step={0.0001} value={String(inertia[2])} onChange={(e) => { const v = Number(e.target.value || 0); setInertia([inertia[0], inertia[1], v]); }} className="transparent-input" style={{ width: 80 }} />
            </div>
          </div>

          <label style={{ fontSize: 13 }}>Arm length (m)</label>
          <input type="number" step={0.001} value={String(armLength)} onChange={(e) => setArmLength(Number(e.target.value || 0))} className="transparent-input" />

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
