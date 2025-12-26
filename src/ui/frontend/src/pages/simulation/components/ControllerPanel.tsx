import React, { useEffect, useState } from 'react';

interface Props {
  velP: number[];
  setVelP: React.Dispatch<React.SetStateAction<number[]>>;
  velI: number[];
  setVelI: React.Dispatch<React.SetStateAction<number[]>>;
  velD: number[];
  setVelD: React.Dispatch<React.SetStateAction<number[]>>;
  attP: number[];
  setAttP: React.Dispatch<React.SetStateAction<number[]>>;
  rateP: number[];
  setRateP: React.Dispatch<React.SetStateAction<number[]>>;
  rateD: number[];
  setRateD: React.Dispatch<React.SetStateAction<number[]>>;
  openPanel: string | null;
  setOpenPanel: React.Dispatch<React.SetStateAction<string | null>>;
  schedulePidSend: (payload: any) => void;
}

const ControllerPanel: React.FC<Props> = ({ velP, setVelP, velI, setVelI, velD, setVelD, attP, setAttP, rateP, setRateP, rateD, setRateD, openPanel, setOpenPanel, schedulePidSend }) => {
  // local string states so inputs can be cleared without immediately becoming 0
  const [localVelP, setLocalVelP] = useState<string[]>([String(velP[0]), String(velP[1]), String(velP[2])]);
  const [localVelI, setLocalVelI] = useState<string[]>([String(velI[0]), String(velI[1]), String(velI[2])]);
  const [localVelD, setLocalVelD] = useState<string[]>([String(velD[0]), String(velD[1]), String(velD[2])]);
  const [localAttP, setLocalAttP] = useState<string[]>([String(attP[0]), String(attP[1]), String(attP[2])]);
  const [localRateP, setLocalRateP] = useState<string[]>([String(rateP[0]), String(rateP[1]), String(rateP[2])]);
  const [localRateD, setLocalRateD] = useState<string[]>([String(rateD[0]), String(rateD[1]), String(rateD[2])]);

  useEffect(() => {
    if (openPanel === 'controller') {
      setLocalVelP([String(velP[0]), String(velP[1]), String(velP[2])]);
      setLocalVelI([String(velI[0]), String(velI[1]), String(velI[2])]);
      setLocalVelD([String(velD[0]), String(velD[1]), String(velD[2])]);
      setLocalAttP([String(attP[0]), String(attP[1]), String(attP[2])]);
      setLocalRateP([String(rateP[0]), String(rateP[1]), String(rateP[2])]);
      setLocalRateD([String(rateD[0]), String(rateD[1]), String(rateD[2])]);
    }
  }, [openPanel, velP, velI, velD, attP, rateP, rateD]);

  const commitArray = (local: string[], prop: number[], setter: (v: React.SetStateAction<number[]>) => void, payloadKey: string) => {
    const next = [...prop];
    local.forEach((raw, idx) => {
      if (raw.trim() !== '') {
        const num = Number(raw);
        if (!Number.isNaN(num)) next[idx] = num;
      }
    });
    setter(next);
    const payload: any = {};
    payload[payloadKey] = next;
    schedulePidSend(payload);
  };

  return (
    <div className="panel-root">
      <div style={{display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 6, cursor: 'pointer'}} onClick={() => setOpenPanel(op => op === 'controller' ? null : 'controller')}>
        <div style={{fontWeight: 700}}>CONTROLLER</div>
        <div className="panel-toggle" aria-hidden>{openPanel === 'controller' ? '▾' : '▸'}</div>
      </div>
      {openPanel === 'controller' && <>
        <div style={{fontSize:12, marginBottom:6}}>Velocity P / I / D</div>
        <div style={{display:'flex', gap:6, marginBottom:6}}>
          <input
            step={0.001}
            placeholder="P1"
            value={localVelP[0]}
            onChange={(e) => setLocalVelP(l => { const c = [...l]; c[0] = e.target.value; return c; })}
            onBlur={() => { commitArray(localVelP, velP, setVelP, 'vel_P_gain'); setLocalVelP(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localVelP, velP, setVelP, 'vel_P_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
          <input
            placeholder="P2"
            value={localVelP[1]}
            onChange={(e) => setLocalVelP(l => { const c = [...l]; c[1] = e.target.value; return c; })}
            onBlur={() => { commitArray(localVelP, velP, setVelP, 'vel_P_gain'); setLocalVelP(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localVelP, velP, setVelP, 'vel_P_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
          <input
            placeholder="P3"
            value={localVelP[2]}
            onChange={(e) => setLocalVelP(l => { const c = [...l]; c[2] = e.target.value; return c; })}
            onBlur={() => { commitArray(localVelP, velP, setVelP, 'vel_P_gain'); setLocalVelP(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localVelP, velP, setVelP, 'vel_P_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
        </div>
        <div style={{fontSize:12, marginBottom:6}}>Velocity I</div>
        <div style={{display:'flex', gap:6, marginBottom:6}}>
          <input
            placeholder="I1"
            value={localVelI[0]}
            onChange={(e) => setLocalVelI(l => { const c = [...l]; c[0] = e.target.value; return c; })}
            onBlur={() => { commitArray(localVelI, velI, setVelI, 'vel_I_gain'); setLocalVelI(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localVelI, velI, setVelI, 'vel_I_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
          <input
            placeholder="I2"
            value={localVelI[1]}
            onChange={(e) => setLocalVelI(l => { const c = [...l]; c[1] = e.target.value; return c; })}
            onBlur={() => { commitArray(localVelI, velI, setVelI, 'vel_I_gain'); setLocalVelI(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localVelI, velI, setVelI, 'vel_I_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
          <input
            placeholder="I3"
            value={localVelI[2]}
            onChange={(e) => setLocalVelI(l => { const c = [...l]; c[2] = e.target.value; return c; })}
            onBlur={() => { commitArray(localVelI, velI, setVelI, 'vel_I_gain'); setLocalVelI(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localVelI, velI, setVelI, 'vel_I_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
        </div>
        <div style={{fontSize:12, marginBottom:6}}>Velocity D</div>
        <div style={{display:'flex', gap:6, marginBottom:6}}>
          <input
            placeholder="D1"
            value={localVelD[0]}
            onChange={(e) => setLocalVelD(l => { const c = [...l]; c[0] = e.target.value; return c; })}
            onBlur={() => { commitArray(localVelD, velD, setVelD, 'vel_D_gain'); setLocalVelD(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localVelD, velD, setVelD, 'vel_D_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
          <input
            placeholder="D2"
            value={localVelD[1]}
            onChange={(e) => setLocalVelD(l => { const c = [...l]; c[1] = e.target.value; return c; })}
            onBlur={() => { commitArray(localVelD, velD, setVelD, 'vel_D_gain'); setLocalVelD(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localVelD, velD, setVelD, 'vel_D_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
          <input
            placeholder="D3"
            value={localVelD[2]}
            onChange={(e) => setLocalVelD(l => { const c = [...l]; c[2] = e.target.value; return c; })}
            onBlur={() => { commitArray(localVelD, velD, setVelD, 'vel_D_gain'); setLocalVelD(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localVelD, velD, setVelD, 'vel_D_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
        </div>
        <div style={{fontSize:12, marginBottom:6}}>Attitude P (roll,pitch,yaw)</div>
        <div style={{display:'flex', gap:6, marginBottom:6}}>
          <input
            placeholder="A1"
            value={localAttP[0]}
            onChange={(e) => setLocalAttP(l => { const c = [...l]; c[0] = e.target.value; return c; })}
            onBlur={() => { commitArray(localAttP, attP, setAttP, 'attitute_p_gain'); setLocalAttP(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localAttP, attP, setAttP, 'attitute_p_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
          <input
            placeholder="A2"
            value={localAttP[1]}
            onChange={(e) => setLocalAttP(l => { const c = [...l]; c[1] = e.target.value; return c; })}
            onBlur={() => { commitArray(localAttP, attP, setAttP, 'attitute_p_gain'); setLocalAttP(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localAttP, attP, setAttP, 'attitute_p_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
          <input
            placeholder="A3"
            value={localAttP[2]}
            onChange={(e) => setLocalAttP(l => { const c = [...l]; c[2] = e.target.value; return c; })}
            onBlur={() => { commitArray(localAttP, attP, setAttP, 'attitute_p_gain'); setLocalAttP(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localAttP, attP, setAttP, 'attitute_p_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
        </div>
        <div style={{fontSize:12, marginBottom:6}}>Rate P / D</div>
        <div style={{display:'flex', gap:6, marginBottom:6}}>
          <input
            placeholder="R_P1"
            value={localRateP[0]}
            onChange={(e) => setLocalRateP(l => { const c = [...l]; c[0] = e.target.value; return c; })}
            onBlur={() => { commitArray(localRateP, rateP, setRateP, 'rate_P_gain'); setLocalRateP(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localRateP, rateP, setRateP, 'rate_P_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
          <input
            placeholder="R_P2"
            value={localRateP[1]}
            onChange={(e) => setLocalRateP(l => { const c = [...l]; c[1] = e.target.value; return c; })}
            onBlur={() => { commitArray(localRateP, rateP, setRateP, 'rate_P_gain'); setLocalRateP(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localRateP, rateP, setRateP, 'rate_P_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
          <input
            placeholder="R_P3"
            value={localRateP[2]}
            onChange={(e) => setLocalRateP(l => { const c = [...l]; c[2] = e.target.value; return c; })}
            onBlur={() => { commitArray(localRateP, rateP, setRateP, 'rate_P_gain'); setLocalRateP(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localRateP, rateP, setRateP, 'rate_P_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
        </div>
        <div style={{display:'flex', gap:6}}>
          <input
            placeholder="R_D1"
            value={localRateD[0]}
            onChange={(e) => setLocalRateD(l => { const c = [...l]; c[0] = e.target.value; return c; })}
            onBlur={() => { commitArray(localRateD, rateD, setRateD, 'rate_D_gain'); setLocalRateD(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localRateD, rateD, setRateD, 'rate_D_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
          <input
            placeholder="R_D2"
            value={localRateD[1]}
            onChange={(e) => setLocalRateD(l => { const c = [...l]; c[1] = e.target.value; return c; })}
            onBlur={() => { commitArray(localRateD, rateD, setRateD, 'rate_D_gain'); setLocalRateD(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localRateD, rateD, setRateD, 'rate_D_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
          <input
            placeholder="R_D3"
            value={localRateD[2]}
            onChange={(e) => setLocalRateD(l => { const c = [...l]; c[2] = e.target.value; return c; })}
            onBlur={() => { commitArray(localRateD, rateD, setRateD, 'rate_D_gain'); setLocalRateD(l => l.map(s => s.trim())); }}
            onKeyDown={(e) => { if (e.key === 'Enter') { e.preventDefault(); commitArray(localRateD, rateD, setRateD, 'rate_D_gain'); } }}
            className="transparent-input"
            style={{width:'33%'}}
          />
        </div>
      </>}
    </div>
  );
}

export default ControllerPanel;
