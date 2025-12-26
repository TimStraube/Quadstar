import React from 'react';

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
  return (
    <div className="panel-root" style={{
      position: 'fixed',
      bottom: 12,
      left: 12,
      zIndex: 250,
      width: 360,
      maxWidth: '90vw',
      pointerEvents: 'auto'
    }}>
      <div style={{display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 6}}>
        <div style={{fontWeight: 700}}>Controller Einstellungen (live)</div>
        <button onClick={() => setOpenPanel(op => op === 'controller' ? null : 'controller')} className="panel-toggle">{openPanel === 'controller' ? '▾' : '▸'}</button>
      </div>
      {openPanel === 'controller' && <>
        <div style={{fontSize:12, marginBottom:6}}>Velocity P / I / D</div>
        <div style={{display:'flex', gap:6, marginBottom:6}}>
          <input step={0.001} value={String(velP[0])} onChange={(e)=>{ const v = Number(e.target.value||0); setVelP(s=>{const n=[...s];n[0]=v; return n}); schedulePidSend({vel_P_gain: [Number(e.target.value||0), velP[1], velP[2]]}); }} className="transparent-input" style={{width:'33%'}} />
          <input value={String(velP[1])} onChange={(e)=>{ const v = Number(e.target.value||0); setVelP(s=>{const n=[...s];n[1]=v; return n}); schedulePidSend({vel_P_gain: [velP[0], Number(e.target.value||0), velP[2]]}); }} className="transparent-input" style={{width:'33%'}} />
          <input value={String(velP[2])} onChange={(e)=>{ const v = Number(e.target.value||0); setVelP(s=>{const n=[...s];n[2]=v; return n}); schedulePidSend({vel_P_gain: [velP[0], velP[1], Number(e.target.value||0)]}); }} className="transparent-input" style={{width:'33%'}} />
        </div>
        <div style={{fontSize:12, marginBottom:6}}>Velocity I</div>
        <div style={{display:'flex', gap:6, marginBottom:6}}>
          <input value={String(velI[0])} onChange={(e)=>{ setVelI(s=>{const n=[...s];n[0]=Number(e.target.value||0); return n}); schedulePidSend({vel_I_gain: [Number(e.target.value||0), velI[1], velI[2]]}); }} className="transparent-input" style={{width:'33%'}} />
          <input value={String(velI[1])} onChange={(e)=>{ setVelI(s=>{const n=[...s];n[1]=Number(e.target.value||0); return n}); schedulePidSend({vel_I_gain: [velI[0], Number(e.target.value||0), velI[2]]}); }} className="transparent-input" style={{width:'33%'}} />
          <input value={String(velI[2])} onChange={(e)=>{ setVelI(s=>{const n=[...s];n[2]=Number(e.target.value||0); return n}); schedulePidSend({vel_I_gain: [velI[0], velI[1], Number(e.target.value||0)]}); }} className="transparent-input" style={{width:'33%'}} />
        </div>
        <div style={{fontSize:12, marginBottom:6}}>Velocity D</div>
        <div style={{display:'flex', gap:6, marginBottom:6}}>
          <input value={String(velD[0])} onChange={(e)=>{ setVelD(s=>{const n=[...s];n[0]=Number(e.target.value||0); return n}); schedulePidSend({vel_D_gain: [Number(e.target.value||0), velD[1], velD[2]]}); }} className="transparent-input" style={{width:'33%'}} />
          <input value={String(velD[1])} onChange={(e)=>{ setVelD(s=>{const n=[...s];n[1]=Number(e.target.value||0); return n}); schedulePidSend({vel_D_gain: [velD[0], Number(e.target.value||0), velD[2]]}); }} className="transparent-input" style={{width:'33%'}} />
          <input value={String(velD[2])} onChange={(e)=>{ setVelD(s=>{const n=[...s];n[2]=Number(e.target.value||0); return n}); schedulePidSend({vel_D_gain: [velD[0], velD[1], Number(e.target.value||0)]}); }} className="transparent-input" style={{width:'33%'}} />
        </div>
        <div style={{fontSize:12, marginBottom:6}}>Attitude P (roll,pitch,yaw)</div>
        <div style={{display:'flex', gap:6, marginBottom:6}}>
          <input value={String(attP[0])} onChange={(e)=>{ setAttP(s=>{const n=[...s];n[0]=Number(e.target.value||0); return n}); schedulePidSend({attitute_p_gain: [Number(e.target.value||0), attP[1], attP[2]]}); }} className="transparent-input" style={{width:'33%'}} />
          <input value={String(attP[1])} onChange={(e)=>{ setAttP(s=>{const n=[...s];n[1]=Number(e.target.value||0); return n}); schedulePidSend({attitute_p_gain: [attP[0], Number(e.target.value||0), attP[2]]}); }} className="transparent-input" style={{width:'33%'}} />
          <input value={String(attP[2])} onChange={(e)=>{ setAttP(s=>{const n=[...s];n[2]=Number(e.target.value||0); return n}); schedulePidSend({attitute_p_gain: [attP[0], attP[1], Number(e.target.value||0)]}); }} className="transparent-input" style={{width:'33%'}} />
        </div>
        <div style={{fontSize:12, marginBottom:6}}>Rate P / D</div>
        <div style={{display:'flex', gap:6, marginBottom:6}}>
          <input value={String(rateP[0])} onChange={(e)=>{ setRateP(s=>{const n=[...s];n[0]=Number(e.target.value||0); return n}); schedulePidSend({rate_P_gain: [Number(e.target.value||0), rateP[1], rateP[2]]}); }} className="transparent-input" style={{width:'33%'}} />
          <input value={String(rateP[1])} onChange={(e)=>{ setRateP(s=>{const n=[...s];n[1]=Number(e.target.value||0); return n}); schedulePidSend({rate_P_gain: [rateP[0], Number(e.target.value||0), rateP[2]]}); }} className="transparent-input" style={{width:'33%'}} />
          <input value={String(rateP[2])} onChange={(e)=>{ setRateP(s=>{const n=[...s];n[2]=Number(e.target.value||0); return n}); schedulePidSend({rate_P_gain: [rateP[0], rateP[1], Number(e.target.value||0)]}); }} className="transparent-input" style={{width:'33%'}} />
        </div>
        <div style={{display:'flex', gap:6}}>
          <input value={String(rateD[0])} onChange={(e)=>{ setRateD(s=>{const n=[...s];n[0]=Number(e.target.value||0); return n}); schedulePidSend({rate_D_gain: [Number(e.target.value||0), rateD[1], rateD[2]]}); }} className="transparent-input" style={{width:'33%'}} />
          <input value={String(rateD[1])} onChange={(e)=>{ setRateD(s=>{const n=[...s];n[1]=Number(e.target.value||0); return n}); schedulePidSend({rate_D_gain: [rateD[0], Number(e.target.value||0), rateD[2]]}); }} className="transparent-input" style={{width:'33%'}} />
          <input value={String(rateD[2])} onChange={(e)=>{ setRateD(s=>{const n=[...s];n[2]=Number(e.target.value||0); return n}); schedulePidSend({rate_D_gain: [rateD[0], rateD[1], Number(e.target.value||0)]}); }} className="transparent-input" style={{width:'33%'}} />
        </div>
      </>}
    </div>
  );
}

export default ControllerPanel;
