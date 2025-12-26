import React, { useMemo, useState } from 'react';

type Sample = { t: number; p: number }; // t = unix seconds, p = watts

 interface Props {
  samples: Sample[];
  // keep these props optional so the panel is independent
  openPanel?: string | null;
  setOpenPanel?: (s: string | null) => void;
  asideList?: React.ReactNode;
 }

// Collapsible SVG sparkline + total energy (last minute)
const EnergyPanel: React.FC<Props> = ({ samples, openPanel, setOpenPanel, asideList }) => {
  const now = Date.now() / 1000;
  const windowStart = now - 60;

  // filtered samples in last 60s
  const last = useMemo(() => samples.filter(s => s.t >= windowStart), [samples, windowStart]);

  // compute energy (J) over last minute by trapezoidal approximation
  const energyJ = useMemo(() => {
    if (last.length < 2) return 0;
    let e = 0;
    for (let i = 1; i < last.length; i++) {
      const dt = last[i].t - last[i - 1].t;
      const avgP = (last[i].p + last[i - 1].p) / 2;
      e += avgP * dt;
    }
    return e;
  }, [last]);

  const latestP = last.length ? last[last.length - 1].p : 0;

  // sparkline path
  const path = useMemo(() => {
    if (last.length === 0) return '';
    const w = 280; const h = 64; const pad = 4;
    const minT = windowStart; const maxT = now;
    let maxP = 0;
    for (const s of last) if (s.p > maxP) maxP = s.p;
    if (maxP <= 0) maxP = 1;
    const coords = last.map(s => {
      const x = pad + ((s.t - minT) / (maxT - minT)) * (w - pad * 2);
      const y = pad + (1 - (s.p / maxP)) * (h - pad * 2);
      return `${x},${y}`;
    });
    return 'M' + coords.join(' L ');
  }, [last, windowStart, now]);

  // manage local open state so EnergyPanel is independent from left-side panels
  const [isOpen, setIsOpen] = useState<boolean>(false);

  // compact header when closed
  if (!isOpen) {
    return (
      <div className="panel-root" style={{ position: 'fixed', right: 12, bottom: 12, width: 220, zIndex: 400, cursor: 'pointer' }} onClick={() => setIsOpen(true)}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
          <div style={{ fontWeight: 700 }}>ENERGY</div>
          <div style={{ fontSize: 12, opacity: 0.9 }}>{(energyJ / 1000).toFixed(3)} kJ</div>
        </div>
      </div>
    );
  }

  return (
    <div className="panel-root" style={{ position: 'fixed', right: 12, bottom: 12, width: 320, zIndex: 400 }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', cursor: 'pointer' }} onClick={() => setIsOpen(false)}>
        <div style={{ fontWeight: 700 }}>Energy (last 60s)</div>
        <div style={{ textAlign: 'right' }}>
          <div style={{ fontSize: 12, opacity: 0.9 }}>{latestP.toFixed(1)} W</div>
          <div style={{ fontSize: 11, opacity: 0.8 }}>{(energyJ / 1000).toFixed(3)} kJ</div>
        </div>
      </div>
      <div style={{ marginTop: 8 }}>
        <svg width="100%" height={80} viewBox={`0 0 280 64`} preserveAspectRatio="none">
          <defs>
            <linearGradient id="g" x1="0" x2="0" y1="0" y2="1">
              <stop offset="0%" stopColor="#ffcc00" stopOpacity="0.9" />
              <stop offset="100%" stopColor="#ffcc00" stopOpacity="0.06" />
            </linearGradient>
          </defs>
          {path && <path d={path} stroke="#ffcc00" strokeWidth={2} fill="none" strokeLinecap="round" strokeLinejoin="round" />}
          {/* filled area under curve */}
          {path && <path d={`${path} L 276 60 L 4 60 Z`} fill="url(#g)" opacity={0.18} />}
        </svg>
      </div>
      {asideList && (
        <div className="energy-aside-list" style={{ marginTop: 8 }}>
          {asideList}
        </div>
      )}
    </div>
  );
};

export default EnergyPanel;
