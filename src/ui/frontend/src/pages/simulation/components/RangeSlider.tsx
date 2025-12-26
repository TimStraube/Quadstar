import React from 'react';

type Props = {
  label: string;
  min: number;
  max: number;
  step?: number;
  value: number;
  onChange: (v: number) => void;
  display?: (v: number) => string;
  valueStyle?: React.CSSProperties;
};

const RangeSlider: React.FC<Props> = ({ label, min, max, step = 0.01, value, onChange, display, valueStyle }) => {
  const fmt = display || ((v: number) => String(v));
  return (
    <div style={{display: 'flex', alignItems: 'center', gap: 8}}>
      <label style={{fontSize: 14, minWidth: 110}}>{label}</label>
      <div style={{width: 60, textAlign: 'left', fontWeight: 600, ...valueStyle}}>{fmt(value)}</div>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={String(value)}
        onChange={(e) => { const v = Number(e.target.value); onChange(v); }}
        style={{flex: 1}}
      />
    </div>
  );
};

export default RangeSlider;
