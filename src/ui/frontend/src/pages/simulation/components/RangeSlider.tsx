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
  precision?: number;
};

const RangeSlider: React.FC<Props> = ({ label, min, max, step = 0.01, value, onChange, display, valueStyle, precision = 2 }) => {
  const defaultFmt = (v: number) => {
    if (typeof v === 'number') return v.toFixed(precision);
    return String(v);
  };
  const fmt = display || defaultFmt;
  return (
    <div style={{display: 'flex', alignItems: 'center', gap: 8, width: '100%'}}>
      <label style={{fontSize: 14, minWidth: 80, flex: '0 0 auto'}}>{label}</label>
      <div style={{display: 'flex', alignItems: 'center', gap: 8, flex: '1 1 auto', minWidth: 0}}>
        <input
          type="range"
          min={min}
          max={max}
          step={step}
          value={String(value)}
          onChange={(e) => { const v = Number(e.target.value); onChange(v); }}
          style={{flex: 1, minWidth: 0}}
        />
        <div style={{whiteSpace: 'nowrap', textAlign: 'right', minWidth: 56, fontWeight: 600, ...valueStyle}}>{fmt(value)}</div>
      </div>
    </div>
  );
};

export default RangeSlider;
