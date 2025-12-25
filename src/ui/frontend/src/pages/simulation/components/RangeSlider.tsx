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
    <>
      <label style={{fontSize: 14}}>{label}</label>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={String(value)}
        onChange={(e) => { const v = Number(e.target.value); onChange(v); }}
        style={{flex: 1}}
      />
      <div style={{width: 90, textAlign: 'right', fontWeight: '600', ...valueStyle}}>{fmt(value)}</div>
    </>
  );
};

export default RangeSlider;
