import React from 'react';
import RangeSlider from './RangeSlider';

interface Props {
  sliderVal: number;
  setSliderVal: (v: number) => void;
  sliderToSpeed: (v: number) => number;
  wpTolerance: number;
  onToleranceChange: (v: number) => void;
  openPanel: string | null;
  setOpenPanel: (s: string | null) => void;
}

const SimulationPanel: React.FC<Props> = ({ sliderVal, setSliderVal, sliderToSpeed, wpTolerance, onToleranceChange, openPanel, setOpenPanel }) => {
  const isOpen = openPanel === 'simulation';
  return (
    <div className="panel-root">
      <div style={{display:'flex', alignItems:'center', justifyContent:'space-between', marginBottom:6}}>
        <div style={{fontWeight:700}}>Simulation</div>
        <div>
          <button title="Toggle simulation panel" onClick={() => setOpenPanel(op => op === 'simulation' ? null : 'simulation')} className="panel-toggle">{isOpen ? '▾' : '▸'}</button>
        </div>
      </div>
      {isOpen && (
        <div style={{display:'flex', flexDirection:'column', gap:8}}>
          <div style={{display:'flex', alignItems:'center', gap:8, flexWrap:'nowrap'}}>
            <div style={{flex:'0 0 140px'}}>
              <RangeSlider
                label="Tolerance"
                min={0}
                max={2}
                step={0.01}
                value={wpTolerance}
                onChange={(v) => onToleranceChange(v)}
                display={(v) => `${v.toFixed(2)} m`}
                valueStyle={{ width: 60 }}
              />
            </div>
          </div>

          <div style={{display:'flex', alignItems:'center', gap:8, flexWrap:'nowrap'}}>
            <div style={{flex:'1 1 auto', minWidth: 180}}>
              <RangeSlider
                label="Speed (log)"
                min={0}
                max={1}
                step={0.001}
                value={sliderVal}
                onChange={(v) => { setSliderVal(v); }}
                display={(v) => `${Number(sliderToSpeed(v)).toFixed(2)}x`}
                valueStyle={{ width: 60 }}
              />
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default SimulationPanel;
