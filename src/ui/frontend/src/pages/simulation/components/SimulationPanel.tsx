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
      <div style={{display:'flex', alignItems:'center', justifyContent:'space-between', marginBottom:6, cursor: 'pointer'}} onClick={() => setOpenPanel(openPanel === 'simulation' ? null : 'simulation')}>
        <div style={{fontWeight:700}}>SIMULATION</div>
        <div className="panel-toggle" aria-hidden>{isOpen ? '▾' : '▸'}</div>
      </div>
      {isOpen && (
        <div style={{display:'flex', flexDirection:'column', gap:8}}>
          <div style={{display:'flex', alignItems:'center', gap:8, flexWrap:'nowrap'}}>
            <div style={{flex:'0 0 120px'}}>
              <RangeSlider
                label="Tolerance"
                min={0}
                max={2}
                step={0.01}
                value={wpTolerance}
                onChange={(v) => onToleranceChange(v)}
                display={(v) => `${Number(v).toFixed(2)} m`}
                precision={2}
                valueStyle={{ width: 60 }}
              />
            </div>
          </div>

          <div style={{display:'flex', alignItems:'center', gap:8, flexWrap:'nowrap'}}>
            <div style={{flex:'1 1 auto', minWidth: 160}}>
              <RangeSlider
                label="Speed (log)"
                min={0}
                max={1}
                step={0.001}
                value={sliderVal}
                onChange={(v) => { setSliderVal(v); }}
                display={(v) => `${Number(sliderToSpeed(v)).toFixed(1)}x`}
                precision={1}
                valueStyle={{ width: 56 }}
              />
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default SimulationPanel;
