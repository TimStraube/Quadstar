import React, { useState } from 'react';

type WP = { x: number; y: number; z: number };

interface Props {
  waypoints: WP[];
  speed: number; // m/s (average cruise speed)
  baseMass: number; // kg (frame + motors without battery)
  payloadMass?: number; // kg
  simulationObjects?: React.MutableRefObject<any>;
}

const OptimizerPanel: React.FC<Props> = ({ waypoints, speed, baseMass, payloadMass = 0, simulationObjects }) => {
  const [energyDensity, setEnergyDensity] = useState<number>(200); // Wh/kg typical LiPo ~150-250
  const [reserveFraction, setReserveFraction] = useState<number>(0.2);
  const [hoverCoeff, setHoverCoeff] = useState<number>(100); // W per kg hover coefficient (W/kg)
  const [minBatteryKg, setMinBatteryKg] = useState<number | null>(null);
  const [requiredWh, setRequiredWh] = useState<number | null>(null);
  const [capacityWh, setCapacityWh] = useState<number | null>(null);
  const [message, setMessage] = useState<string | null>(null);
  const [isOpen, setIsOpen] = useState<boolean>(false);
  const [runningSim, setRunningSim] = useState<boolean>(false);
  const [progress, setProgress] = useState<number>(0);
  const [integratedWh, setIntegratedWh] = useState<number | null>(null);
  const [powerW, setPowerW] = useState<number | null>(null);

  // compute total path distance (open polyline with given order). Units assumed meters.
  const pathDistance = (wps: WP[]) => {
    if (!wps || wps.length < 2) return 0;
    let d = 0;
    for (let i = 0; i < wps.length - 1; i++) {
      const a = wps[i];
      const b = wps[i + 1];
      const dx = a.x - b.x;
      const dy = a.y - b.y;
      const dz = (a.z || 0) - (b.z || 0);
      d += Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    return d;
  };

  // simplified energy model: E (Wh) = hoverCoeff (W/kg) * mass (kg) * (time (s) / 3600)
  // mass = baseMass + payloadMass + batteryMass
  // time = distance / speed
  const simulateEnergyWh = (batteryMassKg: number) => {
    const dist = pathDistance(waypoints);
    const avgSpeed = Math.max(0.1, speed || 1.0);
    const timeSec = dist / avgSpeed;
    const mass = baseMass + payloadMass + batteryMassKg;
    const powerW = hoverCoeff * mass; // W
    const energyWh = (powerW * (timeSec / 3600));
    // add a small cruise penalty proportional to distance (empirical)
    const cruisePenalty = 0.02 * dist; // Wh per meter placeholder
    return energyWh + cruisePenalty;
  };

  // Find minimal battery mass such that battery_mass * energyDensity >= requiredEnergy * (1+reserveFraction)
  // Because requiredEnergy depends on battery mass (through mass), we solve via binary search.
  const computeMinBattery = () => {
    // New flow: run a single simulated flight (using current baseMass + payloadMass, excluding battery mass)
    // integrate energy over the trajectory and then compute required battery mass = requiredWh / energyDensity
    setMessage(null);
    setMinBatteryKg(null);
    setRequiredWh(null);
    setCapacityWh(null);
    setIntegratedWh(null);

    const ed = Number(energyDensity);
    if (!ed || ed <= 0) {
      setMessage('Ungültige Energiedichte');
      return;
    }

    // run async simulation
    runSimulationAndCompute();
  };

  const runSimulationAndCompute = async () => {
    if (!waypoints || waypoints.length < 2) { setMessage('Nicht genügend Waypoints'); return; }
    setRunningSim(true); setProgress(0); setMessage('Starte Simulation (iterative Suche)...');
    const ed = Number(energyDensity);
    if (!ed || ed <= 0) { setMessage('Ungültige Energiedichte'); setRunningSim(false); return; }
    const objs = simulationObjects && simulationObjects.current ? simulationObjects.current : undefined;

    // helper: run a flight simulation for a given battery mass and report energy (Wh)
    const runFlightSimulation = async (batteryMassKg: number, onInnerProgress?: (p:number)=>void) : Promise<number> => {
      // place vehicle at start
      try {
        const start = waypoints[0];
        if (objs) {
          const alt = Number(start.z || 0);
          const nx = -Number(start.y || 0);
          const nz = Number(start.x || 0);
          try { if (objs.model) objs.model.position.set(nx, alt, nz); } catch(e) {}
          try { if (objs.cube1) objs.cube1.position.set(nx, alt, nz); } catch(e) {}
          try { if (objs.cube2) objs.cube2.position.set(nx, alt, nz); } catch(e) {}
          try { if (objs.cube3) objs.cube3.position.set(nx, alt, nz); } catch(e) {}
          try { if (objs.renderer && objs.scene && objs.camera) objs.renderer.render(objs.scene, objs.camera); } catch (e) {}
        }
      } catch(e){ }

      const dt = 0.1;
      const totDist = pathDistance(waypoints);
      if (totDist <= 0) throw new Error('Weglänge 0');
      let covered = 0;
      let energyWh = 0;
      const mass = baseMass + (payloadMass || 0) + Math.max(0, batteryMassKg);

      // quick sanity: ensure scene objects exist
      if (!objs || (!objs.renderer && !objs.scene)) {
        throw new Error('Szene noch nicht initialisiert');
      }

      for (let i = 0; i < waypoints.length - 1; i++) {
        const a = waypoints[i];
        const b = waypoints[i + 1];
        const dx = b.x - a.x; const dy = b.y - a.y; const dz = (b.z || 0) - (a.z || 0);
        const segLen = Math.sqrt(dx * dx + dy * dy + dz * dz);
        const v = Math.max(0.1, speed || 1.0);
        const segTime = segLen / v;
        const steps = Math.max(1, Math.ceil(segTime / dt));

        for (let s = 0; s < steps; s++) {
          const t = (s + 1) / steps;
          const px = a.x + dx * t;
          const py = a.y + dy * t;
          const pz = (a.z || 0) + dz * t;
          // update visual
          try {
            if (objs) {
              const alt = Number(pz || 0);
              const nx = -Number(py || 0);
              const nz = Number(px || 0);
              try { if (objs.model) objs.model.position.set(nx, alt, nz); } catch(e) {}
              try { if (objs.cube1) objs.cube1.position.set(nx, alt, nz); } catch(e) {}
              try { if (objs.cube2) objs.cube2.position.set(nx, alt, nz); } catch(e) {}
              try { if (objs.cube3) objs.cube3.position.set(nx, alt, nz); } catch(e) {}
              try { if (objs.renderer && objs.scene && objs.camera) objs.renderer.render(objs.scene, objs.camera); } catch (e) {}
            }
          } catch(e){}

          const pW = hoverCoeff * mass;
          energyWh += (pW * dt) / 3600.0;

          // update occasional UI values (throttled)
          if (s % 5 === 0) {
            try { setIntegratedWh(energyWh); } catch(e) {}
            try { setPowerW(pW); } catch(e) {}
          }

          const partial = (s + 1) / steps * segLen;
          const currentCovered = covered + partial;
          if (onInnerProgress) onInnerProgress(Math.min(1, currentCovered / totDist));

          if (s % 3 === 0) await new Promise(r => setTimeout(r, 10));
        }

        // cruise penalty
        energyWh += 0.02 * segLen;
        covered += segLen;
      }

      return energyWh;
    };

    // Single-run simulation: fly from first to last waypoint once (exclude battery mass)
    try {
      setMessage('Führe eine einzelne Flug-Simulation durch...');
      // ensure scene is ready
      if (!objs || (!objs.renderer && !objs.scene)) { setMessage('Szene noch nicht bereit'); setRunningSim(false); return; }
      const energy = await runFlightSimulation(0, (p) => setProgress(p));
      setIntegratedWh(energy);
      const reqWh = energy * (1 + Math.max(0, reserveFraction));
      const batteryKg = reqWh / ed;
      setMinBatteryKg(batteryKg);
      setRequiredWh(reqWh);
      setCapacityWh(batteryKg * ed);
      setMessage('Simulation fertig');
      setProgress(1);
    } catch (e:any) {
      console.error('Simulation failed', e);
      setMessage('Simulation fehlgeschlagen');
    } finally {
      setRunningSim(false);
    }
  };

  // collapsible panel that lives inside the left-side panel stack
  if (!isOpen) {
    return (
      <div className="panel-root" style={{ cursor: 'pointer' }} onClick={() => setIsOpen(true)}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
          <div style={{ fontWeight: 700 }}>OPTIMIZER</div>
        </div>
      </div>
    );
  }

  return (
    <div className="panel-root">
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', cursor: 'pointer' }} onClick={() => setIsOpen(false)}>
        <div style={{ fontWeight: 700 }}>OPTIMIZER</div>
      </div>
      <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
        <div style={{ display: 'flex', gap: 8, alignItems: 'center' }}>
          <label style={{ minWidth: 120 }}>Energiedichte (Wh/kg)</label>
          <input className="transparent-input" type="number" value={energyDensity} onChange={e => setEnergyDensity(Number(e.target.value))} style={{ width: 120 }} />
        </div>
        <div style={{ display: 'flex', gap: 8, alignItems: 'center' }}>
          <label style={{ minWidth: 120 }}>Reserve (%)</label>
          <input className="transparent-input" type="number" value={Math.round(reserveFraction * 100)} onChange={e => setReserveFraction(Number(e.target.value) / 100)} style={{ width: 120 }} />
        </div>
        <div style={{ display: 'flex', gap: 8, alignItems: 'center' }}>
          <label style={{ minWidth: 120 }}>Hover K (W/kg)</label>
          <input className="transparent-input" type="number" value={hoverCoeff} onChange={e => setHoverCoeff(Number(e.target.value))} style={{ width: 120 }} />
        </div>
        <div style={{ display: 'flex', gap: 8, alignItems: 'center' }}>
          <label style={{ minWidth: 120 }}>Base Mass (kg)</label>
          <input className="transparent-input" type="number" value={baseMass} readOnly style={{ width: 120, opacity: 0.9 }} />
        </div>
        <div style={{ display: 'flex', gap: 8, alignItems: 'center' }}>
          <label style={{ minWidth: 120 }}>Payload (kg)</label>
          <input className="transparent-input" type="number" value={payloadMass} readOnly style={{ width: 120, opacity: 0.9 }} />
        </div>
        <div style={{ display: 'flex', gap: 8 }}>
          <button className="control-button" onClick={computeMinBattery} disabled={runningSim}>{runningSim ? 'Simuliere...' : 'Berechne optimale Akku-Größe'}</button>
        </div>

        {message && <div style={{ color: '#ffcc88' }}>{message}</div>}
        {runningSim && (
          <div style={{ marginTop: 6 }}>
            <div style={{ fontSize: 12, color: '#ddd' }}>Simulation: {(progress * 100).toFixed(0)}%</div>
            <div style={{ background: 'rgba(255,255,255,0.06)', height: 8, borderRadius: 4 }}>
              <div style={{ width: `${Math.floor(progress * 100)}%`, height: '100%', background: '#4da6ff', borderRadius: 4 }} />
            </div>
          </div>
        )}

        {runningSim && (
          <div style={{ marginTop: 6, fontSize: 12, color: '#ccc' }}>
            <div>Aktuelle Leistung: <b>{powerW ? powerW.toFixed(0) : '–'} W</b></div>
            <div>Verbrauch bisher: <b>{integratedWh ? integratedWh.toFixed(3) : '–'} Wh</b></div>
          </div>
        )}

        {minBatteryKg !== null && (
          <div style={{ display: 'flex', flexDirection: 'column', gap: 6 }}>
            <div>Benötigte Batterie-Masse: <b>{minBatteryKg.toFixed(3)} kg</b></div>
            <div>Erforderliche Kapazität (inkl. Reserve): <b>{(requiredWh || 0).toFixed(1)} Wh</b></div>
            <div>Berechnete Kapazität: <b>{(capacityWh || 0).toFixed(1)} Wh</b></div>
            <div style={{ fontSize: 12, color: '#ccc' }}>Distanz: {pathDistance(waypoints).toFixed(1)} m — Speed: {speed.toFixed(2)} m/s</div>
          </div>
        )}
      </div>
    </div>
  );
};

export default OptimizerPanel;
