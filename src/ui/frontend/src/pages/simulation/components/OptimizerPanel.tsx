import React, { useState } from 'react';

type WP = { x: number; y: number; z: number };

interface Props {
  waypoints: WP[];
  speed: number; // m/s (average cruise speed)
  baseMass: number; // kg (frame + motors without battery)
  payloadMass?: number; // kg
}

const OptimizerPanel: React.FC<Props> = ({ waypoints, speed, baseMass, payloadMass = 0 }) => {
  const [energyDensity, setEnergyDensity] = useState<number>(200); // Wh/kg typical LiPo ~150-250
  const [reserveFraction, setReserveFraction] = useState<number>(0.2);
  const [hoverCoeff, setHoverCoeff] = useState<number>(100); // W per kg hover coefficient (W/kg)
  const [minBatteryKg, setMinBatteryKg] = useState<number | null>(null);
  const [requiredWh, setRequiredWh] = useState<number | null>(null);
  const [capacityWh, setCapacityWh] = useState<number | null>(null);
  const [message, setMessage] = useState<string | null>(null);
  const [isOpen, setIsOpen] = useState<boolean>(false);

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
    setMessage(null);
    setMinBatteryKg(null);
    setRequiredWh(null);
    setCapacityWh(null);

    // bounds
    const low = 0.01; // kg
    const high = 10.0; // kg
    const ed = Number(energyDensity);
    if (!ed || ed <= 0) {
      setMessage('Ungültige Energiedichte');
      return;
    }

    // monotonicity: f(m) = m*ed - (1+res) * E_req(m)
    // search for smallest m where f(m) >= 0
    const resFactor = 1 + Math.max(0, reserveFraction);

    const f = (m: number) => {
      const req = simulateEnergyWh(m);
      return m * ed - resFactor * req;
    };

    // check feasibility at high
    if (f(high) < 0) {
      setMessage('Kein machbares Ergebnis im Suchbereich (erhöhe max battery oder Energiedichte)');
      return;
    }

    let lo = low;
    let hi = high;
    for (let i = 0; i < 60; i++) {
      const mid = (lo + hi) / 2;
      if (f(mid) >= 0) hi = mid; else lo = mid;
    }
    const sol = hi;
    const req = simulateEnergyWh(sol);
    setMinBatteryKg(sol);
    setRequiredWh(req * resFactor);
    setCapacityWh(sol * ed);
    setMessage(null);
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
          <button className="control-button" onClick={computeMinBattery}>Berechne optimale Akku-Größe</button>
        </div>

        {message && <div style={{ color: '#ffcc88' }}>{message}</div>}

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
