import React, { useEffect, useState } from "react";
import { IonCard, IonCardHeader, IonCardTitle, IonCardContent } from '@ionic/react';

import { listenInfoValues, InfoValues } from "../../services/Control/main";

const Controller: React.FC<{ cardClass?: (base: string) => string, startAuthorized?: boolean }> = ({ cardClass = (b) => b, startAuthorized = false }) => {
  const [info, setInfo] = useState<InfoValues | null>(null);
  const [blink, setBlink] = useState(true);
  useEffect(() => {
    const ws = listenInfoValues(setInfo);
    return () => ws.close();
  }, []);
  useEffect(() => {
    if (startAuthorized) {
      const interval = setInterval(() => setBlink(b => !b), 500);
      return () => clearInterval(interval);
    } else {
      setBlink(true);
    }
  }, [startAuthorized]);

  const motorPower = info?.motors?.map(m => m.speed) ?? [0, 0, 0, 0];
  const motorTemp = info?.motors?.map(m => m.temperature) ?? [0, 0, 0, 0];
  const battery = info?.battery ?? 0;
  const stepMotors = info?.stepMotors ?? [
    { position: 0, temperature: 0 },
    { position: 0, temperature: 0 }
  ];

  return (
    <IonCard className={cardClass('blue-card')} style={{width: '100%', flex: '1 1 auto', display: 'flex', flexDirection: 'column', justifyContent: 'center', position: 'relative'}}>
      {/* Blinkendes Lämpchen oben rechts */}
      <div style={{position: 'absolute', top: 18, right: 18, zIndex: 10}}>
        {startAuthorized && (
          <span style={{
            display: 'inline-block',
            width: 28,
            height: 28,
            borderRadius: '50%',
            background: blink ? 'rgba(255,235,59,0.14)' : 'rgba(255,253,231,0.06)',
            boxShadow: blink ? '0 0 16px 6px rgba(255,235,59,0.12)' : 'none',
            border: '2px solid rgba(249,168,37,0.18)',
            transition: 'background 0.2s, box-shadow 0.2s'
          }} />
        )}
      </div>
      <IonCardHeader>
        <IonCardTitle>Controller & Motoren</IonCardTitle>
      </IonCardHeader>
      <IonCardContent>
        <div style={{marginBottom: '16px'}}>Controller: <strong>PID aktiv</strong></div>
        <div style={{marginBottom: '16px', padding: '12px', background: 'var(--input-bg)', borderRadius: '12px', boxShadow: '0 2px 8px rgba(0,0,0,0.3)'}}>
          <strong>Batterie:</strong> {battery}%
        </div>
        {[1,2,3,4].map((motor) => (
          <div key={motor} style={{marginBottom: '32px', display: 'flex', flexDirection: 'row', alignItems: 'center', height: '60px'}}>
            <div style={{width: '80px', fontWeight: 'bold', color: 'var(--panel-color)'}}>Motor {motor}</div>
            {/* Power Bar */}
            <div style={{height: '48px', width: '120px', background: 'linear-gradient(to right, rgba(0,105,92,0.85) 0%, rgba(0,77,64,0.85) 100%)', borderRadius: '16px', position: 'relative', marginRight: '16px'}}>
              <div style={{position: 'absolute', left: 0, top: 0, height: '100%', width: `${motorPower[motor-1]}%`, background: 'rgba(255,255,255,0.06)', borderRadius: '16px', border: '2px solid rgba(0,230,118,0.12)', transition: 'width 0.3s'}}></div>
              <span style={{position: 'absolute', left: '50%', top: '50%', transform: 'translate(-50%, -50%)', fontWeight: 'bold', color: 'var(--panel-color)', fontSize: '1.1em'}}>
                {motorPower[motor-1]}%
              </span>
            </div>
            {/* Temperatur Bar */}
            <div style={{height: '48px', width: '120px', background: 'linear-gradient(to right, rgba(255,152,0,0.85) 0%, rgba(211,47,47,0.85) 100%)', borderRadius: '16px', position: 'relative'}}>
              <div style={{position: 'absolute', left: 0, top: 0, height: '100%', width: `${motorTemp[motor-1]}%`, background: 'rgba(255,255,255,0.06)', borderRadius: '16px', border: '2px solid rgba(255,152,0,0.12)', transition: 'width 0.3s'}}></div>
              <span style={{position: 'absolute', left: '50%', top: '50%', transform: 'translate(-50%, -50%)', fontWeight: 'bold', color: 'var(--panel-color)', fontSize: '1.1em'}}>
                {motorTemp[motor-1]}°C
              </span>
            </div>
          </div>
        ))}
      {/* Stepper Motoren Anzeige */}
      <div style={{marginBottom: '24px', display: 'flex', flexDirection: 'row', gap: '32px', justifyContent: 'center'}}>
        {[0,1].map(idx => (
          <div key={idx} style={{background: 'var(--input-bg)', borderRadius: '12px', padding: '16px 24px', minWidth: '160px', boxShadow: '0 2px 8px rgba(0,0,0,0.28)'}}>
            <div style={{fontWeight: 'bold', color: 'var(--panel-color)', fontSize: '1.1em', marginBottom: '8px'}}>Stepper {idx === 0 ? 'Links' : 'Rechts'}</div>
            <div style={{marginBottom: '6px', color: 'rgba(255,255,255,0.85)'}}>Position: <strong>{stepMotors[idx].position}°</strong></div>
            <div style={{color: 'rgba(255,255,255,0.85)'}}>Temperatur: <strong>{stepMotors[idx].temperature}°C</strong></div>
          </div>
        ))}
      </div>
      </IonCardContent>
    </IonCard>
  );
};

export default Controller;