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
            background: blink ? '#ffeb3b' : '#fffde7',
            boxShadow: blink ? '0 0 16px 6px #ffeb3b' : 'none',
            border: '2px solid #fbc02d',
            transition: 'background 0.2s, box-shadow 0.2s'
          }} />
        )}
      </div>
      <IonCardHeader>
        <IonCardTitle>Controller & Motoren</IonCardTitle>
      </IonCardHeader>
      <IonCardContent>
        <div style={{marginBottom: '16px'}}>Controller: <strong>PID aktiv</strong></div>
        <div style={{marginBottom: '16px', padding: '12px', background: '#e3f2fd', borderRadius: '12px', boxShadow: '0 2px 8px rgba(25, 118, 210, 0.12)'}}>
          <strong>Batterie:</strong> {battery}%
        </div>
        {[1,2,3,4].map((motor) => (
          <div key={motor} style={{marginBottom: '32px', display: 'flex', flexDirection: 'row', alignItems: 'center', height: '60px'}}>
            <div style={{width: '80px', fontWeight: 'bold', color: '#fff'}}>Motor {motor}</div>
            {/* Power Bar */}
            <div style={{height: '48px', width: '120px', background: 'linear-gradient(to right, #00e676 0%, #1976d2 100%)', borderRadius: '16px', position: 'relative', marginRight: '16px', boxShadow: '0 2px 8px rgba(25, 118, 210, 0.2)'}}>
              <div style={{position: 'absolute', left: 0, top: 0, height: '100%', width: `${motorPower[motor-1]}%`, background: 'rgba(255,255,255,0.2)', borderRadius: '16px', border: '2px solid #00e676', transition: 'width 0.3s'}}></div>
              <span style={{position: 'absolute', left: '50%', top: '50%', transform: 'translate(-50%, -50%)', fontWeight: 'bold', color: '#1976d2', fontSize: '1.1em'}}>
                {motorPower[motor-1]}%
              </span>
            </div>
            {/* Temperatur Bar */}
            <div style={{height: '48px', width: '120px', background: 'linear-gradient(to right, #ff9800 0%, #d32f2f 100%)', borderRadius: '16px', position: 'relative', boxShadow: '0 2px 8px rgba(211, 47, 47, 0.2)'}}>
              <div style={{position: 'absolute', left: 0, top: 0, height: '100%', width: `${motorTemp[motor-1]}%`, background: 'rgba(255,255,255,0.2)', borderRadius: '16px', border: '2px solid #ff9800', transition: 'width 0.3s'}}></div>
              <span style={{position: 'absolute', left: '50%', top: '50%', transform: 'translate(-50%, -50%)', fontWeight: 'bold', color: '#d32f2f', fontSize: '1.1em'}}>
                {motorTemp[motor-1]}°C
              </span>
            </div>
          </div>
        ))}
      {/* Stepper Motoren Anzeige */}
      <div style={{marginBottom: '24px', display: 'flex', flexDirection: 'row', gap: '32px', justifyContent: 'center'}}>
        {[0,1].map(idx => (
          <div key={idx} style={{background: '#e3f2fd', borderRadius: '12px', padding: '16px 24px', minWidth: '160px', boxShadow: '0 2px 8px rgba(25, 118, 210, 0.12)'}}>
            <div style={{fontWeight: 'bold', color: '#000', fontSize: '1.1em', marginBottom: '8px'}}>Stepper {idx === 0 ? 'Links' : 'Rechts'}</div>
            <div style={{marginBottom: '6px', color: '#000'}}>Position: <strong>{stepMotors[idx].position}°</strong></div>
            <div style={{color: '#000'}}>Temperatur: <strong>{stepMotors[idx].temperature}°C</strong></div>
          </div>
        ))}
      </div>
      </IonCardContent>
    </IonCard>
  );
};

export default Controller;