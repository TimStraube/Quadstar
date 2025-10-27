import React from "react";
import { IonCard, IonCardHeader, IonCardTitle, IonCardContent } from '@ionic/react';

const motorPower = [80, 78, 82, 79];
const motorTemp = [55, 60, 52, 58];

const Controller: React.FC<{ cardClass?: (base: string) => string }> = ({ cardClass = (b) => b }) => {
  return (
    <IonCard className={cardClass('blue-card')} style={{width: '100%', flex: '1 1 auto', display: 'flex', flexDirection: 'column', justifyContent: 'center'}}>
      <IonCardHeader>
        <IonCardTitle>Controller & Motoren</IonCardTitle>
      </IonCardHeader>
      <IonCardContent>
        <div style={{marginBottom: '16px'}}>Controller: <strong>PID aktiv</strong></div>
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
      </IonCardContent>
    </IonCard>
  );
};

export default Controller;