import { IonContent, IonHeader, IonPage, IonTitle, IonToolbar, IonCard, IonCardHeader, IonCardTitle, IonCardContent, IonGrid, IonRow, IonCol, IonButton } from '@ionic/react';
import React, { useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { useRef } from 'react';
// 3D Cube Component
const RotatingCube: React.FC<{ rotation: number, zoom: number }> = ({ rotation, zoom }) => {
  return (
    <Canvas camera={{ position: [0, 0, zoom] }} style={{ width: '200px', height: '200px' }}>
      <ambientLight intensity={0.5} />
      <directionalLight position={[2, 2, 2]} />
      <mesh rotation={[0.4, rotation, 0]}>
        <boxGeometry args={[1, 1, 1]} />
        <meshStandardMaterial color="#1976d2" />
      </mesh>
    </Canvas>
  );
};
import './Home.css';

const Home: React.FC = () => {
  const [connected, setConnected] = useState(false);
  const [joystickConnected, setJoystickConnected] = useState(false);
  // 3D Controls
  const [cubeRotation, setCubeRotation] = useState(0.5);
  const [cubeZoom, setCubeZoom] = useState(3);
  // Start-Autorisierung und Notmodus
  const [startAuthorized, setStartAuthorized] = useState(false);
  const [emergency, setEmergency] = useState(false);
  // Notmodus prüfen: Wenn Start aktiv und eine Verbindung getrennt wird
  React.useEffect(() => {
    if (startAuthorized && (!connected || !joystickConnected)) {
      setEmergency(true);
    } else {
      setEmergency(false);
    }
    // Start entauthorisieren, falls Verbindung verloren
    if (startAuthorized && (!connected || !joystickConnected)) {
      setStartAuthorized(false);
    }
  }, [startAuthorized, connected, joystickConnected]);

  // Hilfsfunktion für dynamische Card-Klasse
  const cardClass = (base: string) => emergency ? base.replace('blue-card', 'red-card') : base;

  return (
    <IonPage>
      {/* <IonHeader>
        <IonToolbar>
          <IonTitle>Quadcopter Control Panel</IonTitle>
        </IonToolbar>
      </IonHeader> */}
      <IonContent fullscreen className="ion-content">
        <IonGrid style={{height: '100vh', overflow: 'hidden'}}>
          <IonRow style={{height: '100%'}}>
            {/* Connection Container (top left) + Angle/Joystick Container darunter */}
            <IonCol size="3" className="left-col">
              <div style={{display: 'flex', flexDirection: 'column', height: '100%'}}>
                <IonCard className={cardClass('blue-card left-card')} style={{flex: '0 0 auto'}}>
                  <IonCardHeader>
                    <IonCardTitle>Verbindung</IonCardTitle>
                  </IonCardHeader>
                  <IonCardContent>
                    <div style={{display: 'flex', alignItems: 'center', gap: '10px', marginBottom: '10px'}}>
                      <span>Funkverbindung:</span>
                    </div>
                    <div style={{display: 'flex', alignItems: 'center', gap: '10px'}}>
                      <IonButton expand="block" color={connected ? 'danger' : 'success'} className="toggle-btn" onClick={() => setConnected(!connected)} style={{flex: 1}}>
                        {connected ? 'Trennen' : 'Verbinden'}
                      </IonButton>
                      <span className={connected ? 'lamp lamp-green lamp-large' : 'lamp lamp-red lamp-large'} style={{marginLeft: '12px'}}></span>
                    </div>
                    <div style={{marginTop: '18px'}}>
                      <div style={{fontWeight: 'bold', color: '#fff', marginBottom: '6px', display: 'flex', alignItems: 'center', gap: '10px'}}>
                        Joystick Verbindung
                      </div>
                      <div style={{display: 'flex', alignItems: 'center', gap: '10px'}}>
                        <IonButton expand="block" color={joystickConnected ? 'danger' : 'success'} className="toggle-btn" onClick={() => setJoystickConnected(!joystickConnected)} style={{flex: 1}}>
                          {joystickConnected ? 'Trennen' : 'Verbinden'}
                        </IonButton>
                        <span className={joystickConnected ? 'lamp lamp-green lamp-large' : 'lamp lamp-red lamp-large'} style={{marginLeft: '12px'}}></span>
                      </div>
                    </div>
                    {/* Start-Autorisierung Button */}
          <div style={{marginTop: '18px'}}>
            <IonButton
              expand="block"
              color={startAuthorized ? (emergency ? 'danger' : 'success') : 'medium'}
              className="toggle-btn"
              onClick={() => setStartAuthorized(true)}
              disabled={!connected || !joystickConnected || startAuthorized}
            >
              {emergency
                ? 'NOT-AUS'
                : startAuthorized
                  ? 'Gestartet'
                  : 'Start autorisieren'}
            </IonButton>
            {/* Entauthorisieren nur im Normalmodus */}
            {!emergency && startAuthorized && (
              <IonButton
                expand="block"
                color="warning"
                className="toggle-btn"
                style={{marginTop: '8px'}}
                onClick={() => setStartAuthorized(false)}
              >
                Start entauthorisieren
              </IonButton>
            )}
          </div>
                  </IonCardContent>
                </IonCard>
                {/* Neuer Container für Joystick Controls (kompakt) */}
                <IonCard className={cardClass('blue-card left-card')} style={{marginTop: '16px', flex: '1 1 auto', display: 'flex', flexDirection: 'column', justifyContent: 'center', opacity: joystickConnected ? 1 : 0.5, pointerEvents: joystickConnected ? 'auto' : 'none'}}>
                  <IonCardHeader>
                    <IonCardTitle>Joystick Controls</IonCardTitle>
                  </IonCardHeader>
                  <IonCardContent style={{height: '100%', display: 'flex', flexDirection: 'column', justifyContent: 'center'}}>
                    {/* Power Level Plot - Gradient Balken */}
                    <div className="power-bar-container" style={{flex: 1, display: 'flex', flexDirection: 'column', justifyContent: 'center'}}>
                      <div className="power-bar-label" style={{fontSize: '1.1em'}}>Power Level</div>
                      <div className="power-bar" style={{height: '90px', width: '30px'}}>
                        <div className="power-bar-fill" style={{height: '75%'}}></div>
                        <span className="power-bar-value" style={{top: `${100 - 75}%`, fontSize: '1em'}}>75%</span>
                      </div>
                    </div>
                    {/* Joystick Position Plot */}
                    <div className="joystick-pos-container" style={{flex: 1, display: 'flex', flexDirection: 'column', justifyContent: 'center'}}>
                      <div className="joystick-pos-label" style={{fontSize: '1.1em'}}>Position</div>
                      <div className="joystick-pos-rect" style={{width: '160px', height: '100px'}}>
                        <div className="joystick-pos-center" style={{width: '10px', height: '10px'}}></div>
                        <div className="joystick-pos-dot" style={{left: `${80 + 60 * 0.2}px`, top: `${50 - 40 * -0.1}px`, width: '18px', height: '18px'}}></div>
                      </div>
                      <div className="joystick-pos-value" style={{fontSize: '1.1em'}}>X: <strong>0.2</strong> &nbsp; Y: <strong>-0.1</strong></div>
                    </div>
                  </IonCardContent>
                </IonCard>
              </div>
            </IonCol>
            {/* Joystick Controls (Power, Position) im mittleren Container */}
            <IonCol size="5" className="center-col" style={{opacity: connected ? 1 : 0.5, pointerEvents: connected ? 'auto' : 'none'}}>
              <IonCard className={cardClass('blue-card center-card')}>
                <IonCardHeader>
                  <IonCardTitle>Flügelwinkel & 3D-View</IonCardTitle>
                </IonCardHeader>
                <IonCardContent style={{height: 'calc(100vh - 120px)', display: 'flex', flexDirection: 'column', justifyContent: 'space-between'}}>
                  {/* 3D-View oben mit Controls */}
                  <div style={{width: '100%', height: '220px', display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', marginBottom: '16px'}}>
                    <RotatingCube rotation={cubeRotation} zoom={cubeZoom} />
                    <div style={{marginTop: '12px', display: 'flex', gap: '12px', justifyContent: 'center'}}>
                      <IonButton size="small" onClick={() => setCubeRotation(cubeRotation - 0.2)} color="primary">Links drehen</IonButton>
                      <IonButton size="small" onClick={() => setCubeRotation(cubeRotation + 0.2)} color="primary">Rechts drehen</IonButton>
                      <IonButton size="small" onClick={() => setCubeZoom(Math.max(2, cubeZoom - 0.5))} color="secondary">Zoom +</IonButton>
                      <IonButton size="small" onClick={() => setCubeZoom(Math.min(8, cubeZoom + 0.5))} color="secondary">Zoom -</IonButton>
                    </div>
                  </div>
                  {/* Geschwindigkeitsanzeige mittig */}
                  <div style={{width: '100%', display: 'flex', justifyContent: 'center', alignItems: 'center', marginBottom: '24px'}}>
                    <div style={{background: '#e3f2fd', borderRadius: '18px', boxShadow: '0 2px 8px rgba(25, 118, 210, 0.12)', padding: '18px 36px', display: 'flex', flexDirection: 'column', alignItems: 'center'}}>
                      <div style={{fontSize: '1.3em', color: '#1976d2', fontWeight: 'bold', marginBottom: '8px'}}>Geschwindigkeit</div>
                      <svg width="180" height="110" viewBox="0 0 180 110">
                        {/* Skala */}
                        <path d="M20,90 A70,70 0 0,1 160,90" fill="none" stroke="#1976d2" strokeWidth="6" />
                        {/* Ticks */}
                        {Array.from({length: 11}).map((_, i) => {
                          // Skala: 9 Uhr (180°) bis 15 Uhr (360° bzw. 0°)
                          const startAngle = Math.PI;
                          const endAngle = 2 * Math.PI;
                          const angle = startAngle + (endAngle - startAngle) * (i / 10);
                          const x1 = 90 + 65 * Math.cos(angle);
                          const y1 = 90 + 65 * Math.sin(angle);
                          const x2 = 90 + 75 * Math.cos(angle);
                          const y2 = 90 + 75 * Math.sin(angle);
                          return <line key={i} x1={x1} y1={y1} x2={x2} y2={y2} stroke="#1976d2" strokeWidth={i % 5 === 0 ? 4 : 2} />;
                        })}
                        {/* Zahlen */}
                        {[0,2,4,6,8,10].map((v) => {
                          const startAngle = Math.PI;
                          const endAngle = 2 * Math.PI;
                          const angle = startAngle + (endAngle - startAngle) * (v / 10);
                          const x = 90 + 55 * Math.cos(angle);
                          const y = 90 + 55 * Math.sin(angle);
                          return <text key={v} x={x} y={y+7} textAnchor="middle" fontSize="18" fill="#1976d2" alignmentBaseline="middle">{v}</text>;
                        })}
                        {/* Zeiger (für 7.2 m/s) */}
                        {(() => {
                          const speed = 7.2;
                          const maxSpeed = 10;
                          const startAngle = Math.PI;
                          const endAngle = 2 * Math.PI;
                          const angle = startAngle + (endAngle - startAngle) * (speed / maxSpeed);
                          const x2 = 90 + 60 * Math.cos(angle);
                          const y2 = 90 + 60 * Math.sin(angle);
                          return <line x1={90} y1={90} x2={x2} y2={y2} stroke="#d32f2f" strokeWidth="6" />;
                        })()}
                        {/* Mittelpunkt */}
                        <circle cx="90" cy="90" r="10" fill="#1976d2" stroke="#fff" strokeWidth="3" />
                      </svg>
                      <div style={{fontSize: '2em', color: '#1976d2', fontWeight: 'bold', marginTop: '8px'}}>7.2 m/s</div>
                      <div style={{fontSize: '1em', color: '#555'}}>horizontal</div>
                    </div>
                  </div>
                  {/* Winkelanzeigen nebeneinander unten */}
                  <div className="wing-angle-circles" style={{display: 'flex', flexDirection: 'row', justifyContent: 'center', gap: '48px', width: '100%'}}>
                    {/* Linker Flügelwinkel */}
                    <div className="wing-angle-circle">
                      <div className="wing-angle-label" style={{fontSize: '1.3em'}}>Links</div>
                      <svg width="120" height="120" viewBox="0 0 120 120">
                        <circle cx="60" cy="60" r="56" stroke="#fff" strokeWidth="4" fill="#e3f2fd" />
                        <line x1="60" y1="60" x2={60 + 56 * Math.cos((15-90)*Math.PI/180)} y2={60 + 56 * Math.sin((15-90)*Math.PI/180)} stroke="#1976d2" strokeWidth="8" />
                        <text x="60" y="110" textAnchor="middle" fill="#1976d2" fontSize="24">15°</text>
                      </svg>
                    </div>
                    {/* Rechter Flügelwinkel */}
                    <div className="wing-angle-circle">
                      <div className="wing-angle-label" style={{fontSize: '1.3em'}}>Rechts</div>
                      <svg width="120" height="120" viewBox="0 0 120 120">
                        <circle cx="60" cy="60" r="56" stroke="#fff" strokeWidth="4" fill="#e3f2fd" />
                        <line x1="60" y1="60" x2={60 + 56 * Math.cos((17-90)*Math.PI/180)} y2={60 + 56 * Math.sin((17-90)*Math.PI/180)} stroke="#1976d2" strokeWidth="8" />
                        <text x="60" y="110" textAnchor="middle" fill="#1976d2" fontSize="24">17°</text>
                      </svg>
                    </div>
                  </div>
                </IonCardContent>
              </IonCard>
            </IonCol>
            {/* Drehrate & Temperatur Container (rechte Seite) */}
            <IonCol size="4" className="right-col" style={{opacity: connected ? 1 : 0.5, pointerEvents: connected ? 'auto' : 'none'}}>
              <IonCard className={cardClass('blue-card right-card')}>
                <IonCardHeader>
                  <IonCardTitle>Drehrate & ESC Temperatur</IonCardTitle>
                </IonCardHeader>
                <IonCardContent>
                  <div style={{marginBottom: '16px'}}>Controller: <strong>PID aktiv</strong></div>
                  {[1,2,3,4].map((motor) => (
                    <div key={motor} className="motor-bar-row">
                      <div className="motor-label">Motor {motor}</div>
                      {/* Drehrate Bar */}
                      <div className="rpm-bar">
                        <div className="rpm-bar-fill" style={{width: `${[1200,1100,1300,1250][motor-1]/15}%`}}></div>
                        <span className="rpm-bar-value">{[1200,1100,1300,1250][motor-1]} rpm</span>
                      </div>
                      {/* Temperatur Bar */}
                      <div className="temp-bar">
                        <div className="temp-bar-fill" style={{width: `${[55,60,52,58][motor-1]}%`}}></div>
                        <span className="temp-bar-value">{[55,60,52,58][motor-1]}°C</span>
                      </div>
                    </div>
                  ))}
                </IonCardContent>
              </IonCard>
            </IonCol>
            {/* Controller & Motor Container */}
            <IonCol size="4" style={{display: 'flex', flexDirection: 'column', justifyContent: 'center', height: '100%', opacity: connected ? 1 : 0.5, pointerEvents: connected ? 'auto' : 'none'}}>
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
                        <div style={{position: 'absolute', left: 0, top: 0, height: '100%', width: `${[80,78,82,79][motor-1]}%`, background: 'rgba(255,255,255,0.2)', borderRadius: '16px', border: '2px solid #00e676', transition: 'width 0.3s'}}></div>
                        <span style={{position: 'absolute', left: '50%', top: '50%', transform: 'translate(-50%, -50%)', fontWeight: 'bold', color: '#1976d2', fontSize: '1.1em'}}>
                          {[80,78,82,79][motor-1]}%
                        </span>
                      </div>
                      {/* Temperatur Bar */}
                      <div style={{height: '48px', width: '120px', background: 'linear-gradient(to right, #ff9800 0%, #d32f2f 100%)', borderRadius: '16px', position: 'relative', boxShadow: '0 2px 8px rgba(211, 47, 47, 0.2)'}}>
                        <div style={{position: 'absolute', left: 0, top: 0, height: '100%', width: `${[55,60,52,58][motor-1]}%`, background: 'rgba(255,255,255,0.2)', borderRadius: '16px', border: '2px solid #ff9800', transition: 'width 0.3s'}}></div>
                        <span style={{position: 'absolute', left: '50%', top: '50%', transform: 'translate(-50%, -50%)', fontWeight: 'bold', color: '#d32f2f', fontSize: '1.1em'}}>
                          {[55,60,52,58][motor-1]}°C
                        </span>
                      </div>
                    </div>
                  ))}
                </IonCardContent>
              </IonCard>
            </IonCol>
          </IonRow>
        </IonGrid>
      </IonContent>
    </IonPage>
  );
};

export default Home;
