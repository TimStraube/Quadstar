import React from "react";
import { IonCard, IonCardHeader, IonCardTitle, IonCardContent, IonButton } from "@ionic/react";
import { Canvas } from "@react-three/fiber";

const RotatingCube: React.FC<{ rotation: number; zoom: number }> = ({ rotation, zoom }) => (
  <Canvas camera={{ position: [0, 0, zoom] }} style={{ width: "200px", height: "200px" }}>
    <ambientLight intensity={0.5} />
    <directionalLight position={[2, 2, 2]} />
    <mesh rotation={[0.4, rotation, 0]}>
      <boxGeometry args={[1, 1, 1]} />
      <meshStandardMaterial color="#1976d2" />
    </mesh>
  </Canvas>
);

const InfoCard: React.FC<{
  cubeRotation: number;
  setCubeRotation: (v: number) => void;
  cubeZoom: number;
  setCubeZoom: (v: number) => void;
  cardClass: (base: string) => string;
}> = ({ cubeRotation, setCubeRotation, cubeZoom, setCubeZoom, cardClass }) => (
  <IonCard className={cardClass("blue-card center-card")}> 
    <IonCardHeader>
      <IonCardTitle>Flügelwinkel & 3D-View</IonCardTitle>
    </IonCardHeader>
    <IonCardContent style={{ height: "calc(100vh - 120px)", display: "flex", flexDirection: "column", justifyContent: "space-between" }}>
      {/* 3D-View oben mit Controls */}
      <div style={{ width: "100%", height: "220px", display: "flex", flexDirection: "column", alignItems: "center", justifyContent: "center", marginBottom: "16px" }}>
        <RotatingCube rotation={cubeRotation} zoom={cubeZoom} />
        <div style={{ marginTop: "12px", display: "flex", gap: "12px", justifyContent: "center" }}>
          <IonButton size="small" onClick={() => setCubeRotation(cubeRotation - 0.2)} color="primary">Links drehen</IonButton>
          <IonButton size="small" onClick={() => setCubeRotation(cubeRotation + 0.2)} color="primary">Rechts drehen</IonButton>
          <IonButton size="small" onClick={() => setCubeZoom(Math.max(2, cubeZoom - 0.5))} color="secondary">Zoom +</IonButton>
          <IonButton size="small" onClick={() => setCubeZoom(Math.min(8, cubeZoom + 0.5))} color="secondary">Zoom -</IonButton>
        </div>
      </div>
      {/* Geschwindigkeitsanzeige mittig */}
      <div style={{ width: "100%", display: "flex", justifyContent: "center", alignItems: "center", marginBottom: "24px" }}>
        <div style={{ background: "#e3f2fd", borderRadius: "18px", boxShadow: "0 2px 8px rgba(25, 118, 210, 0.12)", padding: "18px 36px", display: "flex", flexDirection: "column", alignItems: "center" }}>
          <div style={{ fontSize: "1.3em", color: "#1976d2", fontWeight: "bold", marginBottom: "8px" }}>Geschwindigkeit</div>
          <svg width="180" height="110" viewBox="0 0 180 110">
            <path d="M20,90 A70,70 0 0,1 160,90" fill="none" stroke="#1976d2" strokeWidth="6" />
            {Array.from({ length: 11 }).map((_, i) => {
              const startAngle = Math.PI;
              const endAngle = 2 * Math.PI;
              const angle = startAngle + (endAngle - startAngle) * (i / 10);
              const x1 = 90 + 65 * Math.cos(angle);
              const y1 = 90 + 65 * Math.sin(angle);
              const x2 = 90 + 75 * Math.cos(angle);
              const y2 = 90 + 75 * Math.sin(angle);
              return <line key={i} x1={x1} y1={y1} x2={x2} y2={y2} stroke="#1976d2" strokeWidth={i % 5 === 0 ? 4 : 2} />;
            })}
            {[0,2,4,6,8,10].map((v) => {
              const startAngle = Math.PI;
              const endAngle = 2 * Math.PI;
              const angle = startAngle + (endAngle - startAngle) * (v / 10);
              const x = 90 + 55 * Math.cos(angle);
              const y = 90 + 55 * Math.sin(angle);
              return <text key={v} x={x} y={y+7} textAnchor="middle" fontSize="18" fill="#1976d2" alignmentBaseline="middle">{v}</text>;
            })}
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
            <circle cx="90" cy="90" r="10" fill="#1976d2" stroke="#fff" strokeWidth="3" />
          </svg>
          <div style={{ fontSize: "2em", color: "#1976d2", fontWeight: "bold", marginTop: "8px" }}>7.2 m/s</div>
          <div style={{ fontSize: "1em", color: "#555" }}>horizontal</div>
        </div>
      </div>
      {/* Winkelanzeigen nebeneinander unten */}
      <div className="wing-angle-circles" style={{ display: "flex", flexDirection: "row", justifyContent: "center", gap: "48px", width: "100%" }}>
        <div className="wing-angle-circle">
          <div className="wing-angle-label" style={{ fontSize: "1.3em" }}>Links</div>
          <svg width="120" height="120" viewBox="0 0 120 120">
            <circle cx="60" cy="60" r="56" stroke="#fff" strokeWidth="4" fill="#e3f2fd" />
            <line x1="60" y1="60" x2={60 + 56 * Math.cos((15-90)*Math.PI/180)} y2={60 + 56 * Math.sin((15-90)*Math.PI/180)} stroke="#1976d2" strokeWidth="8" />
            <text x="60" y="110" textAnchor="middle" fill="#1976d2" fontSize="24">15°</text>
          </svg>
        </div>
        <div className="wing-angle-circle">
          <div className="wing-angle-label" style={{ fontSize: "1.3em" }}>Rechts</div>
          <svg width="120" height="120" viewBox="0 0 120 120">
            <circle cx="60" cy="60" r="56" stroke="#fff" strokeWidth="4" fill="#e3f2fd" />
            <line x1="60" y1="60" x2={60 + 56 * Math.cos((17-90)*Math.PI/180)} y2={60 + 56 * Math.sin((17-90)*Math.PI/180)} stroke="#1976d2" strokeWidth="8" />
            <text x="60" y="110" textAnchor="middle" fill="#1976d2" fontSize="24">17°</text>
          </svg>
        </div>
      </div>
    </IonCardContent>
  </IonCard>
);

export default InfoCard;
