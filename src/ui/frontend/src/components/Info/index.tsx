import React, { useEffect, useState } from "react";
import { IonCard, IonCardHeader, IonCardTitle, IonCardContent } from "@ionic/react";
import { Canvas } from "@react-three/fiber";
import { listenInfoValues, InfoValues } from "../../services/Info";

const RotatingCube: React.FC<{ quaternion: { w: number; x: number; y: number; z: number } }> = ({ quaternion }) => {
  // Quaternion zu Euler (roll, pitch, yaw) konvertieren
  const qw = quaternion.w, qx = quaternion.x, qy = quaternion.y, qz = quaternion.z;
  // Yaw, Pitch, Roll aus Quaternion
  const ysqr = qy * qy;
  // Roll (x-Achse)
  const t0 = +2.0 * (qw * qx + qy * qz);
  const t1 = +1.0 - 2.0 * (qx * qx + ysqr);
  const roll = Math.atan2(t0, t1);
  // Pitch (y-Achse)
  let t2 = +2.0 * (qw * qy - qz * qx);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  const pitch = Math.asin(t2);
  // Yaw (z-Achse)
  const t3 = +2.0 * (qw * qz + qx * qy);
  const t4 = +1.0 - 2.0 * (ysqr + qz * qz);
  const yaw = Math.atan2(t3, t4);
  return (
    <Canvas camera={{ position: [0, 0, 3] }} style={{ width: "200px", height: "200px" }}>
      <ambientLight intensity={0.5} />
      <directionalLight position={[2, 2, 2]} />
      <mesh rotation={[roll, pitch, yaw]}>
        <boxGeometry args={[1, 1, 1]} />
        <meshStandardMaterial color="#90a4ae" />
      </mesh>
    </Canvas>
  );
};

const InfoCard: React.FC<{ cardClass: (base: string) => string }> = ({ cardClass }) => {
  const [info, setInfo] = useState<InfoValues | null>(null);
  useEffect(() => {
    const ws = listenInfoValues(setInfo);
    return () => ws.close();
  }, []);

  const quaternion = info?.quaternion ?? { w: 1, x: 0, y: 0, z: 0 };
  const speed = info?.speed ?? 0;

  return (
    <IonCard className={cardClass("blue-card center-card")}> 
      <IonCardHeader>
        <IonCardTitle>Lage & Geschwindigkeit</IonCardTitle>
      </IonCardHeader>
      <IonCardContent style={{ height: "calc(100vh - 120px)", display: "flex", flexDirection: "column", justifyContent: "space-between" }}>
        {/* 3D-View mit Quaternion */}
        <div style={{ width: "100%", height: "220px", display: "flex", flexDirection: "row", alignItems: "center", justifyContent: "center", gap: "32px", marginBottom: "16px" }}>
          <RotatingCube quaternion={quaternion} />
          {/* Künstliche Flugzeug-Lageanzeige */}
          <div style={{ width: "140px", height: "140px", borderRadius: "50%", background: "var(--input-bg)", boxShadow: "0 2px 8px rgba(0,0,0,0.3)", position: "relative", display: "flex", alignItems: "center", justifyContent: "center" }} className="panel-circle">
            <svg width="140" height="140" viewBox="0 0 140 140" style={{ position: "absolute", left: 0, top: 0 }}>
              <circle cx="70" cy="70" r="68" className="horizon-circle" strokeWidth="4" />
              {/* Horizont-Linie: Pitch */}
              {(() => {
                // Roll und Pitch aus Quaternion
                const qw = quaternion.w, qx = quaternion.x, qy = quaternion.y, qz = quaternion.z;
                const ysqr = qy * qy;
                const t0 = +2.0 * (qw * qx + qy * qz);
                const t1 = +1.0 - 2.0 * (qx * qx + ysqr);
                const roll = Math.atan2(t0, t1);
                let t2 = +2.0 * (qw * qy - qz * qx);
                t2 = t2 > 1.0 ? 1.0 : t2;
                t2 = t2 < -1.0 ? -1.0 : t2;
                const pitch = Math.asin(t2);
                // Horizont-Linie: Mittelpunkt (70,70), Länge 100px, Winkel roll, vertikal verschoben um pitch
                const r = 50;
                const px = 70 + Math.sin(-pitch) * 40; // Pitch nach oben/unten
                const angle = roll;
                const x1 = px + r * Math.cos(angle + Math.PI/2);
                const y1 = 70 + r * Math.sin(angle + Math.PI/2);
                const x2 = px + r * Math.cos(angle - Math.PI/2);
                const y2 = 70 + r * Math.sin(angle - Math.PI/2);
                return <line x1={x1} y1={y1} x2={x2} y2={y2} className="horizon-line" strokeWidth="6" />;
              })()}
              {/* Flugzeug-Symbol */}
              <rect x="60" y="65" width="20" height="10" rx="4" className="aircraft-box" />
              <rect x="67" y="60" width="6" height="20" rx="3" className="aircraft-box" />
            </svg>
          </div>
        </div>
        {/* Geschwindigkeit */}
        <div style={{ width: "100%", display: "flex", justifyContent: "center", alignItems: "center", marginBottom: "24px" }}>
          <div style={{ background: "var(--input-bg)", borderRadius: "18px", boxShadow: "0 2px 8px rgba(0,0,0,0.3)", padding: "18px 36px", display: "flex", flexDirection: "column", alignItems: "center" }}>
            <div style={{ fontSize: "1.3em", color: "var(--panel-color)", fontWeight: "bold", marginBottom: "8px" }}>Geschwindigkeit</div>
            <div style={{ fontSize: "2em", color: "var(--panel-color)", fontWeight: "bold", marginTop: "8px" }}>{speed.toFixed(2)} m/s</div>
            <div style={{ fontSize: "1em", color: "rgba(255,255,255,0.8)" }}>horizontal</div>
          </div>
        </div>
      </IonCardContent>
    </IonCard>
  );
};

export default InfoCard;
