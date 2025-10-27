import React, { useEffect, useState } from "react";
import { IonCard, IonCardHeader, IonCardTitle, IonCardContent } from "@ionic/react";
import { listenJoystickValues, JoystickValues } from "../../services/Joystick";

const JoystickCard: React.FC<{ joystickConnected: boolean }> = ({ joystickConnected }) => {
  const [values, setValues] = useState<JoystickValues>({ thrust: 0, roll: 0, pitch: 0, connected: false });

  useEffect(() => {
    if (!joystickConnected) return;
    const ws = listenJoystickValues(setValues);
    return () => ws.close();
  }, [joystickConnected]);

  // Werte für die Anzeige
  const powerPercent = values.thrust;
  const x = values.roll / 100; // Annahme: Wertebereich -1 bis 1
  const y = values.pitch / 100;

  return (
    <IonCard
      className={"blue-card left-card"}
      style={{
        marginTop: "16px",
        flex: "1 1 auto",
        display: "flex",
        flexDirection: "column",
        justifyContent: "center",
        opacity: joystickConnected ? 1 : 0.5,
        pointerEvents: joystickConnected ? "auto" : "none",
      }}
    >
      <IonCardHeader>
        <IonCardTitle>Joystick Controls</IonCardTitle>
      </IonCardHeader>
      <IonCardContent style={{ height: "100%", display: "flex", flexDirection: "column", justifyContent: "center" }}>
        {/* Power Level Plot - Gradient Balken */}
        <div className="power-bar-container" style={{ flex: 1, display: "flex", flexDirection: "column", justifyContent: "center" }}>
          <div className="power-bar-label" style={{ fontSize: "1.1em" }}>Power Level</div>
          <div className="power-bar" style={{ height: "90px", width: "30px" }}>
            <div className="power-bar-fill" style={{ height: `${powerPercent}%` }}></div>
            <span className="power-bar-value" style={{ top: `${100 - powerPercent}%`, fontSize: "1em" }}>{powerPercent}%</span>
          </div>
        </div>
        {/* Joystick Position Plot */}
        <div className="joystick-pos-container" style={{ flex: 1, display: "flex", flexDirection: "column", justifyContent: "center" }}>
          <div className="joystick-pos-label" style={{ fontSize: "1.1em" }}>Position</div>
          <div className="joystick-pos-rect" style={{ width: "160px", height: "100px" }}>
            <div className="joystick-pos-center" style={{ width: "10px", height: "10px" }}></div>
            <div className="joystick-pos-dot" style={{ left: `${80 + 60 * x}px`, top: `${50 - 40 * y}px`, width: "18px", height: "18px" }}></div>
          </div>
          <div className="joystick-pos-value" style={{ fontSize: "1.1em" }}>
            X: <strong>{x.toFixed(2)}</strong> &nbsp; Y: <strong>{y.toFixed(2)}</strong>
          </div>
        </div>
      </IonCardContent>
    </IonCard>
  );
};

export default JoystickCard;
