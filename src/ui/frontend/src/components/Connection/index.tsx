import React from "react";
import { IonCard, IonCardHeader, IonCardTitle, IonCardContent, IonButton } from "@ionic/react";

const ConnectionCard: React.FC<{
  connected: boolean;
  setConnected: (v: boolean) => void;
  joystickConnected: boolean;
  setJoystickConnected: (v: boolean) => void;
  startAuthorized: boolean;
  setStartAuthorized: (v: boolean) => void;
  emergency: boolean;
  cardClass: (base: string) => string;
}> = ({
  connected,
  setConnected,
  joystickConnected,
  setJoystickConnected,
  startAuthorized,
  setStartAuthorized,
  emergency,
  cardClass,
}) => (
  <IonCard className={cardClass("blue-card left-card")} style={{ flex: "0 0 auto" }}>
    <IonCardHeader>
      <IonCardTitle>Verbindung</IonCardTitle>
    </IonCardHeader>
    <IonCardContent>
      <div style={{ display: "flex", alignItems: "center", gap: "10px", marginBottom: "10px" }}>
        <span>Funkverbindung:</span>
      </div>
      <div style={{ display: "flex", alignItems: "center", gap: "10px" }}>
        <IonButton
          expand="block"
          color={connected ? "danger" : "success"}
          className="toggle-btn"
          onClick={() => setConnected(!connected)}
          style={{ flex: 1 }}
        >
          {connected ? "Trennen" : "Verbinden"}
        </IonButton>
        <span className={connected ? "lamp lamp-green lamp-large" : "lamp lamp-red lamp-large"} style={{ marginLeft: "12px" }}></span>
      </div>
      <div style={{ marginTop: "18px" }}>
        <div style={{ fontWeight: "bold", color: "var(--panel-color)", marginBottom: "6px", display: "flex", alignItems: "center", gap: "10px" }}>
          Joystick Verbindung
        </div>
        <div style={{ display: "flex", alignItems: "center", gap: "10px" }}>
          <IonButton
            expand="block"
            color={joystickConnected ? "danger" : "success"}
            className="toggle-btn"
            onClick={setJoystickConnected}
            style={{ flex: 1 }}
          >
            {joystickConnected ? "Trennen" : "Verbinden"}
          </IonButton>
          <span className={joystickConnected ? "lamp lamp-green lamp-large" : "lamp lamp-red lamp-large"} style={{ marginLeft: "12px" }}></span>
        </div>
      </div>
      <div style={{ marginTop: "18px" }}>
        <IonButton
          expand="block"
          color={startAuthorized ? (emergency ? "danger" : "success") : "medium"}
          className="toggle-btn"
          onClick={() => setStartAuthorized(true)}
          disabled={!connected || !joystickConnected || startAuthorized}
        >
          {emergency ? "NOT-AUS" : startAuthorized ? "Gestartet" : "Start autorisieren"}
        </IonButton>
        {!emergency && startAuthorized && (
          <IonButton
            expand="block"
            color="warning"
            className="toggle-btn"
            style={{ marginTop: "8px" }}
            onClick={() => setStartAuthorized(false)}
          >
            Start entauthorisieren
          </IonButton>
        )}
      </div>
    </IonCardContent>
  </IonCard>
);

export default ConnectionCard;
