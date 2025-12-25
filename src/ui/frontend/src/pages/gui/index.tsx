import "./style.css";

import {
  IonContent,
  IonPage,
  IonGrid,
  IonRow,
  IonCol,
} from "@ionic/react";
import React, { useState } from "react";
import { connectJoystick, disconnectJoystick } from "../../services/Joystick";
import ConnectionCard from "../../components/Connection";
import InfoCard from "../../components/Info";
import JoystickCard from "../../components/Joystick";
import Controller from "../../components/Controller";

const GUI: React.FC = () => {
  const [connected, setConnected] = useState(false);
  const [joystickConnected, setJoystickConnected] = useState(false);
  // Joystick verbinden/trennen
  const handleJoystickConnect = async () => {
    if (!joystickConnected) {
      await connectJoystick();
      setJoystickConnected(true);
    } else {
      await disconnectJoystick();
      setJoystickConnected(false);
    }
  };
  // Start-Autorisierung und Notmodus
  const [
    startAuthorized,
    setStartAuthorized,
  ] = useState(false);
  const [emergency, setEmergency] =
    useState(false);
  // Notmodus prüfen: Wenn Start aktiv und eine Verbindung getrennt wird
  React.useEffect(() => {
    if (
      startAuthorized &&
      (!connected || !joystickConnected)
    ) {
      setEmergency(true);
    } else {
      setEmergency(false);
    }
    // Start entauthorisieren, falls Verbindung verloren
    if (
      startAuthorized &&
      (!connected || !joystickConnected)
    ) {
      setStartAuthorized(false);
    }
  }, [
    startAuthorized,
    connected,
    joystickConnected,
  ]);

  // Hilfsfunktion für dynamische Card-Klasse
  const cardClass = (base: string) =>
    emergency
      ? base.replace(
          "blue-card",
          "red-card"
        )
      : base;

  return (
    <IonPage>
      {/* <IonHeader>
        <IonToolbar>
          <IonTitle>Quadcopter Control Panel</IonTitle>
        </IonToolbar>
      </IonHeader> */}
      <IonContent
        fullscreen
        className="ion-content"
      >
        <IonGrid
          style={{
            height: "100vh",
            overflow: "hidden",
          }}
        >
          <IonRow
            style={{ height: "100%" }}
          >
            {/* Connection Container (top left) + Angle/Joystick Container darunter */}
            <IonCol>
              <ConnectionCard
                connected={connected}
                setConnected={setConnected}
                joystickConnected={joystickConnected}
                setJoystickConnected={handleJoystickConnect}
                startAuthorized={startAuthorized}
                setStartAuthorized={setStartAuthorized}
                emergency={emergency}
                cardClass={cardClass}
              />
            </IonCol>
            <IonCol>
              <InfoCard
                cardClass={cardClass}
              />
            </IonCol>
            <IonCol>
              <JoystickCard joystickConnected={joystickConnected} />
            </IonCol>
            <IonCol>
              <Controller startAuthorized={startAuthorized} />
            </IonCol>
          </IonRow>
        </IonGrid>
      </IonContent>
    </IonPage>
  );
};

export default GUI;
