import React from 'react';
import Footer from '../../components/Footer';
import {
  IonPage,
  IonHeader,
  IonToolbar,
  IonTitle,
  IonContent,
  IonCard,
  IonCardHeader,
  IonCardTitle,
  IonCardContent,
} from '@ionic/react';

const Imprint: React.FC = () => {
  return (
    <IonPage>
      <IonContent fullscreen style={{ padding: 16, background: 'transparent', color: '#ffffff' }}>
        <IonCard style={{ color: '#ffffff' }}>
          <IonCardHeader>
            <IonCardTitle>Impressum</IonCardTitle>
          </IonCardHeader>
          <IonCardContent style={{ color: '#ffffff' }}>
            <p style={{ color: '#ffffff' }}>
              <span style={{ color: '#ffffff' }}>Tim Straube</span><br />
              Sankt Stephansplatz 14<br />
              78462 Konstanz<br />
              Germany<br />
            </p>
          </IonCardContent>
        </IonCard>
      </IonContent>
      <Footer />
    </IonPage>
  );
};

export default Imprint;
