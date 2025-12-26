import React from 'react';
import Footer from '../../components/Footer';
import {
  IonPage,
  IonContent,
  IonCard,
  IonCardHeader,
  IonCardTitle,
  IonCardContent,
} from '@ionic/react';

const ContactPage: React.FC = () => {
  // Minimal contact page — no form

  return (
    <IonPage>
      <IonContent fullscreen style={{ padding: 16, background: 'transparent' }}>
        <IonCard style={{ background: 'transparent', color: 'inherit' }}>
          <IonCardHeader>
            <IonCardTitle>Kontakt</IonCardTitle>
          </IonCardHeader>
          <IonCardContent>
            <p>
              <span style={{ color: '#ffffff' }}>Tim Straube</span><br />
              <a href="mailto:tim.straube@example.com" style={{ color: '#4da6ff' }}>tim.straube@example.com</a><br />
              <a href="https://github.com/TimStraube" target="_blank" rel="noopener noreferrer" style={{ color: '#4da6ff' }}>github.com/TimStraube</a>
            </p>
          </IonCardContent>
        </IonCard>
      </IonContent>
      <Footer />
    </IonPage>
  );
};

export default ContactPage;
