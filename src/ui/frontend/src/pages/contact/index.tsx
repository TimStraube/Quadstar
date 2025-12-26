import React, { useState } from 'react';
import Footer from '../../components/Footer';
import {
  IonPage,
  IonContent,
  IonHeader,
  IonToolbar,
  IonTitle,
  IonCard,
  IonCardHeader,
  IonCardTitle,
  IonCardContent,
  IonItem,
  IonLabel,
  IonInput,
  IonTextarea,
  IonButton,
  IonToast
} from '@ionic/react';

const ContactPage: React.FC = () => {
  const [name, setName] = useState('');
  const [email, setEmail] = useState('');
  const [message, setMessage] = useState('');
  const [showToast, setShowToast] = useState(false);

  const handleSubmit = async (e?: React.FormEvent) => {
    if (e) e.preventDefault();
    // Simple client-side validation
    if (!email || !message) {
      setShowToast(true);
      return;
    }

    // Mock send: replace with real API call if desired
    try {
      console.log('Contact submit', { name, email, message });
      // show success feedback
      setName(''); setEmail(''); setMessage('');
      setShowToast(true);
    } catch (err) {
      console.error('Failed to submit contact', err);
      setShowToast(true);
    }
  };

  return (
    <IonPage>
      <IonContent fullscreen style={{ padding: 16, background: 'transparent' }}>
        <IonCard style={{ background: 'transparent', color: 'inherit' }}>
          <IonCardHeader>
            <IonCardTitle>Kontaktformular</IonCardTitle>
          </IonCardHeader>
          <IonCardContent>
            <form onSubmit={handleSubmit}>
              <IonItem>
                <IonLabel position="stacked">Name (optional)</IonLabel>
                <IonInput value={name} placeholder="Dein Name" onIonChange={e => setName(String(e.detail.value || ''))} />
              </IonItem>

              <IonItem>
                <IonLabel position="stacked">E‑Mail</IonLabel>
                <IonInput value={email} placeholder="name@example.com" type="email" onIonChange={e => setEmail(String(e.detail.value || ''))} required />
              </IonItem>

              <IonItem>
                <IonLabel position="stacked">Nachricht</IonLabel>
                <IonTextarea value={message} placeholder="Deine Nachricht" rows={6} onIonChange={e => setMessage(String(e.detail.value || ''))} required />
              </IonItem>

              <div style={{ display: 'flex', gap: 12, marginTop: 12 }}>
                <IonButton type="submit" color="primary">Senden</IonButton>
                <IonButton fill="outline" onClick={() => { setName(''); setEmail(''); setMessage(''); }}>Zurücksetzen</IonButton>
              </div>
            </form>
            <div style={{ marginTop: 18, color: 'rgba(221,221,221,0.9)' }}>
              Oder schreibe direkt an: <a style={{ color: '#4da6ff' }} href="mailto:support@thetafly.local">support@thetafly.local</a>
            </div>
          </IonCardContent>
        </IonCard>

        <IonToast
          isOpen={showToast}
          onDidDismiss={() => setShowToast(false)}
          message={email && message ? 'Deine Nachricht wurde (simuliert) gesendet.' : 'Bitte E‑Mail und Nachricht ausfüllen.'}
          duration={2200}
        />
      </IonContent>
      <Footer />
    </IonPage>
  );
};

export default ContactPage;
