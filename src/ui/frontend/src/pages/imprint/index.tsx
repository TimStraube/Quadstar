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
      <IonContent fullscreen style={{ padding: 16, background: 'transparent' }}>
        <IonCard>
          <IonCardHeader>
            <IonCardTitle>Angaben gemäß § 5 TMG</IonCardTitle>
          </IonCardHeader>
          <IonCardContent>
            <p>
              Thetafly GmbH<br />
              Musterstraße 12<br />
              12345 Musterstadt
            </p>

            <h3>Vertreten durch</h3>
            <p>Geschäftsführer: Max Mustermann</p>

            <h3>Kontakt</h3>
            <p>
              Telefon: +49 30 1234567<br />
              E‑Mail: <a href="mailto:legal@thetafly.local">legal@thetafly.local</a>
            </p>

            <h3>Registereintrag</h3>
            <p>Eintragung im Handelsregister. Registergericht: Amtsgericht Musterstadt. Registernummer: HRB 12345</p>

            <h3>Umsatzsteuer-ID</h3>
            <p>Umsatzsteuer-Identifikationsnummer gemäß §27 a Umsatzsteuergesetz: DE123456789</p>

            <h3>Wirtschafts-ID</h3>
            <p>Wirtschafts-Identifikationsnummer (falls vorhanden): --</p>

            <h3>Haftung für Inhalte</h3>
            <p>
              Als Diensteanbieter sind wir gemäß § 7 Abs.1 TMG für eigene Inhalte auf diesen Seiten nach den allgemeinen Gesetzen verantwortlich. Nach §§ 8 bis 10 TMG sind wir als Diensteanbieter jedoch nicht verpflichtet, übermittelte oder gespeicherte fremde
              Informationen zu überwachen oder nach Umständen zu forschen, die auf eine rechtswidrige Tätigkeit hinweisen.
            </p>

            <h3>Haftung für Links</h3>
            <p>
              Unser Angebot enthält Links zu externen Websites Dritter, auf deren Inhalte wir keinen Einfluss haben. Deshalb können wir für diese fremden Inhalte auch keine Gewähr übernehmen. Für die Inhalte der verlinkten Seiten ist stets der jeweilige
              Anbieter oder Betreiber der Seiten verantwortlich.
            </p>
          </IonCardContent>
        </IonCard>
      </IonContent>
      <Footer />
    </IonPage>
  );
};

export default Imprint;
