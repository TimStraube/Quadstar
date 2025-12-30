import React from 'react';
import Footer from '../../components/Footer';
import {
  IonPage,
  IonHeader,
  IonToolbar,
  IonTitle,
  IonContent,
} from '@ionic/react';

const Imprint: React.FC = () => {
  return (
    <IonPage>
      <IonContent fullscreen style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', padding: 24, background: 'transparent' }}>
        {/* landing-like background */}
        <div style={{position: 'fixed', left: 0, top: 0, right: 0, bottom: 0, backgroundColor: '#2b2b2b', zIndex: 0, pointerEvents: 'none'}} />
        <div style={{position: 'absolute', left:0, top:0, right:0, bottom:0, backgroundColor: 'rgba(0,0,0,0.0)', zIndex:1, pointerEvents: 'none'}} />

        {/* centered container (block is centered on page) with left-aligned text inside */}
        <div style={{position: 'relative', maxWidth: 700, margin: '0 auto', textAlign: 'center', display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', minHeight: '60vh', zIndex: 3, color: '#fff'}}>
          <div style={{width: '100%', maxWidth: 560, padding: '0 16px'}}>
            <h1 style={{fontSize: 30, marginBottom: 12}}>Impressum</h1>
            <div style={{textAlign: 'left', color: '#ffffff', lineHeight: 1.5, paddingLeft: 8, paddingRight: 8}}>
              <p style={{margin: 0}}><strong>Tim Straube</strong></p>
              <p style={{margin: '6px 0 0 0'}}>Sankt Stephansplatz 14<br />78462 Konstanz<br />Germany</p>
            </div>
          </div>
        </div>
      </IonContent>
      <Footer />
    </IonPage>
  );
};

export default Imprint;
