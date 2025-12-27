import React from 'react';
import Footer from '../../components/Footer';
import { IonPage, IonContent, IonButton, IonCard, IonCardHeader, IonCardTitle, IonCardContent, IonImg, IonIcon } from '@ionic/react';
import { logoPaypal } from 'ionicons/icons';

// try to read version from package.json at frontend package root
// relative path from this file -> ../../../package.json
// If JSON imports are not enabled in TS config, this will need adjustment.
let appVersion = '1.0.0';
try {
	// eslint-disable-next-line @typescript-eslint/no-var-requires
	// @ts-ignore
	const pkg = require('../../../package.json');
	if (pkg && pkg.version) appVersion = String(pkg.version);
} catch (e) {
	// fallback to default above
}

const Landing: React.FC = () => {
	return (
		<IonPage>
			<IonContent fullscreen style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', padding: 24, background: 'transparent' }}>
				{/* fixed full-screen background so Vite/public assets at /img/... are reliably shown */}
					   {/* full-screen solid black background */}
					   <div style={{position: 'fixed', left: 0, top: 0, right: 0, bottom: 0, backgroundColor: '#2b2b2b', zIndex: 0, pointerEvents: 'none'}} />
					   <div style={{position: 'absolute', left:0, top:0, right:0, bottom:0, backgroundColor: 'rgba(0,0,0,0.0)', zIndex:1, pointerEvents: 'none'}} />
					   <div style={{position: 'relative', maxWidth: 800, margin: '0 auto', textAlign: 'center', display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', minHeight: '60vh', zIndex: 3, color: '#fff'}}>
					<h1 style={{marginBottom: 8, color: '#fff'}}>THETAFLY</h1>
					<p style={{color: 'rgba(255,255,255,0.9)', marginBottom: 20}}>Wingcopter simulator</p>

					<div style={{marginTop: 16, width: '100%'}}>
						<div style={{display: 'flex', gap: 18, justifyContent: 'center', flexWrap: 'wrap'}}>
							<IonCard href="/simulation" routerDirection="forward" style={{width: 300, cursor: 'pointer', background: 'linear-gradient(180deg, rgba(255,255,255,0.02), rgba(255,255,255,0.01))', border: '1px solid rgba(255,255,255,0.03)', boxShadow: '0 10px 30px rgba(0,0,0,0.6)', borderRadius: 12, overflow: 'hidden'}}>
								<IonImg src={'/img/quad.png'} style={{height: 160, objectFit: 'cover', filter: 'brightness(0.55) contrast(0.9)'}} />
								<IonCardHeader style={{background: 'transparent', padding: '12px 16px'}}>
									<IonCardTitle style={{ color: '#fff', fontSize: 18, fontWeight: 700 }}>Simulation</IonCardTitle>
								</IonCardHeader>
								<IonCardContent style={{minHeight: 0, paddingBottom: 0}} />
							</IonCard>

							{/* <IonCard href="/gui" routerDirection="forward" style={{width: 300, cursor: 'pointer', background: 'linear-gradient(180deg, rgba(255,255,255,0.02), rgba(255,255,255,0.01))', border: '1px solid rgba(255,255,255,0.03)', boxShadow: '0 10px 30px rgba(0,0,0,0.6)', borderRadius: 12, overflow: 'hidden'}}>
								<IonImg src={'/img/0001.jpeg'} style={{height: 160, objectFit: 'cover', filter: 'brightness(0.5) contrast(0.9)'}} />
								<IonCardHeader style={{background: 'transparent', padding: '12px 16px'}}>
									<IonCardTitle style={{ color: '#fff', fontSize: 18, fontWeight: 700 }}>GUI</IonCardTitle>
								</IonCardHeader>
								<IonCardContent style={{minHeight: 12, paddingBottom: 18}} />
							</IonCard> */}
						</div>
					</div>

					<div style={{marginTop: 36}}>
						<p style={{color: 'rgba(255,255,255,0.9)'}}>If you find this project useful, consider supporting development:</p>
						<a href="https://www.paypal.com/donate" target="_blank" rel="noopener noreferrer" style={{textDecoration: 'none'}}>
							<IonButton className="paypal-button">
								<IonIcon icon={logoPaypal} slot="start" />
								Support
							</IonButton>
						</a>
					</div>

					<div style={{marginTop: 48, color: 'rgba(255,255,255,0.85)', fontSize: 14}}>
						Software version: <strong style={{color: '#fff'}}>{appVersion}</strong>
					</div>
				</div>

				<Footer />
			</IonContent>
		</IonPage>
	);
};

export default Landing;
