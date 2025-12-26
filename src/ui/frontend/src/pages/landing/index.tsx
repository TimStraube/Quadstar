import React from 'react';
import { IonPage, IonContent, IonButton } from '@ionic/react';

// try to read version from package.json at frontend package root
// relative path from this file -> ../../../package.json
// If JSON imports are not enabled in TS config, this will need adjustment.
let appVersion = '0.0.0';
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
					   <div style={{position: 'fixed', left: 0, top: 0, right: 0, bottom: 0, backgroundImage: "url('/img/0001.jpeg')", backgroundSize: 'cover', backgroundPosition: 'center', backgroundRepeat: 'no-repeat', zIndex: 0, pointerEvents: 'none'}} />
					   <div style={{position: 'absolute', left:0, top:0, right:0, bottom:0, backgroundColor: 'rgba(0,0,0,0.35)', zIndex:1, pointerEvents: 'none'}} />
					   <div style={{position: 'relative', maxWidth: 800, margin: '0 auto', textAlign: 'center', display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', minHeight: '60vh', zIndex: 3, color: '#fff'}}>
					<h1 style={{marginBottom: 8, color: '#fff'}}>THETAFLY</h1>
					<p style={{color: 'rgba(255,255,255,0.9)', marginBottom: 20}}>Simulation and testbench frontend for Thetafly — control, visualize and tune your quadcopter.</p>

					<div style={{marginTop: 16}}>
						<div style={{display: 'flex', gap: 12, justifyContent: 'center'}}>
							<IonButton href="/simulation" routerDirection="forward">Open Simulation</IonButton>
							<IonButton href="/gui" routerDirection="forward" color="tertiary">Open GUI</IonButton>
						</div>
					</div>

					<div style={{marginTop: 36}}>
						<p style={{color: 'rgba(255,255,255,0.9)'}}>If you find this project useful, consider supporting development:</p>
						<a href="https://www.patreon.com/" target="_blank" rel="noopener noreferrer" style={{textDecoration: 'none'}}>
							<IonButton color="secondary">Support on Patreon</IonButton>
						</a>
					</div>

					<div style={{marginTop: 48, color: 'rgba(255,255,255,0.85)', fontSize: 14}}>
						Software version: <strong style={{color: '#fff'}}>{appVersion}</strong>
					</div>
				</div>

				<div style={{position: 'fixed', left: 0, right: 0, bottom: 12, display: 'flex', justifyContent: 'center', width: '100%'}}>
					<small style={{color: 'rgba(255,255,255,0.6)'}}>© {new Date().getFullYear()} Thetafly</small>
				</div>
			</IonContent>
		</IonPage>
	);
};

export default Landing;
