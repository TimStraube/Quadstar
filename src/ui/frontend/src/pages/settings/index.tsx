import React, { useEffect, useState } from 'react';
import { IonPage, IonContent, IonHeader, IonToolbar, IonTitle, IonItem, IonLabel, IonToggle, IonButton } from '@ionic/react';

const SettingsPage: React.FC = () => {
	const [follow, setFollow] = useState<boolean>(() => {
		try { return localStorage.getItem('sim.followQuad') === '1'; } catch(e){ return false; }
	});
	const [autoRotate, setAutoRotate] = useState<boolean>(() => {
		try { const v = localStorage.getItem('sim.autoRotate'); return v === null ? true : v === '1'; } catch(e){ return true; }
	});

	useEffect(() => {
		// no-op: keep local state initialized from localStorage
	}, []);

	const apply = (nextFollow: boolean, nextAuto: boolean) => {
		try {
			localStorage.setItem('sim.followQuad', nextFollow ? '1' : '0');
			localStorage.setItem('sim.autoRotate', nextAuto ? '1' : '0');
		} catch(e) {}
		// notify other parts of the app
		try { window.dispatchEvent(new CustomEvent('settingsChanged', { detail: { follow: nextFollow, autoRotate: nextAuto } })); } catch(e) {}
	};

	return (
		<IonPage>
			<IonHeader>
				<IonToolbar>
					<IonTitle>Einstellungen</IonTitle>
				</IonToolbar>
			</IonHeader>
			<IonContent>
				<div style={{padding:12}}>
					<IonItem>
						<IonLabel>Following (Kamera folgt dem Quad)</IonLabel>
						<IonToggle checked={follow} onIonChange={e=>{ const v = Boolean((e.target as HTMLInputElement).checked); setFollow(v); apply(v, autoRotate); }} />
					</IonItem>
					<IonItem>
						<IonLabel>Auto-Rotate (Szenenrotation)</IonLabel>
						<IonToggle checked={autoRotate} onIonChange={e=>{ const v = Boolean((e.target as HTMLInputElement).checked); setAutoRotate(v); apply(follow, v); }} />
					</IonItem>
					<div style={{marginTop:16}}>
						<IonButton onClick={() => apply(follow, autoRotate)}>Übernehmen</IonButton>
						<IonButton style={{marginLeft:8}} routerLink="/simulation">Zurück zur Simulation</IonButton>
					</div>
				</div>
			</IonContent>
		</IonPage>
	);
};

export default SettingsPage;

