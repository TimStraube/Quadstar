import React, { useEffect, useState } from 'react';
import { IonPage, IonContent, IonHeader, IonToolbar, IonTitle, IonItem, IonLabel, IonToggle, IonButton, IonRange } from '@ionic/react';

const SettingsPage: React.FC = () => {
	const [follow, setFollow] = useState<boolean>(() => {
		try { return localStorage.getItem('sim.followQuad') === '1'; } catch(e){ return false; }
	});
	const [autoRotate, setAutoRotate] = useState<boolean>(() => {
		try { const v = localStorage.getItem('sim.autoRotate'); return v === null ? true : v === '1'; } catch(e){ return true; }
	});
	const [showGrid, setShowGrid] = useState<boolean>(() => {
		try { const v = localStorage.getItem('sim.showGrid'); return v === null ? true : v === '1'; } catch(e){ return true; }
	});
	// speed slider mapping matches Simulation's non-linear mapping
	const minSpeed = 0.25;
	const maxSpeed = 100;
	const speedToSlider = (s: number) => {
		if (!s || s <= 0) return 0;
		return Math.log(s / minSpeed) / Math.log(maxSpeed / minSpeed);
	};
	const sliderToSpeed = (v: number) => {
		return minSpeed * Math.pow(maxSpeed / minSpeed, v);
	};
	const [defaultSliderVal, setDefaultSliderVal] = useState<number>(() => {
		try {
			const v = localStorage.getItem('sim.defaultSpeed');
			const num = v === null ? 1.0 : Number(v);
			return speedToSlider(num);
		} catch (e) { return speedToSlider(1.0); }
	});

	useEffect(() => {
		// no-op: keep local state initialized from localStorage
	}, []);

	const apply = (nextFollow: boolean, nextAuto: boolean, nextShowGrid: boolean, nextDefaultSliderVal: number) => {
		const actualSpeed = sliderToSpeed(nextDefaultSliderVal);
		try {
			localStorage.setItem('sim.followQuad', nextFollow ? '1' : '0');
			localStorage.setItem('sim.autoRotate', nextAuto ? '1' : '0');
			localStorage.setItem('sim.showGrid', nextShowGrid ? '1' : '0');
			localStorage.setItem('sim.defaultSpeed', String(actualSpeed));
		} catch(e) {}
		// notify other parts of the app
		try { window.dispatchEvent(new CustomEvent('settingsChanged', { detail: { follow: nextFollow, autoRotate: nextAuto, showGrid: nextShowGrid, defaultSpeed: actualSpeed } })); } catch(e) {}
	};

	return (
		<IonPage>
			<IonContent>
				<div style={{padding:12}}>
					<IonItem>
						<IonLabel>Following (Kamera folgt dem Quad)</IonLabel>
						<IonToggle checked={follow} onIonChange={e=>{ const v = Boolean((e.target as HTMLInputElement).checked); setFollow(v); apply(v, autoRotate, showGrid, defaultSliderVal); }} />
					</IonItem>
					<IonItem>
						<IonLabel>Auto-Rotate (Szenenrotation)</IonLabel>
						<IonToggle checked={autoRotate} onIonChange={e=>{ const v = Boolean((e.target as HTMLInputElement).checked); setAutoRotate(v); apply(follow, v, showGrid, defaultSliderVal); }} />
					</IonItem>
					<IonItem>
						<IonLabel>Gitter anzeigen</IonLabel>
						<IonToggle checked={showGrid} onIonChange={e=>{ const v = Boolean((e.target as HTMLInputElement).checked); setShowGrid(v); apply(follow, autoRotate, v, defaultSliderVal); }} />
					</IonItem>
					<IonItem>
						<IonLabel style={{width: '40%'}}>Startgeschwindigkeit</IonLabel>
						<IonRange min={0} max={1} step={0.01} value={defaultSliderVal} onIonChange={e => {
							const v = Number((e.detail as any).value);
							if (!Number.isNaN(v) && v >= 0 && v <= 1) {
								setDefaultSliderVal(v);
								const actual = sliderToSpeed(v);
								try { localStorage.setItem('sim.defaultSpeed', String(actual)); } catch(e) {}
								try { window.dispatchEvent(new CustomEvent('settingsChanged', { detail: { defaultSpeed: actual } })); } catch(e) {}
							}
						}} />
						<div style={{marginLeft:8, minWidth:80, textAlign:'right'}}>{sliderToSpeed(defaultSliderVal).toFixed(2)}x</div>
					</IonItem>
					<div style={{marginTop:16}}>
						<IonButton onClick={() => { apply(follow, autoRotate, showGrid, defaultSliderVal); window.location.href = '/simulation'; }} style={{marginLeft:8}}>Zurück zur Simulation</IonButton>
					</div>
				</div>
			</IonContent>
		</IonPage>
	);
};

export default SettingsPage;

