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
	const [showMap, setShowMap] = useState<boolean>(() => {
		try { const v = localStorage.getItem('sim.showMap'); return v === null ? false : v === '1'; } catch(e){ return false; }
	});

	const [wpNodeColor, setWpNodeColor] = useState<string>(() => { try { return localStorage.getItem('sim.wpNodeColor') || '#007bff'; } catch(e){ return '#007bff'; } });
	const [wpLineColor, setWpLineColor] = useState<string>(() => { try { return localStorage.getItem('sim.wpLineColor') || '#007bff'; } catch(e){ return '#007bff'; } });
	const [wpLineWidth, setWpLineWidth] = useState<number>(() => { try { const v = localStorage.getItem('sim.wpLineWidth'); return v === null ? 2 : Number(v); } catch(e){ return 2; } });
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

	const apply = (nextFollow: boolean, nextAuto: boolean, nextShowGrid: boolean, nextShowMap: boolean, nextDefaultSliderVal: number, nextWpNodeColor: string, nextWpLineColor: string, nextWpLineWidth: number) => {
		const actualSpeed = sliderToSpeed(nextDefaultSliderVal);
		try {
			localStorage.setItem('sim.followQuad', nextFollow ? '1' : '0');
			localStorage.setItem('sim.autoRotate', nextAuto ? '1' : '0');
			localStorage.setItem('sim.showGrid', nextShowGrid ? '1' : '0');
			localStorage.setItem('sim.showMap', nextShowMap ? '1' : '0');
			localStorage.setItem('sim.wpNodeColor', nextWpNodeColor || '#007bff');
			localStorage.setItem('sim.wpLineColor', nextWpLineColor || '#007bff');
			localStorage.setItem('sim.wpLineWidth', String(nextWpLineWidth || 2));
			localStorage.setItem('sim.defaultSpeed', String(actualSpeed));
		} catch(e) {}
		// notify other parts of the app
		try {
			window.dispatchEvent(new CustomEvent('settingsChanged', { detail: { follow: nextFollow, autoRotate: nextAuto, showGrid: nextShowGrid, showMap: nextShowMap, defaultSpeed: actualSpeed, nodeColor: nextWpNodeColor, lineColor: nextWpLineColor, lineWidth: nextWpLineWidth } }));
		} catch(e) {}
	};

	return (
		<IonPage>
			<IonContent>
				<div style={{padding:12}}>
					<IonItem>
						<IonLabel>Following (Kamera folgt dem Quad)</IonLabel>
						<IonToggle checked={follow} onIonChange={e=>{ const v = Boolean((e.target as HTMLInputElement).checked); setFollow(v); apply(v, autoRotate, showGrid, showMap, defaultSliderVal, wpNodeColor, wpLineColor, wpLineWidth); }} />
					</IonItem>
					<IonItem>
						<IonLabel>Auto-Rotate (Szenenrotation)</IonLabel>
						<IonToggle checked={autoRotate} onIonChange={e=>{ const v = Boolean((e.target as HTMLInputElement).checked); setAutoRotate(v); apply(follow, v, showGrid, showMap, defaultSliderVal, wpNodeColor, wpLineColor, wpLineWidth); }} />
					</IonItem>
					<IonItem>
						<IonLabel>Gitter anzeigen</IonLabel>
						<IonToggle checked={showGrid} onIonChange={e=>{ const v = Boolean((e.target as HTMLInputElement).checked); setShowGrid(v); apply(follow, autoRotate, v, showMap, defaultSliderVal, wpNodeColor, wpLineColor, wpLineWidth); }} />
					</IonItem>
					<IonItem>
						<IonLabel>OpenStreetMap als Plane</IonLabel>
						<IonToggle checked={showMap} onIonChange={e=>{ const v = Boolean((e.target as HTMLInputElement).checked); setShowMap(v); apply(follow, autoRotate, showGrid, v, defaultSliderVal, wpNodeColor, wpLineColor, wpLineWidth); }} />
					</IonItem>
					<IonItem>
						<IonLabel>Knotenfarbe (Waypoints)</IonLabel>
						<div style={{marginLeft:12}}>
							<input type="color" value={wpNodeColor} onChange={e => { const v = (e.target as HTMLInputElement).value; setWpNodeColor(v); apply(follow, autoRotate, showGrid, showMap, defaultSliderVal, v, wpLineColor, wpLineWidth); }} />
						</div>
					</IonItem>
					<IonItem>
						<IonLabel>Linienfarbe (Waypoints)</IonLabel>
						<div style={{marginLeft:12}}>
							<input type="color" value={wpLineColor} onChange={e => { const v = (e.target as HTMLInputElement).value; setWpLineColor(v); apply(follow, autoRotate, showGrid, showMap, defaultSliderVal, wpNodeColor, v, wpLineWidth); }} />
						</div>
					</IonItem>
					<IonItem>
						<IonLabel>Linienstärke</IonLabel>
						<IonRange min={1} max={50} step={1} value={wpLineWidth} onIonChange={e => { const v = Number((e.detail as any).value); if (!Number.isNaN(v)) { setWpLineWidth(v); apply(follow, autoRotate, showGrid, showMap, defaultSliderVal, wpNodeColor, wpLineColor, v); } }} />
						<div style={{marginLeft:8, minWidth:40, textAlign:'right'}}>{wpLineWidth}px</div>
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
						<IonButton onClick={() => { apply(follow, autoRotate, showGrid, showMap, defaultSliderVal, wpNodeColor, wpLineColor, wpLineWidth); window.location.href = '/simulation'; }} style={{marginLeft:8}}>Zurück zur Simulation</IonButton>
					</div>
				</div>
			</IonContent>
		</IonPage>
	);
};

export default SettingsPage;

