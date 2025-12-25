import React, { useEffect, useRef, useState } from "react";
import {
  IonContent,
  IonPage,
  IonButton,
  IonButtons,
} from "@ionic/react";

const Simulation: React.FC = () => {
  const [running, setRunning] = useState(false);
  const [resetFlag, setResetFlag] = useState(false);
  const animationRef = useRef<number | null>(null);
  const pollingRef = useRef<boolean>(false);
  const [threeLoaded, setThreeLoaded] = useState(false);
  const [speed, setSpeed] = useState<number>(1.0); // simulation speed multiplier
  const simulationObjects = useRef<any>({});
  const canvasContainerRef = useRef<HTMLDivElement>(null);
  const pollingDelayRef = useRef<number>(50);

  // --- Simulation Control Functions ---
  const startSimulation = () => {
    console.log('Start pressed');
    setRunning(true);
    setResetFlag(false);
  };
  const pauseSimulation = () => setRunning(false);
  const resetSimulation = () => {
    console.log('Reset pressed');
    setRunning(false);
    setResetFlag(true);
  };

  // Update polling delay when speed changes
  useEffect(() => {
    // base 50ms per poll @ 1x; faster speed -> smaller delay
    const base = 50;
    pollingDelayRef.current = Math.max(5, Math.round(base / speed));
  }, [speed]);

  // --- Three.js and Animation Setup ---
  useEffect(() => {
    // Remove any existing scripts first
    document.querySelectorAll('script[src*="three"]').forEach(s => s.remove());
    setThreeLoaded(false);
    // Remove existing canvas
    document.querySelectorAll('canvas').forEach(c => c.remove());
    // Load Three.js
    const threeScript = document.createElement('script');
    threeScript.src = 'https://cdn.jsdelivr.net/npm/three@0.125.2/build/three.min.js';
    threeScript.onload = () => {
      const controlsScript = document.createElement('script');
      controlsScript.src = 'https://cdn.jsdelivr.net/npm/three@0.125.2/examples/js/controls/OrbitControls.js';
      controlsScript.onload = () => {
        setThreeLoaded(true);
        setupScene();
      };
      document.head.appendChild(controlsScript);
    };
    document.head.appendChild(threeScript);
    return () => {
      document.querySelectorAll('canvas').forEach(c => c.remove());
    };
    // eslint-disable-next-line
  }, [resetFlag]);

  // --- Setup Three.js Scene ---
  const setupScene = () => {
    // @ts-expect-error - THREE.js loaded dynamically
    const THREE = window.THREE;
    if (!THREE) return;
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setClearColor(0x000000, 1);
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.domElement.style.width = '100vw';
    renderer.domElement.style.height = '100vh';
    renderer.domElement.style.display = 'block';
    renderer.domElement.style.position = 'absolute';
    renderer.domElement.style.top = '0';
    renderer.domElement.style.left = '0';
    renderer.domElement.style.zIndex = '1';
    renderer.domElement.style.objectFit = 'cover';
    if (canvasContainerRef.current) {
      canvasContainerRef.current.innerHTML = '';
      canvasContainerRef.current.appendChild(renderer.domElement);
    }
    const controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.enableZoom = true;
    controls.autoRotate = false;
    controls.maxDistance = 10;
    controls.minDistance = 1;
    controls.screenSpacePanning = false;
    const geometry1 = new THREE.BoxGeometry(0.6, 0.02, 0.02);
    const material1 = new THREE.MeshBasicMaterial({color: 0x00ff00});
    const cube1 = new THREE.Mesh(geometry1, material1);
    scene.add(cube1);
    const geometry2 = new THREE.BoxGeometry(0.02, 0.02, 0.6);
    const material2 = new THREE.MeshBasicMaterial({ color: 0x0000ff });
    const cube2 = new THREE.Mesh(geometry2, material2);
    scene.add(cube2);
    const geometry3 = new THREE.BoxGeometry(0.02, 0.2, 0.02);
    const material3 = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    const cube3 = new THREE.Mesh(geometry3, material3);
    cube3.geometry.translate(0, 0.1, 0);
    scene.add(cube3);
    const planeGeometry = new THREE.PlaneGeometry(10, 10);
    const planeMaterial = new THREE.MeshBasicMaterial({
      color: 0x808080,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.3
    });
    const groundPlane = new THREE.Mesh(planeGeometry, planeMaterial);
    groundPlane.rotation.x = -Math.PI / 2;
    groundPlane.position.y = 0;
    scene.add(groundPlane);
    const gridHelper = new THREE.GridHelper(10, 20, 0x444444, 0x444444);
    gridHelper.position.y = 0.01;
    scene.add(gridHelper);
    camera.position.set(0, 5, 3);
    camera.lookAt(0, 0, 0);
    simulationObjects.current = {
      scene, camera, renderer, controls, cube1, cube2, cube3
    };
  };

  // --- Animation & Polling Loop ---
  useEffect(() => {
    if (!running || !threeLoaded) return;
    let t = 0, norden = 0, osten = 0, unten = 0;
    let rollen = 0, nicken = 0, gieren = 0;
    let stop = false;
    pollingRef.current = true;
    // --- Polling Loop ---
    function connectServer() {
      if (!pollingRef.current) return;
      // Backend runs on port 8000 by default in this repo; use that port
      fetch('http://localhost:8000/', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({})
      })
      .then(async response => {
        if (!response.ok) {
          const body = await response.text().catch(() => '<no body>');
          console.error('Server error response', response.status, body);
          // stop polling to avoid flooding the console
          pollingRef.current = false;
          setRunning(false);
          return null;
        }
        return response.json();
      })
      .then(data => {
        if (!data) return;
        const json_data = data;
        // Direct assignment without type/NaN checks or fallbacks
        t = json_data.t;
        norden = json_data.north;
        osten = json_data.east;
        unten = json_data.down !== undefined ? -json_data.down : json_data.down;
        rollen = json_data.roll;
        nicken = json_data.pitch;
        gieren = json_data.yaw;
        const timeElement = document.getElementById('time');
        // Display time with lowercase 't' and ' sec' suffix, formatted to 2 decimals
        if (timeElement) timeElement.textContent = `t = ${Number(t).toFixed(2)} sec`;
        // schedule next poll only while running, delay controlled by slider
        if (pollingRef.current) setTimeout(connectServer, pollingDelayRef.current);
      })
      .catch((err) => {
        console.error('connectServer error', err);
        if (pollingRef.current) setTimeout(connectServer, 500);
      });
    }
    connectServer();
    // --- Animation Loop ---
    function animate() {
      if (!running || stop) return;
      const { cube1, cube2, cube3, controls, renderer, scene, camera } = simulationObjects.current;
      // Update quadcopter position
      cube1.position.x = -osten;
      cube1.position.z = norden;
      cube1.position.y = -unten;
      cube1.rotation.z = rollen;
      cube1.rotation.x = nicken;
      cube1.rotation.y = gieren;
      cube2.position.x = -osten;
      cube2.position.z = norden;
      cube2.position.y = -unten;
      cube2.rotation.z = rollen;
      cube2.rotation.x = nicken;
      cube2.rotation.y = gieren;
      cube3.position.x = -osten;
      cube3.position.z = norden;
      cube3.position.y = -unten;
      cube3.rotation.z = rollen;
      cube3.rotation.x = nicken;
      cube3.rotation.y = gieren;
      controls.update();
      renderer.render(scene, camera);
      animationRef.current = requestAnimationFrame(animate);
    }
    animate();
    return () => {
      stop = true;
      pollingRef.current = false;
      if (animationRef.current) cancelAnimationFrame(animationRef.current);
    };
    // eslint-disable-next-line
  }, [running, threeLoaded]);

  // --- Reset-Flag: Szene neu aufbauen ---
  useEffect(() => {
    if (resetFlag) {
      document.querySelectorAll('canvas').forEach(c => c.remove());
      setTimeout(() => {
        setupScene();
        setResetFlag(false);
      }, 100);
    }
    // eslint-disable-next-line
  }, [resetFlag]);

  return (
    <IonPage>
      <IonContent fullscreen>
        <div style={{
          position: 'fixed',
          top: '5px',
          left: '0',
          right: '0',
          textAlign: 'center',
          zIndex: 100,
          color: 'white',
          fontSize: '24px',
          fontWeight: 'bold',
          textShadow: '2px 2px 4px rgba(0,0,0,0.8)',
          pointerEvents: 'none'
        }}>
          <h1 id="time">t = 0.00 sec</h1>
        </div>
        <div
          id="three-canvas-container"
          ref={canvasContainerRef}
          style={{
            width: '100vw',
            height: '100vh',
            overflow: 'hidden',
            /* kein position: fixed! */
            zIndex: 1,
          }}
        />
        <div style={{
          position: 'fixed',
          bottom: 24,
          right: 24,
          zIndex: 200,
          display: 'flex',
          flexDirection: 'column',
          gap: 12,
          pointerEvents: 'auto',
        }}>
          <IonButton onClick={startSimulation} disabled={running}>Start</IonButton>
          <IonButton onClick={pauseSimulation} disabled={!running}>Pause</IonButton>
          <IonButton onClick={resetSimulation}>Reset</IonButton>
        </div>
        {/* Speed slider - centered at bottom */}
        <div style={{
          position: 'fixed',
          bottom: 24,
          left: '50%',
          transform: 'translateX(-50%)',
          zIndex: 200,
          width: '360px',
          maxWidth: '80%',
          background: 'rgba(0,0,0,0.35)',
          padding: '8px 12px',
          borderRadius: 8,
          color: 'white',
          display: 'flex',
          alignItems: 'center',
          gap: 8,
          pointerEvents: 'auto'
        }}>
          <label style={{fontSize: 14}}>Speed</label>
          <input
            type="range"
            min={0.25}
            max={4}
            step={0.05}
            value={String(speed)}
            onChange={(e) => setSpeed(Number(e.target.value))}
            style={{flex: 1}}
          />
          <div style={{width: 60, textAlign: 'right', fontWeight: '600'}}>{speed.toFixed(2)}x</div>
        </div>
      </IonContent>
    </IonPage>
  );
};

export default Simulation;
