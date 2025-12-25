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
  // sliderVal is linear 0..1 but maps to speed logarithmically between minSpeed and maxSpeed
  const minSpeed = 0.25;
  const maxSpeed = 100;
  const speedToSlider = (s: number) => {
    if (s <= 0) return 0;
    return Math.log(s / minSpeed) / Math.log(maxSpeed / minSpeed);
  };
  const sliderToSpeed = (v: number) => {
    return minSpeed * Math.pow(maxSpeed / minSpeed, v);
  };
  const [sliderVal, setSliderVal] = useState<number>(speedToSlider(speed));
  const simulationObjects = useRef<any>({});
  const canvasContainerRef = useRef<HTMLDivElement>(null);
  const pollingDelayRef = useRef<number>(50);
  const userInteractingRef = useRef<boolean>(false);
  const idleTimerRef = useRef<number | null>(null);
  const CAMERA_DISTANCE = 6; // meters
  const [waypoints, setWaypoints] = useState<Array<{x:number,y:number,z:number}>>(
    // initialize with random defaults: x,y in [-3,3], z (altitude) in [0,3]
    Array.from({length:5}, () => ({x: (Math.random() * 6) - 3, y: (Math.random() * 6) - 3, z: (Math.random() * 3)}))
  );
  const [activeWaypoint, setActiveWaypoint] = useState<number | null>(null);
  // waypoint switching tolerance (meters) - user configurable 0..2
  const [wpTolerance, setWpTolerance] = useState<number>(0.2);

  // send waypoints to backend when changed
  useEffect(() => {
    // update visualization
    // update visualization for all waypoints
    waypoints.forEach((_, i) => {
      try { createOrUpdateWaypoint(i); } catch (e) { /* ignore */ }
    });

    // Always send full waypoint list (5 entries) to backend in realtime
    // Frontend waypoint input uses z as altitude (up). Send altitude as-is;
    // backend will convert to 'down' internally. Also include tolerance.
    const payload = waypoints.map(w => ({ x: Number(w.x), y: Number(w.y), z: Number(w.z) }));
    fetch('http://localhost:8000/waypoints', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ waypoints: payload, tolerance: wpTolerance })
    }).catch(err => console.error('failed to send waypoints', err));
    // eslint-disable-next-line
  }, [waypoints]);

  // --- Simulation Control Functions ---
  const startSimulation = () => {
    console.log('Start pressed');
    setRunning(true);
    setResetFlag(false);
  };

  // --- Waypoint helpers ---
  const createOrUpdateWaypoint = (index: number) => {
    // index 0..4
    const pts = waypoints[index];
    if (!simulationObjects.current || !simulationObjects.current.scene) return;
    // @ts-expect-error
    const THREE = window.THREE;
    if (!THREE) return;

    const grp = simulationObjects.current.waypointGroup;
    if (!grp) return;

    // remove existing mesh
    const existing = simulationObjects.current.waypointMeshes[index];
    if (existing) {
      grp.remove(existing);
      simulationObjects.current.waypointMeshes[index] = null;
    }

    // always create a waypoint sphere (default values are 0)
    if (pts) {
      const isActive = (typeof activeWaypoint === 'number' && index === activeWaypoint);
      const radius = isActive ? 0.12 : 0.08;
      const color = isActive ? 0xff3333 : 0xffaa00;
      const sphereGeo = new THREE.SphereGeometry(radius, 12, 12);
      const sphereMat = new THREE.MeshBasicMaterial({color});
      const sphere = new THREE.Mesh(sphereGeo, sphereMat);
      // mapping: input (x=north, y=east, z=altitude/up) -> scene pos: x = -east, y = alt, z = north
      sphere.position.set(-Number(pts.y), Number(pts.z), Number(pts.x));
      grp.add(sphere);
      simulationObjects.current.waypointMeshes[index] = sphere;
    }

    // update lines connecting waypoints
    updateWaypointLine();
  };

  // when active waypoint changes, refresh waypoint visuals
  useEffect(() => {
    try {
      for (let i = 0; i < waypoints.length; i++) {
        createOrUpdateWaypoint(i);
      }
    } catch (e) { /* ignore */ }
    // eslint-disable-next-line
  }, [activeWaypoint]);

  const updateWaypointLine = () => {
    if (!simulationObjects.current || !simulationObjects.current.scene) return;
    // @ts-expect-error
    const THREE = window.THREE;
    if (!THREE) return;
    const grp = simulationObjects.current.waypointGroup;
    if (!grp) return;

    // remove existing line
    if (simulationObjects.current.waypointLine) {
      try { grp.remove(simulationObjects.current.waypointLine); } catch(e) {}
      try { simulationObjects.current.waypointLine.geometry.dispose(); } catch(e) {}
      try { simulationObjects.current.waypointLine.material.dispose(); } catch(e) {}
      simulationObjects.current.waypointLine = null;
    }
    // remove closing line (last->first)
    if (simulationObjects.current.waypointClosingLine) {
      try { grp.remove(simulationObjects.current.waypointClosingLine); } catch(e) {}
      try { simulationObjects.current.waypointClosingLine.geometry.dispose(); } catch(e) {}
      try { simulationObjects.current.waypointClosingLine.material.dispose(); } catch(e) {}
      simulationObjects.current.waypointClosingLine = null;
    }

    const points: any[] = [];
    for (let i = 0; i < waypoints.length; i++) {
      const p = waypoints[i];
      if (p && p.x !== null && p.y !== null && p.z !== null) {
        // use same mapping as above: z is altitude(up) in inputs
        points.push(new THREE.Vector3(-p.y, p.z, p.x));
      }
    }
    if (points.length >= 2) {
      const geom = new THREE.BufferGeometry().setFromPoints(points);
      const mat = new THREE.LineBasicMaterial({color: 0xffaa00});
      const line = new THREE.Line(geom, mat);
      grp.add(line);
      simulationObjects.current.waypointLine = line;
      // draw closing line from last to first
      try {
        const last = points[points.length - 1];
        const first = points[0];
        const closingGeom = new THREE.BufferGeometry().setFromPoints([last, first]);
        const closingMat = new THREE.LineBasicMaterial({color: 0xffaa00});
        const closingLine = new THREE.Line(closingGeom, closingMat);
        grp.add(closingLine);
        simulationObjects.current.waypointClosingLine = closingLine;
      } catch (e) { /* ignore closing line errors */ }
    }
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
    // Avoid loading Three.js multiple times: if window.THREE exists, reuse it.
    setThreeLoaded(false);
    // Remove existing canvas only (don't remove script tags if other parts of app use them)
    document.querySelectorAll('canvas').forEach(c => c.remove());
    // If already loaded, reuse
    // @ts-ignore
    if (typeof window !== 'undefined' && (window as any).THREE) {
      setThreeLoaded(true);
      setupScene();
      return;
    }
    // Load Three.js dynamically
    const threeScript = document.createElement('script');
    threeScript.src = 'https://cdn.jsdelivr.net/npm/three@0.125.2/build/three.min.js';
    threeScript.onload = () => {
      // Load OrbitControls only if not present
      // @ts-ignore
      if ((window as any).THREE && !(window as any).THREE.OrbitControls) {
        const controlsScript = document.createElement('script');
        controlsScript.src = 'https://cdn.jsdelivr.net/npm/three@0.125.2/examples/js/controls/OrbitControls.js';
        controlsScript.onload = () => {
          setThreeLoaded(true);
          setupScene();
        };
        document.head.appendChild(controlsScript);
      } else {
        setThreeLoaded(true);
        setupScene();
      }
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
    // make autorotation faster and rotate to the right (negative = opposite direction)
    controls.autoRotateSpeed = -2; // faster, right-handed
    controls.maxDistance = 10;
    controls.minDistance = 1;
    controls.screenSpacePanning = false;
    // Position camera at fixed distance CAMERA_DISTANCE with a small elevation
    const camElev = 0.5;
    // slight right offset for a 'biased' initial view
    const camX = 1.5;
    const camZ = Math.sqrt(Math.max(0, CAMERA_DISTANCE * CAMERA_DISTANCE - camElev * camElev - camX * camX));
    camera.position.set(camX, camElev, camZ);
    // look slightly off-center (keeps scene feeling a bit 'wrong' initially)
    camera.lookAt(0.2, 0, 0);
    controls.target.set(0.2, 0, 0);

    // Interactivity handling: pause auto-rotate while user interacts, resume after idle
    controls.addEventListener('start', () => {
      userInteractingRef.current = true;
      controls.autoRotate = false;
      if (idleTimerRef.current) { window.clearTimeout(idleTimerRef.current); idleTimerRef.current = null; }
    });
    controls.addEventListener('end', () => {
      userInteractingRef.current = false;
      // resume auto-rotate after 3s of inactivity
      if (idleTimerRef.current) window.clearTimeout(idleTimerRef.current);
      idleTimerRef.current = window.setTimeout(() => {
        controls.autoRotate = true;
        idleTimerRef.current = null;
      }, 3000) as unknown as number;
    });
    // If the user never interacts, enable auto-rotate after initial idle
    if (idleTimerRef.current) window.clearTimeout(idleTimerRef.current);
    idleTimerRef.current = window.setTimeout(() => { controls.autoRotate = true; idleTimerRef.current = null; }, 3000) as unknown as number;
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
    // prepare waypoint group and storage
    const waypointGroup = new THREE.Group();
    scene.add(waypointGroup);

    simulationObjects.current = {
      scene, camera, renderer, controls, cube1, cube2, cube3,
      waypointGroup,
      waypointMeshes: Array(5).fill(null),
      waypointLine: null,
      waypointClosingLine: null
    };
  };

  // --- Animation & Polling Loop ---
  useEffect(() => {
    if (!running || !threeLoaded) return;
    let t = 0, norden = 0, osten = 0, unten = 0;
    let rollen = 0, nicken = 0, gieren = 0;
    // altitude (positive up) derived from backend 'down' (positive down)
    let altitude = 0;
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
        // backend returns 'down' (positive downwards). Convert to altitude (positive up).
          // backend returns 'down' (positive downwards). Convert to altitude (positive up).
          altitude = json_data.down !== undefined ? -json_data.down : json_data.down;
        rollen = json_data.roll;
        nicken = json_data.pitch;
        gieren = json_data.yaw;
        const timeElement = document.getElementById('time');
        // Display time with lowercase 't' and ' sec' suffix, formatted to 2 decimals
        if (timeElement) timeElement.textContent = `t = ${Number(t).toFixed(2)} sec`;
        // schedule next poll only while running, delay controlled by slider
        if (pollingRef.current) setTimeout(connectServer, pollingDelayRef.current);
        // update active waypoint index if provided by backend
        if (json_data.current_wp_index !== undefined && json_data.current_wp_index !== null) {
          setActiveWaypoint(Number(json_data.current_wp_index));
        } else {
          setActiveWaypoint(null);
        }
      })
      .catch((err) => {
        console.error('connectServer error', err);
        if (pollingRef.current) setTimeout(connectServer, 500);
      });
    }
    connectServer();
    // --- Animation Loop ---
    function animate() {
      try {
        if (!running || stop) return;
        const { cube1, cube2, cube3, controls, renderer, scene, camera } = simulationObjects.current;
        // Update quadcopter position
        cube1.position.x = -osten;
        cube1.position.z = norden;
        cube1.position.y = altitude;
        cube1.rotation.z = rollen;
        cube1.rotation.x = nicken;
        cube1.rotation.y = gieren;
        cube2.position.x = -osten;
        cube2.position.z = norden;
        cube2.position.y = altitude;
        cube2.rotation.z = rollen;
        cube2.rotation.x = nicken;
        cube2.rotation.y = gieren;
        cube3.position.x = -osten;
        cube3.position.z = norden;
        cube3.position.y = altitude;
        cube3.rotation.z = rollen;
        cube3.rotation.x = nicken;
        cube3.rotation.y = gieren;
        controls.update();
        renderer.render(scene, camera);
        animationRef.current = requestAnimationFrame(animate);
      } catch (err) {
        console.error('Animation error', err);
        // stop animation loop to avoid repeated errors
        stop = true;
        pollingRef.current = false;
        setRunning(false);
      }
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
          <IonButton onClick={() => { setRunning(r => !r); }}>
            {running ? 'Pause' : 'Play'}
          </IonButton>
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
          <label style={{fontSize: 14}}>Speed (log)</label>
          <input
            type="range"
            min={0}
            max={1}
            step={0.001}
            value={String(sliderVal)}
            onChange={(e) => {
              const v = Number(e.target.value);
              setSliderVal(v);
              const s = sliderToSpeed(v);
              setSpeed(s);
            }}
            style={{flex: 1}}
          />
          <div style={{width: 90, textAlign: 'right', fontWeight: '600'}}>{Number(speed).toFixed(2)}x</div>
        </div>
        {/* Tolerance slider - just above bottom */}
        <div style={{
          position: 'fixed',
          bottom: 80,
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
          <label style={{fontSize: 14}}>Tolerance</label>
          <input
            type="range"
            min={0}
            max={2}
            step={0.01}
            value={String(wpTolerance)}
            onChange={(e) => {
              const v = Number(e.target.value);
              setWpTolerance(v);
              // send current waypoints + new tolerance to backend
              const payload = waypoints.map(w => ({ x: Number(w.x), y: Number(w.y), z: Number(w.z) }));
              fetch('http://localhost:8000/waypoints', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ waypoints: payload, tolerance: v })
              }).catch(err => console.error('failed to send waypoints', err));
            }}
            style={{flex: 1}}
          />
          <div style={{width: 60, textAlign: 'right', fontWeight: '600'}}>{wpTolerance.toFixed(2)} m</div>
        </div>
        {/* Waypoint editor - top-left panel */}
        <div style={{
          position: 'fixed',
          top: 12,
          left: 12,
          zIndex: 250,
          width: 320,
          maxWidth: '90vw',
          background: 'rgba(0,0,0,0.5)',
          color: 'white',
          padding: 8,
          borderRadius: 8,
          pointerEvents: 'auto'
        }}>
          <div style={{fontWeight: 700, marginBottom: 6}}>Waypoints (north, east, alt)</div>
          {waypoints.map((wp, i) => (
            <div key={i} style={{display: 'flex', gap: 6, marginBottom: 6, alignItems: 'center'}}>
              <input type="number" step="0.1" placeholder="north" value={wp.x} onChange={(e)=>{
                const v = e.target.value === '' ? 0 : Number(e.target.value);
                const copy = [...waypoints]; copy[i] = {x:v,y:copy[i].y,z:copy[i].z}; setWaypoints(copy);
              }} style={{width: '32%', background: '#ffffff', color: '#000000', padding: '4px', borderRadius: 4, border: '1px solid rgba(0,0,0,0.2)'}} />
              <input type="number" step="0.1" placeholder="east" value={wp.y} onChange={(e)=>{
                const v = e.target.value === '' ? 0 : Number(e.target.value);
                const copy = [...waypoints]; copy[i] = {x:copy[i].x,y:v,z:copy[i].z}; setWaypoints(copy);
              }} style={{width: '32%', background: '#ffffff', color: '#000000', padding: '4px', borderRadius: 4, border: '1px solid rgba(0,0,0,0.2)'}} />
                  <input type="number" min={0} max={3} step="0.1" placeholder="alt" value={wp.z} onChange={(e)=>{
                    const raw = e.target.value;
                    const num = raw === '' ? 0 : Number(raw);
                    let v = Number.isNaN(num) ? 0 : Math.max(0, num);
                    v = Math.min(3, v);
                    const copy = [...waypoints]; copy[i] = {x:copy[i].x,y:copy[i].y,z:v}; setWaypoints(copy);
                  }} style={{width: '32%', background: '#ffffff', color: '#000000', padding: '4px', borderRadius: 4, border: '1px solid rgba(0,0,0,0.2)'}} />
            </div>
          ))}
          <div style={{display:'flex', gap:6, marginTop:6}}>
            <IonButton onClick={() => {
              // regenerate random waypoints: x,y in [-3,3], z (altitude) in [0,3]
              const newwps = Array.from({length:5}, () => ({x: (Math.random() * 6) - 3, y: (Math.random() * 6) - 3, z: (Math.random() * 3)}));
              setWaypoints(newwps);
            }}>
              Regenerate Waypoints
            </IonButton>
          </div>
        </div>
      </IonContent>
    </IonPage>
  );
};

export default Simulation;
