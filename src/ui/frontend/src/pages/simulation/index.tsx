import React, { useEffect, useRef, useState } from "react";
import WaypointsPanel from './components/WaypointsPanel';
import ControllerPanel from './components/ControllerPanel';
import ParamsPanel from './components/ParamsPanel';
import SimulationPanel from './components/SimulationPanel';
import { getPidValues, setPidValues, postWaypoints } from '../../services/Info';
import {
  IonContent,
  IonPage,
  IonButton
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
  // enable/disable automatic scene rotation
  const [autoRotateEnabled, setAutoRotateEnabled] = useState<boolean>(true);
  // mutable ref that always holds latest auto-rotate state for callbacks created earlier
  const autoRotateRef = useRef<boolean>(autoRotateEnabled);
  // follow mode: when true, camera will follow the quad (unless user interacts)
  const [followQuad, setFollowQuad] = useState<boolean>(false);
  const followQuadRef = useRef<boolean>(followQuad);
  // initialize settings from localStorage and listen for changes from settings page
  useEffect(() => {
    try {
      const f = localStorage.getItem('sim.followQuad');
      const a = localStorage.getItem('sim.autoRotate');
      if (f !== null) {
        const fv = f === '1'; setFollowQuad(fv); followQuadRef.current = fv;
      }
      if (a !== null) {
        const av = a === '1'; setAutoRotateEnabled(av); autoRotateRef.current = av;
      }
    } catch (e) {}
    const handler = (ev: any) => {
      try {
        const d = ev?.detail || {};
        if (typeof d.follow === 'boolean') { setFollowQuad(d.follow); followQuadRef.current = d.follow; }
        if (typeof d.autoRotate === 'boolean') { setAutoRotateEnabled(d.autoRotate); autoRotateRef.current = d.autoRotate; }
        // if controls exist, update them immediately
        try { const c = simulationObjects.current?.controls; if (c) c.autoRotate = autoRotateRef.current && !userInteractingRef.current; } catch(e) {}
      } catch(e){}
    };
    window.addEventListener('settingsChanged', handler as EventListener);
    return () => { window.removeEventListener('settingsChanged', handler as EventListener); };
  }, []);
  // which panel is currently open: 'waypoints' | 'pid' | 'position' | null
  const [openPanel, setOpenPanel] = useState<string | null>('waypoints');
  const [singlePos, setSinglePos] = useState<{north:number,east:number,alt:number}>({north:0,east:0,alt:0});
  // PID parameters state (each is array of 3 numbers: [roll, pitch, yaw] or [x,y,z])
  const [velP, setVelP] = useState<number[]>([5.0, 5.0, 4.0]);
  const [velI, setVelI] = useState<number[]>([5.0, 5.0, 5.0]);
  const [velD, setVelD] = useState<number[]>([0.5, 0.5, 0.5]);
  const [attP, setAttP] = useState<number[]>([8.0, 8.0, 1.5]);
  const [rateP, setRateP] = useState<number[]>([1.5, 1.5, 1.0]);
  const [rateD, setRateD] = useState<number[]>([0.04, 0.04, 0.1]);
  const pidDebounceRef = useRef<number | null>(null);
  // physical parameters configurable by user
  const [mass, setMass] = useState<number>(1.2);
  const [inertia, setInertia] = useState<number[]>([0.01, 0.01, 0.02]);
  const [armLength, setArmLength] = useState<number>(0.18);

  // Apply physical params visually to the Three.js scene (scaling/model params)
  const applyParamsToScene = () => {
    try {
      const objs = simulationObjects.current;
      if (!objs) return;
      // store params for potential future use
      objs.params = { mass, inertia, armLength };
      const defaultArm = 0.18;
      const scale = (armLength > 0) ? (armLength / defaultArm) : 1.0;
      if (objs.model) {
        try { objs.model.scale.set(scale, scale, scale); } catch (e) {}
      } else {
        try { if (objs.cube1) objs.cube1.scale.set(scale, scale, scale); } catch (e) {}
        try { if (objs.cube2) objs.cube2.scale.set(scale, scale, scale); } catch (e) {}
        try { if (objs.cube3) objs.cube3.scale.set(scale, scale, scale); } catch (e) {}
      }
      try { if (objs.renderer && objs.scene && objs.camera) objs.renderer.render(objs.scene, objs.camera); } catch (e) {}
      console.log('Applied params to scene', objs.params);
    } catch (e) {
      console.warn('applyParamsToScene error', e);
    }
  };

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
    (async () => {
      try {
        await postWaypoints({ waypoints: payload, tolerance: wpTolerance });
      } catch (e) {
        console.error('failed to send waypoints', e);
      }
    })();
    // eslint-disable-next-line
  }, [waypoints]);

  // Load current PID values from backend on mount (delegated to service)
  useEffect(() => {
    (async () => {
      const obj = await getPidValues();
      if (!obj) return;
      if (obj.vel_P_gain) setVelP(obj.vel_P_gain);
      if (obj.vel_I_gain) setVelI(obj.vel_I_gain);
      if (obj.vel_D_gain) setVelD(obj.vel_D_gain);
      if (obj.attitute_p_gain) setAttP(obj.attitute_p_gain);
      if (obj.rate_P_gain) setRateP(obj.rate_P_gain);
      if (obj.rate_D_gain) setRateD(obj.rate_D_gain);
    })().catch(() => {});
  }, []);

  // helper: send PID to backend (debounced)
  const schedulePidSend = (payload: any) => {
    if (pidDebounceRef.current) { window.clearTimeout(pidDebounceRef.current); pidDebounceRef.current = null; }
    pidDebounceRef.current = window.setTimeout(() => {
      // sanitize payload: prevent zero/negative gains
      const minGain = 1e-3;
      const sanitize = (obj) => {
        const out = {};
        for (const k of Object.keys(obj)) {
          const v = obj[k];
          if (Array.isArray(v)) out[k] = v.map(x => Math.max(Number(x) || 0, minGain));
          else if (typeof v === 'number') out[k] = Math.max(v, minGain);
          else out[k] = v;
        }
        return out;
      };
      const safePayload = sanitize(payload);
      (async () => {
        const j = await setPidValues(safePayload);
        if (!j) return;
        if (j.vel_P_gain) setVelP(j.vel_P_gain);
        if (j.vel_I_gain) setVelI(j.vel_I_gain);
        if (j.vel_D_gain) setVelD(j.vel_D_gain);
        if (j.attitute_p_gain) setAttP(j.attitute_p_gain);
        if (j.rate_P_gain) setRateP(j.rate_P_gain);
        if (j.rate_D_gain) setRateD(j.rate_D_gain);
      })().catch(() => {});
    }, 250) as unknown as number;
  };

  // --- Simulation Control Functions ---
    // Keep OrbitControls.autoRotate in sync with React state
    useEffect(() => {
      try {
        const objs = simulationObjects.current;
        if (objs && objs.controls) {
          objs.controls.autoRotate = Boolean(autoRotateEnabled) && !userInteractingRef.current;
          // update ref so older timeouts use current value
          autoRotateRef.current = Boolean(autoRotateEnabled);
        }
      } catch (e) { /* ignore */ }
    }, [autoRotateEnabled, threeLoaded]);

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
      } catch (exception) { 
        console.error('failed to draw closing waypoint line', exception);
      }
      // render scene once so lines appear even while paused
      try {
        const objs = simulationObjects.current;
        if (objs && objs.renderer && objs.scene && objs.camera) objs.renderer.render(objs.scene, objs.camera);
      } catch (err) { /* ignore */ }
    }
  };
  const resetSimulation = () => {
    console.log('Reset pressed');
    setRunning(false);
    setResetFlag(true);
  };

  const focusCamera = () => {
    try {
      const objs = simulationObjects.current;
      // @ts-ignore
      const THREE = (window as any).THREE;
      if (!objs || !objs.controls || !objs.camera || !THREE) return;
      let target = new THREE.Vector3(0,0,0);
      if (objs.model && objs.model.position) target.copy(objs.model.position);
      else if (objs.cube1) target.set(objs.cube1.position.x, objs.cube1.position.y, objs.cube1.position.z);
      const camOffset = new THREE.Vector3(4, 2, 0);
      objs.camera.position.copy(target).add(camOffset);
      objs.controls.target.copy(target);
      objs.controls.update();
    } catch (e) { console.error('focus camera error', e); }
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
    // Renderer settings suitable for environment maps
    try {
      renderer.toneMapping = THREE.ACESFilmicToneMapping;
      renderer.toneMappingExposure = 1.0;
      renderer.outputEncoding = THREE.sRGBEncoding;
    } catch (e) { /* ignore if properties missing */ }
    // Try to load an EXR environment map from public/models
    (async () => {
      try {
        // dynamic import of EXRLoader
        // @ts-ignore
        const mod = await import('https://cdn.jsdelivr.net/npm/three@0.125.2/examples/jsm/loaders/EXRLoader.js');
        const EXRLoader = mod.EXRLoader || (mod as any).default;
        const exrLoader = new EXRLoader();
        exrLoader.load('/models/autumn_field_puresky_512.exr', (texture: any) => {
          try {
            const pmremGenerator = new THREE.PMREMGenerator(renderer);
            pmremGenerator.compileEquirectangularShader();
            const envMap = pmremGenerator.fromEquirectangular(texture).texture;
            scene.environment = envMap;
            scene.background = envMap;
            texture.dispose();
            pmremGenerator.dispose();
            // render once so background appears immediately
            try { renderer.render(scene, camera); } catch (e) {}
          } catch (e) {
            console.error('Failed to apply EXR as env map', e);
          }
        }, undefined, (err: any) => {
          console.warn('Failed to load EXR environment', err);
        });
      } catch (err) {
        console.warn('EXRLoader dynamic import failed', err);
      }
    })();
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
    const camElev = 2.0;
    // stronger right offset for a more side-looking camera
    const camX = 4.0;
    const camZ = Math.sqrt(Math.max(0, CAMERA_DISTANCE * CAMERA_DISTANCE - camElev * camElev - camX * camX));
    camera.position.set(camX, camElev, camZ);
    // look slightly off-center to the right
    camera.lookAt(1.0, 0, 0);
    controls.target.set(1.0, 0, 0);

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
        try { controls.autoRotate = !!autoRotateRef.current; } catch (e) {}
        idleTimerRef.current = null;
      }, 3000) as unknown as number;
    });
    // If the user never interacts, enable auto-rotate after initial idle
    if (idleTimerRef.current) window.clearTimeout(idleTimerRef.current);
    idleTimerRef.current = window.setTimeout(() => { try { controls.autoRotate = !!autoRotateRef.current; } catch(e) {} idleTimerRef.current = null; }, 3000) as unknown as number;
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
    // Add simple lighting so GLTF models are visible
    try {
      const hemi = new THREE.HemisphereLight(0xffffff, 0x444444, 0.6);
      hemi.position.set(0, 20, 0);
      scene.add(hemi);
      const dir = new THREE.DirectionalLight(0xffffff, 0.8);
      dir.position.set(-5, 10, 5);
      scene.add(dir);
      const amb = new THREE.AmbientLight(0x666666, 0.4);
      scene.add(amb);
    } catch (e) { /* ignore if THREE missing */ }
    camera.position.set(0, 5, 3);
    camera.lookAt(0, 0, 0);
    // Try to load a local GLTF model if present in `public/models/model.glb`.
    // If loading succeeds, replace the placeholder cubes with the model.
    (async () => {
      try {
        // dynamic import of GLTFLoader (match Three.js version used above)
        // @ts-ignore
        const mod = await import('https://cdn.jsdelivr.net/npm/three@0.125.2/examples/jsm/loaders/GLTFLoader.js');
        const GLTFLoader = mod.GLTFLoader || (mod as any).default;
        const loader = new GLTFLoader();
        loader.load('/models/model.glb', (gltf: any) => {
          try {
            const model = gltf.scene || gltf.scenes?.[0];
            if (!model) {
              console.warn('GLTF loaded but contains no scene');
              return;
            }
            model.traverse((o: any) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
            // Adjust scale/rotation if the model is too large/small (scale down ~4x)
            model.scale.set(0.25, 0.25, 0.25);
            model.rotation.set(0, 0, 0);
            model.position.set(0, 0, 0);
            model.name = 'quadModel';
            scene.add(model);
            simulationObjects.current.model = model;
            // remove placeholder geometry so the model is visible instead
            try { scene.remove(cube1); } catch (e) {}
            try { scene.remove(cube2); } catch (e) {}
            try { scene.remove(cube3); } catch (e) {}
            // render once so the loaded model appears immediately
            try { renderer.render(scene, camera); } catch (e) {}
          } catch (e) {
            console.error('Error when adding GLTF to scene', e);
          }
        }, undefined, (err: any) => {
          console.error('Failed to load GLTF model /models/model.glb', err);
        });
      } catch (err) {
        console.warn('GLTFLoader dynamic import failed', err);
      }
    })();
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
    // set initial quad visual positions and render once so scene is visible before Play
    try {
      const objs = simulationObjects.current;
      objs.cube1.position.set(0, 0, 0);
      objs.cube2.position.set(0, 0, 0);
      objs.cube3.position.set(0, 0, 0);
      objs.cube1.rotation.set(0,0,0);
      objs.cube2.rotation.set(0,0,0);
      objs.cube3.rotation.set(0,0,0);
      objs.renderer.render(objs.scene, objs.camera);
    } catch (e) { /* ignore */ }
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
      // Backend runs on port 5001 (changed to avoid system service using 5000)
      fetch('http://localhost:5001/', {
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
        const { cube1, cube2, cube3, controls, renderer, scene, camera, model } = simulationObjects.current;
        // If a GLTF model was loaded, move/rotate it. Otherwise, update the placeholder cubes.
        if (model) {
          try {
            // mapping: scene x = -east, y = altitude (up), z = north
            model.position.set(-osten, altitude, norden);
            // maintain same rotation mapping as the cubes: rotation.set(x=nicken, y=gieren, z=rollen)
            model.rotation.set(nicken || 0, gieren || 0, rollen || 0);
          } catch (e) { /* ignore model update errors */ }
        } else {
          // Update quadcopter position on placeholder parts
          try {
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
          } catch (e) { /* ignore cube update errors */ }
        }
        // Camera follow mode: smoothly follow the quad when enabled and user not interacting
        try {
          // ensure we use the latest follow flag
          if (followQuadRef.current && !userInteractingRef.current) {
            // @ts-ignore
            const THREE = window.THREE;
            if (THREE && (model || cube1)) {
              const target = new THREE.Vector3();
              if (model && model.position) target.copy(model.position);
              else target.set(cube1.position.x, cube1.position.y, cube1.position.z);
              // desired camera offset relative to target
              const camOffset = new THREE.Vector3(4, 2, 0);
              const desired = new THREE.Vector3().copy(target).add(camOffset);
              // smooth lerp the camera and controls.target
              camera.position.lerp(desired, 0.16);
              controls.target.lerp(target, 0.18);
            }
          }
        } catch (e) { /* ignore follow errors */ }
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
        {/* Settings button top-right */}
        <div style={{position: 'fixed', top: 12, right: 12, zIndex: 300, pointerEvents: 'auto'}}>
          <IonButton href="/settings" routerDirection="forward" color="tertiary">Einstellungen</IonButton>
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
        
        {/* Centered Play / Reset bar above sliders */}
        <div style={{
          position: 'fixed',
          bottom: 100,
          left: '50%',
          transform: 'translateX(-50%)',
          zIndex: 210,
          width: '360px',
          maxWidth: '80%',
          background: 'rgba(0,0,0,0.35)',
          padding: '8px 12px',
          borderRadius: 8,
          color: 'white',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          gap: 12,
          pointerEvents: 'auto'
        }}>
          <IonButton onClick={() => { setRunning(r => !r); }}>
            {running ? 'Pause' : 'Play'}
          </IonButton>
          <IonButton onClick={resetSimulation}>Reset</IonButton>
          <IonButton onClick={focusCamera}>Focus</IonButton>
        </div>
        <div className="panel-stack">
          <WaypointsPanel
            waypoints={waypoints}
            setWaypoints={setWaypoints}
            wpTolerance={wpTolerance}
            setWpTolerance={setWpTolerance}
            openPanel={openPanel}
            setOpenPanel={setOpenPanel}
            activeWaypoint={activeWaypoint}
            setActiveWaypoint={setActiveWaypoint}
            singlePos={singlePos}
            setSinglePos={setSinglePos}
            simulationObjects={simulationObjects}
          />
          <SimulationPanel
            sliderVal={sliderVal}
            setSliderVal={(v) => { setSliderVal(v); setSpeed(sliderToSpeed(v)); }}
            sliderToSpeed={sliderToSpeed}
            wpTolerance={wpTolerance}
            onToleranceChange={(v) => {
              setWpTolerance(v);
              const payload = waypoints.map(w => ({ x: Number(w.x), y: Number(w.y), z: Number(w.z) }));
              (async () => {
                try { await postWaypoints({ waypoints: payload, tolerance: v }); } catch (err) { console.error('failed to send waypoints', err); }
              })();
            }}
            openPanel={openPanel}
            setOpenPanel={setOpenPanel}
          />
          <ParamsPanel
            mass={mass}
            setMass={setMass}
            inertia={inertia}
            setInertia={setInertia}
            armLength={armLength}
            setArmLength={setArmLength}
            openPanel={openPanel}
            setOpenPanel={setOpenPanel}
            onApplyScene={applyParamsToScene}
          />
          <ControllerPanel
          velP={velP}
          setVelP={setVelP}
          velI={velI}
          setVelI={setVelI}
          velD={velD}
          setVelD={setVelD}
          attP={attP}
          setAttP={setAttP}
          rateP={rateP}
          setRateP={setRateP}
          rateD={rateD}
          setRateD={setRateD}
          openPanel={openPanel}
          setOpenPanel={setOpenPanel}
          schedulePidSend={schedulePidSend}
          />
        </div>
      </IonContent>
    </IonPage>
  );
};

export default Simulation;
