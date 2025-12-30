import React, {
  useEffect,
  useRef,
  useState,
} from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { EXRLoader } from "three/examples/jsm/loaders/EXRLoader";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";
import VehiclesPanel from "./components/VehiclesPanel";
import WaypointsPanel from "./components/WaypointsPanel";
import ControllerPanel from "./components/ControllerPanel";
import ParamsPanel from "./components/ParamsPanel";
import SimulationPanel from "./components/SimulationPanel";
import PositionPanel from "./components/PositionPanel";
import EnergyPanel from "./components/EnergyPanel";
import OptimizerPanel from "./components/OptimizerPanel";
import {
  getPidValues,
  setPidValues,
  postWaypoints,
} from "../../services/Info";
import {
  IonContent,
  IonPage,
  IonButton,
  IonIcon,
  useIonViewWillEnter,
} from "@ionic/react";
import {
  settings as settingsIcon,
  home as homeIcon,
} from "ionicons/icons";

const Simulation: React.FC = () => {
  const [running, setRunning] =
    useState(true); // start simulation automatically when page opens
  const [resetFlag, setResetFlag] =
    useState(false);
  const animationRef = useRef<
    number | null
  >(null);
  const pollingRef =
    useRef<boolean>(false);
  const [threeLoaded, setThreeLoaded] =
    useState(false);
  const [speed, setSpeed] =
    useState<number>(() => {
      try {
        const v = localStorage.getItem(
          "sim.defaultSpeed"
        );
        return v === null
          ? 1.0
          : Number(v);
      } catch (e) {
        return 1.0;
      }
    }); // simulation speed multiplier
  // sliderVal is linear 0..1 but maps to speed logarithmically between minSpeed and maxSpeed
  const minSpeed = 0.25;
  const maxSpeed = 100;
  const speedToSlider = (s: number) => {
    if (s <= 0) return 0;
    return (
      Math.log(s / minSpeed) /
      Math.log(maxSpeed / minSpeed)
    );
  };
  const sliderToSpeed = (v: number) => {
    return (
      minSpeed *
      Math.pow(maxSpeed / minSpeed, v)
    );
  };
  const [sliderVal, setSliderVal] =
    useState<number>(
      speedToSlider(speed)
    );
  const speedRef =
    useRef<number>(speed);
  useEffect(() => {
    speedRef.current = speed;
    try {
      setSliderVal(
        speedToSlider(speed)
      );
    } catch (e) {}
  }, [speed]);
  const [
    energySamples,
    setEnergySamples,
  ] = useState<
    Array<{ t: number; p: number }>
  >([]);
  const simulationObjects = useRef<any>(
    {}
  );
  const canvasContainerRef =
    useRef<HTMLDivElement>(null);
  const pollingDelayRef =
    useRef<number>(50);
  const userInteractingRef =
    useRef<boolean>(false);
  const idleTimerRef = useRef<
    number | null
  >(null);
  const CAMERA_DISTANCE = 6; // meters
  const [waypoints, setWaypoints] =
    useState<
      Array<{
        x: number;
        y: number;
        z: number;
      }>
    >(
      // initialize with random defaults: x,y in [-3,3], z (altitude) in [0,3]
      Array.from({ length: 5 }, () => ({
        x: Math.random() * 6 - 3,
        y: Math.random() * 6 - 3,
        z: Math.random() * 3,
      }))
    );
  const [
    activeWaypoint,
    setActiveWaypoint,
  ] = useState<number | null>(null);
  // vehicles: list of quadcopter systems that can be placed on the map
  const [vehicles, setVehicles] =
    useState<
      Array<{
        name: string;
        x: number;
        y: number;
        z: number;
      }>
    >(
      Array.from({ length: 1 }, () => ({
        name: "veh0",
        x: 0,
        y: 0,
        z: 0,
      }))
    );
  const [
    activeVehicle,
    setActiveVehicle,
  ] = useState<number | null>(null);
  // waypoint switching tolerance (meters) - user configurable 0..2
  const [wpTolerance, setWpTolerance] =
    useState<number>(0.2);
  // enable/disable automatic scene rotation
  const [
    autoRotateEnabled,
    setAutoRotateEnabled,
  ] = useState<boolean>(true);
  // mutable ref that always holds latest auto-rotate state for callbacks created earlier
  const autoRotateRef = useRef<boolean>(
    autoRotateEnabled
  );
  // follow mode: when true, camera will follow the quad (unless user interacts)
  const [followQuad, setFollowQuad] =
    useState<boolean>(false);
  const followQuadRef =
    useRef<boolean>(followQuad);
  const followPrevPosRef =
    useRef<any>(null);
  // show/hide grid in the scene
  const [showGrid, setShowGrid] =
    useState<boolean>(() => {
      try {
        const v = localStorage.getItem(
          "sim.showGrid"
        );
        return v === null
          ? true
          : v === "1";
      } catch (e) {
        return true;
      }
    });
  const showGridRef =
    useRef<boolean>(showGrid);
  // show OpenStreetMap plane under the scene
  const [showMap, setShowMap] =
    useState<boolean>(() => {
      try {
        const v = localStorage.getItem(
          "sim.showMap"
        );
        return v === "1";
      } catch (e) {
        return false;
      }
    });
  const showMapRef =
    useRef<boolean>(showMap);
  // waypoint visuals settings (colors, line width)
  const wpNodeColorRef =
    useRef<string>("#007bff");
  const wpLineColorRef =
    useRef<string>("#007bff");
  const wpLineWidthRef =
    useRef<number>(2);
  // initialize settings from localStorage and listen for changes from settings page
  useEffect(() => {
    try {
      const f = localStorage.getItem(
        "sim.followQuad"
      );
      const a = localStorage.getItem(
        "sim.autoRotate"
      );
      const g = localStorage.getItem(
        "sim.showGrid"
      );
      const ds = localStorage.getItem(
        "sim.defaultSpeed"
      );
      if (f !== null) {
        const fv = f === "1";
        setFollowQuad(fv);
        followQuadRef.current = fv;
      }
      if (a !== null) {
        const av = a === "1";
        setAutoRotateEnabled(av);
        autoRotateRef.current = av;
      }
      if (g !== null) {
        const gv = g === "1";
        setShowGrid(gv);
        showGridRef.current = gv;
      }
      if (ds !== null) {
        const v = Number(ds);
        if (!Number.isNaN(v) && v > 0) {
          setSpeed(v);
          speedRef.current = v;
          try {
            setSliderVal(
              speedToSlider(v)
            );
          } catch (e) {}
        }
      }
      try {
        const nc = localStorage.getItem(
          "sim.wpNodeColor"
        );
        if (nc !== null)
          wpNodeColorRef.current = nc;
      } catch (e) {}
      try {
        const lc = localStorage.getItem(
          "sim.wpLineColor"
        );
        if (lc !== null)
          wpLineColorRef.current = lc;
      } catch (e) {}
      try {
        const lw = localStorage.getItem(
          "sim.wpLineWidth"
        );
        if (lw !== null)
          wpLineWidthRef.current =
            Number(lw);
      } catch (e) {}
    } catch (e) {}
    const handler = (ev: any) => {
      try {
        const d = ev?.detail || {};
        if (
          typeof d.follow === "boolean"
        ) {
          setFollowQuad(d.follow);
          followQuadRef.current =
            d.follow;
        }
        if (
          typeof d.autoRotate ===
          "boolean"
        ) {
          setAutoRotateEnabled(
            d.autoRotate
          );
          autoRotateRef.current =
            d.autoRotate;
        }
        if (
          typeof d.showGrid ===
          "boolean"
        ) {
          setShowGrid(d.showGrid);
          showGridRef.current =
            d.showGrid;
          try {
            const gh =
              simulationObjects.current
                ?.gridHelper;
            if (gh)
              gh.visible = d.showGrid;
          } catch (e) {}
          try {
            const gp =
              simulationObjects.current
                ?.groundPlane;
            if (gp)
              gp.visible = d.showGrid;
          } catch (e) {}
        }
        if (
          typeof d.showMap === "boolean"
        ) {
          setShowMap(d.showMap);
          showMapRef.current =
            d.showMap;
          try {
            if (d.showMap)
              addMapPlane();
            else removeMapPlane();
          } catch (e) {}
        }
        if (
          typeof d.defaultSpeed !==
          "undefined"
        ) {
          const v = Number(
            d.defaultSpeed
          );
          if (
            !Number.isNaN(v) &&
            v > 0
          ) {
            setSpeed(v);
            speedRef.current = v;
            try {
              setSliderVal(
                speedToSlider(v)
              );
            } catch (e) {}
          }
        }
        if (
          typeof d.nodeColor ===
          "string"
        ) {
          try {
            wpNodeColorRef.current =
              d.nodeColor;
            for (
              let i = 0;
              i <
              (simulationObjects.current
                ?.waypointMeshes
                ?.length || 0);
              i++
            )
              try {
                createOrUpdateWaypoint(
                  i
                );
              } catch (e) {}
          } catch (e) {}
        }
        if (
          typeof d.lineColor ===
          "string"
        ) {
          try {
            wpLineColorRef.current =
              d.lineColor;
            try {
              updateWaypointLine();
            } catch (e) {}
          } catch (e) {}
        }
        if (
          typeof d.lineWidth ===
          "number"
        ) {
          try {
            wpLineWidthRef.current =
              Number(d.lineWidth);
            try {
              updateWaypointLine();
            } catch (e) {}
          } catch (e) {}
        }
        // if controls exist, update them immediately
        try {
          const c =
            simulationObjects.current
              ?.controls;
          if (c)
            c.autoRotate =
              autoRotateRef.current &&
              !userInteractingRef.current;
        } catch (e) {}
      } catch (e) {}
    };
    window.addEventListener(
      "settingsChanged",
      handler as EventListener
    );
    return () => {
      window.removeEventListener(
        "settingsChanged",
        handler as EventListener
      );
    };
  }, []);

  // Re-apply settings when the view becomes active (handles Ionic page caching)
  useIonViewWillEnter(() => {
    try {
      const f = localStorage.getItem(
        "sim.followQuad"
      );
      const a = localStorage.getItem(
        "sim.autoRotate"
      );
      const g = localStorage.getItem(
        "sim.showGrid"
      );
      const ds = localStorage.getItem(
        "sim.defaultSpeed"
      );
      if (f !== null) {
        const fv = f === "1";
        setFollowQuad(fv);
        followQuadRef.current = fv;
      }
      if (a !== null) {
        const av = a === "1";
        setAutoRotateEnabled(av);
        autoRotateRef.current = av;
      }
      if (g !== null) {
        const gv = g === "1";
        setShowGrid(gv);
        showGridRef.current = gv;
        try {
          const gh =
            simulationObjects.current
              ?.gridHelper;
          if (gh) gh.visible = gv;
        } catch (e) {}
        try {
          const gp =
            simulationObjects.current
              ?.groundPlane;
          if (gp) gp.visible = gv;
        } catch (e) {}
      }
      if (ds !== null) {
        const v = Number(ds);
        if (!Number.isNaN(v) && v > 0) {
          setSpeed(v);
          speedRef.current = v;
          try {
            setSliderVal(
              speedToSlider(v)
            );
          } catch (e) {}
        }
      }
      try {
        const sm = localStorage.getItem(
          "sim.showMap"
        );
        if (sm !== null) {
          const m = sm === "1";
          setShowMap(m);
          showMapRef.current = m;
        }
      } catch (e) {}
      // Update controls immediately if present
      try {
        const c =
          simulationObjects.current
            ?.controls;
        if (c)
          c.autoRotate =
            autoRotateRef.current &&
            !userInteractingRef.current;
      } catch (e) {}
    } catch (e) {}
  });
  // which panel is currently open: 'waypoints' | 'pid' | 'position' | null
  const [openPanel, setOpenPanel] =
    useState<string | null>(
      "waypoints"
    );
  const [singlePos, setSinglePos] =
    useState<{
      north: number;
      east: number;
      alt: number;
    }>({ north: 0, east: 0, alt: 0 });
  // PID parameters state (each is array of 3 numbers: [roll, pitch, yaw] or [x,y,z])
  const [velP, setVelP] = useState<
    number[]
  >([5.0, 5.0, 4.0]);
  const [velI, setVelI] = useState<
    number[]
  >([5.0, 5.0, 5.0]);
  const [velD, setVelD] = useState<
    number[]
  >([0.5, 0.5, 0.5]);
  const [attP, setAttP] = useState<
    number[]
  >([8.0, 8.0, 1.5]);
  const [rateP, setRateP] = useState<
    number[]
  >([1.5, 1.5, 1.0]);
  const [rateD, setRateD] = useState<
    number[]
  >([0.04, 0.04, 0.1]);
  const pidDebounceRef = useRef<
    number | null
  >(null);
  // physical parameters configurable by user
  const [mass, setMass] =
    useState<number>(1.2);
  const [inertia, setInertia] =
    useState<number[]>([
      0.01, 0.01, 0.02,
    ]);
  const [armLength, setArmLength] =
    useState<number>(0.18);

  // Apply physical params visually to the Three.js scene (scaling/model params)
  const applyParamsToScene = () => {
    try {
      const objs =
        simulationObjects.current;
      if (!objs) return;
      // store params for potential future use
      objs.params = {
        mass,
        inertia,
        armLength,
      };
      const defaultArm = 0.18;
      const scale =
        armLength > 0
          ? armLength / defaultArm
          : 1.0;
      if (objs.model) {
        try {
          objs.model.scale.set(
            scale,
            scale,
            scale
          );
        } catch (e) {}
      } else {
        try {
          if (objs.cube1)
            objs.cube1.scale.set(
              scale,
              scale,
              scale
            );
        } catch (e) {}
        try {
          if (objs.cube2)
            objs.cube2.scale.set(
              scale,
              scale,
              scale
            );
        } catch (e) {}
        try {
          if (objs.cube3)
            objs.cube3.scale.set(
              scale,
              scale,
              scale
            );
        } catch (e) {}
      }
      try {
        if (
          objs.renderer &&
          objs.scene &&
          objs.camera
        )
          objs.renderer.render(
            objs.scene,
            objs.camera
          );
      } catch (e) {}
      console.log(
        "Applied params to scene",
        objs.params
      );
    } catch (e) {
      console.warn(
        "applyParamsToScene error",
        e
      );
    }
  };

  // send waypoints to backend when changed
  useEffect(() => {
    // cleanup any leftover waypoint meshes for indices that no longer exist
    try {
      const objs =
        simulationObjects.current || {};
      if (
        objs &&
        Array.isArray(
          objs.waypointMeshes
        ) &&
        objs.waypointGroup
      ) {
        for (
          let idx = waypoints.length;
          idx <
          objs.waypointMeshes.length;
          idx++
        ) {
          const existing =
            objs.waypointMeshes[idx];
          if (existing) {
            try {
              objs.waypointGroup.remove(
                existing
              );
            } catch (e) {}
            try {
              if (existing.geometry)
                existing.geometry.dispose();
            } catch (e) {}
            try {
              if (existing.material)
                existing.material.dispose();
            } catch (e) {}
            objs.waypointMeshes[idx] =
              null;
          }
        }
      }
    } catch (e) {
      /* ignore cleanup errors */
    }

    // update visualization for all current waypoints
    waypoints.forEach((_, i) => {
      try {
        createOrUpdateWaypoint(i);
      } catch (e) {
        /* ignore */
      }
    });

    // Always send full waypoint list (5 entries) to backend in realtime
    // Frontend waypoint input uses z as altitude (up). Send altitude as-is;
    // backend will convert to 'down' internally. Also include tolerance.
    const payload = waypoints.map(
      (w) => ({
        x: Number(w.x),
        y: Number(w.y),
        z: Number(w.z),
      })
    );
    (async () => {
      try {
        await postWaypoints({
          waypoints: payload,
          tolerance: wpTolerance,
        });
      } catch (e) {
        console.error(
          "failed to send waypoints",
          e
        );
      }
    })();
    // eslint-disable-next-line
  }, [waypoints]);

  // Load current PID values from backend on mount (delegated to service)
  useEffect(() => {
    (async () => {
      const obj = await getPidValues();
      if (!obj) return;
      if (obj.vel_P_gain)
        setVelP(obj.vel_P_gain);
      if (obj.vel_I_gain)
        setVelI(obj.vel_I_gain);
      if (obj.vel_D_gain)
        setVelD(obj.vel_D_gain);
      if (obj.attitute_p_gain)
        setAttP(obj.attitute_p_gain);
      if (obj.rate_P_gain)
        setRateP(obj.rate_P_gain);
      if (obj.rate_D_gain)
        setRateD(obj.rate_D_gain);
    })().catch(() => {});
  }, []);

  // helper: send PID to backend (debounced)
  const schedulePidSend = (
    payload: any
  ) => {
    if (pidDebounceRef.current) {
      window.clearTimeout(
        pidDebounceRef.current
      );
      pidDebounceRef.current = null;
    }
    pidDebounceRef.current =
      window.setTimeout(() => {
        // sanitize payload: prevent zero/negative gains
        const minGain = 1e-3;
        const sanitize = (
          obj: Record<string, unknown>
        ) => {
          const out: Record<
            string,
            unknown
          > = {};
          for (const k of Object.keys(
            obj
          )) {
            const v = (obj as any)[k];
            if (Array.isArray(v))
              out[k] = (v as any[]).map(
                (x) =>
                  Math.max(
                    Number(x) || 0,
                    minGain
                  )
              );
            else if (
              typeof v === "number"
            )
              out[k] = Math.max(
                v,
                minGain
              );
            else out[k] = v;
          }
          return out;
        };
        const safePayload =
          sanitize(payload);
        (async () => {
          const j = await setPidValues(
            safePayload
          );
          if (!j) return;
          if (j.vel_P_gain)
            setVelP(j.vel_P_gain);
          if (j.vel_I_gain)
            setVelI(j.vel_I_gain);
          if (j.vel_D_gain)
            setVelD(j.vel_D_gain);
          if (j.attitute_p_gain)
            setAttP(j.attitute_p_gain);
          if (j.rate_P_gain)
            setRateP(j.rate_P_gain);
          if (j.rate_D_gain)
            setRateD(j.rate_D_gain);
        })().catch(() => {});
      }, 250) as unknown as number;
  };

  // --- Simulation Control Functions ---
  // Keep OrbitControls.autoRotate in sync with React state
  useEffect(() => {
    try {
      const objs =
        simulationObjects.current;
      if (objs && objs.controls) {
        objs.controls.autoRotate =
          Boolean(autoRotateEnabled) &&
          !userInteractingRef.current;
        // update ref so older timeouts use current value
        autoRotateRef.current = Boolean(
          autoRotateEnabled
        );
      }
    } catch (e) {
      /* ignore */
    }
  }, [autoRotateEnabled, threeLoaded]);

  // --- Waypoint helpers ---
  const createOrUpdateWaypoint = (
    index: number
  ) => {
    // index 0..4
    const pts = waypoints[index];
    if (
      !simulationObjects.current ||
      !simulationObjects.current.scene
    )
      return;
    // @ts-expect-error
    const THREE = window.THREE;
    if (!THREE) return;

    const grp =
      simulationObjects.current
        .waypointGroup;
    if (!grp) return;

    // remove existing mesh
    const existing =
      simulationObjects.current
        .waypointMeshes[index];
    if (existing) {
      grp.remove(existing);
      simulationObjects.current.waypointMeshes[
        index
      ] = null;
    }

    // always create a waypoint sphere (default values are 0)
    if (pts) {
      const isActive =
        typeof activeWaypoint ===
          "number" &&
        index === activeWaypoint;
      const radius = isActive
        ? 0.12
        : 0.08;
      // Active waypoint remains red; inactive color configurable
      const inactiveColor =
        wpNodeColorRef.current ||
        "#007bff";
      const color = isActive
        ? 0xff3333
        : inactiveColor;
      const sphereGeo =
        new THREE.SphereGeometry(
          radius,
          12,
          12
        );
      const sphereMat =
        new THREE.MeshBasicMaterial({
          color,
        });
      const sphere = new THREE.Mesh(
        sphereGeo,
        sphereMat
      );
      // mapping: input (x=north, y=east, z=altitude/up) -> scene pos: x = -east, y = alt, z = north
      sphere.position.set(
        -Number(pts.y),
        Number(pts.z),
        Number(pts.x)
      );
      grp.add(sphere);
      simulationObjects.current.waypointMeshes[
        index
      ] = sphere;
    }

    // update lines connecting waypoints
    updateWaypointLine();
  };

  // when active waypoint changes, refresh waypoint visuals
  useEffect(() => {
    try {
      for (
        let i = 0;
        i < waypoints.length;
        i++
      ) {
        createOrUpdateWaypoint(i);
      }
    } catch (e) {
      /* ignore */
    }
    // eslint-disable-next-line
  }, [activeWaypoint]);

  const updateWaypointLine = () => {
    if (
      !simulationObjects.current ||
      !simulationObjects.current.scene
    )
      return;
    // @ts-expect-error
    const THREE = window.THREE;
    if (!THREE) return;
    const grp =
      simulationObjects.current
        .waypointGroup;
    if (!grp) return;

    // remove existing line
    if (
      simulationObjects.current
        .waypointLine
    ) {
      try {
        grp.remove(
          simulationObjects.current
            .waypointLine
        );
      } catch (e) {}
      try {
        simulationObjects.current.waypointLine.geometry.dispose();
      } catch (e) {}
      try {
        simulationObjects.current.waypointLine.material.dispose();
      } catch (e) {}
      simulationObjects.current.waypointLine =
        null;
    }
    // remove closing line (last->first)
    if (
      simulationObjects.current
        .waypointClosingLine
    ) {
      try {
        grp.remove(
          simulationObjects.current
            .waypointClosingLine
        );
      } catch (e) {}
      try {
        simulationObjects.current.waypointClosingLine.geometry.dispose();
      } catch (e) {}
      try {
        simulationObjects.current.waypointClosingLine.material.dispose();
      } catch (e) {}
      simulationObjects.current.waypointClosingLine =
        null;
    }

    const points: any[] = [];
    for (
      let i = 0;
      i < waypoints.length;
      i++
    ) {
      const p = waypoints[i];
      if (
        p &&
        p.x !== null &&
        p.y !== null &&
        p.z !== null
      ) {
        // use same mapping as above: z is altitude(up) in inputs
        points.push(
          new THREE.Vector3(
            -p.y,
            p.z,
            p.x
          )
        );
      }
    }
    if (points.length >= 2) {
      const lineColor =
        wpLineColorRef &&
        wpLineColorRef.current
          ? wpLineColorRef.current
          : "#007bff";
      const lineWidthPx =
        wpLineWidthRef &&
        wpLineWidthRef.current
          ? wpLineWidthRef.current
          : 2;
      // Some platforms ignore LineBasicMaterial.linewidth. For visible thick lines everywhere,
      // build the line as a series of thin cylinders (mesh) whose radius is derived from the requested px width.
      const useMeshLines = true; // always enable robust mesh-based thick lines
      if (useMeshLines) {
        // remove any previous mesh group
        try {
          if (
            simulationObjects.current
              .waypointLineMeshGroup
          ) {
            const oldGroup =
              simulationObjects.current
                .waypointLineMeshGroup;
            try {
              grp.remove(oldGroup);
            } catch (e) {}
            oldGroup.traverse(
              (o: any) => {
                try {
                  if (o.geometry)
                    o.geometry.dispose();
                } catch (e) {}
                try {
                  if (o.material)
                    o.material.dispose();
                } catch (e) {}
              }
            );
            simulationObjects.current.waypointLineMeshGroup =
              null;
          }
        } catch (e) {}

        const meshGroup =
          new THREE.Group();
        const radiusWorld = Math.max(
          0.01,
          lineWidthPx * 0.01
        ); // heuristic: px->meters
        for (
          let i = 0;
          i < points.length - 1;
          i++
        ) {
          const a = points[i];
          const b = points[i + 1];
          const dir =
            new THREE.Vector3().subVectors(
              b,
              a
            );
          const length = dir.length();
          if (length <= 0) continue;
          const midpoint =
            new THREE.Vector3()
              .addVectors(a, b)
              .multiplyScalar(0.5);
          // cylinder aligned with Y axis; create then rotate
          const cylGeo =
            new THREE.CylinderGeometry(
              radiusWorld,
              radiusWorld,
              length,
              8
            );
          const cylMat =
            new THREE.MeshBasicMaterial(
              { color: lineColor }
            );
          const cyl = new THREE.Mesh(
            cylGeo,
            cylMat
          );
          // orient cylinder to match segment direction
          cyl.position.copy(midpoint);
          const up = new THREE.Vector3(
            0,
            1,
            0
          );
          const quat =
            new THREE.Quaternion().setFromUnitVectors(
              up,
              dir.clone().normalize()
            );
          cyl.quaternion.copy(quat);
          meshGroup.add(cyl);
        }
        // closing segment
        try {
          const last =
            points[points.length - 1];
          const first = points[0];
          const dir =
            new THREE.Vector3().subVectors(
              first,
              last
            );
          const length = dir.length();
          if (length > 0) {
            const midpoint =
              new THREE.Vector3()
                .addVectors(last, first)
                .multiplyScalar(0.5);
            const cylGeo =
              new THREE.CylinderGeometry(
                radiusWorld,
                radiusWorld,
                length,
                8
              );
            const cylMat =
              new THREE.MeshBasicMaterial(
                { color: lineColor }
              );
            const cyl = new THREE.Mesh(
              cylGeo,
              cylMat
            );
            cyl.position.copy(midpoint);
            const up =
              new THREE.Vector3(
                0,
                1,
                0
              );
            const quat =
              new THREE.Quaternion().setFromUnitVectors(
                up,
                dir.clone().normalize()
              );
            cyl.quaternion.copy(quat);
            meshGroup.add(cyl);
          }
        } catch (e) {
          /* ignore closing errors */
        }
        grp.add(meshGroup);
        simulationObjects.current.waypointLineMeshGroup =
          meshGroup;
        // no separate waypointLine/ClosingLine when using meshGroup
        simulationObjects.current.waypointLine =
          null;
        simulationObjects.current.waypointClosingLine =
          null;
      } else {
        // fallback to simple Line
        const geom =
          new THREE.BufferGeometry().setFromPoints(
            points
          );
        const mat =
          new THREE.LineBasicMaterial({
            color: lineColor,
            linewidth: lineWidthPx,
          });
        const line = new THREE.Line(
          geom,
          mat
        );
        grp.add(line);
        simulationObjects.current.waypointLine =
          line;
        // closing line
        try {
          const last =
            points[points.length - 1];
          const first = points[0];
          const closingGeom =
            new THREE.BufferGeometry().setFromPoints(
              [last, first]
            );
          const closingMat =
            new THREE.LineBasicMaterial(
              {
                color: lineColor,
                linewidth: lineWidthPx,
              }
            );
          const closingLine =
            new THREE.Line(
              closingGeom,
              closingMat
            );
          grp.add(closingLine);
          simulationObjects.current.waypointClosingLine =
            closingLine;
        } catch (exception) {
          console.error(
            "failed to draw closing waypoint line",
            exception
          );
        }
      }
      // render scene once so lines appear even while paused
      try {
        const objs =
          simulationObjects.current;
        if (
          objs &&
          objs.renderer &&
          objs.scene &&
          objs.camera
        )
          objs.renderer.render(
            objs.scene,
            objs.camera
          );
      } catch (err) {
        /* ignore */
      }
    }
  };

  // --- Vehicle helpers ---
  const createOrUpdateVehicle = (
    index: number
  ) => {
    if (
      !simulationObjects.current ||
      !simulationObjects.current.scene
    )
      return;
    // @ts-expect-error
    const THREE = window.THREE;
    if (!THREE) return;

    const grp =
      simulationObjects.current
        .vehicleGroup;
    if (!grp) return;

    // ensure array exists
    if (
      !Array.isArray(
        simulationObjects.current
          .vehicleMeshes
      )
    )
      simulationObjects.current.vehicleMeshes =
        [];

    // remove existing mesh for this index
    const existing =
      simulationObjects.current
        .vehicleMeshes[index];
    if (existing) {
      try {
        grp.remove(existing);
      } catch (e) {}
      try {
        if (existing.geometry)
          existing.geometry.dispose();
      } catch (e) {}
      try {
        if (existing.material)
          existing.material.dispose();
      } catch (e) {}
      simulationObjects.current.vehicleMeshes[
        index
      ] = null;
    }

    const v = vehicles[index];
    if (!v) return;

    // If a GLTF model was loaded into simulationObjects.current.model, clone it for each vehicle
    try {
      if (
        simulationObjects.current.model
      ) {
        const mdl =
          simulationObjects.current.model.clone(
            true
          );
        mdl.name = `vehicle_${index}`;
        mdl.userData.pickable = true;
        // position mapping: input x=north, y=east, z=alt -> scene pos x=-east, y=alt, z=north
        mdl.position.set(
          -Number(v.y),
          Number(v.z),
          Number(v.x)
        );
        grp.add(mdl);
        simulationObjects.current.vehicleMeshes[
          index
        ] = mdl;
        // add to pickable list
        try {
          simulationObjects.current.pickable =
            (
              simulationObjects.current
                .pickable || []
            ).concat([mdl]);
        } catch (e) {}
        return;
      }
    } catch (e) {
      console.warn(
        "vehicle clone failed",
        e
      );
    }

    // Fallback: create a simple placeholder quad (cross-shaped) so it's visible
    try {
      const body =
        new THREE.BoxGeometry(
          0.2,
          0.05,
          0.2
        );
      const bodyMat =
        new THREE.MeshStandardMaterial({
          color: 0x222222,
        });
      const bodyMesh = new THREE.Mesh(
        body,
        bodyMat
      );
      bodyMesh.position.set(
        -Number(v.y),
        Number(v.z) + 0.05,
        Number(v.x)
      );
      bodyMesh.userData.pickable = true;

      // add four small arms
      const armGeo =
        new THREE.BoxGeometry(
          0.02,
          0.02,
          0.6
        );
      const armMat =
        new THREE.MeshStandardMaterial({
          color: 0x888888,
        });
      const arm1 = new THREE.Mesh(
        armGeo,
        armMat
      );
      arm1.rotation.set(0, 0, 0);
      arm1.position.set(0, 0, 0);
      bodyMesh.add(arm1);
      const arm2 = arm1.clone();
      arm2.rotation.set(
        0,
        Math.PI / 2,
        0
      );
      bodyMesh.add(arm2);

      grp.add(bodyMesh);
      simulationObjects.current.vehicleMeshes[
        index
      ] = bodyMesh;
      try {
        simulationObjects.current.pickable =
          (
            simulationObjects.current
              .pickable || []
          ).concat([bodyMesh]);
      } catch (e) {}
    } catch (e) {
      console.error(
        "failed to create vehicle placeholder",
        e
      );
    }
  };

  // watch vehicles array and update scene meshes
  useEffect(() => {
    try {
      const objs =
        simulationObjects.current;
      if (!objs || !objs.scene) return;
      // ensure vehicleGroup exists
      if (!objs.vehicleGroup) {
        objs.vehicleGroup = new (
          window as any
        ).THREE.Group();
        objs.scene.add(
          objs.vehicleGroup
        );
      }
      // cleanup extra meshes if vehicles were removed
      if (
        !Array.isArray(
          objs.vehicleMeshes
        )
      )
        objs.vehicleMeshes = [];
      for (
        let i = vehicles.length;
        i < objs.vehicleMeshes.length;
        i++
      ) {
        const ex =
          objs.vehicleMeshes[i];
        if (ex) {
          try {
            objs.vehicleGroup.remove(
              ex
            );
          } catch (e) {}
          try {
            if (ex.geometry)
              ex.geometry.dispose();
          } catch (e) {}
          try {
            if (ex.material)
              ex.material.dispose();
          } catch (e) {}
        }
      }
      objs.vehicleMeshes.length =
        vehicles.length;

      // create or update each vehicle mesh
      vehicles.forEach((_, i) => {
        try {
          createOrUpdateVehicle(i);
        } catch (e) {
          console.warn(
            "createOrUpdateVehicle failed",
            e
          );
        }
      });
    } catch (e) {
      console.warn(
        "vehicles effect error",
        e
      );
    }
    // eslint-disable-next-line
  }, [vehicles]);

  // Add an OpenStreetMap tile as a horizontal plane under the scene
  const addMapPlane = (
    zoom = 13,
    x = 4096,
    y = 2720,
    size = 200
  ) => {
    try {
      const objs =
        simulationObjects.current;
      // @ts-ignore
      const THREE = window.THREE;
      if (
        !THREE ||
        !objs ||
        !objs.scene
      )
        return;
      // if already exists, remove first
      if (objs.mapPlane) {
        try {
          objs.scene.remove(
            objs.mapPlane
          );
        } catch (e) {}
        try {
          if (objs.mapPlane.geometry)
            objs.mapPlane.geometry.dispose();
        } catch (e) {}
        try {
          if (objs.mapPlane.material)
            objs.mapPlane.material.dispose();
        } catch (e) {}
        objs.mapPlane = null;
      }
      // Stitch multiple tiles (3x3) into a single canvas to increase resolution and coverage.
      const tileCount = 3;
      const half = Math.floor(
        tileCount / 2
      );
      const tileSize = 256;
      const canvasSize =
        tileSize * tileCount;
      const canvas =
        document.createElement(
          "canvas"
        );
      canvas.width = canvasSize;
      canvas.height = canvasSize;
      const ctx =
        canvas.getContext("2d");
      if (!ctx) {
        console.warn(
          "Failed to create canvas 2D context for map stitching"
        );
        return;
      }
      let remaining =
        tileCount * tileCount;
      const baseX = x,
        baseY = y;
      for (
        let ix = -half;
        ix <= half;
        ix++
      ) {
        for (
          let iy = -half;
          iy <= half;
          iy++
        ) {
          const tx = baseX + ix;
          const ty = baseY + iy;
          const url = `https://a.tile.openstreetmap.org/${zoom}/${tx}/${ty}.png`;
          const img = new Image();
          img.crossOrigin = "anonymous";
          img.onload = () => {
            try {
              const dx =
                (ix + half) * tileSize;
              const dy =
                (iy + half) * tileSize;
              ctx.drawImage(
                img,
                dx,
                dy,
                tileSize,
                tileSize
              );
            } catch (e) {
              console.warn(
                "drawImage failed",
                e
              );
            }
            remaining -= 1;
            if (remaining === 0) {
              try {
                const tex =
                  new THREE.CanvasTexture(
                    canvas
                  );
                tex.needsUpdate = true;
                try {
                  tex.encoding =
                    THREE.sRGBEncoding;
                } catch (e) {}
                try {
                  tex.generateMipmaps =
                    true;
                  tex.minFilter =
                    THREE.LinearMipMapLinearFilter;
                  tex.magFilter =
                    THREE.LinearFilter;
                } catch (e) {}
                try {
                  const maxAniso =
                    objs.renderer &&
                    objs.renderer
                      .capabilities &&
                    objs.renderer
                      .capabilities
                      .getMaxAnisotropy
                      ? objs.renderer.capabilities.getMaxAnisotropy()
                      : 4;
                  tex.anisotropy =
                    maxAniso || 1;
                } catch (e) {
                  tex.anisotropy =
                    tex.anisotropy || 1;
                }
                const geom =
                  new THREE.PlaneGeometry(
                    size,
                    size
                  );
                const mat: any =
                  new THREE.MeshBasicMaterial(
                    {
                      map: tex,
                      side: THREE.DoubleSide,
                    }
                  );
                try {
                  mat.toneMapped =
                    false;
                } catch (e) {}
                const plane =
                  new THREE.Mesh(
                    geom,
                    mat
                  );
                plane.name = "osmPlane";
                plane.userData.osm = {
                  zoom,
                  baseX,
                  baseY,
                  tileCount,
                };
                plane.rotation.x =
                  -Math.PI / 2;
                // place slightly lower so it's closer to ground but avoid z-fighting
                plane.position.y = 0.05;
                objs.scene.add(plane);
                objs.mapPlane = plane;
                try {
                  const box =
                    new THREE.BoxHelper(
                      plane,
                      0xff0000
                    );
                  box.name =
                    "osmPlaneHelper";
                  objs.scene.add(box);
                  objs.mapPlaneHelper =
                    box;
                } catch (e) {}
                try {
                  console.log(
                    "stitched mapPlane bbox",
                    new THREE.Box3()
                      .setFromObject(
                        plane
                      )
                      .toArray()
                  );
                } catch (e) {}
                try {
                  console.log(
                    "camera pos",
                    objs.camera
                      .position,
                    "near/far",
                    objs.camera.near,
                    objs.camera.far
                  );
                } catch (e) {}
                try {
                  if (
                    objs.renderer &&
                    objs.scene &&
                    objs.camera
                  )
                    objs.renderer.render(
                      objs.scene,
                      objs.camera
                    );
                } catch (e) {}
              } catch (e) {
                console.warn(
                  "Failed to create CanvasTexture",
                  e
                );
              }
            }
          };
          img.onerror = (err) => {
            console.warn(
              "Failed to load tile",
              url,
              err
            );
            remaining -= 1;
            if (remaining === 0) {
              // even if some tiles failed, create texture from whatever we have
              try {
                const tex =
                  new THREE.CanvasTexture(
                    canvas
                  );
                try {
                  tex.encoding =
                    THREE.sRGBEncoding;
                } catch (e) {}
                try {
                  tex.generateMipmaps =
                    true;
                  tex.minFilter =
                    THREE.LinearMipMapLinearFilter;
                  tex.magFilter =
                    THREE.LinearFilter;
                } catch (e) {}
                objs.mapPlane =
                  new THREE.Mesh(
                    new THREE.PlaneGeometry(
                      size,
                      size
                    ),
                    new THREE.MeshBasicMaterial(
                      {
                        map: tex,
                        side: THREE.DoubleSide,
                      }
                    )
                  );
                objs.mapPlane.rotation.x =
                  -Math.PI / 2;
                objs.mapPlane.position.y = 0.05;
                objs.scene.add(
                  objs.mapPlane
                );
                try {
                  if (
                    objs.renderer &&
                    objs.scene &&
                    objs.camera
                  )
                    objs.renderer.render(
                      objs.scene,
                      objs.camera
                    );
                } catch (e) {}
              } catch (e) {
                console.warn(
                  "Failed to create fallback CanvasTexture",
                  e
                );
              }
            }
          };
          img.src = url;
        }
      }
    } catch (e) {
      console.warn(
        "addMapPlane error",
        e
      );
    }
  };

  const removeMapPlane = () => {
    try {
      const objs =
        simulationObjects.current;
      if (!objs || !objs.scene) return;
      if (objs.mapPlane) {
        try {
          objs.scene.remove(
            objs.mapPlane
          );
        } catch (e) {}
        try {
          if (objs.mapPlane.geometry)
            objs.mapPlane.geometry.dispose();
        } catch (e) {}
        try {
          if (objs.mapPlane.material)
            objs.mapPlane.material.dispose();
        } catch (e) {}
        objs.mapPlane = null;
        try {
          if (
            objs.renderer &&
            objs.scene &&
            objs.camera
          )
            objs.renderer.render(
              objs.scene,
              objs.camera
            );
        } catch (e) {}
      }
    } catch (e) {
      console.warn(
        "removeMapPlane error",
        e
      );
    }
  };
  const resetSimulation = () => {
    console.log("Reset pressed");
    setRunning(false);
    setResetFlag(true);
  };

  const focusCamera = () => {
    try {
      const objs =
        simulationObjects.current;
      // @ts-ignore
      const THREE = (window as any)
        .THREE;
      if (
        !objs ||
        !objs.controls ||
        !objs.camera ||
        !THREE
      )
        return;
      let target = new THREE.Vector3(
        0,
        0,
        0
      );
      let refObj: any = null;
      if (
        objs.model &&
        objs.model.position
      ) {
        target.copy(
          objs.model.position
        );
        refObj = objs.model;
      } else if (objs.cube1) {
        target.set(
          objs.cube1.position.x,
          objs.cube1.position.y,
          objs.cube1.position.z
        );
        refObj = objs.cube1;
      }

      // Compute a forward-facing camera position relative to the vehicle's orientation.
      // Assume the vehicle's local +X axis is the forward direction for the simple placeholder/model used here.
      const forwardLocal =
        new THREE.Vector3(1, 0, 0);
      try {
        const worldQuat =
          new THREE.Quaternion();
        if (refObj)
          refObj.getWorldQuaternion(
            worldQuat
          );
        forwardLocal
          .applyQuaternion(worldQuat)
          .normalize();
      } catch (e) {
        // fallback: use world X axis
      }

      // Onboard view: place the camera at the vehicle (small eye offset) and look along its forward vector
      const eyeHeight = 0.12; // small offset above vehicle center (meters)
      const camPos =
        new THREE.Vector3().copy(
          target
        );
      camPos.y += eyeHeight;
      // look point far ahead along forward vector
      const lookAtPoint =
        new THREE.Vector3()
          .copy(target)
          .add(
            forwardLocal
              .clone()
              .multiplyScalar(10)
          );

      objs.camera.position.copy(camPos);
      // Tell controls to target the forward look-at point so orientation matches
      objs.controls.target.copy(
        lookAtPoint
      );
      objs.controls.update();
      // Toggle follow mode: if already following, disable; otherwise enable and prime prev position
      try {
        if (followQuadRef.current) {
          try {
            setFollowQuad(false);
          } catch (e) {}
          followQuadRef.current = false;
          followPrevPosRef.current =
            null;
          try {
            if (objs.controls)
              objs.controls.enabled =
                true;
          } catch (e) {}
        } else {
          try {
            setFollowQuad(true);
          } catch (e) {}
          followQuadRef.current = true;
          // prime previous position so delta-follow starts smoothly
          try {
            followPrevPosRef.current =
              target.clone();
          } catch (e) {
            followPrevPosRef.current =
              null;
          }
          try {
            if (objs.controls)
              objs.controls.enabled =
                true;
          } catch (e) {}
        }
      } catch (e) {}
    } catch (e) {
      console.error(
        "focus camera error",
        e
      );
    }
  };

  // Update polling delay when speed changes
  useEffect(() => {
    // base 50ms per poll @ 1x; faster speed -> smaller delay
    const base = 50;
    pollingDelayRef.current = Math.max(
      5,
      Math.round(base / speed)
    );
  }, [speed]);

  // --- Three.js and Animation Setup ---
  useEffect(() => {
    // Avoid loading Three.js multiple times: if window.THREE exists, reuse it.
    setThreeLoaded(false);
    // Remove existing canvas only (don't remove script tags if other parts of app use them)
    document
      .querySelectorAll("canvas")
      .forEach((c) => c.remove());
    try {
      // ensure the bundled THREE and OrbitControls are available globally for legacy code
      (window as any).THREE = THREE;
      (
        window as any
      ).THREE.OrbitControls =
        OrbitControls;
    } catch (e) {}
    setThreeLoaded(true);
    setupScene();
    return () => {
      document
        .querySelectorAll("canvas")
        .forEach((c) => c.remove());
    };
    // eslint-disable-next-line
  }, [resetFlag]);

  // --- Setup Three.js Scene ---
  const setupScene = () => {
    // @ts-expect-error - THREE.js loaded dynamically
    const THREE = window.THREE;
    if (!THREE) return;
    const scene = new THREE.Scene();
    const camera =
      new THREE.PerspectiveCamera(
        75,
        window.innerWidth /
          window.innerHeight,
        0.1,
        1000
      );
    const renderer =
      new THREE.WebGLRenderer({
        antialias: true,
      });
    renderer.setClearColor(0x000000, 1);
    // respect device pixel ratio for crisper textures
    try {
      renderer.setPixelRatio(
        window.devicePixelRatio || 1
      );
    } catch (e) {}
    renderer.setSize(
      window.innerWidth,
      window.innerHeight
    );
    renderer.domElement.style.width =
      "100vw";
    renderer.domElement.style.height =
      "100vh";
    renderer.domElement.style.display =
      "block";
    renderer.domElement.style.position =
      "absolute";
    renderer.domElement.style.top = "0";
    renderer.domElement.style.left =
      "0";
    renderer.domElement.style.zIndex =
      "1";
    renderer.domElement.style.objectFit =
      "cover";
    if (canvasContainerRef.current) {
      canvasContainerRef.current.innerHTML =
        "";
      canvasContainerRef.current.appendChild(
        renderer.domElement
      );
    }
    // Renderer settings suitable for environment maps
    try {
      renderer.toneMapping =
        THREE.ACESFilmicToneMapping;
      renderer.toneMappingExposure = 1.0;
      renderer.outputEncoding =
        THREE.sRGBEncoding;
    } catch (e) {
      /* ignore if properties missing */
    }
    // Try to load an EXR environment map from public/models
    (async () => {
      try {
        // Use bundled EXRLoader
        const exrLoader =
          new EXRLoader();
        exrLoader.load(
          "/models/autumn_field_puresky_512.exr",
          (texture: any) => {
            try {
              const pmremGenerator =
                new THREE.PMREMGenerator(
                  renderer
                );
              pmremGenerator.compileEquirectangularShader();
              const envMap =
                pmremGenerator.fromEquirectangular(
                  texture
                ).texture;
              scene.environment =
                envMap;
              scene.background = envMap;
              texture.dispose();
              pmremGenerator.dispose();
              try {
                renderer.render(
                  scene,
                  camera
                );
              } catch (e) {}
            } catch (e) {
              console.error(
                "Failed to apply EXR as env map",
                e
              );
            }
          },
          undefined,
          (err: any) => {
            console.warn(
              "Failed to load EXR environment",
              err
            );
          }
        );
      } catch (err) {
        console.warn(
          "EXRLoader load failed",
          err
        );
      }
    })();
    const controls = new OrbitControls(
      camera,
      renderer.domElement
    );
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.enableZoom = true;
    controls.autoRotate = false;
    // make autorotation faster and rotate to the right (negative = opposite direction)
    controls.autoRotateSpeed = -2; // faster, right-handed
    // allow the user to zoom out much further
    controls.maxDistance = 200;
    controls.minDistance = 1;
    controls.screenSpacePanning = false;
    // Position camera at fixed distance CAMERA_DISTANCE with a small elevation
    const camElev = 2.0;
    // stronger right offset for a more side-looking camera
    const camX = 4.0;
    const camZ = Math.sqrt(
      Math.max(
        0,
        CAMERA_DISTANCE *
          CAMERA_DISTANCE -
          camElev * camElev -
          camX * camX
      )
    );
    camera.position.set(
      camX,
      camElev,
      camZ
    );
    // look slightly off-center to the right
    camera.lookAt(1.0, 0, 0);
    controls.target.set(1.0, 0, 0);

    // Interactivity handling: pause auto-rotate while user interacts, resume after idle
    controls.addEventListener(
      "start",
      () => {
        userInteractingRef.current =
          true;
        controls.autoRotate = false;
        if (idleTimerRef.current) {
          window.clearTimeout(
            idleTimerRef.current
          );
          idleTimerRef.current = null;
        }
      }
    );
    controls.addEventListener(
      "end",
      () => {
        userInteractingRef.current =
          false;
        // resume auto-rotate after 3s of inactivity
        if (idleTimerRef.current)
          window.clearTimeout(
            idleTimerRef.current
          );
        idleTimerRef.current =
          window.setTimeout(() => {
            try {
              controls.autoRotate =
                !!autoRotateRef.current;
            } catch (e) {}
            idleTimerRef.current = null;
          }, 3000) as unknown as number;
      }
    );
    // If the user never interacts, enable auto-rotate after initial idle
    if (idleTimerRef.current)
      window.clearTimeout(
        idleTimerRef.current
      );
    idleTimerRef.current =
      window.setTimeout(() => {
        try {
          controls.autoRotate =
            !!autoRotateRef.current;
        } catch (e) {}
        idleTimerRef.current = null;
      }, 3000) as unknown as number;
    const geometry1 =
      new THREE.BoxGeometry(
        0.6,
        0.02,
        0.02
      );
    const material1 =
      new THREE.MeshBasicMaterial({
        color: 0x00ff00,
      });
    const cube1 = new THREE.Mesh(
      geometry1,
      material1
    );
    cube1.userData.pickable = true;
    scene.add(cube1);
    const geometry2 =
      new THREE.BoxGeometry(
        0.02,
        0.02,
        0.6
      );
    const material2 =
      new THREE.MeshBasicMaterial({
        color: 0x0000ff,
      });
    const cube2 = new THREE.Mesh(
      geometry2,
      material2
    );
    cube2.userData.pickable = true;
    scene.add(cube2);
    const geometry3 =
      new THREE.BoxGeometry(
        0.02,
        0.2,
        0.02
      );
    const material3 =
      new THREE.MeshBasicMaterial({
        color: 0xff0000,
      });
    const cube3 = new THREE.Mesh(
      geometry3,
      material3
    );
    cube3.geometry.translate(0, 0.1, 0);
    cube3.userData.pickable = true;
    scene.add(cube3);
    // make these pickable (for clicking the quad to open params)
    const pickable = [
      cube1,
      cube2,
      cube3,
    ];
    const planeGeometry =
      new THREE.PlaneGeometry(10, 10);
    const planeMaterial =
      new THREE.MeshBasicMaterial({
        color: 0x808080,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.3,
      });
    const groundPlane = new THREE.Mesh(
      planeGeometry,
      planeMaterial
    );
    groundPlane.rotation.x =
      -Math.PI / 2;
    groundPlane.position.y = 0;
    scene.add(groundPlane);
    const gridHelper =
      new THREE.GridHelper(
        10,
        20,
        0x444444,
        0x444444
      );
    gridHelper.position.y = 0.01;
    scene.add(gridHelper);
    try {
      gridHelper.visible =
        !!showGridRef.current;
    } catch (e) {}
    try {
      if (showMapRef.current)
        addMapPlane();
    } catch (e) {}
    try {
      groundPlane.visible =
        !!showGridRef.current;
    } catch (e) {}
    // Add simple lighting so GLTF models are visible
    try {
      const hemi =
        new THREE.HemisphereLight(
          0xffffff,
          0x444444,
          0.6
        );
      hemi.position.set(0, 20, 0);
      scene.add(hemi);
      const dir =
        new THREE.DirectionalLight(
          0xffffff,
          0.8
        );
      dir.position.set(-5, 10, 5);
      scene.add(dir);
      const amb =
        new THREE.AmbientLight(
          0x666666,
          0.4
        );
      scene.add(amb);
    } catch (e) {
      /* ignore if THREE missing */
    }
    camera.position.set(0, 5, 3);
    camera.lookAt(0, 0, 0);
    // Try to load a local GLTF model if present in `public/models/model.glb`.
    // If loading succeeds, replace the placeholder cubes with the model.
    (async () => {
      try {
        // Use bundled GLTFLoader
        const loader = new GLTFLoader();
        loader.load(
          "/models/model.glb",
          (gltf: any) => {
            try {
              const model =
                gltf.scene ||
                gltf.scenes?.[0];
              if (!model) {
                console.warn(
                  "GLTF loaded but contains no scene"
                );
                return;
              }
              model.traverse(
                (o: any) => {
                  if (o.isMesh) {
                    o.castShadow = true;
                    o.receiveShadow =
                      true;
                  }
                }
              );
              model.scale.set(
                0.25,
                0.25,
                0.25
              );
              model.rotation.set(
                0,
                0,
                0
              );
              model.position.set(
                0,
                0,
                0
              );
              model.name = "quadModel";
              scene.add(model);
              model.userData.pickable =
                true;
              simulationObjects.current.model =
                model;
              try {
                scene.remove(cube1);
              } catch (e) {}
              try {
                scene.remove(cube2);
              } catch (e) {}
              try {
                scene.remove(cube3);
              } catch (e) {}
              try {
                if (
                  simulationObjects.current &&
                  Array.isArray(
                    simulationObjects
                      .current.pickable
                  )
                ) {
                  simulationObjects.current.pickable =
                    [model];
                }
              } catch (e) {}
              try {
                renderer.render(
                  scene,
                  camera
                );
              } catch (e) {}
            } catch (e) {
              console.error(
                "Error when adding GLTF to scene",
                e
              );
            }
          },
          undefined,
          (err: any) => {
            console.error(
              "Failed to load GLTF model /models/model.glb",
              err
            );
          }
        );
      } catch (err) {
        console.warn(
          "GLTFLoader load failed",
          err
        );
      }
    })();
    // prepare waypoint group and storage
    const waypointGroup =
      new THREE.Group();
    scene.add(waypointGroup);
    // prepare vehicle group and storage (one mesh per vehicle)
    const vehicleGroup =
      new THREE.Group();
    scene.add(vehicleGroup);

    // raycaster for pointer interaction
    const raycaster =
      new THREE.Raycaster();
    const mouse = new THREE.Vector2();

    // pointer handler: when clicking on the quad/model, open params panel
    const onPointerDown = (
      ev: PointerEvent
    ) => {
      try {
        // allow pointer interactions while in follow mode so the user can look around;
        // do not automatically disable follow here
        console.debug(
          "pointerdown",
          ev.clientX,
          ev.clientY,
          "target=",
          (ev.target as any)?.tagName ||
            ev.target
        );
        const rect =
          renderer.domElement.getBoundingClientRect();
        mouse.x =
          ((ev.clientX - rect.left) /
            rect.width) *
            2 -
          1;
        mouse.y =
          -(
            (ev.clientY - rect.top) /
            rect.height
          ) *
            2 +
          1;
        raycaster.setFromCamera(
          mouse,
          camera
        );
        // Raycast against all scene children and find the first object that (or whose ancestor) is pickable
        const children =
          scene.children || [];
        const intersects =
          raycaster.intersectObjects(
            children,
            true
          );
        console.debug(
          "intersects count",
          intersects.length
        );
        let picked = null as any;
        for (let it of intersects) {
          let obj: any = it.object;
          // walk up parents to find pickable flag
          while (obj) {
            if (
              obj.userData &&
              obj.userData.pickable
            ) {
              picked = obj;
              break;
            }
            obj = obj.parent;
          }
          if (picked) break;
        }
        if (picked) {
          console.debug(
            "picked object",
            picked.name || picked.type,
            picked
          );
          try {
            setOpenPanel("params");
          } catch (e) {
            console.warn(e);
          }
        }
      } catch (e) {
        console.error(
          "onPointerDown error",
          e
        );
      }
    };

    // register on both canvas and window as a fallback (some UI may capture events)
    try {
      renderer.domElement.addEventListener(
        "pointerdown",
        onPointerDown
      );
    } catch (e) {
      console.warn(
        "failed to add pointer listener to canvas",
        e
      );
    }
    try {
      window.addEventListener(
        "pointerdown",
        onPointerDown
      );
    } catch (e) {
      console.warn(
        "failed to add pointer listener to window",
        e
      );
    }

    simulationObjects.current = {
      scene,
      camera,
      renderer,
      controls,
      cube1,
      cube2,
      cube3,
      waypointGroup,
      vehicleGroup,
      waypointMeshes:
        Array(5).fill(null),
      vehicleMeshes: [],
      waypointLine: null,
      waypointClosingLine: null,
      mapPlane: null,
      pickable,
      raycaster,
      gridHelper,
      groundPlane,
      _onPointerDown: onPointerDown,
    };
    try {
      (window as any).getSimObjects =
        () => simulationObjects.current;
    } catch (e) {}
    // set initial quad visual positions and render once so scene is visible before Play
    try {
      const objs =
        simulationObjects.current;
      objs.cube1.position.set(0, 0, 0);
      objs.cube2.position.set(0, 0, 0);
      objs.cube3.position.set(0, 0, 0);
      objs.cube1.rotation.set(0, 0, 0);
      objs.cube2.rotation.set(0, 0, 0);
      objs.cube3.rotation.set(0, 0, 0);
      objs.renderer.render(
        objs.scene,
        objs.camera
      );
    } catch (e) {
      /* ignore */
    }
  };

  // --- Animation & Polling Loop ---
  useEffect(() => {
    if (!running || !threeLoaded)
      return;
    let t = 0,
      norden = 0,
      osten = 0,
      unten = 0;
    let rollen = 0,
      nicken = 0,
      gieren = 0;
    // altitude (positive up) derived from backend 'down' (positive downwards)
    let altitude = 0;
    let stop = false;
    pollingRef.current = true;
    // --- Polling Loop ---
    function connectServer() {
      if (!pollingRef.current) return;
      // Backend runs on port 5001 in dev; use API_BASE so dev/prod can differ.
      // API_BASE is taken from import.meta.env.VITE_API_BASE or defaults to '/api/'
      // Use Vite env `VITE_API_BASE` if present, otherwise fall back to proxied '/api/'
      const apiTarget =
        (import.meta as any)?.env
          ?.VITE_API_BASE ?? "/api/";
      fetch(apiTarget, {
        method: "POST",
        headers: {
          "Content-Type":
            "application/json",
        },
        body: JSON.stringify({}),
      })
        .then(async (response) => {
          if (!response.ok) {
            const body = await response
              .text()
              .catch(() => "<no body>");
            console.error(
              "Server error response",
              response.status,
              body
            );
            // stop polling to avoid flooding the console
            pollingRef.current = false;
            setRunning(false);
            return null;
          }
          return response.json();
        })
        .then((data) => {
          if (!data) return;
          const json_data = data;
          // Direct assignment without type/NaN checks or fallbacks
          t = json_data.t;
          norden = json_data.north;
          osten = json_data.east;
          // backend returns 'down' (positive downwards). Convert to altitude (positive up).
          // backend returns 'down' (positive downwards). Convert to altitude (positive up).
          altitude =
            json_data.down !== undefined
              ? -json_data.down
              : json_data.down;
          rollen = json_data.roll;
          nicken = json_data.pitch;
          gieren = json_data.yaw;
          const timeElement =
            document.getElementById(
              "time"
            );
          // Display time with lowercase 't' and ' sec' suffix, formatted to 2 decimals
          if (timeElement)
            timeElement.textContent = `t = ${Number(
              t
            ).toFixed(2)} s`;
          try {
            const speedElem =
              document.getElementById(
                "speedLabel"
              );
            if (speedElem)
              speedElem.textContent = `${Number(
                speedRef.current || 1
              ).toFixed(2)}x`;
          } catch (e) {}
          // schedule next poll only while running, delay controlled by slider
          if (pollingRef.current)
            setTimeout(
              connectServer,
              pollingDelayRef.current
            );
          // update active waypoint index if provided by backend
          if (
            json_data.current_wp_index !==
              undefined &&
            json_data.current_wp_index !==
              null
          ) {
            setActiveWaypoint(
              Number(
                json_data.current_wp_index
              )
            );
          } else {
            setActiveWaypoint(null);
          }
          // --- Energy / Power sampling from telemetry (prefer electrical or motor data)
          try {
            // Prefer a real Unix timestamp for plotting. The backend `t` is a simulation time
            // starting at 0; if it's not a Unix epoch (> 1e9) we use wall-clock time instead.
            let ts: number;
            if (
              typeof json_data.t ===
                "number" &&
              json_data.t > 1e9
            )
              ts = Number(json_data.t);
            else ts = Date.now() / 1000;
            let powerW: number | null =
              null;
            // 1) If backend provides voltage/current, compute electrical power
            const voltageKeys = [
              "voltage",
              "bat_voltage",
              "battery_voltage",
            ];
            const currentKeys = [
              "current",
              "bat_current",
              "battery_current",
              "current_mA",
              "bat_current_mA",
            ];
            let voltage:
              | number
              | undefined;
            for (const k of voltageKeys) {
              if (
                json_data[k] !==
                undefined
              ) {
                voltage = Number(
                  json_data[k]
                );
                break;
              }
            }
            let current:
              | number
              | undefined;
            for (const k of currentKeys) {
              if (
                json_data[k] !==
                undefined
              ) {
                current = Number(
                  json_data[k]
                );
                break;
              }
            }
            if (
              voltage !== undefined &&
              current !== undefined &&
              !Number.isNaN(voltage) &&
              !Number.isNaN(current)
            ) {
              // if current looks like mA (large number), convert to A
              if (
                Math.abs(current) > 500
              )
                current =
                  current / 1000.0;
              powerW =
                voltage * current;
            }
            // 2) If no electrical data, try motor speeds (many backends expose `wMotor` or similar)
            if (powerW === null) {
              const motorKeys = [
                "wMotor",
                "wmotor",
                "motor_speeds",
                "motor_rpms",
                "motorSpeeds",
                "w_motors",
              ];
              let motors: any = null;
              for (const k of motorKeys) {
                if (
                  json_data[k] !==
                  undefined
                ) {
                  motors = json_data[k];
                  break;
                }
              }
              if (
                motors &&
                Array.isArray(motors) &&
                motors.length >= 1
              ) {
                // determine units: assume rad/s by default; if values are large (~1000..20000) they might be RPM
                const sample0 = Number(
                  motors[0]
                );
                if (
                  !Number.isNaN(sample0)
                ) {
                  let motorsRadS =
                    motors.map(
                      (v: any) =>
                        Number(v)
                    );
                  // heuristics: if values are > 2000 assume RPM -> convert to rad/s
                  if (
                    motorsRadS.some(
                      (v: number) =>
                        Math.abs(v) >
                        2000
                    )
                  ) {
                    motorsRadS =
                      motorsRadS.map(
                        (rpm: number) =>
                          (rpm *
                            2 *
                            Math.PI) /
                          60.0
                      );
                  }
                  // use torque coefficient kTo if provided by backend, else fallback to default from model
                  const kTo =
                    json_data.kTo !==
                    undefined
                      ? Number(
                          json_data.kTo
                        )
                      : 1.632e-7;
                  // P_i = torque * omega = kTo * w^2 * w = kTo * w^3
                  let totalP = 0;
                  for (const w of motorsRadS) {
                    if (
                      !Number.isNaN(w)
                    )
                      totalP +=
                        kTo * w * w * w;
                  }
                  // don't accept negative nonsense
                  if (
                    Number.isFinite(
                      totalP
                    ) &&
                    totalP > 0
                  )
                    powerW = totalP;
                }
              }
            }
            // 3) If still null, fallback to null (no sample)
            if (
              powerW !== null &&
              Number.isFinite(powerW)
            ) {
              // push sample, throttle to ~5Hz
              const nowMs = Date.now();
              const objs =
                simulationObjects.current ||
                {};
              if (
                !objs._lastEnergyPush ||
                nowMs -
                  objs._lastEnergyPush >
                  180
              ) {
                try {
                  console.debug(
                    "telemetry energy sample",
                    {
                      ts,
                      powerW,
                      voltage,
                      current,
                      motors:
                        Array.isArray(
                          motors
                        )
                          ? motors.slice(
                              0,
                              4
                            )
                          : motors,
                    }
                  );
                } catch (e) {}
                setEnergySamples(
                  (s) => {
                    const next =
                      s.concat([
                        {
                          t: ts,
                          p: Math.max(
                            0,
                            powerW as number
                          ),
                        },
                      ]);
                    const cutoff =
                      ts - 60;
                    return next.filter(
                      (x) =>
                        x.t >= cutoff
                    );
                  }
                );
                objs._lastEnergyPush =
                  nowMs;
                simulationObjects.current =
                  objs;
              }
            }
          } catch (e) {
            console.debug(
              "energy telemetry parse error",
              e
            );
          }
        })
        .catch((err) => {
          console.error(
            "connectServer error",
            err
          );
          if (pollingRef.current)
            setTimeout(
              connectServer,
              500
            );
        });
    }
    connectServer();
    // --- Animation Loop ---
    function animate() {
      try {
        if (!running || stop) return;
        const {
          cube1,
          cube2,
          cube3,
          controls,
          renderer,
          scene,
          camera,
          model,
        } = simulationObjects.current;
        // If a GLTF model was loaded, move/rotate it. Otherwise, update the placeholder cubes.
        if (model) {
          try {
            // mapping: scene x = -east, y = altitude (up), z = north
            model.position.set(
              -osten,
              altitude,
              norden
            );
            // maintain same rotation mapping as the cubes: rotation.set(x=nicken, y=gieren, z=rollen)
            model.rotation.set(
              nicken || 0,
              gieren || 0,
              rollen || 0
            );
          } catch (e) {
            /* ignore model update errors */
          }
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
          } catch (e) {
            /* ignore cube update errors */
          }
        }
        // Energy is now sampled from backend telemetry (voltage/current or motor speeds)
        // Camera follow mode: smoothly follow the quad when enabled and user not interacting
        try {
          // ensure we use the latest follow flag
          if (
            followQuadRef.current &&
            !userInteractingRef.current
          ) {
            // @ts-ignore
            const THREE = window.THREE;
            if (
              THREE &&
              (model || cube1)
            ) {
              const target =
                new THREE.Vector3();
              const refObj: any = model
                ? model
                : cube1;
              if (
                model &&
                model.position
              )
                target.copy(
                  model.position
                );
              else
                target.set(
                  cube1.position.x,
                  cube1.position.y,
                  cube1.position.z
                );

              // Follow by translating camera/target by the quad's movement delta so the camera
              // stays attached to the vehicle while retaining the user's current rotation.
              if (
                followQuadRef.current
              ) {
                try {
                  // compute current vehicle world position
                  const forwardLocal =
                    new THREE.Vector3(
                      1,
                      0,
                      0
                    ); // unused here but kept for future
                  const current =
                    new THREE.Vector3();
                  if (
                    model &&
                    model.position
                  )
                    current.copy(
                      model.position
                    );
                  else
                    current.set(
                      cube1.position.x,
                      cube1.position.y,
                      cube1.position.z
                    );

                  if (
                    !followPrevPosRef.current
                  )
                    followPrevPosRef.current =
                      current.clone();
                  const prev =
                    followPrevPosRef.current;
                  const delta =
                    new THREE.Vector3().subVectors(
                      current,
                      prev
                    );
                  // move camera and controls.target by the delta so they follow the vehicle
                  if (
                    delta.lengthSq() > 0
                  ) {
                    camera.position.add(
                      delta
                    );
                    controls.target.add(
                      delta
                    );
                  }
                  // maintain a small eye offset above the vehicle
                  try {
                    const eyeHeight = 0.12;
                    const desiredY =
                      current.y +
                      eyeHeight;
                    if (
                      Math.abs(
                        camera.position
                          .y - desiredY
                      ) > 1e-3
                    )
                      camera.position.y =
                        desiredY;
                  } catch (e) {}
                  followPrevPosRef.current =
                    current.clone();
                } catch (e) {
                  /* ignore follow errors */
                }
                // ensure user can still orbit/look around
                try {
                  if (
                    controls &&
                    !controls.enabled
                  )
                    controls.enabled =
                      true;
                } catch (e) {}
              } else {
                // not following: ensure controls are enabled so the user can orbit
                try {
                  if (
                    controls &&
                    !controls.enabled
                  )
                    controls.enabled =
                      true;
                } catch (e) {}
                followPrevPosRef.current =
                  null;
              }
            }
          }
        } catch (e) {
          /* ignore follow errors */
        }
        // If showMap is enabled but mapPlane is missing, try to add it again (recover from resets)
        try {
          const objs =
            simulationObjects.current ||
            {};
          if (
            showMapRef.current &&
            objs &&
            !objs.mapPlane &&
            !objs._addingMapPlane
          ) {
            objs._addingMapPlane = true;
            console.log(
              "mapPlane missing — attempting to addMapPlane()"
            );
            try {
              addMapPlane();
            } catch (e) {
              console.warn(
                "addMapPlane failed",
                e
              );
            }
            // allow re-tries after 1s
            setTimeout(() => {
              try {
                const o =
                  simulationObjects.current;
                if (o)
                  o._addingMapPlane =
                    false;
              } catch (e) {}
            }, 1000);
          }
        } catch (e) {}
        controls.update();
        renderer.render(scene, camera);
        animationRef.current =
          requestAnimationFrame(
            animate
          );
      } catch (err) {
        console.error(
          "Animation error",
          err
        );
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
      if (animationRef.current)
        cancelAnimationFrame(
          animationRef.current
        );
    };
    // eslint-disable-next-line
  }, [running, threeLoaded]);

  // --- Reset-Flag: Szene neu aufbauen ---
  useEffect(() => {
    if (resetFlag) {
      // remove pointer handler if present
      try {
        const objs =
          simulationObjects.current;
        if (
          objs &&
          objs.renderer &&
          objs._onPointerDown
        ) {
          try {
            objs.renderer.domElement.removeEventListener(
              "pointerdown",
              objs._onPointerDown
            );
          } catch (e) {}
          objs._onPointerDown = null;
        }
      } catch (e) {}
      document
        .querySelectorAll("canvas")
        .forEach((c) => c.remove());
      setTimeout(() => {
        try {
          const ds =
            localStorage.getItem(
              "sim.defaultSpeed"
            );
          if (ds !== null) {
            const v = Number(ds);
            if (
              !Number.isNaN(v) &&
              v > 0
            ) {
              setSpeed(v);
              speedRef.current = v;
              try {
                setSliderVal(
                  speedToSlider(v)
                );
              } catch (e) {}
            }
          }
        } catch (e) {}
        setupScene();
        setResetFlag(false);
      }, 100);
    }
    // eslint-disable-next-line
  }, [resetFlag]);

  return (
    <IonPage>
      <IonContent fullscreen>
        <div
          style={{
            position: "fixed",
            top: "5px",
            left: "0",
            right: "0",
            textAlign: "center",
            zIndex: 100,
            color: "white",
            fontSize: "24px",
            fontWeight: "bold",
            textShadow:
              "2px 2px 4px rgba(0,0,0,0.8)",
            pointerEvents: "none",
          }}
        >
          <h1 id="time">t = 0.00 s</h1>
          <div
            id="speedLabel"
            style={{
              fontSize: "12px",
              marginTop: "2px",
              opacity: 0.9,
            }}
          >
            1.00x
          </div>
        </div>
        {/* Home button top-left */}
        <div
          style={{
            position: "fixed",
            top: 12,
            left: 12,
            zIndex: 300,
            pointerEvents: "auto",
            display: "flex",
            gap: "8px",
          }}
        >
          <IonButton
            href="/"
            routerDirection="forward"
            title="Landing"
            aria-label="Landing"
            className="panel-icon-button"
            style={{ marginRight: 4 }}
          >
            <IonIcon
              icon={homeIcon}
              slot="icon-only"
              style={{ fontSize: 20 }}
            />
          </IonButton>
        </div>

        {/* Settings button remains top-right */}
        <div
          style={{
            position: "fixed",
            top: 12,
            right: 12,
            zIndex: 300,
            pointerEvents: "auto",
            display: "flex",
            gap: "8px",
          }}
        >
          <IonButton
            href="/settings"
            routerDirection="forward"
            title="Einstellungen"
            aria-label="Einstellungen"
            className="panel-icon-button"
          >
            <IonIcon
              icon={settingsIcon}
              slot="icon-only"
              style={{ fontSize: 20 }}
            />
          </IonButton>
        </div>
        <div
          id="three-canvas-container"
          ref={canvasContainerRef}
          style={{
            width: "100vw",
            height: "100vh",
            overflow: "hidden",
            /* kein position: fixed! */
            zIndex: 1,
          }}
        />

        {/* Bottom docked Play / Reset pill */}
        <div className="control-bar">
          <button
            className="control-button"
            onClick={() => {
              setRunning((r) => !r);
            }}
          >
            {running ? "Pause" : "Play"}
          </button>
          <button
            className="control-button"
            onClick={resetSimulation}
          >
            Reset
          </button>
          <button
            className="control-button"
            onClick={focusCamera}
          >
            Focus
          </button>
        </div>
        <div className="panel-stack">
          <WaypointsPanel
            waypoints={waypoints}
            setWaypoints={setWaypoints}
            wpTolerance={wpTolerance}
            setWpTolerance={
              setWpTolerance
            }
            openPanel={openPanel}
            setOpenPanel={setOpenPanel}
            activeWaypoint={
              activeWaypoint
            }
            setActiveWaypoint={
              setActiveWaypoint
            }
            singlePos={singlePos}
            setSinglePos={setSinglePos}
            simulationObjects={
              simulationObjects
            }
          />
          <PositionPanel
            singlePos={singlePos}
            setSinglePos={setSinglePos}
            simulationObjects={
              simulationObjects
            }
            openPanel={openPanel}
            setOpenPanel={setOpenPanel}
          />
          <SimulationPanel
            sliderVal={sliderVal}
            setSliderVal={(v) => {
              setSliderVal(v);
              setSpeed(
                sliderToSpeed(v)
              );
            }}
            sliderToSpeed={
              sliderToSpeed
            }
            wpTolerance={wpTolerance}
            onToleranceChange={(v) => {
              setWpTolerance(v);
              const payload =
                waypoints.map((w) => ({
                  x: Number(w.x),
                  y: Number(w.y),
                  z: Number(w.z),
                }));
              (async () => {
                try {
                  await postWaypoints({
                    waypoints: payload,
                    tolerance: v,
                  });
                } catch (err) {
                  console.error(
                    "failed to send waypoints",
                    err
                  );
                }
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
            onApplyScene={
              applyParamsToScene
            }
          />
          <VehiclesPanel
            vehicles={vehicles}
            setVehicles={setVehicles}
            openPanel={openPanel}
            setOpenPanel={setOpenPanel}
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
            schedulePidSend={
              schedulePidSend
            }
          />
        </div>
        <div className="panel-stack-bottom-right">
          <EnergyPanel
            samples={energySamples}
            openPanel={openPanel}
            setOpenPanel={setOpenPanel}
          />
          <OptimizerPanel
            waypoints={waypoints}
            speed={sliderToSpeed(
              sliderVal
            )}
            baseMass={mass}
            payloadMass={0}
            simulationObjects={
              simulationObjects
            }
          />
        </div>
      </IonContent>
    </IonPage>
  );
};

export default Simulation;
