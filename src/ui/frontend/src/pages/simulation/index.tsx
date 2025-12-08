import {
  IonContent,
  IonPage,
} from "@ionic/react";
import React, { useEffect } from "react";

const Simulation: React.FC = () => {
  
  useEffect(() => {
    // Remove any existing scripts first  
    document.querySelectorAll('script[src*="three"]').forEach(s => s.remove());
    
    // Load Three.js
    const threeScript = document.createElement('script');
    threeScript.src = 'https://cdn.jsdelivr.net/npm/three@0.125.2/build/three.min.js';
    threeScript.onload = () => {
      const controlsScript = document.createElement('script');
      controlsScript.src = 'https://cdn.jsdelivr.net/npm/three@0.125.2/examples/js/controls/OrbitControls.js';
      controlsScript.onload = () => {
        setTimeout(() => startSimulation(), 100);
      };
      document.head.appendChild(controlsScript);
    };
    document.head.appendChild(threeScript);

    const startSimulation = () => {
      // @ts-expect-error - THREE.js loaded dynamically
      const THREE = window.THREE;
      if (!THREE) {
        console.error('THREE.js not loaded');
        return;
      }

      // Variables from main.js
      let t = 0, norden = 0, osten = 0, unten = 0;
      let rollen = 0, nicken = 0, gieren = 0;

      function connectServer() {
        fetch('http://localhost:5000/', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({})
        })
        .then(response => response.json())
        .then(data => {
          const json_data = JSON.parse(JSON.stringify(data));
          t = json_data.t;
          norden = json_data.north;
          osten = json_data.east;
          unten = -json_data.down;
          rollen = json_data.roll;
          nicken = json_data.pitch;
          gieren = json_data.yaw;

          const timeElement = document.getElementById('time');
          const distanceElement = document.getElementById('distance');
          if (timeElement) timeElement.textContent = "T = " + t.toFixed(2);
          if (distanceElement) {
            distanceElement.textContent = "D = " + 
              (Math.abs(norden) + Math.abs(osten) + Math.abs(unten)).toFixed(4) + "m";
          }

          setTimeout(connectServer, 50);
        })
        .catch(error => console.error(error));
      }

      // Remove existing canvas
      document.querySelectorAll('canvas').forEach(c => c.remove());

      // Three.js setup - exactly like main.js
      const scene = new THREE.Scene();
      const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
      
      const renderer = new THREE.WebGLRenderer();
      renderer.setSize(window.innerWidth, window.innerHeight);
      
      // Find the Ionic content container and append canvas there
      const ionContent = document.querySelector('ion-content');
      if (ionContent) {
        renderer.domElement.style.position = 'absolute';
        renderer.domElement.style.top = '0';
        renderer.domElement.style.left = '0';
        renderer.domElement.style.zIndex = '1';
        ionContent.appendChild(renderer.domElement);
      } else {
        document.body.appendChild(renderer.domElement);
      }

      // Orbit controls
      const controls = new THREE.OrbitControls(camera, renderer.domElement);
      controls.enableDamping = true;
      controls.dampingFactor = 0.05;
      controls.enableZoom = true;
      controls.autoRotate = false;
      controls.maxDistance = 10;
      controls.minDistance = 1;
      controls.screenSpacePanning = false;

      // Quadcopter components - exactly like main.js
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

      // Ground plane
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

      // Grid helper
      const gridHelper = new THREE.GridHelper(10, 20, 0x444444, 0x444444);
      gridHelper.position.y = 0.01;
      scene.add(gridHelper);

      // Camera position
      camera.position.set(0, 5, 3);
      camera.lookAt(0, 0, 0);

      // Animation loop
      function animate() {
        requestAnimationFrame(animate);

        // Update quadcopter position - exact same as main.js
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
      }

      // Start everything
      connectServer();
      animate();
      console.log("Simulation started");
    };

    // Cleanup on unmount
    return () => {
      document.querySelectorAll('canvas').forEach(c => c.remove());
    };
  }, []);

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
          <h1 id="time">T = 0.00</h1>
        </div>
        <div style={{
          position: 'fixed',
          top: '60px',
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
          <h1 id="distance">D = 0.00m</h1>
        </div>
      </IonContent>
    </IonPage>
  );
};

export default Simulation;
