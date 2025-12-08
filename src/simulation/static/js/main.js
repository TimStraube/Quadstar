let t = 0;
let norden = 0;
let osten = 0;
let unten = 0;
let rollen = 0;
let nicken = 0;
let gieren = 0;

// Trajectory tracking
let trajectoryPoints = [];
let maxTrajectoryPoints = 300; // Reduced for better performance
let trajectoryLine;

function connectServer() {
    fetch('/', {
        method: 'POST',
        headers: 
        {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({})
    })
    .then(response => response.json())
        .then(data => {
            const json_data = JSON.parse(JSON.stringify(data));
            t = json_data.t;
            norden = json_data.north
            osten = json_data.east
            unten = -json_data.down
            rollen = json_data.roll
            nicken = json_data.pitch
            gieren = json_data.yaw
            
            // Add trajectory point every 5th frame for better performance
            // if (Math.floor(t * 50) % 5 === 0) {
            //     updateTrajectory(norden, osten, -unten);
            // }
            
            // Optimized: 50ms delay = 20 FPS, reduced lag but still smooth
            setTimeout(connectServer, 50); // 20 FPS für weniger Lag

            // Increased distance threshold for circle trajectory (radius 2m)
            if (Math.sqrt(Math.pow(norden, 2) + Math.pow(osten, 2) + Math.pow(unten, 2)) > 10) {
                location.reload(); // Only reload if quadcopter is really far away
            }
    }).catch(error => {
        console.error(error);
    });
}

function updateTrajectory(x, z, y) {
    // Add new point to trajectory
    trajectoryPoints.push(new THREE.Vector3(x, y, z));
    
    // Remove oldest points if we exceed maximum
    if (trajectoryPoints.length > maxTrajectoryPoints) {
        trajectoryPoints.shift();
    }
    
    // Update trajectory line geometry
    if (trajectoryPoints.length > 1) {
        trajectoryLine.geometry.setFromPoints(trajectoryPoints);
        trajectoryLine.geometry.verticesNeedUpdate = true;
    }
}

connectServer();

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(
    75, 
    window.innerWidth / window.innerHeight, 
    0.1, 
    1000);

const renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// Add orbit controls for 3D navigation
const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true; // Smooth camera movement
controls.dampingFactor = 0.05;
controls.enableZoom = true;
controls.autoRotate = false;
controls.maxDistance = 10;
controls.minDistance = 1;
controls.screenSpacePanning = false;

// Green: North axis (X-direction in our coordinate system)
const geometry1 = new THREE.BoxGeometry(0.6, 0.02, 0.02);
const material1 = new THREE.MeshBasicMaterial({color: 0x00ff00});
const cube1 = new THREE.Mesh(geometry1, material1);
scene.add(cube1);

// Blue: East axis (Z-direction in our coordinate system) 
const geometry2 = new THREE.BoxGeometry(0.02, 0.02, 0.6);
const material2 = new THREE.MeshBasicMaterial({ color: 0x0000ff });
const cube2 = new THREE.Mesh(geometry2, material2);
scene.add(cube2);
const geometry3 = new THREE.BoxGeometry(0.02, 0.2, 0.02);
const material3 = new THREE.MeshBasicMaterial({ color: 0xff0000 });
const cube3 = new THREE.Mesh(geometry3, material3);
cube3.geometry.translate(0, 0.1, 0); // Translate up in Y-axis (our new up direction)
scene.add(cube3);

// Add ground plane (Y=0 is now ground level)
const planeGeometry = new THREE.PlaneGeometry(10, 10);
const planeMaterial = new THREE.MeshBasicMaterial({
    color: 0x808080,
    side: THREE.DoubleSide,
    transparent: true,
    opacity: 0.3
});
const groundPlane = new THREE.Mesh(planeGeometry, planeMaterial);
groundPlane.rotation.x = -Math.PI / 2; // Rotate to be horizontal
groundPlane.position.y = 0; // Place at y=0 (ground level)
scene.add(groundPlane);

// Add grid helper for better depth perception
const gridHelper = new THREE.GridHelper(10, 20, 0x444444, 0x444444);
gridHelper.position.y = 0.01; // Slightly above ground plane
scene.add(gridHelper);

// Initialize trajectory line
const trajectoryGeometry = new THREE.BufferGeometry();
const trajectoryMaterial = new THREE.LineBasicMaterial({ 
    color: 0xff00ff, // Magenta color for trajectory
    linewidth: 2
});
trajectoryLine = new THREE.Line(trajectoryGeometry, trajectoryMaterial);
scene.add(trajectoryLine);

// Set camera position - use standard Y-up and invert Z in rendering
camera.position.set(0, 5, 3); // Start behind and above the quadcopter
camera.lookAt(0, 0, 0);

const timeElement = document.getElementById("time");
const distanceElement = document.getElementById("distance");

function animate() {
    requestAnimationFrame(animate);

    // Convert NED to standard Y-up coordinate system for visualization
    cube1.position.x = -osten;   // East -> -X (korrigiertes Vorzeichen für blaue Achse)
    cube1.position.z = norden;   // North -> Z (vor/zurück ohne Negation)  
    cube1.position.y = -unten;   // Down -> -Y (hoch/runter, negiert)
    cube1.rotation.z = rollen;   // Roll around Z
    cube1.rotation.x = nicken;   // Pitch around X
    cube1.rotation.y = gieren;   // Yaw around Y
    
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

    timeElement.textContent = "T = " + t.toFixed(2);
    distanceElement.textContent = "D = " + (
        Math.abs(norden) + 
        Math.abs(osten) + 
        Math.abs(unten)).toFixed(4) + "m";

    // Update controls
    controls.update();
    
    renderer.render(scene, camera);
}

animate();

window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}, false);

