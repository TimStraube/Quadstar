let t = 0;
// Norden = +y
let norden = 0;
// Osten
let osten = 0;
let unten = 0;
let rollen = 0;
let nicken = 0;
let gieren = 0;

function connectServer() {
    fetch('/sensortest/', {
        method: 'POST',
        headers: 
        {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({})
    })
    .then(response => response.json())
        .then(data => {
            const json_data = JSON.parse(
                JSON.stringify(data)
            );
            t = json_data.t;
            norden = json_data.norden
            osten = json_data.osten
            unten = -json_data.unten
            rollen = json_data.rollen
            nicken = json_data.nicken
            gieren = json_data.gieren
            console.log("Rollen: " + rollen)
            console.log("Nicken: " + nicken)
            console.log("Gieren: " + gieren)
            connectServer();
    }).catch(error => {
        console.error(error);
    });
}

connectServer();

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(
    30, 
    window.innerWidth / window.innerHeight, 
    0.1, 
    1000
);

const renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const geometrynorden = new THREE.BoxGeometry(0.01, 0.3, 0.01);
const materialnorden = new THREE.MeshBasicMaterial({
    color: 0x0000ff
});
const nordenPfeil = new THREE.Mesh(geometrynorden, materialnorden);
nordenPfeil.position.x = 1
nordenPfeil.position.y = 1.15
nordenPfeil.position.z = 0
scene.add(nordenPfeil)

const geometryosten = new THREE.BoxGeometry(0.3, 0.01, 0.01);
const materialosten = new THREE.MeshBasicMaterial({
    color: 0x00ff00
});
const ostenPfeil = new THREE.Mesh(geometryosten, materialosten);
ostenPfeil.position.x = 1.15
ostenPfeil.position.y = 1
ostenPfeil.position.z = 0
scene.add(ostenPfeil)

const geometryunten = new THREE.BoxGeometry(0.01, 0.01, 0.3);
const materialunten = new THREE.MeshBasicMaterial({
    color: 0xff0000
});
const untenPfeil = new THREE.Mesh(geometryunten, materialunten);
untenPfeil.position.x = 1
untenPfeil.position.y = 1
untenPfeil.position.z = -0.15
scene.add(untenPfeil)

const geometry1 = new THREE.BoxGeometry(0.3, 0.02, 0.02);
const material1 = new THREE.MeshBasicMaterial({
    color: 0x00ff00
});
const cube1 = new THREE.Mesh(geometry1, material1);
cube1.geometry.translate(0.15, 0, 0);
scene.add(cube1);
const geometry2 = new THREE.BoxGeometry(0.02, 0.3, 0.02);
const material2 = new THREE.MeshBasicMaterial({
    color: 0x0000ff
});
const cube2 = new THREE.Mesh(geometry2, material2);

cube2.geometry.translate(0, 0.15, 0);
scene.add(cube2);
const geometry3 = new THREE.BoxGeometry(0.02, 0.02, 0.3);
const material3 = new THREE.MeshBasicMaterial({
    color: 0xff0000
});
const cube3 = new THREE.Mesh(geometry3, material3);
cube3.geometry.translate(0, 0, -0.15);
scene.add(cube3);

const geometry = new THREE.SphereGeometry(1, 32, 32); 
const material = new THREE.MeshBasicMaterial({
    color: 0x0000ff,
    transparent: true,
    opacity: 0.2
}); 
const sphere = new THREE.Mesh(geometry, material); 
scene.add(sphere);

camera.position.z = 5;
camera.lookAt(0, 0, 0);

const timeElement = document.getElementById("time");
const distanceElement = document.getElementById("distance");

function animate() {
    requestAnimationFrame(animate);

    cube1.position.y = norden;
    cube1.position.x = osten;
    cube1.position.z = unten;
    cube1.rotation.y = rollen;
    cube1.rotation.x = -nicken;
    cube1.rotation.z = gieren;
    cube2.position.y = norden;
    cube2.position.x = osten;
    cube2.position.z = unten;
    cube2.rotation.y = rollen;
    cube2.rotation.x = -nicken;
    cube2.rotation.z = gieren;
    cube3.position.y = norden;
    cube3.position.x = osten;
    cube3.position.z = unten;
    cube3.rotation.y = rollen;
    cube3.rotation.x = -nicken;
    cube3.rotation.z = gieren;

    timeElement.textContent = "T = " + t.toFixed(2);
    distanceElement.textContent = "D = " + (
        Math.abs(norden) + 
        Math.abs(osten) + 
        Math.abs(unten)).toFixed(4) + "m";

    renderer.render(scene, camera);
}

animate();

window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}, false);

