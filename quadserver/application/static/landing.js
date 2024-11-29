import * as THREE from 'three'; 
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js'; 
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

document.addEventListener('DOMContentLoaded', () => {
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(75, window.innerWidth / 500, 0.1, 1000);
    const renderer = new THREE.WebGLRenderer();
    renderer.setSize(window.innerWidth, 500);
    document.body.appendChild(renderer.domElement);


    const geometry = new THREE.BoxGeometry();
    const material = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    const quad = new THREE.Mesh(geometry, material);
    scene.add(quad);

    camera.position.z = 5;

    function animate() {
        requestAnimationFrame(animate);
        renderer.render(scene, camera);
    }
    
    document.getElementById('saveModel').addEventListener('click', function(event) {
        event.preventDefault(); // Prevent the default form submission

        // Obtain the CSRF token from the hidden input field
        var csrfToken = document.querySelector('[name="csrfmiddlewaretoken"]').value;

        var formData = {    
            'saveModel': true, 
        };

        // Send AJAX request using Fetch API
        fetch('/inputUpdate/', {  
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'X-CSRFToken': csrfToken
            },
            body: JSON.stringify(formData)
        })
        .then(response => response.json())
        .then(data => {
            if (data.status === 'success') {
                console.log("Model saved successfully");
            } else {
                alert('Error adding patient');
            }
        })
        .catch(error => {
            console.error('Error adding patient:', error);
            alert('Error adding patient');
        });
    });

    let t = 0;
    let north = 0;
    let east = 0;
    let down = 0;
    let roll = 0;
    let pitch = 0;
    let yaw = 0;

    function updateQuad() {
        fetch('/updateQuad/', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'X-CSRFToken': csrfToken
            },
            body: JSON.stringify({})
        })
        .then(response => response.json())
            .then(data => {
                const json_data = JSON.parse(
                    JSON.stringify(data)
                );
                t = json_data.t;
                north = json_data.north
                east = json_data.east
                down = -json_data.down
                roll = json_data.roll
                pitch = json_data.pitch
                yaw = json_data.yaw

                quad.position.x = east;
                quad.position.y = down;
                quad.position.z = north;
                quad.rotation.x = pitch;
                quad.rotation.y = roll;
                quad.rotation.z = yaw;
                
                connectServer();

        }).catch(error => {
            console.error(error);
        });
    }


    // Execute only when all DOM elements are loaded
    window.addEventListener('load', () => {
        // Start animation loop
        animate();
    });
});