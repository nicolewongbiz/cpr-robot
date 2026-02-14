// Import Three.js and addons
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// Scene setup
let scene, camera, renderer, controls;
let model = null;
let mixer = null;
let clock = new THREE.Clock();

// Initialize the scene
function init() {
    // Create scene
    scene = new THREE.Scene();
    
    // Create camera
    camera = new THREE.PerspectiveCamera(
        60, // Field of view (reduced for more zoom)
        window.innerWidth / window.innerHeight, // Aspect ratio
        0.1, // Near clipping plane
        1000 // Far clipping plane
    );
    camera.position.set(0, 0, 2.5);
    
    // Create renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.5; // Increased exposure further to make glow more visible
    
    // Add renderer to DOM
    const container = document.getElementById('canvas-container');
    container.appendChild(renderer.domElement);
    
    // Add lighting to match Blender's appearance
    // Reduced ambient light to make emission/glow more prominent
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
    scene.add(ambientLight);
    
    // Directional light (like sun) - reduced intensity
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
    directionalLight.position.set(5, 5, 5);
    scene.add(directionalLight);
    
    // Hemisphere light for more natural lighting - reduced intensity
    const hemisphereLight = new THREE.HemisphereLight(0xffffff, 0x444444, 0.3);
    scene.add(hemisphereLight);
    
    // Setup OrbitControls for mouse rotation
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true; // Smooth rotation
    controls.dampingFactor = 0.05;
    controls.enableZoom = true;
    controls.enablePan = false; // Disable panning, only rotation
    controls.minDistance = 2;
    controls.maxDistance = 10;
    
    // Load the model
    loadModel();
    
    // Handle window resize
    window.addEventListener('resize', onWindowResize);
    
    // Start animation loop
    animate();
}

// Load the glTF/GLB model
function loadModel() {
    const loader = new GLTFLoader();
    
    // Update this path to your model file
    const modelPath = 'models/heart.glb';
    
    loader.load(
        modelPath,
        // onLoad callback
        function(gltf) {
            model = gltf.scene;
            scene.add(model);
            
            // Traverse model and ensure emission properties are preserved
            // Also collect spheres to change their colors
            const spheres = [];
            model.traverse((child) => {
                if (child.isMesh && child.material) {
                    // Handle both single materials and material arrays
                    const materials = Array.isArray(child.material) ? child.material : [child.material];
                    
                    materials.forEach((material) => {
                        // Ensure emission is enabled if it exists
                        if (material.emissive) {
                            // Material already has emission, ensure it's visible
                            material.emissiveIntensity = material.emissiveIntensity || 1.0;
                        }
                        
                        // Ensure emission maps are properly set
                        if (material.emissiveMap) {
                            material.emissiveMap.needsUpdate = true;
                        }
                        
                        // Ensure textures are properly updated
                        if (material.map) {
                            material.map.needsUpdate = true;
                        }
                        
                        // Make sure material is not overridden by lighting
                        // Emission should be visible regardless of lighting
                        material.needsUpdate = true;
                    });
                    
                    // Collect sphere meshes (assuming they're named or can be identified)
                    // Check if it's a sphere by geometry type or name
                    if (child.geometry && (child.name.toLowerCase().includes('sphere') || 
                        child.name.toLowerCase().includes('ball') ||
                        child.geometry.type === 'SphereGeometry')) {
                        spheres.push(child);
                    }
                }
            });
            
            // Change colors of spheres: first to deep red, second to deep blue with glow
            const deepRed = 0x990000; // Deep red
            const deepBlue = 0x000099; // Deep blue
            const glowIntensity = 5.0; // Very strong glow intensity
            
            if (spheres.length >= 2) {
                // First sphere to deep red with glow
                const redMaterial = Array.isArray(spheres[0].material) ? spheres[0].material[0] : spheres[0].material;
                redMaterial.color.set(deepRed); // Deep red
                redMaterial.emissive = redMaterial.emissive || new THREE.Color();
                redMaterial.emissive.set(0xff3333); // Bright red emission for glow
                redMaterial.emissiveIntensity = glowIntensity; // Very strong glow intensity
                
                // Second sphere to deep blue with glow
                const blueMaterial = Array.isArray(spheres[1].material) ? spheres[1].material[0] : spheres[1].material;
                blueMaterial.color.set(deepBlue); // Deep blue
                blueMaterial.emissive = blueMaterial.emissive || new THREE.Color();
                blueMaterial.emissive.set(0x3333ff); // Bright blue emission for glow
                blueMaterial.emissiveIntensity = glowIntensity; // Very strong glow intensity
            } else if (spheres.length === 1) {
                // If only one sphere found, make it deep red with glow
                const redMaterial = Array.isArray(spheres[0].material) ? spheres[0].material[0] : spheres[0].material;
                redMaterial.color.set(deepRed); // Deep red
                redMaterial.emissive = redMaterial.emissive || new THREE.Color();
                redMaterial.emissive.set(0xff3333); // Bright red emission for glow
                redMaterial.emissiveIntensity = glowIntensity; // Very strong glow intensity
            } else {
                // If no spheres found by name, try to find all meshes and change first two
                const allMeshes = [];
                model.traverse((child) => {
                    if (child.isMesh) {
                        allMeshes.push(child);
                    }
                });
                
                if (allMeshes.length >= 2) {
                    // First mesh to deep red with glow
                    const redMaterial = Array.isArray(allMeshes[0].material) ? allMeshes[0].material[0] : allMeshes[0].material;
                    redMaterial.color.set(deepRed); // Deep red
                    redMaterial.emissive = redMaterial.emissive || new THREE.Color();
                    redMaterial.emissive.set(0xff3333); // Bright red emission for glow
                    redMaterial.emissiveIntensity = glowIntensity; // Very strong glow intensity
                    
                    // Second mesh to deep blue with glow
                    const blueMaterial = Array.isArray(allMeshes[1].material) ? allMeshes[1].material[0] : allMeshes[1].material;
                    blueMaterial.color.set(deepBlue); // Deep blue
                    blueMaterial.emissive = blueMaterial.emissive || new THREE.Color();
                    blueMaterial.emissive.set(0x3333ff); // Bright blue emission for glow
                    blueMaterial.emissiveIntensity = glowIntensity; // Very strong glow intensity
                }
            }
            
            // Center and scale the model if needed
            const box = new THREE.Box3().setFromObject(model);
            const center = box.getCenter(new THREE.Vector3());
            const size = box.getSize(new THREE.Vector3());
            
            // Center the model
            model.position.x -= center.x;
            model.position.y -= center.y;
            model.position.z -= center.z;
            
            // Scale to fit if model is too large/small
            const maxDim = Math.max(size.x, size.y, size.z);
            const scale = 3.2 / maxDim; // Adjusted scale for better fit
            model.scale.multiplyScalar(scale);
            
            // Setup animation mixer
            if (gltf.animations && gltf.animations.length > 0) {
                mixer = new THREE.AnimationMixer(model);
                
                // Play ALL animation clips, not just the first one
                const targetDuration = 11 / 24; // ~0.458 seconds
                const animationSpeed = 0.5; // Slow down animation (0.5 = half speed)
                
                gltf.animations.forEach((clip) => {
                    // Create animation action for this clip
                    const action = mixer.clipAction(clip);
                    
                    // Set animation to loop
                    action.setLoop(THREE.LoopRepeat);
                    
                    // Adjust timeScale to slow down the animation
                    if (clip.duration !== targetDuration) {
                        action.timeScale = (clip.duration / targetDuration) * animationSpeed;
                    } else {
                        action.timeScale = animationSpeed;
                    }
                    
                    // Play the animation
                    action.play();
                });
            }
            
            // Hide loading indicator
            const loadingIndicator = document.getElementById('loading-indicator');
            loadingIndicator.classList.add('hidden');
        },
        // onProgress callback
        function(xhr) {
            if (xhr.lengthComputable) {
                const percentComplete = (xhr.loaded / xhr.total) * 100;
                console.log('Model loading: ' + Math.round(percentComplete) + '%');
            }
        },
        // onError callback
        function(error) {
            console.error('Error loading model:', error);
            const loadingIndicator = document.getElementById('loading-indicator');
            loadingIndicator.textContent = 'Error loading model. Please check the file path.';
            loadingIndicator.style.color = '#ff0000';
        }
    );
}

// Animation loop
function animate() {
    requestAnimationFrame(animate);
    
    const delta = clock.getDelta();
    
    // Update animation mixer
    if (mixer) {
        mixer.update(delta);
    }
    
    // Update controls
    controls.update();
    
    // Render the scene
    renderer.render(scene, camera);
}

// Handle window resize
function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

// Initialize when page loads
init();

