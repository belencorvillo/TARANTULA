// Visor 3D de la Tarántula: carga el CAD (models/tarantula.glb) y monta la
// cadena cinemática idéntica a Leg.cpp / Tarantula.cpp.
//
// Sistema del código: X adelante, Y derecha, Z arriba (ver MainWindow::updateTrotVelocity).
// Como es un sistema "zurdo", todo el robot cuelga de un grupo con escala Y = -1
// para que en pantalla la pata 1 (DD) aparezca realmente delante-derecha.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

// Constantes de Leg.h / Tarantula.cpp
export const R_HIP = 0.15, L1 = 0.08, L2 = 0.188, L3 = 0.211;
export const LEG_THETA = [Math.PI / 4, -Math.PI / 4, -3 * Math.PI / 4, 3 * Math.PI / 4];
const DEG = Math.PI / 180;
const FOOT_RADIUS = 0.006;      // radio aproximado de la punta del pie (CAD)
const BODY_BOTTOM_Z = -0.032;   // cara inferior de la placa inferior (CAD)

export class Spider3D {
  constructor(container) {
    this.container = container;
    this.legs = [];
    this.target = Array.from({ length: 4 }, () => [0, 20, -100]); // grados
    this.current = this.target.map(a => [...a]);
    this.bodyTarget = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
    this.body = { ...this.bodyTarget };
    this.grounded = [false, false, false, false];
    this.highlightLeg = -1;
    this.groundOffset = 0.15;
    this.onLegClick = null;
    this.ready = this.#init();
  }

  async #init() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    const renderer = this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(w, h);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.0;
    this.container.appendChild(renderer.domElement);

    const scene = this.scene = new THREE.Scene();
    const pmrem = new THREE.PMREMGenerator(renderer);
    scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    scene.environmentIntensity = 0.55;

    const camera = this.camera = new THREE.PerspectiveCamera(38, w / h, 0.01, 20);
    camera.up.set(0, 0, 1);
    camera.position.set(-0.95, -0.85, 0.75);

    const controls = this.controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, 0.12);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.minDistance = 0.35;
    controls.maxDistance = 4;
    controls.maxPolarAngle = Math.PI * 0.495;

    // Luces
    scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x1a1d24, 0.6));
    const key = new THREE.DirectionalLight(0xffffff, 2.2);
    key.position.set(-1.2, -0.8, 2.2);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    Object.assign(key.shadow.camera, { left: -0.8, right: 0.8, top: 0.8, bottom: -0.8, near: 0.1, far: 5 });
    key.shadow.bias = -0.0004;
    key.shadow.radius = 4;
    scene.add(key);
    const rim = new THREE.DirectionalLight(0x8fb4ff, 1.4);
    rim.position.set(1.5, 1.2, 0.8);
    scene.add(rim);

    // Suelo: sombra + rejilla + marcador de "delante"
    const shadowPlane = new THREE.Mesh(new THREE.PlaneGeometry(8, 8), new THREE.ShadowMaterial({ opacity: 0.32 }));
    shadowPlane.receiveShadow = true;
    scene.add(shadowPlane);
    const grid = this.grid = new THREE.GridHelper(4, 40, 0x3a4150, 0x262b35);
    grid.rotation.x = Math.PI / 2;
    grid.material.transparent = true;
    grid.material.opacity = 0.55;
    grid.position.z = 0.0005;
    scene.add(grid);
    const arrow = new THREE.Mesh(
      new THREE.ShapeGeometry(new THREE.Shape([new THREE.Vector2(0.62, 0), new THREE.Vector2(0.54, 0.035), new THREE.Vector2(0.54, -0.035)])),
      new THREE.MeshBasicMaterial({ color: 0xf5c451, transparent: true, opacity: 0.85 }));
    arrow.position.z = 0.001;
    scene.add(arrow);

    // Raíz del robot (espejo Y) -> pose del cuerpo -> patas
    this.root = new THREE.Group();
    this.root.scale.y = -1;
    scene.add(this.root);
    this.bodyGroup = new THREE.Group();
    this.bodyGroup.rotation.order = 'ZYX';
    this.root.add(this.bodyGroup);

    const gltf = await new GLTFLoader().loadAsync('models/tarantula.glb');
    const parts = Object.fromEntries(gltf.scene.children.map(n => [n.name, n]));

    const prep = (obj) => {
      obj.traverse(o => {
        if (!o.isMesh) return;
        o.castShadow = true;
        o.receiveShadow = true;
        const m = o.material;
        // Aclarar ligeramente el negro puro del CAD para que se lean los volúmenes
        if (m.color.r < 0.02) m.color.setRGB(0.035, 0.037, 0.042);
        m.envMapIntensity = 1.0;
      });
      return obj;
    };
    const body = prep(parts.body.clone());
    body.position.set(0, 0, 0);
    this.bodyGroup.add(body);

    const footGeo = new THREE.RingGeometry(0.018, 0.026, 40);
    for (let i = 0; i < 4; i++) {
      const base = new THREE.Group();
      base.position.set(R_HIP * Math.cos(LEG_THETA[i]), R_HIP * Math.sin(LEG_THETA[i]), 0);
      base.rotation.z = LEG_THETA[i];
      this.bodyGroup.add(base);

      const coxaJ = new THREE.Group(); base.add(coxaJ);
      const hipJ = new THREE.Group(); hipJ.position.x = L1; coxaJ.add(hipJ);
      const kneeJ = new THREE.Group(); kneeJ.position.x = L2; hipJ.add(kneeJ);
      const foot = new THREE.Object3D(); foot.position.x = L3; kneeJ.add(foot);

      const meshes = [parts.coxa, parts.femur, parts.tibia].map(p => prep(p.clone(true)));
      meshes[0].position.set(0, 0, 0); coxaJ.add(meshes[0]);
      meshes[1].position.set(0, 0, 0); hipJ.add(meshes[1]);
      meshes[2].position.set(0, 0, 0); kneeJ.add(meshes[2]);
      meshes.forEach(m => m.traverse(o => { if (o.isMesh) o.userData.leg = i; }));

      // Anillo de apoyo bajo el pie (en el suelo, fuera de la jerarquía)
      const ring = new THREE.Mesh(footGeo, new THREE.MeshBasicMaterial({ color: 0x7ee2a8, transparent: true, opacity: 0, depthWrite: false }));
      ring.position.z = 0.002;
      scene.add(ring);

      this.legs.push({ coxaJ, hipJ, kneeJ, foot, meshes, ring });
    }

    // Selección de pata con clic
    const ray = new THREE.Raycaster();
    const ndc = new THREE.Vector2();
    let downAt = null;
    renderer.domElement.addEventListener('pointerdown', e => { downAt = [e.clientX, e.clientY]; });
    renderer.domElement.addEventListener('pointerup', e => {
      if (!downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 4) return;
      const r = renderer.domElement.getBoundingClientRect();
      ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
      ray.setFromCamera(ndc, camera);
      const hit = ray.intersectObject(this.bodyGroup, true).find(h => h.object.userData.leg !== undefined);
      if (hit && this.onLegClick) this.onLegClick(hit.object.userData.leg + 1);
    });

    new ResizeObserver(() => this.#resize()).observe(this.container);
    this.clock = new THREE.Clock();
    renderer.setAnimationLoop(() => this.#frame());
  }

  #resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  /** angles: [[q1,q2,q3] x4] en grados (convención de Leg::solveIK) */
  setJointAngles(angles) {
    angles.forEach((a, i) => { if (a) this.target[i] = a.map(v => (Number.isFinite(v) ? v : 0)); });
  }
  setBodyPose(p) { Object.assign(this.bodyTarget, p); }
  setGrounded(g) { this.grounded = g; }
  setHighlight(legId) {
    this.highlightLeg = legId - 1;
    this.legs.forEach((l, i) => l.meshes.forEach(m => m.traverse(o => {
      if (!o.isMesh) return;
      if (!o.userData.baseEmissive) { o.material = o.material.clone(); o.userData.baseEmissive = true; }
      o.material.emissive.setHex(i === this.highlightLeg ? 0x0f2550 : 0x000000);
    })));
  }

  resetView() {
    this.camera.position.set(-0.95, -0.85, 0.75);
    this.controls.target.set(0, 0, 0.12);
  }

  #frame() {
    const dt = Math.min(this.clock.getDelta(), 0.1);
    const k = 1 - Math.exp(-dt * 18); // suavizado (~55 ms) para ocultar el muestreo de la red
    for (let i = 0; i < 4; i++)
      for (let j = 0; j < 3; j++) this.current[i][j] += (this.target[i][j] - this.current[i][j]) * k;
    for (const key in this.body) this.body[key] += (this.bodyTarget[key] - this.body[key]) * k;

    // Cadena cinemática (rotaciones como en Leg::forwardKinematics)
    this.legs.forEach((l, i) => {
      const [q1, q2, q3] = this.current[i];
      l.coxaJ.rotation.z = q1 * DEG;
      l.hipJ.rotation.y = -q2 * DEG;
      l.kneeJ.rotation.y = -q3 * DEG;
    });
    const b = this.body;
    this.bodyGroup.position.set(b.x / 1000, b.y / 1000, b.z / 1000);
    this.bodyGroup.rotation.set(b.roll * DEG, b.pitch * DEG, b.yaw * DEG);

    // Apoyar el robot en el suelo: el punto más bajo (pie o panza) toca z = 0
    this.root.position.z = 0;
    this.root.updateMatrixWorld(true);
    const v = new THREE.Vector3();
    let minZ = Infinity;
    const feet = this.legs.map(l => { l.foot.getWorldPosition(v); minZ = Math.min(minZ, v.z - FOOT_RADIUS); return v.clone(); });
    for (const [x, y] of [[0.13, 0.13], [0.13, -0.13], [-0.13, 0.13], [-0.13, -0.13]]) {
      v.set(x, y, BODY_BOTTOM_Z); this.bodyGroup.localToWorld(v); minZ = Math.min(minZ, v.z);
    }
    this.groundOffset += (-minZ - this.groundOffset) * k;
    this.root.position.z = this.groundOffset;

    // Anillos de apoyo
    this.legs.forEach((l, i) => {
      l.ring.position.x = feet[i].x;
      l.ring.position.y = feet[i].y;
      const tgt = this.grounded[i] ? 0.9 : 0;
      l.ring.material.opacity += (tgt - l.ring.material.opacity) * k;
    });

    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
