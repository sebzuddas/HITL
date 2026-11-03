import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const PITCH_EXAGGERATION = 3;   // pitch angles are small; scale for visibility

export function createView(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(40, 1, 0.1, 50);
  camera.position.set(1.5, 1.05, 1.7);
  const controls = new OrbitControls(camera, canvas);
  controls.target.set(0, 0.5, -0.1);
  controls.enableDamping = true;

  scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 1.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.4); sun.position.set(3, 5, 2); scene.add(sun);

  const mat = (color, extra = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.2, ...extra });
  const ground = new THREE.Mesh(new THREE.CircleGeometry(1.6, 64), mat(0x2b3440, { roughness: 1 }));
  ground.rotation.x = -Math.PI / 2; scene.add(ground);
  const ring = new THREE.Mesh(new THREE.RingGeometry(1.2, 1.215, 96), new THREE.MeshBasicMaterial({ color: 0x7a8794, side: THREE.DoubleSide }));
  ring.rotation.x = -Math.PI / 2; ring.position.y = 0.002; scene.add(ring);
  // 0 deg travel marker
  const mark = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.004, 0.16), mat(0x4db3ff)); mark.position.set(0, 0.004, -1.2); scene.add(mark);

  const PIVOT_Y = 0.55, ARM = 0.5, COUNTER = 0.42;
  const post = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.05, PIVOT_Y, 24), mat(0x9aa5b1));
  post.position.y = PIVOT_Y / 2; scene.add(post);

  const yaw = new THREE.Group(); yaw.position.y = PIVOT_Y; scene.add(yaw);      // travel
  const elev = new THREE.Group(); yaw.add(elev);                                // elevation
  const armMesh = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.03, ARM + COUNTER), mat(0xdfe5ea));
  armMesh.position.z = (COUNTER - ARM) / 2; elev.add(armMesh);                  // arm runs along -z (head) / +z (counterweight)
  const weight = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.05, 24), mat(0x6b7683));
  weight.position.z = COUNTER; weight.rotation.x = Math.PI / 2; elev.add(weight);

  const head = new THREE.Group(); head.position.z = -ARM; elev.add(head);
  const pitchGroup = new THREE.Group(); head.add(pitchGroup);
  const bar = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.02, 0.02), mat(0xdfe5ea)); pitchGroup.add(bar);

  const fans = [];
  for (const side of [1, -1]) {                       // fan A at +x, fan B at -x
    const g = new THREE.Group(); g.position.set(side * 0.17, 0.045, 0); pitchGroup.add(g);
    const duct = new THREE.Mesh(new THREE.TorusGeometry(0.06, 0.008, 12, 40), mat(side > 0 ? 0xd95319 : 0x0072bd));
    duct.rotation.x = Math.PI / 2; g.add(duct);
    const blades = new THREE.Group(); g.add(blades);
    for (let i = 0; i < 3; i++) {
      const b = new THREE.Mesh(new THREE.BoxGeometry(0.11, 0.004, 0.02), mat(0x222a33)); b.rotation.y = i * Math.PI / 3; blades.add(b);
    }
    const flow = new THREE.Mesh(new THREE.ConeGeometry(0.05, 0.1, 20, 1, true), new THREE.MeshBasicMaterial({ color: 0x9fd8ff, transparent: true, opacity: 0.0, side: THREE.DoubleSide }));
    flow.position.y = -0.06; flow.rotation.x = Math.PI; g.add(flow);
    fans.push({ blades, flow });
  }

  function resize() {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix();
  }
  new ResizeObserver(resize).observe(canvas); resize();

  return {
    /** angles in rad, fan voltages in V, dt seconds of wall time for spin animation */
    update({ E, P, T, va, vb }, dt = 0) {
      yaw.rotation.y = -T;
      elev.rotation.x = E;
      pitchGroup.rotation.z = P * PITCH_EXAGGERATION;
      [va, vb].forEach((v, i) => {
        fans[i].blades.rotation.y += dt * v * 6;
        fans[i].flow.material.opacity = Math.min(0.35, Math.max(0, (v - 2) / 12));
      });
      controls.update();
      renderer.render(scene, camera);
    },
  };
}
