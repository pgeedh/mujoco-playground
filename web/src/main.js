
import * as THREE           from 'three';
import { GUI              } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls    } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { DragStateManager } from './utils/DragStateManager.js';
import { setupGUI, loadSceneFromURL, drawTendonsAndFlex, getPosition, getQuaternion, toMujocoPos, standardNormal } from './mujocoUtils.js';
import { G1_PARK_SCENE, downloadG1ParkScene } from './g1Scene.js';
import { G1Controller } from './g1Control.js';
import { initMenu } from './menu.js';
import   load_mujoco        from '../node_modules/@mujoco/mujoco/mujoco.js';

// Load the MuJoCo Module
// The .wasm binary ships separately from the .js loader, so point Emscripten
// at it explicitly; this resolves correctly from both ./src and the esbuild bundle.
const mujoco = await load_mujoco({
  locateFile: (path, prefix) => path.endsWith(".wasm") ?
    new URL('../node_modules/@mujoco/mujoco/mujoco.wasm', import.meta.url).href : prefix + path
});

// Set up Emscripten's Virtual File System. Our scene has an <include> and
// mesh assets, so it can't be loaded synchronously like the demo's
// single-file humanoid.xml — model/data stay null until init() finishes
// downloading everything and calls loadSceneFromURL.
var initialScene = G1_PARK_SCENE;
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');

export class MuJoCoDemo {
  constructor() {
    this.mujoco = mujoco;

    this.model = null;
    this.data  = null;
    this.g1Controller = new G1Controller();

    // Define Random State Variables
    this.params = { scene: initialScene, paused: false, help: false, ctrlnoiserate: 0.0, ctrlnoisestd: 0.0, keyframeNumber: 0 };
    this.mujoco_time = 0.0;
    this.bodies  = {}, this.lights = {};
    this.tmpVec  = new THREE.Vector3();
    this.tmpQuat = new THREE.Quaternion();
    this.updateGUICallbacks = [];

    this.container = document.createElement( 'div' );
    document.body.appendChild( this.container );

    this.scene = new THREE.Scene();
    this.scene.name = 'scene';

    this.camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 0.001, 100 );
    this.camera.name = 'PerspectiveCamera';
    this.camera.position.set(-2.2, 1.5, 1.8); // behind the robot so the course ahead is in view
    this.scene.add(this.camera);
    // The camera looks down local -Z by default; this model's forward
    // (the direction it walks under W) is local +X, so first-person mode
    // needs this fixed correction to look the right way.
    this._fpvCorrection = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -Math.PI / 2);

    this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
    this.scene.fog = new THREE.Fog(this.scene.background, 15, 25.5 );

    this.ambientLight = new THREE.AmbientLight( 0xffffff, 0.1 * 3.14 );
    this.ambientLight.name = 'AmbientLight';
    this.scene.add( this.ambientLight );

    this.spotlight = new THREE.SpotLight();
    this.spotlight.angle = 1.11;
    this.spotlight.distance = 10000;
    this.spotlight.penumbra = 0.5;
    this.spotlight.castShadow = true; // default false
    this.spotlight.intensity = this.spotlight.intensity * 3.14 * 10.0;
    this.spotlight.shadow.mapSize.width = 1024; // default
    this.spotlight.shadow.mapSize.height = 1024; // default
    this.spotlight.shadow.camera.near = 0.1; // default
    this.spotlight.shadow.camera.far = 100; // default
    this.spotlight.position.set(0, 3, 3);
    const targetObject = new THREE.Object3D();
    this.scene.add(targetObject);
    this.spotlight.target = targetObject;
    targetObject.position.set(0, 1, 0);
    this.scene.add( this.spotlight );

    this.renderer = new THREE.WebGLRenderer( { antialias: true } );
    this.renderer.setPixelRatio(1.0);////window.devicePixelRatio );
    this.renderer.setSize( window.innerWidth, window.innerHeight );
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap; // default THREE.PCFShadowMap
    THREE.ColorManagement.enabled = false;
    this.renderer.outputColorSpace = THREE.LinearSRGBColorSpace;
    //this.renderer.outputColorSpace = THREE.LinearSRGBColorSpace;
    //this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    //this.renderer.toneMappingExposure = 2.0;
    this.renderer.useLegacyLights = true;

    this.renderer.setAnimationLoop( this.render.bind(this) );

    this.container.appendChild( this.renderer.domElement );

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.7, 0);
    this.controls.panSpeed = 2;
    this.controls.zoomSpeed = 1;
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.10;
    this.controls.screenSpacePanning = true;
    this.controls.update();

    window.addEventListener('resize', this.onWindowResize.bind(this));

    // Initialize the Drag State Manager.
    this.dragStateManager = new DragStateManager(this.scene, this.renderer, this.camera, this.container.parentElement, this.controls);

    // C toggles third-person (free orbit camera) vs first-person (glued to
    // the robot's head, looking the way it's facing).
    this.cameraMode = 'third';
    this._prevKeyC = false;
    window.addEventListener('keydown', (e) => {
      if (e.code === 'KeyC' && !this._prevKeyC) {
        if (this.cameraMode === 'third') {
          // Leaving third-person: remember exactly where the orbit camera
          // was so switching back restores it, instead of OrbitControls
          // re-syncing its orbit radius from wherever first-person left
          // the camera (which produced a bizarre inside-the-chest close-up).
          this._savedThirdPersonPos = this.camera.position.clone();
          this._savedThirdPersonQuat = this.camera.quaternion.clone();
          this.cameraMode = 'first';
          this.controls.enabled = false;
        } else {
          this.cameraMode = 'third';
          if (this._savedThirdPersonPos) {
            this.camera.position.copy(this._savedThirdPersonPos);
            this.camera.quaternion.copy(this._savedThirdPersonQuat);
          }
          this.controls.enabled = true;
          this.controls.update();
        }
      }
      if (e.code === 'KeyC') this._prevKeyC = true;
    });
    window.addEventListener('keyup', (e) => { if (e.code === 'KeyC') this._prevKeyC = false; });
  }

  async init() {
    // Download our G1 + obstacles scene (model XML + meshes) to MuJoCo's virtual file system
    await downloadG1ParkScene(mujoco);

    // Initialize the three.js Scene using the .xml Model in initialScene
    [this.model, this.data, this.bodies, this.lights] =
      await loadSceneFromURL(mujoco, initialScene, this);

    // loadSceneFromURL never runs forward kinematics, so data.xpos for every
    // body reads as [0,0,0] until the first mj_step/mj_forward — without
    // this, the very first delivery-distance check would see every body
    // (props included) as coincident at the origin and instantly (and
    // sticky-ly) mark them delivered.
    mujoco.mj_forward(this.model, this.data);

    this.g1Controller.bindModel(this.model, mujoco);
    await this.g1Controller.load();

    this.torsoBodyId = mujoco.mj_name2id(this.model, mujoco.mjtObj.mjOBJ_BODY.value, "torso_link");

    this.gui = new GUI();
    setupGUI(this);
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize( window.innerWidth, window.innerHeight );
  }

  render(timeMS) {
    if (!this.model || !this.data || !this.g1Controller.ready) { return; }
    if (this.cameraMode === 'third') { this.controls.update(); }
    this.g1Controller.update(this.data);

    if (!this.params["paused"]) {
      let timestep = this.model.opt.timestep;
      // Cap the physics catch-up debt at 35ms; clamping to (timeMS - 35) rather
      // than timeMS ensures at least some steps run even after a slow frame.
      if (timeMS - this.mujoco_time > 35.0) { this.mujoco_time = timeMS - 35.0; }
      while (this.mujoco_time < timeMS) {

        // Jitter the control state with gaussian random noise
        if (this.params["ctrlnoisestd"] > 0.0) {
          let rate  = Math.exp(-timestep / Math.max(1e-10, this.params["ctrlnoiserate"]));
          let scale = this.params["ctrlnoisestd"] * Math.sqrt(1 - rate * rate);
          let currentCtrl = this.data.ctrl;
          for (let i = 0; i < currentCtrl.length; i++) {
            currentCtrl[i] = rate * currentCtrl[i] + scale * standardNormal();
            this.params["Actuator " + i] = currentCtrl[i];
          }
        }

        // Clear old perturbations, apply new ones.
        for (let i = 0; i < this.data.qfrc_applied.length; i++) { this.data.qfrc_applied[i] = 0.0; }
        let dragged = this.dragStateManager.physicsObject;
        if (dragged && dragged.bodyID) {
          for (let b = 0; b < this.model.nbody; b++) {
            if (this.bodies[b]) {
              getPosition  (this.data.xpos , b, this.bodies[b].position);
              getQuaternion(this.data.xquat, b, this.bodies[b].quaternion);
              this.bodies[b].updateWorldMatrix();
            }
          }
          let bodyID = dragged.bodyID;
          this.dragStateManager.update(); // Update the world-space force origin
          let force = toMujocoPos(this.dragStateManager.currentWorld.clone().sub(this.dragStateManager.worldHit).multiplyScalar(this.model.body_mass[bodyID] * 250));
          let point = toMujocoPos(this.dragStateManager.worldHit.clone());
          mujoco.mj_applyFT(this.model, this.data, [force.x, force.y, force.z], [0, 0, 0], [point.x, point.y, point.z], bodyID, this.data.qfrc_applied);

          // TODO: Apply pose perturbations (mocap bodies only).
        }

        this.g1Controller.beforeStep(this.data, timestep);
        mujoco.mj_step(this.model, this.data);

        this.mujoco_time += timestep * 1000.0;
      }

    } else if (this.params["paused"]) {
      this.dragStateManager.update(); // Update the world-space force origin
      let dragged = this.dragStateManager.physicsObject;
      if (dragged && dragged.bodyID) {
        let b = dragged.bodyID;
        getPosition  (this.data.xpos , b, this.tmpVec , false); // Get raw coordinate from MuJoCo
        getQuaternion(this.data.xquat, b, this.tmpQuat, false); // Get raw coordinate from MuJoCo

        let offset = toMujocoPos(this.dragStateManager.currentWorld.clone()
          .sub(this.dragStateManager.worldHit).multiplyScalar(0.3));
        if (this.model.body_mocapid[b] >= 0) {
          // Set the root body's mocap position...
          console.log("Trying to move mocap body", b);
          let addr = this.model.body_mocapid[b] * 3;
          let pos  = this.data.mocap_pos;
          pos[addr+0] += offset.x;
          pos[addr+1] += offset.y;
          pos[addr+2] += offset.z;
        } else {
          // Set the root body's position directly...
          let root = this.model.body_rootid[b];
          let addr = this.model.jnt_qposadr[this.model.body_jntadr[root]];
          let pos  = this.data.qpos;
          pos[addr+0] += offset.x;
          pos[addr+1] += offset.y;
          pos[addr+2] += offset.z;
        }
      }

      mujoco.mj_forward(this.model, this.data);
    }

    // Update body transforms.
    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition  (this.data.xpos , b, this.bodies[b].position);
        getQuaternion(this.data.xquat, b, this.bodies[b].quaternion);
        this.bodies[b].updateWorldMatrix();
      }
    }

    // First-person camera: glued to the torso, looking the way it's
    // facing. The camera's default look direction is local -Z, but this
    // model's "forward" (the direction it actually walks under W) is
    // local +X, hence the fixed -90° yaw correction.
    if (this.cameraMode === 'first' && this.torsoBodyId >= 0) {
      const torsoPos = getPosition(this.data.xpos, this.torsoBodyId, new THREE.Vector3());
      const torsoQuat = getQuaternion(this.data.xquat, this.torsoBodyId, new THREE.Quaternion());
      const eyeOffset = new THREE.Vector3(0.08, 0.32, 0).applyQuaternion(torsoQuat);
      this.camera.position.copy(torsoPos).add(eyeOffset);
      this.camera.quaternion.copy(torsoQuat).multiply(this._fpvCorrection);
    }

    // Update light transforms.
    for (let l = 0; l < this.model.nlight; l++) {
      if (this.lights[l]) {
        getPosition(this.data.light_xpos, l, this.lights[l].position);
        getPosition(this.data.light_xdir, l, this.tmpVec);
        this.lights[l].lookAt(this.tmpVec.add(this.lights[l].position));
      }
    }

    // Draw Tendons and Flex verts
    drawTendonsAndFlex(this.mujocoRoot, this.model, this.data);

    // Render!
    this.renderer.render( this.scene, this.camera );
  }
}

let demo = new MuJoCoDemo();
window.demo = demo; // for console debugging
await demo.init();

// Hold the sim paused behind the robot-select screen; picking a robot
// starts it, and Esc re-opens the menu (paused again).
demo.params.paused = true;
initMenu({
  onStart: () => { demo.mujoco_time = performance.now(); demo.params.paused = false; },
  onOpen:  () => { demo.params.paused = true; },
});
