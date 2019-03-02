let _renderer;
let _scene;
let _camera;
let _cameraControl;
let _physicsWorld;
let _clock;
let _rigidBodies = [];

Ammo().then(init);

function init(Ammo)
{
	_renderer = new THREE.WebGLRenderer({antialias:true});
	_renderer.setPixelRatio(window.devicePixelRatio);
	_renderer.setSize(window.innerWidth, window.innerHeight);
	_renderer.shadowMap.enabled = true;
	document.body.appendChild(_renderer.domElement);
	window.addEventListener('resize', resize, false);

	_scene = new THREE.Scene();
	_scene.background = new THREE.Color(0xf0f0f0);
	_clock = new THREE.Clock();

	_camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.01, 100);
	_camera.position.set(1, 1,2, 1)

    _cameraControl = new THREE.OrbitControls(_camera);
    _cameraControl.target.y = 0;

    _scene.add(new THREE.AmbientLight(0xf0f0f0));
    let light = new THREE.DirectionalLight(0xffffff, 1);
    light.castShadow = true;
	light.shadow.mapSize.width = 1024;
	light.shadow.mapSize.height = 1024
    _scene.add(light)

	let collisionConfiguration = new Ammo.btSoftBodyRigidBodyCollisionConfiguration();
	let dispatcher = new Ammo.btCollisionDispatcher(collisionConfiguration);
	let broadphase = new Ammo.btDbvtBroadphase();
	let solver = new Ammo.btSequentialImpulseConstraintSolver();
	let softBodySolver = new Ammo.btDefaultSoftBodySolver();
	_physicsWorld = new Ammo.btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration, softBodySolver);
	_physicsWorld.setGravity(new Ammo.btVector3(0, -9.8, 0));
	_physicsWorld.getWorldInfo().set_m_gravity(new Ammo.btVector3(0, -9.8, 0));

    let planeGeometry = new THREE.PlaneBufferGeometry(5, 5);
	planeGeometry.rotateX(-Math.PI / 2);
	let planeMaterial = new THREE.ShadowMaterial({opacity: 0.2});
	let planeMesh = new THREE.Mesh(planeGeometry, planeMaterial);
	planeMesh.position.y = 0;
	planeMesh.receiveShadow = true;
	createRigidBody(planeMesh, new Ammo.btBoxShape(new Ammo.btVector3(10.0, 0.1, 10.0), 0), 0, new THREE.Vector3(0.0, -0.1, 0.0));

	let helper = new THREE.GridHelper(5, 50);
	helper.position.y = 0;
	helper.material.opacity = 0.25;
	helper.material.transparent = true;
	_scene.add(helper);

    let boxMesh = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.2, 0.2), new THREE.MeshPhongMaterial({color: 0x999999}));
    boxMesh.position.set(0, 1, 0);
    boxMesh.castShadow = true;
    createRigidBody(boxMesh, new Ammo.btBoxShape(new Ammo.btVector3(0.1, 0.1, 0.1)), 1);

    update();
}


function createRigidBody(object, shape, mass, tOffset)
{
	let transform = new Ammo.btTransform();
	transform.setIdentity();
	if(tOffset !== undefined) {
		object.tOffset = tOffset;
		transform.setOrigin(new Ammo.btVector3(object.position.x + tOffset.x, object.position.y + tOffset.y, object.position.z + tOffset.z));
	}
	else {
		transform.setOrigin(new Ammo.btVector3(object.position.x, object.position.y, object.position.z));
	}
	
    transform.setRotation(new Ammo.btQuaternion(object.quaternion.x, object.quaternion.y, object.quaternion.z, object.quaternion.w));
    let motionState = new Ammo.btDefaultMotionState(transform);
    let localInertia = new Ammo.btVector3(0, 0, 0);
	shape.calculateLocalInertia(mass, localInertia);
	let rbInfo = new Ammo.btRigidBodyConstructionInfo(mass, motionState, shape, localInertia);
	let rigidBody = new Ammo.btRigidBody(rbInfo);
	object.userData.physicsBody = rigidBody;
	if(mass > 0) {
		_rigidBodies.push(object)
		rigidBody.setActivationState(4);
	}
	_scene.add(object)
	_physicsWorld.addRigidBody(rigidBody)
}


function updateRigidBodies()
{
	let transform = new Ammo.btTransform()
	for(let i = 0; i < _rigidBodies.length; ++ i) {
		let object = _rigidBodies[i];
		let rigidBody = object.userData.physicsBody;
		let motionState = rigidBody.getMotionState();
		if(motionState) {
			motionState.getWorldTransform(transform);
			let p = transform.getOrigin();
			let q = transform.getRotation();
			if(object.tOffset !== undefined) {
				object.position.set(p.x() - object.tOffset.x, p.y() - object.tOffset.y, p.z() - object.tOffset.z)
			}
			else {
				object.position.set(p.x(), p.y(), p.z())
			}
			
			object.quaternion.set(q.x(), q.y(), q.z(), q.w())
		}
	}	
}

function update()
{
	let delta = _clock.getDelta();
	_physicsWorld.stepSimulation(delta, 10);
	updateRigidBodies();

	_renderer.render(_scene, _camera);
	requestAnimationFrame(update);
}

function resize()
{
	_camera.aspect = window.innerWidth / window.innerHeight;
    _camera.updateProjectionMatrix();
	_renderer.setSize(window.innerWidth, window.innerHeight);
}

