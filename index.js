let _renderer;
let _scene;
let _camera;
let _cameraControl;
let _physicsWorld;
let _clock;
let _rigidBodies = [];
let _debugDrawer;

Ammo().then(init);

function initPhysics()
{
    let collisionConfiguration = new Ammo.btSoftBodyRigidBodyCollisionConfiguration();
    let dispatcher = new Ammo.btCollisionDispatcher(collisionConfiguration);
    let broadphase = new Ammo.btDbvtBroadphase();
    let solver = new Ammo.btSequentialImpulseConstraintSolver();
    let softBodySolver = new Ammo.btDefaultSoftBodySolver();
    _physicsWorld = new Ammo.btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration, softBodySolver);
    _physicsWorld.setGravity(new Ammo.btVector3(0, -9.8, 0));
    _physicsWorld.getWorldInfo().set_m_gravity(new Ammo.btVector3(0, -9.8, 0));
}

function initGraphics()
{
    _renderer = new THREE.WebGLRenderer({antialias:true});
    _renderer.setPixelRatio(window.devicePixelRatio);
    _renderer.setSize(window.innerWidth, window.innerHeight);
    document.body.appendChild(_renderer.domElement);
}

function loadDefaultScene()
{
    let physicsData;

    let gltfLoader = new THREE.GLTFLoader();
    gltfLoader.load(
        "defaultScene.glb",
        function(gltf) {
            _scene = gltf.scene;
            _scene.background = new THREE.Color(0xf0f0f0);
            _camera = gltf.cameras[0];
            _debugDrawer = new THREE.AmmoDebugDrawer(_scene, _physicsWorld);
            _debugDrawer.enable();
            resize();
            update();
            if(physicsData !== undefined) {
                createPhysicsScene(_scene, physicsData);
            }
        },
        function(xhr) {

        },
        function(error) {

        }
    );

    let jsonLoader = new THREE.FileLoader();
    jsonLoader.load(
        "defaultScene.json",
        function(data) {
            physicsData = JSON.parse(data);
            if(_scene !== undefined) {
                createPhysicsScene(_scene, physicsData);
            }
        },
        function(xhr) {
            
        },
        function(err) {
        }
    );
}

function createPhysicsScene(scene, jsonData)
{
    jsonData.rigid_bodys.forEach(function(rigidBodyData) {
        let object = scene.getObjectByName(rigidBodyData.name.replace(/\./g, ""));
        if(object !== undefined && object instanceof THREE.Mesh) {
            //let worldScale = new THREE.Vector3();
            //object.getWorldScale(worldScale);
            let geometry = object.geometry;
            let shape;
            switch(rigidBodyData.collision_shape) {
                case "BOX":
                {
                    geometry.computeBoundingBox();
                    let bound = geometry.boundingBox;
                    let halfExtents = bound.max.clone();
                    halfExtents.sub(bound.min);
                    //halfExtents.multiply(worldScale);
                    halfExtents.multiplyScalar(0.5);
                    shape = new Ammo.btBoxShape(new Ammo.btVector3(halfExtents.x, halfExtents.y, halfExtents.z));
                    break; 
                } 
                case "SPHERE":
                {
                    geometry.computeBoundingSphere();
                    let bound = geometry.boundingSphere;
                    shape = new Ammo.btSphereShape(bound.radius);
                    break;
                }
                case "CAPSULE":
                {
                    break;
                }
                case "":
                {
                    break;
                }
                default:
                {
                    break;
                }
            }
            if(shape !== undefined) {
                let rigidBody = createRigidBody(object, shape, rigidBodyData.mass);
                if(rigidBodyData.kinematic) {

                }
            }

        }

    });
}

function init(Ammo)
{
    initPhysics();
    initGraphics();
    _clock = new THREE.Clock();
    loadDefaultScene();
    window.addEventListener('resize', resize, false);
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
    return rigidBody;
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
    if(_scene !== undefined && _camera !== undefined) {
        let delta = _clock.getDelta();
        _physicsWorld.stepSimulation(delta, 10);
        updateRigidBodies();
        _debugDrawer.update();
        _renderer.render(_scene, _camera);
        requestAnimationFrame(update);
    }
}

function resize()
{
    if(_camera !== undefined) {
        _camera.aspect = window.innerWidth / window.innerHeight;
        _camera.updateProjectionMatrix();
    }
    _renderer.setSize(window.innerWidth, window.innerHeight);
}

