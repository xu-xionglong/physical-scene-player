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
            //_cameraControl = new THREE.OrbitControls(_camera, _renderer.domElement);
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
    let rigidbodyMap = new Map();
    jsonData.rigid_bodys.forEach(function(rigidBodyData) {
        let object = scene.getObjectByName(rigidBodyData.name.replace(/\./g, ""));
        if(object !== undefined && object instanceof THREE.Mesh) {
            //let worldScale = new THREE.Vector3();
            //object.getWorldScale(worldScale);
            let geometry = object.geometry;
            let shape = null;
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
            }
            if(shape !== null) {
                let rigidBody = createRigidBody(object, shape, rigidBodyData.mass);
                if(rigidBodyData.kinematic) {
                    rigidBody.setCollisionFlags(rigidBody.getCollisionFlags() | 2);
                }
                rigidbodyMap.set(rigidBodyData.name, rigidBody);
            }

        }

    });

    jsonData.constraints.forEach(function(constraintData) {
        let objectA = constraintData.object1 !== undefined ? rigidbodyMap.get(constraintData.object1) : null;
        let objectB = constraintData.object2 !== undefined ? rigidbodyMap.get(constraintData.object2) : null;
        if(objectA === null && objectB !== null) {
            objectA = objectB;
            objectB = null;
        }
        if(objectA !== null) {
            let constraint = null;
            let translationOffsetA = constraintData.translation_offset_a;
            let pivotA = new Ammo.btVector3(translationOffsetA[0], translationOffsetA[1], translationOffsetA[2]);
            let rotationOffsetA = new THREE.Quaternion();
            rotationOffsetA.fromArray(constraintData.rotation_offset_a);


            let pivotB;
            let rotationOffsetB;
            if(objectB !== null) {
                let translationOffsetB = constraintData.translation_offset_b;
                pivotB = new Ammo.btVector3(translationOffsetB[0], translationOffsetB[1], translationOffsetB[2]);
                rotationOffsetB = new THREE.Quaternion();
                rotationOffsetB.fromArray(constraintData.rotation_offset_b);
            }
            switch(constraintData.type) {
                case "FIXED":
                {
                    
                    break;
                }
                case "POINT":
                {
                    if(objectB === null) {
                        constraint = new Ammo.btPoint2PointConstraint(objectA, pivotA);
                    }
                    else {
                        constraint = new Ammo.btPoint2PointConstraint(objectA, objectB, pivotA, pivotB);
                    }
                    break;
                }
                case "HINGE":
                {
                    let axisA = new THREE.Vector3(0, 0, 1);
                    axisA.applyQuaternion(rotationOffsetA);

                    if(objectB === null) {
                        constraint = new Ammo.btHingeConstraint(objectA, pivotA, new Ammo.btVector3(axisA.x, axisA.y, axisA.z));
                    }
                    else {
                        let axisB = new THREE.Vector3(0, 0, 1);
                        axisB.applyQuaternion(rotationOffsetB);
                        constraint = new Ammo.btHingeConstraint(objectA, objectB, pivotA, pivotB, new Ammo.btVector3(axisA.x, axisA.y, axisA.z), new Ammo.btVector3(axisB.x, axisB.y, axisB.z));
                    }
                    break;
                }
                case "SLIDER":
                {
                    break;
                }
                case "PISTON":
                {
                    break;
                }
                case "GENERIC":
                {
                    break;
                }
                case "GENERIC_SPRING":
                {
                    break;
                }
                default:
            }
            if(constraint !== null) {
                _physicsWorld.addConstraint(constraint, constraintData.disable_collisions);
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

