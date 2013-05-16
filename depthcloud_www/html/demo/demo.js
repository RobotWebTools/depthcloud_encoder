InteractiveMarkerDisplay=new (function(THREE) {

  var camera, cameraControls, scene, renderer;

  var selectableObjs;

  var directionalLight;

  var mouseHandler, highlighter;

  var imClient, imViewer;

  init();
  animate();

  function init() {

    // setup camera
    camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.01, 1000);
    camera.position.x = 3;
    camera.position.y = 3;
    camera.position.z = 3;

    // setup scene
    scene = new THREE.Scene();

    // setup camera mouse control
    cameraControls = new THREE.RosOrbitControls(scene,camera);

    // add node to host selectable objects
    selectableObjs = new THREE.Object3D;
    scene.add(selectableObjs);

    // add lights
    scene.add(new THREE.AmbientLight(0x555555));
    directionalLight = new THREE.DirectionalLight(0xffffff);
    scene.add(directionalLight);

    // add x/y grid
    var numCells = 50;
    var gridGeom = new THREE.PlaneGeometry(numCells, numCells, numCells, numCells);
    var gridMaterial = new THREE.MeshBasicMaterial({
      color : 0x999999,
      wireframe : true,
      wireframeLinewidth : 1,
      transparent : true
    });
    var gridObj = new THREE.Mesh(gridGeom, gridMaterial);
    scene.add(gridObj);

    // add coordinate frame visualization
    axes = new THREE.Axes();
    scene.add(axes);

    renderer = new THREE.WebGLRenderer({
      antialias : true
    });
    renderer.setClearColorHex(0x333333, 1.0);
    renderer.sortObjects = false;
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.shadowMapEnabled = false;
    renderer.autoClear = false;

    var container = document.getElementById("container");
    container.appendChild(renderer.domElement);

    // propagates mouse events to three.js objects
    mouseHandler = new ThreeInteraction.MouseHandler(renderer, camera, selectableObjs, cameraControls);

    // highlights the receiver of mouse events
    highlighter = new ThreeInteraction.Highlighter(mouseHandler);

    // connect to rosbridge
    var rosUrl = "ws://"+window.location.hostname+':9099';
    console.log("Connecting to ROS at ", rosUrl );
    var ros = new ROS(rosUrl);

    var meshBaseUrl = "http://"+window.location.hostname+':8000/resources/';
    console.log("Getting meshes from ", meshBaseUrl );

    var depthNode = new THREE.Object3D;
    scene.add(depthNode);

    // inital position
    depthNode.position.x = 0.0;
    depthNode.position.y = 0.0;
    depthNode.position.z = 0.0;
    depthNode.useQuaternion = true;
    depthNode.quaternion.x = Math.sqrt(1/2);
    depthNode.quaternion.y = 0.0;
    depthNode.quaternion.z = 0.0;
    depthNode.quaternion.w = -1*Math.sqrt(1/2);
    
    var video_stream_url = "http://"+window.location.hostname+':9999/streams/depthcloud_encoded.webm';
    	
    cloudStream = new DepthCloud.Viewer({
      url : video_stream_url,
      sceneNode : depthNode,
      f : 525.0,
      shaderUrl: '../depthcloudjs/shaders/'
    });

    cloudStream.startStream();

    var tfClient = new TfClient( {
      ros: ros,
      fixedFrame: 'base_link',
      angularThres: 2.0,
      transThres: 0.01
    } );

    tfClient.subscribe('head_mount_kinect_rgb_optical_frame',function(transform){
      //console.log(transform);
      depthNode.position.x = transform.translation.x;
      depthNode.position.y = transform.translation.y;
      depthNode.position.z = transform.translation.z;
      depthNode.useQuaternion = true;
      depthNode.quaternion.x = transform.rotation.x;
      depthNode.quaternion.y = transform.rotation.y;
      depthNode.quaternion.z = transform.rotation.z;
      depthNode.quaternion.w = transform.rotation.w;

      axes.position.x = transform.translation.x;
      axes.position.y = transform.translation.y;
      axes.position.z = transform.translation.z;
      axes.useQuaternion = true;
      axes.quaternion.x = transform.rotation.x;
      axes.quaternion.y = transform.rotation.y;
      axes.quaternion.z = transform.rotation.z;
      axes.quaternion.w = transform.rotation.w;
    });

    // show interactive markers
    imClient = new ImProxy.Client(ros,tfClient);
    imViewer = new ImThree.Viewer(selectableObjs, camera, imClient, meshBaseUrl);
  }

  this.subscribe = function( topic ) {
    imClient.subscribe(topic);
  }

  this.unsubscribe = function( topic ) {
    imClient.unsubscribe();
  }

  function animate() {

    cameraControls.update();

    // put light to the top-left of the camera
    directionalLight.position = camera.localToWorld(new THREE.Vector3(-1, 1, 0));
    directionalLight.position.normalize();

    renderer.clear(true, true, true);
    renderer.render(scene, camera);

    highlighter.renderHighlight(renderer, scene, camera);

    requestAnimationFrame(animate);
  }

})(THREE);

