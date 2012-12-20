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
    cameraControls = new THREE.RosOrbitControls(camera);

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
    var ros = new ROS('ws://localhost:9090');
    
    var meshBaseUrl = 'http://localhost:8000/resources/';

    // show interactive markers
    imClient = new ImProxy.Client(ros);
    imViewer = new ImThree.Viewer(selectableObjs, camera, imClient, meshBaseUrl);
    
    var depthNode = new THREE.Object3D;
    scene.add(depthNode);

    cloudStream = new DepthCloud.Viewer({
      url : 'http://localhost:8888/streams/depth_color_combined.webm?',
      sceneNode : depthNode,
      f : 505.0
    });
    
    cloudStream.startStream();
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
