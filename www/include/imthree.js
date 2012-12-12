(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['three', './markersthree'], factory);
  }
  else {
    root.ImThree = factory(root.THREE, root.MarkersThree);
  }
}(this, function (THREE, MarkersThree) {

  var ImThree = {};
  
  var InteractiveMarker = ImThree.InteractiveMarker = function(handle, camera, meshBaseUrl) {
    THREE.Object3D.call(this);
    THREE.EventTarget.call(this);
  
    var that = this;
  
    this.name = handle.name;
  
    this.dragging = false;
    this.onServerSetPose({
      pose : handle.pose
    });
  
    this.dragStart = {
      position : new THREE.Vector3(),
      orientation : new THREE.Quaternion(),
      positionWorld : new THREE.Vector3(),
      orientationWorld : new THREE.Quaternion(),
      event3d : {}
    };
  
    handle.controls.forEach(function(control) {
      that.add(new ImThree.InteractiveMarkerControl(that, control, camera, meshBaseUrl));
    });
    
    if ( handle.menuEntries.length > 0 ) {
      this.menu = new Menu( handle.menuEntries );
      
      // forward menu select events
      this.menu.addEventListener( "menu_select", function( event ) {
        that.dispatchEvent( event );
      });
    }
  };
  
  InteractiveMarker.prototype = Object.create(THREE.Object3D.prototype);

  var projector = new THREE.Projector();
  
  var findClosestPoint = function(target_ray, mouse_ray) {
    // Find the closest point on target_ray to any point on mouse_ray.
    // Math taken from http://paulbourke.net/geometry/lineline3d/
    // line P1->P2 is target_ray
    // line P3->P4 is mouse_ray

    var v13 = new THREE.Vector3;
    v13.sub(target_ray.origin, mouse_ray.origin);
    var v43 = mouse_ray.direction.clone();
    var v21 = target_ray.direction.clone();
    var d1343 = v13.dot(v43);
    var d4321 = v43.dot(v21);
    var d1321 = v13.dot(v21);
    var d4343 = v43.dot(v43);
    var d2121 = v21.dot(v21);

    var denom = d2121 * d4343 - d4321 * d4321;
    if (Math.abs(denom) <= 0.0001) {
      return undefined;
    }
    var numer = d1343 * d4321 - d1321 * d4343;

    var mua = numer / denom;
    return mua;
  };

  var closestAxisPoint = function(axisRay, camera, mousePos) {
    // project axis onto screen
    var o = axisRay.origin.clone();
    projector.projectVector(o, camera);

    var o2 = axisRay.direction.clone().addSelf(axisRay.origin);
    projector.projectVector(o2, camera);

    // d is the axis vector in screen space
    var d = o2.clone().subSelf(o);
    // d = o2-o;

    // t is the 2d ray param of perpendicular projection
    // of mousePos onto o
    var tmp = new THREE.Vector2;
    var t = tmp.sub(mousePos, o).dot(d) / d.dot(d);
    // t = (mousePos - o) * d / (d*d);

    // mp is the final 2d-projected mouse pos
    var mp = new THREE.Vector2;
    mp.add(o, d.clone().multiplyScalar(t));
    // mp = o + d*t;

    // go back to 3d by shooting a ray
    var vector = new THREE.Vector3(mp.x, mp.y, 0.5);
    projector.unprojectVector(vector, camera);
    var mpRay = new THREE.Ray(camera.position, vector.subSelf(camera.position).normalize());
    var mua = findClosestPoint(axisRay, mpRay, mua);

    return mua;
  };

  var intersectPlane = function(mouseRay, planeOrigin, planeNormal) {

    var vector = new THREE.Vector3();
    var intersectPoint = new THREE.Vector3();

    vector.sub(planeOrigin, mouseRay.origin);
    dot = mouseRay.direction.dot(planeNormal);

    // bail if ray and plane are parallel
    if (Math.abs(dot) < mouseRay.precision)
      return null;

    // calc distance to plane
    scalar = planeNormal.dot(vector) / dot;

    // if negative distance, then plane is behind ray
    //if (scalar < 0)
    //  return null;
      
    intersectPoint.add( mouseRay.origin, mouseRay.direction.clone().multiplyScalar( scalar ) );
    return intersectPoint;
  };
  
  InteractiveMarker.prototype.showMenu=function(event)
  {
    if ( this.menu ) {
      this.menu.show(event);
    }
  }
  
  InteractiveMarker.prototype.moveAxis = function(currentControlOri, origAxis, event3d) {
    if (this.dragging) {
      //console.log(currentControlOri);
      var axis = currentControlOri.multiplyVector3(origAxis.clone());
      // get move axis in world coords
      var originWorld = this.dragStart.event3d.intersection.point;
      var axisWorld = this.dragStart.orientationWorld.clone().multiplyVector3(axis.clone());
      
      var axisRay = new THREE.Ray(originWorld, axisWorld);
      
      // find closest point to mouse on axis
      var t = closestAxisPoint(axisRay, event3d.camera, event3d.mousePos);
  
      // offset from drag start position
      var p = new THREE.Vector3;
      p.add(this.dragStart.position, this.dragStart.orientation.multiplyVector3(axis.clone()).multiplyScalar(t));
      this.setPosition(p);
      
      event3d.stopPropagation();
    }
  };
  
  InteractiveMarker.prototype.movePlane = function(currentControlOri, origNormal, event3d) {
    if (this.dragging) {
      var normal = currentControlOri.multiplyVector3(origNormal.clone());
      // get plane params in world coords
      var originWorld = this.dragStart.event3d.intersection.point;
      var normalWorld = this.dragStart.orientationWorld.multiplyVector3(normal.clone());
  
      // intersect mouse ray with plane
      var intersection = intersectPlane(event3d.mouseRay, originWorld, normalWorld);
  
      // offset from drag start position
      var p = new THREE.Vector3;
      p.sub(intersection, originWorld);
      p.addSelf(this.dragStart.positionWorld);
      this.setPosition(p);
      event3d.stopPropagation();
    }
  };
  
  function printVec(v) {
    //console.log(Math.round(v.x*10)/10,Math.round(v.y*10)/10,Math.round(v.y*10)/10);
  }
  function printQuat(v) {
    //console.log(Math.round(v.x*10)/10,Math.round(v.y*10)/10,Math.round(v.y*10)/10,Math.round(v.w*10)/10);
  }

  InteractiveMarker.prototype.rotateAxis = function(control, origOrientation, event3d) {
    if (this.dragging) {
      control.updateMatrixWorld();
      
      //console.log("------------------_");
      
      var currentControlOri = control.currentControlOri;
      var orientation = currentControlOri.clone().multiplySelf(origOrientation.clone());
      
      printQuat(currentControlOri);
      printQuat(orientation);
            
      var normal = orientation.multiplyVector3(new THREE.Vector3(1, 0, 0));
  
      // get plane params in world coords
      var originWorld = this.dragStart.event3d.intersection.point;
      var normalWorld = this.dragStart.orientationWorld.multiplyVector3(normal);

      printVec(normal);
      printVec(normalWorld);
  
      // intersect mouse ray with plane
      var intersection = intersectPlane(event3d.mouseRay, originWorld, normalWorld);
      
      // offset local origin to lie on intersection plane
      var normalRay = new THREE.Ray( this.dragStart.positionWorld, normalWorld );
      var rotOrigin = intersectPlane(normalRay, originWorld, normalWorld);
  
      // rotates from world to plane coords
      var orientationWorld = this.dragStart.orientationWorld.clone().multiplySelf(orientation);
      var orientationWorldInv = orientationWorld.clone().inverse();
      
      // rotate original and current intersection into local coords
      intersection.subSelf( rotOrigin );
      orientationWorldInv.multiplyVector3(intersection);
  
      var origIntersection = this.dragStart.event3d.intersection.point.clone();
      origIntersection.subSelf( rotOrigin );
      orientationWorldInv.multiplyVector3(origIntersection);
      
      // compute relative 2d angle
      var a1 = Math.atan2(intersection.y,intersection.z);
      var a2 = Math.atan2(origIntersection.y,origIntersection.z);
      var a = a2 - a1;
      
      var rot = new THREE.Quaternion();
      rot.setFromAxisAngle( normal, a );
      
      // rotate
  //    this.setOrientation( rot.multiplySelf(this.dragStart.orientationWorld) );
      this.setOrientation( rot.multiplySelf(this.dragStart.orientationWorld) );
      
      // offset from drag start position
      event3d.stopPropagation();
    }
  };
  
  InteractiveMarker.prototype.startDrag = function(event3d) {
    if (event3d.domEvent.button !== 0) {
      return;
    }
    event3d.stopPropagation();
    this.dragging = true;
    this.updateMatrixWorld(true);
    var scale = new THREE.Vector3();
    this.matrixWorld.decompose(this.dragStart.positionWorld, this.dragStart.orientationWorld, scale);
    this.dragStart.position = this.position.clone();
    this.dragStart.orientation = this.quaternion.clone();
    this.dragStart.event3d = event3d;
  
    this.dispatchEvent({
      type : "user_mouse_down",
      position : this.position,
      orientation : this.quaternion
    });
  }
  
  InteractiveMarker.prototype.stopDrag = function(event3d) {
    if (event3d.domEvent.button !== 0) {
      return;
    }
    event3d.stopPropagation();
    this.dragging = false;
    this.dragStart.event3d = {};
    this.onServerSetPose(this.bufferedPoseEvent);
    this.bufferedPoseEvent = undefined;
  
    this.dispatchEvent({
      type : "user_mouse_up",
      position : this.position,
      orientation : this.quaternion
    });
  }
  
  InteractiveMarker.prototype.buttonClick = function(event3d) {
    event3d.stopPropagation();
  
    this.dispatchEvent({
      type : "user_button_click",
      position : this.position,
      orientation : this.quaternion
    });
  }
  
  InteractiveMarker.prototype.setPosition = function(position) {
    this.position = position;
    this.dispatchEvent({
      type : "user_changed_pose",
      position : this.position,
      orientation : this.quaternion
    });
  }
  
  InteractiveMarker.prototype.setOrientation = function(orientation) {
    orientation.normalize();
    this.quaternion = orientation;
    this.dispatchEvent({
      type : "user_changed_pose",
      position : this.position,
      orientation : this.quaternion
    });
  }
  
  InteractiveMarker.prototype.onServerSetPose = function(event) {
    if (event === undefined) {
      return;
    }
  
    if (this.dragging) {
      this.bufferedPoseEvent = event;
      return;
    }
  
    var pose = event.pose;
  
    this.position.x = pose.position.x;
    this.position.y = pose.position.y;
    this.position.z = pose.position.z;
  
    this.useQuaternion = true;
    this.quaternion = new THREE.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  
    this.updateMatrixWorld(true);
  }

  // --------------------------------------------------------
  
  var InteractiveMarkerControl = ImThree.InteractiveMarkerControl = function(parent, control, camera, meshBaseUrl) {
    THREE.Object3D.call(this);
    THREE.EventTarget.call(this);
  
    this.parent = parent;
    this.dragging = false;
  
    var that = this;
  
    var NONE = 0;
    var MENU = 1;
    var BUTTON = 2;
    var MOVE_AXIS = 3;
    var MOVE_PLANE = 4;
    var ROTATE_AXIS = 5;
    var MOVE_ROTATE = 6;
  
    var controlOri = new THREE.Quaternion(control.orientation.x, control.orientation.y, control.orientation.z, control.orientation.w);
    controlOri.normalize();
  
    // transform x axis into local frame
    var controlAxis = new THREE.Vector3(1, 0, 0);
    controlOri.multiplyVector3(controlAxis);
    
    this.currentControlOri = new THREE.Quaternion();
  
    // determine mouse interaction
    switch(control.interaction_mode) {
      case MOVE_AXIS:
        this.addEventListener("mousemove", parent.moveAxis.bind(parent, this.currentControlOri, controlAxis));
        break;
      case ROTATE_AXIS:
        this.addEventListener("mousemove", parent.rotateAxis.bind(parent, this, controlOri));
        break;
      case MOVE_PLANE:
        this.addEventListener("mousemove", parent.movePlane.bind(parent, this.currentControlOri, controlAxis));
        break;
      case BUTTON:
        this.addEventListener("click", parent.buttonClick.bind(parent));
        break;
      default:
        break;
    }
  
    // install default listeners for highlighting / dragging
    function stopPropagation(event) {
      event.stopPropagation();
    }
  
    if (control.interaction_mode != NONE) {
      this.addEventListener('mousedown', parent.startDrag.bind(parent));
      this.addEventListener('mouseup', parent.stopDrag.bind(parent));
      this.addEventListener('contextmenu', parent.showMenu.bind(parent));
      this.addEventListener('mouseover', stopPropagation);
      this.addEventListener('mouseout', stopPropagation);
      this.addEventListener('click', stopPropagation);
    }
  
    // define rotation behaviour
    var INHERIT = 0;
    var FIXED = 1;
    var VIEW_FACING = 2;
  
    var rotInv = new THREE.Quaternion();
    var posInv = parent.position.clone().multiplyScalar(-1);
  
    switch(control.orientation_mode) {
      case INHERIT:
        rotInv = parent.quaternion.clone().inverse();
        that.updateMatrixWorld = function(force) {
          ImThree.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
          that.currentControlOri.copy(that.quaternion);
          that.currentControlOri.normalize();
        }
        break;
      case FIXED:
        that.updateMatrixWorld = function(force) {
          that.useQuaternion = true;
          that.quaternion = that.parent.quaternion.clone().inverse();
          that.updateMatrix();
          that.matrixWorldNeedsUpdate = true;
          ImThree.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
          //that.currentControlOri.copy(that.quaternion);
        }
        break;
      case VIEW_FACING:
        var independent_marker_orientation = control.independent_marker_orientation;
        that.updateMatrixWorld = function(force) {

          camera.updateMatrixWorld();
          var cameraRot = new THREE.Matrix4().extractRotation( camera.matrixWorld );
          
          var ros2Gl = new THREE.Matrix4();
          var r90 = Math.PI*0.5;
          var rv = new THREE.Vector3( -r90,0,r90 );
          ros2Gl.setRotationFromEuler( rv );

          var worldToLocal = new THREE.Matrix4();
          worldToLocal.getInverse( that.parent.matrixWorld );
          
          cameraRot.multiply( cameraRot, ros2Gl );
          cameraRot.multiply( worldToLocal, cameraRot );
          
          that.currentControlOri.setFromRotationMatrix( cameraRot );
          
          if ( !independent_marker_orientation ) {
            that.useQuaternion = true;
            that.quaternion.copy(that.currentControlOri);
            that.updateMatrix();
            that.matrixWorldNeedsUpdate = true;
          }
          
          ImThree.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
        }
        break;
      default:
        break;
    }
  
    // create visuals (markers)
    control.markers.forEach(function(markerMsg) {
      var markerHelper = new MarkersThree.MarkerHelper(markerMsg, meshBaseUrl);
  
      if ( markerMsg.header.frame_id !== "" )
      {
        // if the marker lives in its own coordinate frame,
        // convert position into IM's local coordinate frame
        markerHelper.position.addSelf(posInv);
        rotInv.multiplyVector3(markerHelper.position);
        markerHelper.quaternion.multiply(rotInv, markerHelper.quaternion);
        markerHelper.updateMatrixWorld();
      }
  
      that.add(markerHelper);
    });
  
  }
  
  InteractiveMarkerControl.prototype = Object.create(THREE.Object3D.prototype);
  
  Menu = ImThree.Menu = function( menuEntries )
  {
    var allMenus = [];
    allMenus[0] = { children:[] };

    THREE.EventTarget.call(this);
    
    var that = this;
    
    this.menuDomElem = document.createElement("div");
    this.menuDomElem.style.position = "absolute";
    this.menuDomElem.className = "interactive_marker_menu";
    this.menuDomElem.addEventListener( "contextmenu", function(event) {
      event.preventDefault();
    });

    this.overlayDomElem = document.createElement("div");
    this.overlayDomElem.style.visibility = "hidden";
    this.overlayDomElem.className = "interactive_marker_overlay";

    this.hideListener = this.hide.bind(this);
    this.overlayDomElem.addEventListener("contextmenu", this.hideListener);
    this.overlayDomElem.addEventListener("click", this.hideListener);

    document.body.appendChild(this.overlayDomElem);
    document.body.appendChild(this.menuDomElem);
    
    // parse all entries
    for (var i=0; i<menuEntries.length; i++) {
      var entry = menuEntries[i];
      var id = entry.id;
      allMenus[id] = { 
        title: entry.title,
        id : id,
        children: []
      };
    }
    
    // link children to parents
    for (var i=0; i<menuEntries.length; i++) {
      var entry = menuEntries[i];
      var id = entry.id;
      var menu = allMenus[ id ];
      var parent = allMenus[ entry.parent_id ];
      parent.children.push( menu );
    }
    
    function emitMenuSelect( menuEntry, domEvent )
    {
      this.dispatchEvent({
        type: "menu_select",
        domEvent: domEvent,
        id: menuEntry.id
        });
      this.hide( domEvent );        
    }

    // create html menu, starting from root (id 0)    
    function makeUl( parentDomElem, parentMenu ) {
      
      var ulElem = document.createElement("ul");
      parentDomElem.appendChild(ulElem);
      
      var children = parentMenu.children;
      
      for (var i=0; i<children.length; i++) {
        var liElem = document.createElement("li");
        var divElem = document.createElement("div");
        divElem.appendChild(document.createTextNode( children[i].title ));
        ulElem.appendChild( liElem );
        liElem.appendChild( divElem );

        if ( children[i].children.length > 0 ) {
          makeUl( liElem, children[i] );
          divElem.addEventListener( "click", that.hide.bind( that ) );
        } else {
          divElem.addEventListener( "click", emitMenuSelect.bind( that, children[i] ) );
          divElem.className = "interactive_marker_menuentry";
        }
      }
      
    }
    
    // construct dom element
    makeUl( this.menuDomElem, allMenus[0] );
  }
  
  Menu.prototype.show = function(event) {
    this.overlayDomElem.style.visibility = "visible";
    this.menuDomElem.style.visibility = "visible";
    
    this.menuDomElem.style.left = event.domEvent.clientX + 'px';
    this.menuDomElem.style.top = event.domEvent.clientY  + 'px';
  }

  Menu.prototype.hide = function(event) {
    event.preventDefault();
    this.overlayDomElem.style.visibility = "hidden";
    this.menuDomElem.style.visibility = "hidden";
  }

  // --------------------------------------------------------
  
  var Viewer = ImThree.Viewer = function ( scene, camera, intMarkerClient, meshBaseUrl ) {
    this.scene = scene;
    this.camera = camera;
    this.root = new THREE.Object3D();
    this.meshBaseUrl = meshBaseUrl;
    scene.add(this.root);
    
    var that=this;

    intMarkerClient.on('created_marker', this.addMarker.bind(this));
    intMarkerClient.on('deleted_marker', this.eraseMarker.bind(this));
  };

  Viewer.prototype.addMarker = function(intMarkerHandle) {
    var intMarker = new InteractiveMarker(intMarkerHandle, this.camera, this.meshBaseUrl);
    this.root.add(intMarker);
    
    intMarkerHandle.on('server_updated_pose', function(pose) {
      intMarker.onServerSetPose({
        pose : pose
      });
    });
    
    intMarker.addEventListener('user_changed_pose', intMarkerHandle.setPoseFromClient.bind(intMarkerHandle));
    intMarker.addEventListener('user_mouse_down',intMarkerHandle.onMouseDown.bind(intMarkerHandle));
    intMarker.addEventListener('user_mouse_up', intMarkerHandle.onMouseUp.bind(intMarkerHandle));
    intMarker.addEventListener('user_button_click', intMarkerHandle.onButtonClick.bind(intMarkerHandle));
    intMarker.addEventListener('menu_select', intMarkerHandle.onMenuSelect.bind(intMarkerHandle));
  };

  Viewer.prototype.eraseMarker = function(name) {
    var marker = this.root.getChildByName(name);
    if ( marker == undefined ) {
      console.log("internal error: cannot delete " + name + " from scene");
    }
    this.root.remove(marker);
  };
  
  return ImThree;
}));
