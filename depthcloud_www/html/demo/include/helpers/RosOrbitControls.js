/**
 * @author dgossow / https://github.com/dgossow
 * @author qiao / https://github.com/qiao
 * @author mrdoob / http://mrdoob.com
 * @author alteredq / http://alteredqualia.com/
 */

// behaves like THREE.OrbitControls, but uses
// right-handed coords and z as up vector
(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['three'], factory);
  }
  else {
    factory(root.THREE);
  }
}(this, function (THREE) {

  THREE.RosOrbitControls = function(scene, object) {

    THREE.EventTarget.call(this);

    this.object = object;

    // In ROS, z is pointing upwards
    this.object.up = new THREE.Vector3(0, 0, 1);

    // API

    this.center = new THREE.Vector3();

    this.userZoom = true;
    this.userZoomSpeed = 1.0;

    this.userRotate = true;
    this.userRotateSpeed = 1.0;

    this.autoRotate = false;
    this.autoRotateSpeed = 2.0;
    // 30 seconds per round when fps is 60

    this.axes = new THREE.Axes( {
      shaftRadius: 0.025,
      headRadius: 0.07,
      headLength: 0.2
      });
    scene.add(this.axes);
    this.axes.traverse( function(obj){obj.visible = false;});

    // internals

    var scope = this;

    var EPS = 0.000001;
    var PIXELS_PER_ROUND = 1800;

    var rotateStart = new THREE.Vector2();
    var rotateEnd = new THREE.Vector2();
    var rotateDelta = new THREE.Vector2();

    var zoomStart = new THREE.Vector2();
    var zoomEnd = new THREE.Vector2();
    var zoomDelta = new THREE.Vector2();

    var moveStartCenter = new THREE.Vector3();
    var moveStartNormal = new THREE.Vector3();
    var moveStartPosition = new THREE.Vector3();
    var moveStartIntersection = new THREE.Vector3();

    var touchStartDist = 0;
    var touchStartScale = 1;

    var phiDelta = 0;
    var thetaDelta = 0;
    var scale = 1;

    var lastPosition = new THREE.Vector3();

    var STATE = {
      NONE : -1,
      ROTATE : 0,
      ZOOM : 1,
      MOVE : 2
    };
    var state = STATE.NONE;

    this.showAxes = function() {
      scope.axes.traverse( function(obj){obj.visible = true;});
      if ( this.hideTimeout ) {
        clearTimeout( this.hideTimeout );
      }
      scope.hideTimeout = setTimeout( function(){
        scope.axes.traverse( function(obj){obj.visible = false;});
        scope.hideTimeout = false;
      }, 1000);
    }

    // events

    var changeEvent = {
      type : 'change'
    };

    this.rotateLeft = function(angle) {

      if (angle === undefined) {

        angle = getAutoRotationAngle();

      }

      thetaDelta -= angle;

    };

    this.rotateRight = function(angle) {

      if (angle === undefined) {

        angle = getAutoRotationAngle();

      }

      thetaDelta += angle;

    };

    this.rotateUp = function(angle) {

      if (angle === undefined) {

        angle = getAutoRotationAngle();

      }

      phiDelta -= angle;

    };

    this.rotateDown = function(angle) {

      if (angle === undefined) {

        angle = getAutoRotationAngle();

      }

      phiDelta += angle;

    };

    this.zoomIn = function(zoomScale) {

      if (zoomScale === undefined) {

        zoomScale = getZoomScale();

      }

      scale /= zoomScale;

    };

    this.zoomOut = function(zoomScale) {

      if (zoomScale === undefined) {

        zoomScale = getZoomScale();

      }

      scale *= zoomScale;

    };

    this.update = function() {

      var position = this.object.position;
      var offset = position.clone().subSelf(this.center)

      // x->y, y->z, z->x

      // angle from z-axis around y-axis
      var theta = Math.atan2(offset.y, offset.x);

      // angle from y-axis
      var phi = Math.atan2(Math.sqrt(offset.y * offset.y + offset.x * offset.x), offset.z);

      if (this.autoRotate) {

        this.rotateLeft(getAutoRotationAngle());

      }

      theta += thetaDelta;
      phi += phiDelta;

      // restrict phi to be betwee EPS and PI-EPS

      phi = Math.max(EPS, Math.min(Math.PI - EPS, phi));

      var radius = offset.length();
      offset.y = radius * Math.sin(phi) * Math.sin(theta);
      offset.z = radius * Math.cos(phi);
      offset.x = radius * Math.sin(phi) * Math.cos(theta);
      offset.multiplyScalar(scale);

      position.copy(this.center).addSelf(offset);

      this.object.lookAt(this.center);

      radius = offset.length();
      this.axes.position = this.center.clone();
      this.axes.scale.x = this.axes.scale.y = this.axes.scale.z = radius * 0.05;
      this.axes.updateMatrixWorld(true);

      thetaDelta = 0;
      phiDelta = 0;
      scale = 1;

      if (lastPosition.distanceTo(this.object.position) > 0) {

        this.dispatchEvent(changeEvent);

        lastPosition.copy(this.object.position);

      }
    };

    function getAutoRotationAngle() {

      return 2 * Math.PI / 60 / 60 * scope.autoRotateSpeed;

    }

    function getZoomScale() {

      return Math.pow(0.95, scope.userZoomSpeed);

    }

    function intersectViewPlane(mouseRay, planeOrigin, planeNormal) {

      var vector = new THREE.Vector3();
      var intersection = new THREE.Vector3();

      vector.sub(planeOrigin, mouseRay.origin);
      dot = mouseRay.direction.dot(planeNormal);

      // bail if ray and plane are parallel
      if (Math.abs(dot) < mouseRay.precision)
        return null;

      // calc distance to plane
      scalar = planeNormal.dot(vector) / dot;

      intersection = mouseRay.direction.clone().multiplyScalar( scalar );
      return intersection;
    };


    function startRotate(event) {
      state = STATE.ROTATE;
      rotateStart.set(event.clientX, event.clientY);
    }

    function getTouchDist(event) {
      if ( event.touches.length < 2 ) return 0;
      var dx = event.touches[0].clientX - event.touches[1].clientX;
      var dy = event.touches[0].clientY - event.touches[1].clientY;
      return Math.sqrt( dx*dx + dy*dy );
    }

    function getTouchCenter(event) {
      var x,y;
      if ( event.touches.length < 2 ) {
        x = event.touches[0].clientX;
        y = event.touches[0].clientY;
      } else {
        x = (event.touches[0].clientX + event.touches[1].clientX)/2.0;
        y = (event.touches[0].clientY + event.touches[1].clientY)/2.0;
      }
      return { x:x,y:y };
    }

    function onTouchStart(event3d) {

      var event = event3d.domEvent;
      event.preventDefault();

      if ( event.touches.length == 1 ) {
        event.clientX = event.touches[0].clientX;
        event.clientY = event.touches[0].clientY;
        event.button = 0;
        startRotate(event);
      } else if ( event.touches.length == 2 ) {
        if ( state != STATE.NONE ) {
          onTouchEnd( event3d );
        }
        touchStartDist = getTouchDist(event);
        touchStartScale = scale;
      }

      scope.showAxes();
    }

    function onTouchMove(event3d) {

      var event = event3d.domEvent;
      event.preventDefault();

      if ( event.touches.length == 1 )
      {
        event.clientX = event.touches[0].clientX;
        event.clientY = event.touches[0].clientY;
        event.button = 0;
        onMouseMove(event3d);
      } else if ( event.touches.length == 2 )
      {
        var touchDist = getTouchDist(event);
        scale *= touchStartScale * touchStartDist / touchDist;
        touchStartDist = touchDist;
        console.log( "touchDist:: ", touchDist );
      }

      scope.showAxes();
    }

    function onTouchEnd(event3d) {

      var event = event3d.domEvent;
      event.preventDefault();
      state = STATE.NONE;
    }

    function onMouseDown(event3d) {

      var event = event3d.domEvent;

      event.preventDefault();

      switch( event.button ) {
        case 0:
          startRotate(event);
          break;
        case 1:
          state = STATE.MOVE;

          moveStartNormal = new THREE.Vector3(0,0,1);
          var rMat = new THREE.Matrix4().extractRotation( object.matrix );
          rMat.multiplyVector3( moveStartNormal );

          moveStartCenter=scope.center.clone();
          moveStartPosition=scope.object.position.clone();
          moveStartIntersection=intersectViewPlane(event3d.mouseRay, moveStartCenter, moveStartNormal);
          break;
        case 2:
          state = STATE.ZOOM;
          zoomStart.set(event.clientX, event.clientY);
          break;
      }

      this.showAxes();
    }

    function onMouseMove(event3d) {

      var event = event3d.domEvent;

      if (state === STATE.ROTATE) {

        rotateEnd.set(event.clientX, event.clientY);
        rotateDelta.sub(rotateEnd, rotateStart);

        scope.rotateLeft(2 * Math.PI * rotateDelta.x / PIXELS_PER_ROUND * scope.userRotateSpeed);
        scope.rotateUp(2 * Math.PI * rotateDelta.y / PIXELS_PER_ROUND * scope.userRotateSpeed);

        rotateStart.copy(rotateEnd);
        scope.showAxes();

      } else if (state === STATE.ZOOM) {

        zoomEnd.set(event.clientX, event.clientY);
        zoomDelta.sub(zoomEnd, zoomStart);

        if (zoomDelta.y > 0) {

          scope.zoomIn();

        } else {

          scope.zoomOut();

        }

        zoomStart.copy(zoomEnd);
        scope.showAxes();

      } else if (state === STATE.MOVE) {

        var intersection = intersectViewPlane( event3d.mouseRay, scope.center, moveStartNormal );

        if ( !intersection ) return;

        var delta = new THREE.Vector3().sub( moveStartIntersection.clone(), intersection.clone() );

        scope.center.add( moveStartCenter.clone(), delta.clone() );
        scope.object.position.add( moveStartPosition.clone(), delta.clone() );
        scope.update();
        scope.object.updateMatrixWorld();
        scope.showAxes();
      }

    }

    function onMouseUp(event3d) {

      if (!scope.userRotate)
        return;

      state = STATE.NONE;

    }

    function onMouseWheel(event3d) {

      if (!scope.userZoom)
        return;

      var event = event3d.domEvent;

      if (event.wheelDelta > 0) {

        scope.zoomOut();

      } else {

        scope.zoomIn();

      }

      this.showAxes();
    }


    THREE.EventTarget.call(this);

    this.addEventListener('touchstart', onTouchStart);
    this.addEventListener('touchend', onTouchEnd);
    this.addEventListener('touchmove', onTouchMove);

    this.addEventListener('mousedown', onMouseDown);
    this.addEventListener('mouseup', onMouseUp);
    this.addEventListener('mousemove', onMouseMove);
    this.addEventListener('mousewheel', onMouseWheel);

    //this.addEventListener('touchMove', onMouseMove);
  };

}));