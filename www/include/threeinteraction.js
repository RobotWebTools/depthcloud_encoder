(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['three'], factory);
  }
  else {
    root.ThreeInteraction = factory(root.THREE);
  }
}(this, function (THREE) {

  var ThreeInteraction = {};
  
  var MouseHandler = ThreeInteraction.MouseHandler = function(renderer, camera, rootObj, fallbackTarget) {
  
    if (!renderer || !renderer.domElement || !camera || !rootObj) {
      return;
    }
  
    THREE.EventTarget.call(this);
  
    this.camera = camera;
    this.rootObj = rootObj;
    this.renderer = renderer;
    this.projector = new THREE.Projector();
    this.lastTarget = fallbackTarget;
    this.dragging = null;
    this.fallbackTarget = fallbackTarget;
  
    // listen to DOM events
    var eventNames = ["contextmenu", "click", "dblclick", "mouseout", "mousedown", "mouseup", "mousemove", "mousewheel"];
    this.listeners = {};
  
    eventNames.forEach(function(eventName) {
      this.listeners[eventName] = this.processDomEvent.bind(this);
      this.renderer.domElement.addEventListener(eventName, this.listeners[eventName], false);
    }, this);
  }
  
  MouseHandler.prototype.destroy = function() {
    this.listeners.forEach(function(listener) {
      this.renderer.domElement.removeEventListener(eventName, listener, false);
    }, this);
  }
  
  MouseHandler.prototype.processDomEvent = function(domEvent) {
  
    domEvent.preventDefault();
  
    var intersections = [];
  
    // compute normalized device coords and 3d mouse ray
    var target = domEvent.target;
    var deviceX = (domEvent.clientX - target.offsetLeft) / target.clientWidth * 2 - 1;
    var deviceY = -(domEvent.clientY - target.offsetTop) / target.clientHeight * 2 + 1;
    
    var vector = new THREE.Vector3(deviceX, deviceY, 0.5);
    this.projector.unprojectVector(vector, this.camera);
  
    var mouseRaycaster = new THREE.Raycaster(this.camera.position.clone(), vector.subSelf(this.camera.position).normalize());
    var mouseRay = mouseRaycaster.ray;
  
    // make our 3d mouse event
    var event3d = {
      mousePos : new THREE.Vector2(deviceX, deviceY),
      mouseRay : mouseRay,
      domEvent : domEvent,
      camera : this.camera,
      intersection : this.lastIntersection
    };
  
    // While the user is holding the mouse down,
    // stay on the same target
    if (this.dragging) {
      this.notify(this.lastTarget, domEvent.type, event3d);
      if (domEvent.type === "mouseup") {
        this.dragging = false;
      }
      return;
    }
  
    // if the mouse leaves the dom element, stop everything
    if (domEvent.type == "mouseout") {
      this.dragging = false;
      this.notify(this.lastTarget, "mouseout", event3d);
      this.lastTarget=null;
      return;
    }
  
    var target = this.lastTarget;
  
    // In the normal case, we need to check what is under the mouse
    intersections = mouseRaycaster.intersectObject(this.rootObj, true);
    if (intersections.length > 0) {
      target = intersections[0].object;
      event3d.intersection = this.lastIntersection = intersections[0];
    } else {
      target = this.fallbackTarget;
    }
  
    // if the mouse moves from one object to another
    // (or from/to the 'null' object), notify both
    if (target !== this.lastTarget) {
      
      var eventAccepted = this.notify(target, 'mouseover', event3d);
      
      if (eventAccepted) {
        this.notify(this.lastTarget, 'mouseout', event3d);
      } else {
        // if target was null or no target has caught our event, fall back
        target = this.fallbackTarget;
        if (target !== this.lastTarget) {
          this.notify(target, 'mouseover', event3d);
          this.notify(this.lastTarget, 'mouseout', event3d);
        }
      }
    }
  
    // pass through event
    this.notify(target, domEvent.type, event3d);
  
    if (domEvent.type === "mousedown") {
      this.dragging = true;
    }
  
    this.lastTarget = target;
  }
  
  MouseHandler.prototype.notify = function(target, type, event3d) {
    event3d.type = type;
  
    // make the event cancelable
    event3d.cancelBubble = false;
    event3d.stopPropagation = function() {
      event3d.cancelBubble = true;
    }
    // walk up graph until event is canceled
    // or root node has been reached
    event3d.currentTarget = target;
    while (event3d.currentTarget) {
      // try to fire event on object
      if (event3d.currentTarget.dispatchEvent && event3d.currentTarget.dispatchEvent instanceof Function) {
        event3d.currentTarget.dispatchEvent(event3d);
        if (event3d.cancelBubble) {
          this.dispatchEvent(event3d);
          return true;
        }
      }
  
      // walk up
      event3d.currentTarget = event3d.currentTarget.parent;
    }
    return false;
  }
  
  var Highlighter = ThreeInteraction.Highlighter = function(mouseHandler) {
    mouseHandler.addEventListener("mouseover", this.onMouseOver.bind(this));
    mouseHandler.addEventListener("mouseout", this.onMouseOut.bind(this));
    this.hoverObjs=[];
  }

  Highlighter.prototype.onMouseOver = function(event) {
    this.hoverObjs.push(event.currentTarget);
  }

  Highlighter.prototype.onMouseOut = function(event) {
    this.hoverObjs.splice(this.hoverObjs.indexOf(event.currentTarget), 1);
  }
  
  Highlighter.prototype.getWebglObjects = function(scene, objects, renderList) {
    var objlist = scene.__webglObjects;
    // get corresponding webgl objects
    for (var c = 0; c < objects.length; c++) {
      if (!objects[c]) {
        continue;
      }
      for (var o = objlist.length - 1; o >= 0; o--) {
        if (objlist[o].object === objects[c]) {
          renderList.push(objlist[o]);
          break;
        }
      }
      // recurse into children
      this.getWebglObjects(scene, objects[c].children, renderList);
    }
  };

  Highlighter.prototype.renderHighlight = function(renderer, scene, camera) {

    // get webgl objects
    var renderList = [];
    this.getWebglObjects(scene, this.hoverObjs, renderList);

    //define highlight material
    var overrideMaterial = new THREE.MeshBasicMaterial({
      fog : false,
      opacity : 0.5,
      depthTest : true,
      depthWrite : false,
      polygonOffset : true,
      polygonOffsetUnits : -1,
      side : THREE.DoubleSide
    });
    scene.overrideMaterial = overrideMaterial;

    // swap render lists, render, undo
    var oldWebglObjects = scene.__webglObjects;
    scene.__webglObjects = renderList;

    renderer.render(scene, camera);

    scene.__webglObjects = oldWebglObjects;
    scene.overrideMaterial = null;
  }
  
  return ThreeInteraction;
}));