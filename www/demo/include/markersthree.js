(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['three', 'colladaloader'], factory);
  }
  else {
    root.MarkersThree = factory(root.THREE, root.ColladaLoader);
  }
}(this, function (THREE, ColladaLoader) {

  var MarkersThree = {};
  
  var meshWarningPrinted = false;
  
  var MarkerHelper = MarkersThree.MarkerHelper = function(markerMsg, meshBaseUrl) {
    
    THREE.Object3D.call(this);
    
    if (meshBaseUrl !== undefined) {
      this.meshBaseUrl = meshBaseUrl;
      if(this.meshBaseUrl.substr(this.meshBaseUrl.length-1)!= "/") {
       this.meshBaseUrl = this.meshBaseUrl + "/";
      }
    } else if ( !meshWarningPrinted ) {
        console.log( "Warning: no mesh base URL given. Will not be able to display mesh markers." );
    }
  
    var that = this;
    var geom = null;
  
    function makeColorMaterial(r, g, b, a) {
      var color = new THREE.Color();
      color.setRGB(r, g, b);
      if (a <= 0.99) {
        return new THREE.MeshBasicMaterial({
          color : color.getHex(),
          opacity : a+0.1,
          transparent : true,
          depthWrite : true,
          blendSrc : THREE.SrcAlphaFactor,
          blendDst : THREE.OneMinusSrcAlphaFactor,
          blendEquation : THREE.ReverseSubtractEquation,
          blending : THREE.NormalBlending,
        });
      } else {
        return new THREE.MeshLambertMaterial({
          color : color.getHex(),
          opacity : a,
          blending : THREE.NormalBlending,
        });
      }
    }
  
    function addMesh(geom, mat) {
      var mesh = new THREE.Mesh(geom, mat);
      that.add(mesh);
    }
  
    function pointMsgToVector3(msg) {
      return new THREE.Vector3(msg.x, msg.y, msg.z);
    }
  
    var ARROW = 0;
    var CUBE = 1;
    var SPHERE = 2
    var CYLINDER = 3;
    var LINE_STRIP = 4;
    var LINE_LIST = 5;
    var CUBE_LIST = 6;
    var SPHERE_LIST = 7;
    var POINTS = 8;
    var TEXT_VIEW_FACING = 9;
    var MESH_RESOURCE = 10;
    var TRIANGLE_LIST = 11;
  
    this.setPose(markerMsg.pose);
  
    var colorMaterial = makeColorMaterial(markerMsg.color.r, markerMsg.color.g, markerMsg.color.b, markerMsg.color.a);
  
    switch( markerMsg.type ) {
      case ARROW:
        var len = markerMsg.scale.x;
        var headLen = len * 0.23;
        var headR = markerMsg.scale.y;
        var shaftR = headR * 0.5;
  
        if (markerMsg.points.length == 2) {
          var p1 = pointMsgToVector3(markerMsg.points[0]);
          var p2 = pointMsgToVector3(markerMsg.points[1]);
          var dir = p1.clone().negate().addSelf(p2);
          // dir = p2 - p1;
          len = dir.length();
          headR = markerMsg.scale.y;
          shaftR = markerMsg.scale.x;
  
          if (markerMsg.scale.z != 0.0) {
            headLen = markerMsg.scale.z;
          }
        }
  
        this.add(new THREE.ArrowMarkerHelper({
          dir : dir,
          origin : p1,
          length : len,
          headLength : headLen,
          shaftDiameter : shaftR,
          headDiameter : headR,
          material : colorMaterial
        }));
        break;
  
      case CUBE:
        var geom = new THREE.CubeGeometry(markerMsg.scale.x, markerMsg.scale.y, markerMsg.scale.z);
        addMesh(geom, colorMaterial);
        break;
  
      case SPHERE:
        var geom = new THREE.SphereGeometry(0.5);
        var mesh = new THREE.Mesh(geom, colorMaterial);
        mesh.scale.x = markerMsg.scale.x;
        mesh.scale.y = markerMsg.scale.y;
        mesh.scale.z = markerMsg.scale.z;
        that.add(mesh);
        break;
  
      case CYLINDER:
        var geom = new THREE.CylinderGeometry( 0.5, 0.5, 1, 16, 1, false );
        var mesh = new THREE.Mesh(geom, colorMaterial);
        mesh.useQuaternion = true;
        mesh.quaternion.setFromAxisAngle( new THREE.Vector3(1,0,0), Math.PI*0.5 );
        mesh.scale = pointMsgToVector3( markerMsg.scale );
        this.add(mesh);
        break;
  
      case LINE_STRIP:
        addMesh(new THREE.CubeGeometry(0.1, 0.1, 0.1), colorMaterial);
        break;
  
      case LINE_LIST:
        addMesh(new THREE.CubeGeometry(0.1, 0.1, 0.1), colorMaterial);
        break;
  
      case CUBE_LIST:
      case SPHERE_LIST:
      case POINTS:
        var geometry = new THREE.Geometry();
        var material = new THREE.ParticleBasicMaterial( { size: markerMsg.scale.x } );

        for ( var i=0; i<markerMsg.points.length; i++ ) {
          var vertex = new THREE.Vector3();
          vertex.x = markerMsg.points[i].x;
          vertex.y = markerMsg.points[i].y;
          vertex.z = markerMsg.points[i].z;
          geometry.vertices.push( vertex );
        }
        
        if ( markerMsg.colors.length == markerMsg.points.length ) {
          material.vertexColors = true;
          for ( var i=0; i<markerMsg.points.length; i++ ) {
            var color = new THREE.Color();
            color.setRGB( markerMsg.colors[i].r, markerMsg.colors[i].g, markerMsg.colors[i].b );
            geometry.colors.push( color );
          }
        } else {
          material.color.setRGB( markerMsg.color.r, markerMsg.color.g, markerMsg.color.b );
        }

        var particles = new THREE.ParticleSystem( geometry, material );
        this.add( particles );
        break;
  
      case TEXT_VIEW_FACING:
        addMesh(new THREE.CubeGeometry(0.1, 0.1, 0.1), colorMaterial);
        break;
  
      case MESH_RESOURCE:
        var meshMarker = new THREE.MeshMarkerHelper( markerMsg, this.meshBaseUrl );
        this.add(meshMarker);
        break;
  
      case TRIANGLE_LIST:
        var tri = new MarkersThree.TriangleListMarkerHelper(colorMaterial, markerMsg.points, markerMsg.colors);
        tri.scale = pointMsgToVector3( markerMsg.scale );
        this.add(tri);
        break;
  
      default:
        addMesh(new THREE.CubeGeometry(0.1, 0.1, 0.1), colorMaterial);
        break;
    }
  
  };
  
  MarkerHelper.prototype = Object.create(THREE.Object3D.prototype);
  
  MarkerHelper.prototype.setPose = function(pose) {
    this.position.x = pose.position.x;
    this.position.y = pose.position.y;
    this.position.z = pose.position.z;
  
    this.useQuaternion = true;
    this.quaternion = new THREE.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    this.quaternion.normalize();
  
    this.updateMatrixWorld();
  }

  /* Triangle List Marker */

  var TriangleListMarkerHelper = MarkersThree.TriangleListMarkerHelper = function(material, vertices, colors) {
    THREE.Object3D.call(this);
  
    if (material === undefined)
      material = new THREE.MeshBasicMaterial();
  
    material.side = THREE.DoubleSide;
  
    var geometry = new THREE.Geometry();
  
    for ( i = 0; i < vertices.length; i++) {
      geometry.vertices.push(new THREE.Vector3(vertices[i].x, vertices[i].y, vertices[i].z));
    }
  
    if (colors.length === vertices.length) {
      // use per-vertex color
      for ( i = 0; i < vertices.length; i += 3) {
        var face = new THREE.Face3(i, i + 1, i + 2);
        for ( j = i * 3; j < i * 3 + 3; i++) {
          var color = new THREE.Color();
          color.setRGB(colors[i].r, colors[i].g, colors[i].b);
          face.vertexColors.push(color);
        }
        geometry.faces.push(face);
      }
      material.vertexColors = THREE.VertexColors;
    } else if (colors.length === vertices.length / 3) {
      // use per-triangle color
      for ( i = 0; i < vertices.length; i += 3) {
        var face = new THREE.Face3(i, i + 1, i + 2);
        face.color.setRGB(colors[i/3].r, colors[i/3].g, colors[i/3].b);
        geometry.faces.push(face);
      }
      material.vertexColors = THREE.FaceColors;
    } else {
      // use marker color
      for ( i = 0; i < vertices.length; i += 3) {
        var face = new THREE.Face3(i, i + 1, i + 2);
        geometry.faces.push(face);
      }
    }
  
    geometry.computeBoundingBox();
    geometry.computeBoundingSphere();
    geometry.computeCentroids();
    geometry.computeFaceNormals();
  
    this.mesh = new THREE.Mesh(geometry, material);
    this.add(this.mesh);
  };
  
  MarkersThree.TriangleListMarkerHelper.prototype = Object.create(THREE.Object3D.prototype);
  
  MarkersThree.TriangleListMarkerHelper.prototype.setColor = function(hex) {
    this.mesh.material.color.setHex(hex);
  };
  
  /* Mesh Marker */

  THREE.MeshMarkerHelper = function(markerMsg, meshBaseUrl) {
    
    if ( meshBaseUrl == undefined )
    {
      THREE.Mesh.call(this,new THREE.CubeGeometry(0.01, 0.01, 0.01), new THREE.MeshBasicMaterial());
    } else {
      THREE.Mesh.call(this,new THREE.CubeGeometry(0.01, 0.01, 0.01), new THREE.MeshBasicMaterial());
//      THREE.Object3D.call(this);
  
      var loader = new THREE.ColladaLoader();
      var url = meshBaseUrl + markerMsg.mesh_resource.substr(10);
      
      var that = this;
      
      loader.load(url, function colladaReady(collada) {
        var sceneObj = collada.scene;
        //sceneObj.children[0].material = new THREE.MeshLambertMaterial({
        //  color:0x888888
        //  });
        that.add(sceneObj);
      });
    }
  };
  
  THREE.MeshMarkerHelper.prototype = Object.create(THREE.Mesh.prototype);

  /* Arrrow Marker */

  THREE.ArrowMarkerHelper = function( options ) {
  
    var origin = options.origin || new THREE.Vector3(0,0,0);
    var dir = options.dir || new THREE.Vector3(1,0,0);
    
    var length = options.length || 1;
    var headLength = options.headLength || 0.2;
    
    var shaftDiameter = options.shaftDiameter || 0.05;
    var headDiameter = options.headDiameter || 0.1;
    
    var material = options.material || new THREE.MeshBasicMaterial();
  
    var shaftLength = length - headLength;
  
    // create and merge geometry
    var geometry = new THREE.CylinderGeometry(shaftDiameter * 0.5,
        shaftDiameter * 0.5, shaftLength, 12, 1);
  
    var m = new THREE.Matrix4;
    m.setPosition( new THREE.Vector3(0, shaftLength * 0.5, 0) );
    geometry.applyMatrix(m);
    
    var coneGeometry = new THREE.CylinderGeometry(0, headDiameter * 0.5,
        headLength, 12, 1);
  
    m.setPosition( new THREE.Vector3(0, shaftLength + (headLength * 0.5), 0) );
    coneGeometry.applyMatrix(m);
        
    THREE.GeometryUtils.merge( geometry, coneGeometry );
  
    THREE.Mesh.call(this,geometry,material);
  
    this.position = origin;
    this.setDirection(dir);
  };
  
  THREE.ArrowMarkerHelper.prototype = Object.create(THREE.Mesh.prototype);
  
  THREE.ArrowMarkerHelper.prototype.setDirection = function(dir) {
  
    var axis = new THREE.Vector3(0, 1, 0).crossSelf(dir);
  
    var radians = Math.acos(new THREE.Vector3(0, 1, 0).dot(dir.clone()
        .normalize()));
  
    this.matrix = new THREE.Matrix4().makeRotationAxis(axis.normalize(), radians);
  
    this.rotation.setEulerFromRotationMatrix(this.matrix, this.eulerOrder);
  
  };
  
  THREE.ArrowMarkerHelper.prototype.setLength = function(length) {
  
    this.scale.set(length, length, length);
  
  };
  
  THREE.ArrowMarkerHelper.prototype.setColor = function(hex) {
  
    this.line.material.color.setHex(hex);
    this.cone.material.color.setHex(hex);
  };

  return MarkersThree;
}));
