(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['three'], factory);
  }
  else {
    root.DepthCloud = factory(root.THREE);
  }
}(this, function (THREE) {

  var DepthCloud = {};
  
  var Viewer = DepthCloud.Viewer = function(options) {
    
    this.url = options.url;
    this.sceneNode = options.sceneNode;
    
    // f defaults to standard Kinect calibration 
    this.f = ( options.f !== undefined ) ? options.f : 526;
    
    this.pointSize = ( options.pointSize !== undefined ) ? options.pointSize : 3;
  }
  
  Viewer.prototype.startStream = function()
  {
  }

  Viewer.prototype.stopStream = function()
  {
  }

  return DepthCloud;
}));
