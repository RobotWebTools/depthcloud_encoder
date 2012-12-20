(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['eventemitter2','actionclient'], factory);
  }
  else {
    root.TfClient = factory(root.EventEmitter2,root.ActionClient);
  }
}(this, function (EventEmitter2, ActionClient) {

  var TfClient = function(options) {
    this.ros = options.ros;
    this.fixedFrame = options.fixedFrame || 'base_link';
    this.angularThres = options.angularThres || 2.0;
    this.transThres = options.transThres || 0.01;
        
    var options = {
      ros: this.ros,
      serverName: options.serverName || "/tf2_web_republisher",
      actionName: "tf2_web_republisher/TFSubscriptionAction"
    };
    
    this.actionClient = new ActionClient( options );
    this.currentGoal = false;
    this.frame_cbs = {};
    this.needUpdate = false;
    
    this.updateTimer = setInterval(this.updateGoal.bind(this), 100);
  };

  TfClient.prototype.__proto__ = EventEmitter2.prototype;
  
  TfClient.prototype.tfUpdate = function(tfMsg) {
    var that = this;
    tfMsg.transforms.forEach( function(transform) {
      var frameId = transform.child_frame_id;
      var cbs = that.frame_cbs[frameId];
      if ( cbs != undefined ) {
        cbs.forEach(function(cb) {
          cb(transform.transform);
        });
      }
    });
  }
  
  // anytime the list of frames changes, we will need to send a new goal 
  TfClient.prototype.updateGoal = function() {
    if ( this.needUpdate ) {
      // cancel old goal, if any
      if ( this.currentGoal ) {
        this.currentGoal.cancel();
      }
      
      var goalMsg = {
        source_frames: [],
         target_frame: this.fixedFrame,
         angular_thres: this.angularThres,
         trans_thres: this.transThres     
      };
      
      var source_frames = [];
      for (frame in this.frame_cbs ) {
        goalMsg.source_frames.push(frame);
      };
      
      this.currentGoal = new this.actionClient.Goal(goalMsg);
      this.currentGoal.on('feedback', this.tfUpdate.bind(this));
      this.currentGoal.send();
      this.needUpdate = false;
    }
  }

  TfClient.prototype.subscribe = function(frameId,callback) {
    // if there is no callback registered for the given frame,
    // create emtpy callback list
    if ( this.frame_cbs[frameId] == undefined ) {
      this.frame_cbs[frameId] = [];
      this.needUpdate = true;      
    }
    this.frame_cbs[frameId].push( callback );
  };
  
  TfClient.prototype.unsubscribe = function(frameId,callback) {
    var cbs = this.frame_cbs[frameId];
    if ( cbs != undefined ) {
      var cbIndex = cbs.indexOf( callback );
      if ( cbIndex >= 0 ) {
        cbs.splice(cbIndex, 1);
        if ( cbs.length == 0 ) {
          delete this.frame_cbs[frameId];
        }
      this.needUpdate = true;      
      }
    }
  }
  
  return TfClient;
}));
