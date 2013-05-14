;!function(exports, undefined) {

  var isArray = Array.isArray ? Array.isArray : function _isArray(obj) {
    return Object.prototype.toString.call(obj) === "[object Array]";
  };
  var defaultMaxListeners = 10;

  function init() {
    this._events = new Object;
  }

  function configure(conf) {
    if (conf) {
      conf.delimiter && (this.delimiter = conf.delimiter);
      conf.wildcard && (this.wildcard = conf.wildcard);
      if (this.wildcard) {
        this.listenerTree = new Object;
      }
    }
  }

  function EventEmitter(conf) {
    this._events = new Object;
    configure.call(this, conf);
  }

  //
  // Attention, function return type now is array, always !
  // It has zero elements if no any matches found and one or more
  // elements (leafs) if there are matches
  //
  function searchListenerTree(handlers, type, tree, i) {
    if (!tree) {
      return [];
    }
    var listeners=[], leaf, len, branch, xTree, xxTree, isolatedBranch, endReached,
        typeLength = type.length, currentType = type[i], nextType = type[i+1];
    if (i === typeLength && tree._listeners) {
      //
      // If at the end of the event(s) list and the tree has listeners
      // invoke those listeners.
      //
      if (typeof tree._listeners === 'function') {
        handlers && handlers.push(tree._listeners);
        return [tree];
      } else {
        for (leaf = 0, len = tree._listeners.length; leaf < len; leaf++) {
          handlers && handlers.push(tree._listeners[leaf]);
        }
        return [tree];
      }
    }

    if ((currentType === '*' || currentType === '**') || tree[currentType]) {
      //
      // If the event emitted is '*' at this part
      // or there is a concrete match at this patch
      //
      if (currentType === '*') {
        for (branch in tree) {
          if (branch !== '_listeners' && tree.hasOwnProperty(branch)) {
            listeners = listeners.concat(searchListenerTree(handlers, type, tree[branch], i+1));
          }
        }
        return listeners;
      } else if(currentType === '**') {
        endReached = (i+1 === typeLength || (i+2 === typeLength && nextType === '*'));
        if(endReached && tree._listeners) {
          // The next element has a _listeners, add it to the handlers.
          listeners = listeners.concat(searchListenerTree(handlers, type, tree, typeLength));
        }

        for (branch in tree) {
          if (branch !== '_listeners' && tree.hasOwnProperty(branch)) {
            if(branch === '*' || branch === '**') {
              if(tree[branch]._listeners && !endReached) {
                listeners = listeners.concat(searchListenerTree(handlers, type, tree[branch], typeLength));
              }
              listeners = listeners.concat(searchListenerTree(handlers, type, tree[branch], i));
            } else if(branch === nextType) {
              listeners = listeners.concat(searchListenerTree(handlers, type, tree[branch], i+2));
            } else {
              // No match on this one, shift into the tree but not in the type array.
              listeners = listeners.concat(searchListenerTree(handlers, type, tree[branch], i));
            }
          }
        }
        return listeners;
      }

      listeners = listeners.concat(searchListenerTree(handlers, type, tree[currentType], i+1));
    }

    xTree = tree['*'];
    if (xTree) {
      //
      // If the listener tree will allow any match for this part,
      // then recursively explore all branches of the tree
      //
      searchListenerTree(handlers, type, xTree, i+1);
    }
    
    xxTree = tree['**'];
    if(xxTree) {
      if(i < typeLength) {
        if(xxTree._listeners) {
          // If we have a listener on a '**', it will catch all, so add its handler.
          searchListenerTree(handlers, type, xxTree, typeLength);
        }
        
        // Build arrays of matching next branches and others.
        for(branch in xxTree) {
          if(branch !== '_listeners' && xxTree.hasOwnProperty(branch)) {
            if(branch === nextType) {
              // We know the next element will match, so jump twice.
              searchListenerTree(handlers, type, xxTree[branch], i+2);
            } else if(branch === currentType) {
              // Current node matches, move into the tree.
              searchListenerTree(handlers, type, xxTree[branch], i+1);
            } else {
              isolatedBranch = {};
              isolatedBranch[branch] = xxTree[branch];
              searchListenerTree(handlers, type, { '**': isolatedBranch }, i+1);
            }
          }
        }
      } else if(xxTree._listeners) {
        // We have reached the end and still on a '**'
        searchListenerTree(handlers, type, xxTree, typeLength);
      } else if(xxTree['*'] && xxTree['*']._listeners) {
        searchListenerTree(handlers, type, xxTree['*'], typeLength);
      }
    }

    return listeners;
  }

  function growListenerTree(type, listener) {

    type = typeof type === 'string' ? type.split(this.delimiter) : type.slice();
    
    //
    // Looks for two consecutive '**', if so, don't add the event at all.
    //
    for(var i = 0, len = type.length; i+1 < len; i++) {
      if(type[i] === '**' && type[i+1] === '**') {
        return;
      }
    }

    var tree = this.listenerTree;
    var name = type.shift();

    while (name) {

      if (!tree[name]) {
        tree[name] = new Object;
      }

      tree = tree[name];

      if (type.length === 0) {

        if (!tree._listeners) {
          tree._listeners = listener;
        }
        else if(typeof tree._listeners === 'function') {
          tree._listeners = [tree._listeners, listener];
        }
        else if (isArray(tree._listeners)) {

          tree._listeners.push(listener);

          if (!tree._listeners.warned) {

            var m = defaultMaxListeners;
            
            if (typeof this._events.maxListeners !== 'undefined') {
              m = this._events.maxListeners;
            }

            if (m > 0 && tree._listeners.length > m) {

              tree._listeners.warned = true;
              console.error('(node) warning: possible EventEmitter memory ' +
                            'leak detected. %d listeners added. ' +
                            'Use emitter.setMaxListeners() to increase limit.',
                            tree._listeners.length);
              console.trace();
            }
          }
        }
        return true;
      }
      name = type.shift();
    }
    return true;
  };

  // By default EventEmitters will print a warning if more than
  // 10 listeners are added to it. This is a useful default which
  // helps finding memory leaks.
  //
  // Obviously not all Emitters should be limited to 10. This function allows
  // that to be increased. Set to zero for unlimited.

  EventEmitter.prototype.delimiter = '.';

  EventEmitter.prototype.setMaxListeners = function(n) {
    this._events || init.call(this);
    this._events.maxListeners = n;
  };

  EventEmitter.prototype.event = '';

  EventEmitter.prototype.once = function(event, fn) {
    this.many(event, 1, fn);
    return this;
  };

  EventEmitter.prototype.many = function(event, ttl, fn) {
    var self = this;

    if (typeof fn !== 'function') {
      throw new Error('many only accepts instances of Function');
    }

    function listener() {
      if (--ttl === 0) {
        self.off(event, listener);
      }
      fn.apply(this, arguments);
    };

    listener._origin = fn;

    this.on(event, listener);

    return self;
  };

  EventEmitter.prototype.emit = function() {
    this._events || init.call(this);

    var type = arguments[0];

    if (type === 'newListener') {
      if (!this._events.newListener) { return false; }
    }

    // Loop through the *_all* functions and invoke them.
    if (this._all) {
      var l = arguments.length;
      var args = new Array(l - 1);
      for (var i = 1; i < l; i++) args[i - 1] = arguments[i];
      for (i = 0, l = this._all.length; i < l; i++) {
        this.event = type;
        this._all[i].apply(this, args);
      }
    }

    // If there is no 'error' event listener then throw.
    if (type === 'error') {
      
      if (!this._all && 
        !this._events.error && 
        !(this.wildcard && this.listenerTree.error)) {

        if (arguments[1] instanceof Error) {
          throw arguments[1]; // Unhandled 'error' event
        } else {
          throw new Error("Uncaught, unspecified 'error' event.");
        }
        return false;
      }
    }

    var handler;

    if(this.wildcard) {
      handler = [];
      var ns = typeof type === 'string' ? type.split(this.delimiter) : type.slice();
      searchListenerTree.call(this, handler, ns, this.listenerTree, 0);
    }
    else {
      handler = this._events[type];
    }

    if (typeof handler === 'function') {
      this.event = type;
      if (arguments.length === 1) {
        handler.call(this);
      }
      else if (arguments.length > 1)
        switch (arguments.length) {
          case 2:
            handler.call(this, arguments[1]);
            break;
          case 3:
            handler.call(this, arguments[1], arguments[2]);
            break;
          // slower
          default:
            var l = arguments.length;
            var args = new Array(l - 1);
            for (var i = 1; i < l; i++) args[i - 1] = arguments[i];
            handler.apply(this, args);
        }
      return true;
    }
    else if (handler) {
      var l = arguments.length;
      var args = new Array(l - 1);
      for (var i = 1; i < l; i++) args[i - 1] = arguments[i];

      var listeners = handler.slice();
      for (var i = 0, l = listeners.length; i < l; i++) {
        this.event = type;
        listeners[i].apply(this, args);
      }
      return (listeners.length > 0) || this._all;
    }
    else {
      return this._all;
    }

  };

  EventEmitter.prototype.on = function(type, listener) {
    
    if (typeof type === 'function') {
      this.onAny(type);
      return this;
    }

    if (typeof listener !== 'function') {
      throw new Error('on only accepts instances of Function');
    }
    this._events || init.call(this);

    // To avoid recursion in the case that type == "newListeners"! Before
    // adding it to the listeners, first emit "newListeners".
    this.emit('newListener', type, listener);

    if(this.wildcard) {
      growListenerTree.call(this, type, listener);
      return this;
    }

    if (!this._events[type]) {
      // Optimize the case of one listener. Don't need the extra array object.
      this._events[type] = listener;
    }
    else if(typeof this._events[type] === 'function') {
      // Adding the second element, need to change to array.
      this._events[type] = [this._events[type], listener];
    }
    else if (isArray(this._events[type])) {
      // If we've already got an array, just append.
      this._events[type].push(listener);

      // Check for listener leak
      if (!this._events[type].warned) {

        var m = defaultMaxListeners;
        
        if (typeof this._events.maxListeners !== 'undefined') {
          m = this._events.maxListeners;
        }

        if (m > 0 && this._events[type].length > m) {

          this._events[type].warned = true;
          console.error('(node) warning: possible EventEmitter memory ' +
                        'leak detected. %d listeners added. ' +
                        'Use emitter.setMaxListeners() to increase limit.',
                        this._events[type].length);
          console.trace();
        }
      }
    }
    return this;
  };

  EventEmitter.prototype.onAny = function(fn) {

    if(!this._all) {
      this._all = [];
    }

    if (typeof fn !== 'function') {
      throw new Error('onAny only accepts instances of Function');
    }

    // Add the function to the event listener collection.
    this._all.push(fn);
    return this;
  };

  EventEmitter.prototype.addListener = EventEmitter.prototype.on;

  EventEmitter.prototype.off = function(type, listener) {
    if (typeof listener !== 'function') {
      throw new Error('removeListener only takes instances of Function');
    }

    var handlers,leafs=[];

    if(this.wildcard) {
      var ns = typeof type === 'string' ? type.split(this.delimiter) : type.slice();
      leafs = searchListenerTree.call(this, null, ns, this.listenerTree, 0);
    }
    else {
      // does not use listeners(), so no side effect of creating _events[type]
      if (!this._events[type]) return this;
      handlers = this._events[type];
      leafs.push({_listeners:handlers});
    }

    for (var iLeaf=0; iLeaf<leafs.length; iLeaf++) {
      var leaf = leafs[iLeaf];
      handlers = leaf._listeners;
      if (isArray(handlers)) {

        var position = -1;

        for (var i = 0, length = handlers.length; i < length; i++) {
          if (handlers[i] === listener ||
            (handlers[i].listener && handlers[i].listener === listener) ||
            (handlers[i]._origin && handlers[i]._origin === listener)) {
            position = i;
            break;
          }
        }

        if (position < 0) {
          return this;
        }

        if(this.wildcard) {
          leaf._listeners.splice(position, 1)
        }
        else {
          this._events[type].splice(position, 1);
        }

        if (handlers.length === 0) {
          if(this.wildcard) {
            delete leaf._listeners;
          }
          else {
            delete this._events[type];
          }
        }
      }
      else if (handlers === listener ||
        (handlers.listener && handlers.listener === listener) ||
        (handlers._origin && handlers._origin === listener)) {
        if(this.wildcard) {
          delete leaf._listeners;
        }
        else {
          delete this._events[type];
        }
      }
    }

    return this;
  };

  EventEmitter.prototype.offAny = function(fn) {
    var i = 0, l = 0, fns;
    if (fn && this._all && this._all.length > 0) {
      fns = this._all;
      for(i = 0, l = fns.length; i < l; i++) {
        if(fn === fns[i]) {
          fns.splice(i, 1);
          return this;
        }
      }
    } else {
      this._all = [];
    }
    return this;
  };

  EventEmitter.prototype.removeListener = EventEmitter.prototype.off;

  EventEmitter.prototype.removeAllListeners = function(type) {
    if (arguments.length === 0) {
      !this._events || init.call(this);
      return this;
    }

    if(this.wildcard) {
      var ns = typeof type === 'string' ? type.split(this.delimiter) : type.slice();
      var leafs = searchListenerTree.call(this, null, ns, this.listenerTree, 0);

      for (var iLeaf=0; iLeaf<leafs.length; iLeaf++) {
        var leaf = leafs[iLeaf];
        leaf._listeners = null;
      }
    }
    else {
      if (!this._events[type]) return this;
      this._events[type] = null;
    }
    return this;
  };

  EventEmitter.prototype.listeners = function(type) {
    if(this.wildcard) {
      var handlers = [];
      var ns = typeof type === 'string' ? type.split(this.delimiter) : type.slice();
      searchListenerTree.call(this, handlers, ns, this.listenerTree, 0);
      return handlers;
    }

    this._events || init.call(this);

    if (!this._events[type]) this._events[type] = [];
    if (!isArray(this._events[type])) {
      this._events[type] = [this._events[type]];
    }
    return this._events[type];
  };

  EventEmitter.prototype.listenersAny = function() {

    if(this._all) {
      return this._all;
    }
    else {
      return [];
    }

  };

  if (typeof define === 'function' && define.amd) {
    define(function() {
      return EventEmitter;
    });
  } else {
    exports.EventEmitter2 = EventEmitter; 
  }

}(typeof process !== 'undefined' && typeof process.title !== 'undefined' && typeof exports !== 'undefined' ? exports : window);

// Ros.js can be included using <script src="ros.js"> or AMD.  The next few
// lines provide support for both formats and are based on the Universal Module
// Definition.
//
// See:
//  * AMD - http://bryanforbes.github.com/amd-commonjs-modules-presentation/2011-10-29/)
//  * UMD - https://github.com/umdjs/umd/blob/master/amdWeb.js
(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['eventemitter2'], factory);
  }
  else {
    root.ROS = factory(root.EventEmitter2);
  }
}(this, function (EventEmitter2) {

  // Takes in the URL of the WebSocket server.
  // Emits the following events:
  //  * 'error' - there was an error with ROS
  //  * 'connection' - connected to the WebSocket server
  //  * 'close' - disconnected to the WebSocket server
  var ROS = function(url) {
    var ros = this;
    ros.socket = null;

    // Provides a unique ID for each message sent to the server.
    ros.idCounter = 0;

    // Socket Handling
    // ---------------

    function onOpen(event) {
      ros.emit('connection', event);
    };

    function onClose(event) {
      ros.emit('close', event);
    };

    function onError(event) {
      ros.emit('error', event);
    };

    function decompressPng(data, callback) {
      // Uncompresses the data before sending it through (use image/canvas to do so).
      var image = new Image();
      // When the image loads, extracts the raw data (JSON message).
      image.onload = function() {
        // Creates a local canvas to draw on.
        var canvas  = document.createElement('canvas');
        var context = canvas.getContext('2d');

        // Sets width and height.
        canvas.width = image.width;
        canvas.height = image.height;

        // Puts the data into the image.
        context.drawImage(image, 0, 0);
        // Grabs the raw, uncompressed data.
        var imageData = context.getImageData(0, 0, image.width, image.height).data;

        // Constructs the JSON.
        var jsonData = '';
        for (var i = 0; i < imageData.length; i += 4) {
          // RGB
          jsonData += String.fromCharCode(imageData[i], imageData[i+1], imageData[i+2]);
        }
        var decompressedData = JSON.parse(jsonData);
        callback(decompressedData);
      };
      // Sends the image data to load.
      image.src = 'data:image/png;base64,' + data.data;
    }

    // Parses message responses from rosbridge and sends to the appropriate
    // topic, service, or param.
    function onMessage(message) {
      function handleMessage(message) {
        if (message.op === 'publish') {
          ros.emit(message.topic, message.msg);
        }
        else if (message.op === 'service_response') {
          ros.emit(message.id, message.values);
        }
      };

      var data = JSON.parse(message.data);
      if (data.op === 'png') {
        decompressPng(data, function(decompressedData) {
          handleMessage(decompressedData);
        });
      }
      else {
        handleMessage(data);
      }
    };

    // Sends the message over the WebSocket, but queues the message up if not
    // yet connected.
    function callOnConnection(message) {
      var messageJson = JSON.stringify(message);

      if (ros.socket.readyState !== WebSocket.OPEN) {
        ros.once('connection', function() {
          ros.socket.send(messageJson);
        });
      }
      else {
        ros.socket.send(messageJson);
      }
    };

    // Connect to the specified WebSocket
    ros.connect = function(url) {
      ros.socket = new WebSocket(url);
      ros.socket.onopen    = onOpen;
      ros.socket.onclose   = onClose;
      ros.socket.onerror   = onError;
      ros.socket.onmessage = onMessage;
    };

    // Disconnect from the WebSocket
    ros.close = function() {
      if (ros.socket) {
        ros.socket.close();
      }
    };

    if (url) {
      ros.connect(url);
    }

    // Topics
    // ------

    // Retrieves list of topics in ROS as an array.
    ros.getTopics = function(callback) {
      var topicsClient = new ros.Service({
        name        : '/rosapi/topics'
      , serviceType : 'rosapi/Topics'
      });

      var request = new ros.ServiceRequest();

      topicsClient.callService(request, function(result) {
        callback(result.topics);
      });
    };

    // Message objects are used for publishing and subscribing to and from
    // topics. Takes in an object matching the fields defined in the .msg
    // definition file.
    ros.Message = function(values) {
      var message = this;
      if (values) {
        Object.keys(values).forEach(function(name) {
          message[name] = values[name];
        });
      }
    };

    // Publish and/or subscribe to a topic in ROS. Options include:
    //  * node - the name of the node to register under
    //  * name - the topic name, like /cmd_vel
    //  * messageType - the message type, like 'std_msgs/String'
    ros.Topic = function(options) {
      var topic          = this;
      options            = options || {};
      topic.node         = options.node;
      topic.name         = options.name;
      topic.messageType  = options.messageType;
      topic.isAdvertised = false;
      topic.compression  = options.compression || 'none';

      // Check for valid compression types
      if (topic.compression && topic.compression !== 'png' && topic.compression !== 'none') {
        topic.emit('warning', topic.compression + ' compression is not supported. No comression will be used.');
      }

      // Every time a message is published for the given topic, the callback
      // will be called with the message object.
      topic.subscribe = function(callback) {
        topic.on('message', function(message) {
          callback(message);
        });

        ros.on(topic.name, function(data) {
          var message = new ros.Message(data);
          topic.emit('message', message);
        });

        ros.idCounter++;
        var subscribeId = 'subscribe:' + topic.name + ':' + ros.idCounter;
        var call = {
          op          : 'subscribe'
        , id          : subscribeId
        , type        : topic.messageType
        , topic       : topic.name
        , compression : topic.compression
        };

        callOnConnection(call);
      };

      // Unregisters as a subscriber for the topic. Unsubscribing will remove
      // all subscribe callbacks.
      topic.unsubscribe = function() {
        ros.removeAllListeners([topic.name]);
        ros.idCounter++;
        var unsubscribeId = 'unsubscribe:' + topic.name + ':' + ros.idCounter;
        var call = {
          op    : 'unsubscribe'
        , id    : unsubscribeId
        , topic : topic.name
        };
        callOnConnection(call);
      };

      // Registers as a publisher for the topic.
      topic.advertise = function() {
        ros.idCounter++;
        var advertiseId = 'advertise:' + topic.name + ':' + ros.idCounter;
        var call = {
          op    : 'advertise'
        , id    : advertiseId
        , type  : topic.messageType
        , topic : topic.name
        };
        callOnConnection(call);
        topic.isAdvertised = true;
      };

      // Unregisters as a publisher for the topic.
      topic.unadvertise = function() {
        ros.idCounter++;
        var unadvertiseId = 'unadvertise:' + topic.name + ':' + ros.idCounter;
        var call = {
          op    : 'unadvertise'
        , id    : unadvertiseId
        , topic : topic.name
        };
        callOnConnection(call);
        topic.isAdvertised = false;
      };

      // Publish the message. Takes in a ros.Message.
      topic.publish = function(message) {
        if (!topic.isAdvertised) {
          topic.advertise();
        }

        ros.idCounter++;
        var publishId = 'publish:' + topic.name + ':' + ros.idCounter;
        var call = {
          op    : 'publish'
        , id    : publishId
        , topic : topic.name
        , msg   : message
        };
        callOnConnection(call);
      };
    };
    ros.Topic.prototype.__proto__ = EventEmitter2.prototype;

    // Services
    // --------

    // Retrieves list of active service names in ROS as an array.
    ros.getServices = function(callback) {
      var servicesClient = new ros.Service({
        name        : '/rosapi/services'
      , serviceType : 'rosapi/Services'
      });

      var request = new ros.ServiceRequest();

      servicesClient.callService(request, function(result) {
        callback(result.services);
      });
    };

    // A ServiceRequest is passed into the service call. Takes in an object
    // matching the values of the request part from the .srv file.
    ros.ServiceRequest = function(values) {
      var serviceRequest = this;
      if (values) {
        Object.keys(values).forEach(function(name) {
          serviceRequest[name] = values[name];
        });
      }
    };

    // A ServiceResponse is returned from the service call. Takes in an object
    // matching the values of the response part from the .srv file.
    ros.ServiceResponse = function(values) {
      var serviceResponse = this;
      if (values) {
        Object.keys(values).forEach(function(name) {
          serviceResponse[name] = values[name];
        });
      }
    };

    // A ROS service client. Options include:
    //  * name - the service name, like /add_two_ints
    //  * serviceType - the service type, like 'rospy_tutorials/AddTwoInts'
    ros.Service = function(options) {
      var service         = this;
      options             = options || {};
      service.name        = options.name;
      service.serviceType = options.serviceType;

      // Calls the service. Returns the service response in the callback.
      service.callService = function(request, callback) {
        ros.idCounter++;
        serviceCallId = 'call_service:' + service.name + ':' + ros.idCounter;

        ros.once(serviceCallId, function(data) {
          var response = new ros.ServiceResponse(data);
          callback(response);
        });

        var requestValues = [];
        Object.keys(request).forEach(function(name) {
          requestValues.push(request[name]);
        });

        var call = {
          op      : 'call_service'
        , id      : serviceCallId
        , service : service.name
        , args    : requestValues
        };
        callOnConnection(call);
      };
    };
    ros.Service.prototype.__proto__ = EventEmitter2.prototype;

    // Params
    // ------

    // Retrieves list of param names from the ROS Parameter Server as an array.
    ros.getParams = function(callback) {
      var paramsClient = new ros.Service({
        name        : '/rosapi/get_param_names'
      , serviceType : 'rosapi/GetParamNames'
      });

      var request = new ros.ServiceRequest();
      paramsClient.callService(request, function(result) {
        callback(result.names);
      });
    };

    // A ROS param. Options include:
    //  * name - the param name, like max_vel_x
    ros.Param = function(options) {
      var param  = this;
      options    = options || {};
      param.name = options.name;

      // Fetches the value of the param and returns in the callback.
      param.get = function(callback) {
        var paramClient = new ros.Service({
          name        : '/rosapi/get_param'
        , serviceType : 'rosapi/GetParam'
        });

        var request = new ros.ServiceRequest({
          name  : param.name
        , value : JSON.stringify('')
        });

        paramClient.callService(request, function(result) {
          var value = JSON.parse(result.value);
          callback(value);
        });
      };

      // Sets the value of the param in ROS.
      param.set = function(value) {
        var paramClient = new ros.Service({
          name        : '/rosapi/set_param'
        , serviceType : 'rosapi/SetParam'
        });

        var request = new ros.ServiceRequest({
          name: param.name
        , value: JSON.stringify(value)
        });

        paramClient.callService(request, function() {});
      };
    };
    ros.Param.prototype.__proto__ = EventEmitter2.prototype;

  };
  ROS.prototype.__proto__ = EventEmitter2.prototype;

  return ROS;

}));

