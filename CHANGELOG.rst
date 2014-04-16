^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package depthcloud_encoder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (2014-04-16)
------------------
* source cleanup
* basic cleanup
* depricated launch package removed
* Contributors: Russell Toris

0.0.3 (2013-05-24)
------------------
* removing depthcloud_www from package.xml
* 0.0.2 -> 0.0.3
* adding nodelet to catkin dependencies
* adding nodelet to package.xml
* cleaning up depthcloud encoder repo
* adding travis.yml
* Contributors: Julius Kammerl

0.0.2 (2013-05-16)
------------------
* 0.0.1 -> 0.0.2
* Merge branch 'groovy-devel' of github.com:RobotWebTools/depthcloudjs into groovy-devel
* adding depthcloud javascript & shader files
* fixing conflict
* removing launch file from www folder
* adding ros launch file
* adding default orientation to depthcloudjs
* reorganization of depthcloud_www
* first version of working launch file
* moving simple_webserver.py to depthcloud_www/html
* adding redirecting index.html
* adding depthcloud_launch package
* adding another crossdomain fix/patch to make CORS happy
* removing launch file from www folder
* adding ros launch file
* minor
* minor
* sample html files
* adding catkin files
* adding meta package
* minor
* adding ROS node & nodelet wrapper
* adding depthcloud encoder class
* depthcloud encoder nodelet xml
* revised catkin scripts
* renamed www->depthcloud_www
* removing tf encoding from pointcloud_image.cpp
* added low-res pointcloud streaming + fixed param server lookup in pointcloud_image.cpp
* Update CMakeLists.txt
* fixing package.xml
* removing lowres-demo folder.. creating demo/demo-low-res.html
* package reorganization & adding pointcloud_image_streamer node
* merged jkammerl/groovy-devel into upstream
* merged package.xml from jkammerl `#2 <https://github.com/RobotWebTools/depthcloud_encoder/issues/2>`_
* merged package.xml from jkammerl
* Merge pull request `#7 <https://github.com/RobotWebTools/depthcloud_encoder/issues/7>`_ from rctoris/groovy-devel
  fixes catkin_make build problem
* fixed html page title for demo
* fixes catkin_make build problem
* adding low-res version
* adding catkin as buildtool dependency
* Merge pull request `#5 <https://github.com/RobotWebTools/depthcloud_encoder/issues/5>`_ from jkammerl/groovy-devel
  removing review tag from groovy-devel
* removing review tag from groovy-devel
* adding CMakeLists.txt to depthcoudjs
* various fixes in demo code & moved shaders up
* merged dgossow into jkammerl fork, removed pointcloud_image_streamer, moved depthcloud.js to root, updated interactivemarkers.js to most recent version
* modified code to work in ros_video_streamer, and fixed coordinate computation
* added tf support to demo
* attempt to fix crossdomain access issues
* depthcloud.js integration
* reorganizing repository / adding pointcloud_image package
* added shader url option to streamer
* initial commit of demo code
* minor
* initial depth cloud plugin
* dummy depthcloud module
* Initial commit
* Contributors: David Gossow, Interactive Manipulation, Julius Kammerl, Russell Toris
