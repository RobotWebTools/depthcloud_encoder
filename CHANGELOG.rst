^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package depthcloud_encoder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2019-05-05)
------------------
* Add dynamic reconfigure gen as dependency chain (`#14 <https://github.com/RobotWebTools/depthcloud_encoder/issues/14>`_)
* Contributors: Jihoon Lee

0.1.0 (2019-03-31)
------------------
* Add ability to latch encoded topic (`#12 <https://github.com/RobotWebTools/depthcloud_encoder/issues/12>`_)
* Make target resolution / crop size a parameter (`#11 <https://github.com/RobotWebTools/depthcloud_encoder/issues/11>`_)
* Add dynamic reconfigure server + fetch focal length from camera info topic (`#10 <https://github.com/RobotWebTools/depthcloud_encoder/issues/10>`_)
  * Add dynamic reconfigure server for some params
  * Fetch focal length from camera info topic
* Parameterize max_depth_per_tile (`#8 <https://github.com/RobotWebTools/depthcloud_encoder/issues/8>`_)
* Update travis config to build for indigo and kinetic (`#9 <https://github.com/RobotWebTools/depthcloud_encoder/issues/9>`_)
* Contributors: Jihoon Lee, Viktor Kunovski

0.0.5 (2015-08-18)
------------------
* Merge pull request #5 from psoetens/develop-closecloud
  Develop closecloud encoding
* depthcloud: allow user to force use of depthmap or cloud source in case multiple options are available
  Without this, we would always subscribe to the cloud topic, even if it
  was not set in the launch file, but it was set in the rosparam server.
  Signed-off-by: Peter Soetens <peter@intermodalics.eu>
* package: add dependencies to pcl_conversions and tf_conversions.
  Signed-off-by: Peter Soetens <peter@thesourceworks.com>
* update package.xml to add pcl dependencies.
  Signed-off-by: Peter Soetens <peter@thesourceworks.com>
* Multiply resolution by 6 by taking closer images and more efficient use of color channels.
  This patch also needs a change on the Depthcloud.js side.
  We could encode these mapping settings on a separate topic
  for Depthcloud.js to pickup again.
  Signed-off-by: Peter Soetens <peter@thesourceworks.com>
* encoder: support encoding of PCL point clouds
  This does add a dependency to TF and PCL but makes the
  encoder much more flexible in what it can encode.
  Signed-off-by: Peter Soetens <peter@thesourceworks.com>
* Changed to explicit casting
* Contributors: Akin Sisbot, Peter Soetens, Russell Toris

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
