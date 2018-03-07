^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package leap_motion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.12 (2018-03-06)
-------------------
* Reimplementation of the entire Leap Motion driver for ROS
* Old implementation of the driver is now deprecated and will be removed in a year.
* Contributors: Nowittyusername

0.0.11 (2017-01-14)
-------------------
* [fix] ROS Hydro onward requires queue_size option `#31 <https://github.com/ros-drivers/leap_motion/issues/31>`_
* Contributors: Kei Okada

0.0.10 (2016-06-18)
-------------------
* [fix][sys] launch and camera_info directory should be installed
  [fix][sys] a bug of the check method of LEAP_SDK environment value `#27 <https://github.com/ros-drivers/leap_motion/issues/28>`_
* [sys] Add tests `#25 <https://github.com/ros-drivers/leap_motion/issues/25>`_
* Contributors: Kenta Yonekura, Isaac I.Y. Saito

0.0.9 (2015-12-14)
------------------
* [feat] Added individual coordinates for each finger bone (`#23 <https://github.com/ros-drivers/leap_motion/issues/23>`_)
* [feat] Better way of parameter declaration (`#17 <https://github.com/ros-drivers/leap_motion/issues/17>`_)
* [sys] Update travis conf to Ubuntu Trusty and ROS I-J (`#22 <https://github.com/ros-drivers/leap_motion/issues/22>`_)
* Contributors: Isaac I.Y. Saito, Tu-Hoa Pham

0.0.8 (2015-04-30)
------------------
* (Feature) Take ROS msg publish frequency from commandline, set it to Parameter Server
* (Maintenance) Add unittest structure.
* Contributors: Isaac I.Y. Saito

0.0.7 (2014-12-24)
------------------
* Build some binaries only when SDK is installed (Fix to https://github.com/ros-drivers/leap_motion/issues/7).
* Contributors: Isaac I.Y. Saito, Yu Ohara

0.0.6 (2014-11-18)
------------------
* Fix typo in dependency. `#5 <https://github.com/ros-drivers/leap_motion/issues/5>`_ from 130s/typo_dependency
* Contributors: Isaac IY Saito

0.0.5 (2014-11-17)
------------------
* Enhancements for skeleton/finger tf frame publishing as well as code cleanup
* Code cleanup
* skeleton sender added
* Adjusted to new LeapMotion API (Fall 2013)
* Changed license to BSD
* Contributors: Florian Lier, Igor Zubrycki, Mirza Shah, Isaac IY Saito

0.0.4 (2013-10-28)
--------------------

0.0.3 (2013-10-24)
--------------------
* Initial release as ROS package. 
* Provides basic publisher for Leap Motion device.
