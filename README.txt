
GHController
======================

This library is a c++ package to control robotic systems in dynamics

dependencies
------------

It depends on:

* Eigen3 (with unsupported element: Lgsm)
* quadprog
* orc_framework


installation
------------

To install this library, use cmake (or ccmake for gui)::

  mkdir build
  cd build
  cmake .. [-DCMAKE_BUILD_TYPE=Debug] [-DCMAKE_INSTALL_PREFIX=/home/user/folder/]
  make
  [sudo] make install


