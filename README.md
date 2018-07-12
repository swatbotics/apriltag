apriltag
========

Small modifications/additions to  http://april.eecs.umich.edu/media/apriltag/apriltag-2015-03-18.tgz

Added a new quad detector and a few various speedups.

Dependencies
============

  - OpenCV (optional)

Building
========

    cd /path/to/apriltag
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j4
    
Running
=======

You can run `aprilag_opencv_demo` to do stuff, run with `-h` to get help.

So for example, you can run

    ./apriltag_opencv_demo -B    ../images/mapping_feb_2014/*.JPG
    ./apriltag_opencv_demo -B -c ../images/mapping_feb_2014/*.JPG

to benchmark the new code against the old code.

Python
======

I recently added the ability to estimate 3D tag poses to the Python wrapper.
To run this, after building the software, go to the python directory and run

    python ../python/apriltag.py -c -k '(765.00, 764.18, 393.72, 304.66)' -s .127 ../images/mapping_feb_2014/*JPG

To estimate tag pose, you need to know the [intrinsic camera parameters](https://docs.opencv.org/3.3.1/dc/dbb/tutorial_py_calibration.html), which
can be estimated using the <python/calibrate_camera.py> script.

You also need to know the tag size in order to scale the estimated translation
vectors correctly.

Although this functionality is implmemented in the C library, it is not yet
coded into the C/C++ demos; I may add it someday if requested.
