apriltag
========

Small modifications/additions to  http://april.eecs.umich.edu/media/apriltag/apriltag-2015-03-18.tgz

Added a new quad detector and a few various speedups.

***Please note:*** I am not the maintainer of the pypi package listed at https://pypi.org/project/apriltag/ – GitHub issues filed here reporting problems with that package will be summarily closed. Sorry, I don't have time to support someone else's unofficial package.


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

If you want to install the library and important binaries to your system directories, you can then type

    sudo make install

Running
=======

You can run `aprilag_opencv_demo` to do stuff, run with `-h` to get help.

So for example, you can run

    ./apriltag_opencv_demo -B    ../images/mapping_feb_2014/*.JPG
    ./apriltag_opencv_demo -B -c ../images/mapping_feb_2014/*.JPG

to benchmark the new code against the old code.

Python
======

***Note that you must build the software per the instructions above before the Python wrapper can 
be used.*** If you did not install the libraries to the system-wide library directory and you 
are not running Python code from the python directory in this repository, your Python code
must specify the  path for the apriltag shared library when constructing an
`apriltag.Detector` object.

I recently added the ability to estimate 3D tag poses to the Python wrapper.
To demonstrate this, after building the software, go to the python directory and run

    python apriltag.py -c -k '(765.00, 764.18, 393.72, 304.66)' -s .127 ../images/mapping_feb_2014/*JPG

To estimate tag pose, you need to know the [intrinsic camera parameters](https://docs.opencv.org/3.3.1/dc/dbb/tutorial_py_calibration.html), which
can be estimated using the [calibrate_camera.py](python/calibrate_camera.py) script.

You also need to know the tag size in order to scale the estimated translation
vectors correctly.

Although this functionality is implmemented in the C library, it is not yet
coded into the C/C++ demos; I may add it someday if requested.
