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




