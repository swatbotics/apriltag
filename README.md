c-apriltag - Matt's port

Port from http://april.eecs.umich.edu/software/apriltag-2015-03-18.tgz

Added a new quad detector and a few various speedups.

Build dependencies:

  - OpenCV (optional)

To build, go to root directory of repo and then run the following:

    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j4

You can run `aprilag_opencv_demo` to do stuff, run with `-h` to get help.

So for example, you can run

    ./apriltag_opencv_demo -B    ../images/mapping_feb_2014/*.JPG
    ./apriltag_opencv_demo -B -c ../images/mapping_feb_2014/*.JPG

to benchmark the new code against the old code.




