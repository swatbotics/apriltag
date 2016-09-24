/* (C) 2013-2015, The Regents of The University of Michigan
All rights reserved.

This software may be available under alternative licensing
terms. Contact Edwin Olson, ebolson@umich.edu, for more information.

   Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project.
 */

#include "apriltag_opencv.h"
#include "apriltag_family.h"
#include "getopt.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// Invoke:
//
// tagtest [options] input.pnm

int main(int argc, char *argv[])
{
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 'i', "iters", "1", "Repeat processing on input set this many times");

    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");
    getopt_add_bool(getopt, 'c', "contours", 0, "Use new contour-based quad detection");
    getopt_add_bool(getopt, 'n', "no-gui", 0, "Suppress GUI output from OpenCV");
    getopt_add_bool(getopt, 'B', "benchmark", 0, "Benchmark mode (assumes -n)");

    if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options] <input files>\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    const zarray_t *inputs = getopt_get_extra_args(getopt);

    const char *famname = getopt_get_string(getopt, "family");

    apriltag_family_t *tf = apriltag_family_create(famname);

    if (!tf) {
      printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
      exit(-1);
    }

    tf->black_border = getopt_get_int(getopt, "border");

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    if (getopt_get_bool(getopt, "contours")) {
      apriltag_detector_enable_quad_contours(td, 1);
    }
    
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");
    td->refine_decode = getopt_get_bool(getopt, "refine-decode");
    td->refine_pose = getopt_get_bool(getopt, "refine-pose");

    int quiet = getopt_get_bool(getopt, "quiet");

    int nogui = getopt_get_bool(getopt, "no-gui");

    int benchmark = getopt_get_bool(getopt, "benchmark");

    int maxiters = getopt_get_int(getopt, "iters");
    
    if (benchmark) {
      nogui = 1;
    }

    int total_detections = 0;
    uint64_t total_time = 0;

    const int hamm_hist_max = 10;

    for (int iter = 0; iter < maxiters; iter++) {

      if (maxiters > 1 && !benchmark) {
        printf("iter %d / %d\n", iter + 1, maxiters);
      }

      for (int input = 0; input < zarray_size(inputs); input++) {

        int hamm_hist[hamm_hist_max];
        memset(hamm_hist, 0, sizeof(hamm_hist));

        char *path;
        zarray_get(inputs, input, &path);

        if (benchmark) {
          int l=strlen(path);
          while (l && path[l-1] != '/') { --l; }
          printf("%s", path+l);
        } else if (!quiet) {
          printf("loading %s\n", path);
        }

        cv::Mat orig = cv::imread(path);
            
        Mat8uc1 gray;

        if (orig.channels() == 3) {
          cv::cvtColor(orig, gray, cv::COLOR_RGB2GRAY);
        } else {
          orig.copyTo(gray);
        }

        image_u8_t* im8 = cv2im8_copy(gray);

        if (gray.empty()) {
          fprintf(stderr, "error loading %s\n", path);
          continue;
        }

        zarray_t *detections = apriltag_detector_detect(td, im8);
      
        cv::Mat display;

        if (!nogui) {
          display = detectionsImage(detections, orig.size(), orig.type());
        }
        
        total_detections += zarray_size(detections);

        for (int i = 0; i < zarray_size(detections); i++) {
          
          apriltag_detection_t *det;
          zarray_get(detections, i, &det);

          if (benchmark) {
            printf(" %d", det->id);
          } else if (!quiet) {
            printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, "
                   "goodness %8.3f, margin %8.3f\n",
                   i, det->family->d*det->family->d, det->family->h,
                   det->id, det->hamming, det->goodness, det->decision_margin);
          }

          hamm_hist[det->hamming]++;
        
        }

        apriltag_detections_destroy(detections);

        if (!benchmark) {

          if (!quiet) {
            timeprofile_display(td->tp);
            printf("nedges: %d, nsegments: %d, nquads: %d\n",
                   td->nedges, td->nsegments, td->nquads);
          }

          if (!quiet)
            printf("Hamming histogram: ");

          for (int i = 0; i < hamm_hist_max; i++)
            printf("%5d", hamm_hist[i]);

          if (quiet) {
            printf("%12.3f", timeprofile_total_utime(td->tp) / 1.0E3);
          }
        

        }
      
        printf("\n");

        if (!nogui) {
          display = 0.5*display + 0.5*orig;
          cv::imshow("foo", orig);
          cv::waitKey();
          cv::imshow("foo", display);
          cv::waitKey();
        }

        image_u8_destroy(im8);
        total_time += timeprofile_total_utime(td->tp);

      }

    }

    if (benchmark) {
      int nin = zarray_size(inputs)*maxiters;
      fprintf(stderr, "%d detections over %d images in %.3f ms (%.3f ms per frame)\n",
              total_detections, nin,
              (total_time*1e-3), (total_time*1e-3)/nin);
    }

    // don't deallocate contents of inputs; those are the argv
    apriltag_detector_destroy(td);

    apriltag_family_destroy(tf);
    return 0;
}
