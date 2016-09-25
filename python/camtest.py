#!/usr/bin/env python

'''Demonstrate Python wrapper of C apriltag library by running on camera frames.'''

from argparse import ArgumentParser
import cv2
import apriltag

# for some reason pylint complains about members being undefined :(
# pylint: disable=E1101

def main():

    '''Main function.'''

    parser = ArgumentParser(
        description='test apriltag Python bindings')

    parser.add_argument('device_or_movie', metavar='INPUT', nargs='?', default=0,
                        help='Movie to load or integer ID of camera device')

    apriltag.add_arguments(parser)

    options = parser.parse_args()

    try:
        cap = cv2.VideoCapture(int(options.device_or_movie))
    except ValueError:
        cap = cv2.VideoCapture(options.device_or_movie)

    window = 'Camera'
    cv2.namedWindow(window)

    detector = apriltag.Detector(options)

    while True:

        success, frame = cap.read()
        if not success:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        detections, dimg = detector.detect(gray, return_image=True)

        num_detections = len(detections)
        print 'Detected {} tags.\n'.format(num_detections)

        for i, detection in enumerate(detections):
            print 'Detection {} of {}:'.format(i+1, num_detections)
            print
            print detection.tostring(indent=2)
            print

        overlay = frame / 2 + dimg[:, :, None] / 2

        cv2.imshow(window, overlay)
        k = cv2.waitKey(1)

        if k == 27:
            break

if __name__ == '__main__':
    main()
