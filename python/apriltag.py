#!/usr/bin/env python

"""Python wrapper for C version of apriltags. This program creates two
classes that are used to detect apriltags and extract information from
them. Using this module, you can identify all apriltags visible in an
image, and get information about the location and orientation of the
tags.

Original author: Isaac Dulin, Spring 2016
Updates: Matt Zucker, Fall 2016

"""
from __future__ import division
from __future__ import print_function

import ctypes
import collections
import os
import re
import numpy

_HAVE_CV2 = False

if __name__ == '__main__':
    try:
        import cv2
        _HAVE_CV2 = True
    except:
        pass

######################################################################

# pylint: disable=R0903

class _ImageU8(ctypes.Structure):
    '''Wraps image_u8 C struct.'''
    _fields_ = [
        ('width', ctypes.c_int),
        ('height', ctypes.c_int),
        ('stride', ctypes.c_int),
        ('buf', ctypes.POINTER(ctypes.c_uint8))
    ]

class _Matd(ctypes.Structure):
    '''Wraps matd C struct.'''
    _fields_ = [
        ('nrows', ctypes.c_int),
        ('ncols', ctypes.c_int),
        ('data', ctypes.c_double*1),
    ]

class _ZArray(ctypes.Structure):
    '''Wraps zarray C struct.'''
    _fields_ = [
        ('el_sz', ctypes.c_size_t),
        ('size', ctypes.c_int),
        ('alloc', ctypes.c_int),
        ('data', ctypes.c_void_p)
    ]

class _ApriltagFamily(ctypes.Structure):
    '''Wraps apriltag_family C struct.'''
    _fields_ = [
        ('ncodes', ctypes.c_int32),
        ('codes', ctypes.POINTER(ctypes.c_int64)),
        ('black_border', ctypes.c_int32),
        ('d', ctypes.c_int32),
        ('h', ctypes.c_int32),
        ('name', ctypes.c_char_p),
    ]

class _ApriltagDetection(ctypes.Structure):
    '''Wraps apriltag_detection C struct.'''
    _fields_ = [
        ('family', ctypes.POINTER(_ApriltagFamily)),
        ('id', ctypes.c_int),
        ('hamming', ctypes.c_int),
        ('goodness', ctypes.c_float),
        ('decision_margin', ctypes.c_float),
        ('H', ctypes.POINTER(_Matd)),
        ('c', ctypes.c_double*2),
        ('p', (ctypes.c_double*2)*4)
    ]

class _ApriltagDetector(ctypes.Structure):
    '''Wraps apriltag_detector C struct.'''
    _fields_ = [
        ('nthreads', ctypes.c_int),
        ('quad_decimate', ctypes.c_float),
        ('quad_sigma', ctypes.c_float),
        ('refine_edges', ctypes.c_int),
        ('refine_decode', ctypes.c_int),
        ('refine_pose', ctypes.c_int),
        ('debug', ctypes.c_int),
        ('quad_contours', ctypes.c_int),
    ]

######################################################################

def _ptr_to_array2d(datatype, ptr, rows, cols):
    array_type = (datatype*cols)*rows
    array_buf = array_type.from_address(ctypes.addressof(ptr))
    return numpy.ctypeslib.as_array(array_buf, shape=(rows, cols))

def _image_u8_get_array(img_ptr):
    return _ptr_to_array2d(ctypes.c_uint8,
                           img_ptr.contents.buf.contents,
                           img_ptr.contents.height,
                           img_ptr.contents.stride)

def _matd_get_array(mat_ptr):
    return _ptr_to_array2d(ctypes.c_double,
                           mat_ptr.contents.data,
                           int(mat_ptr.contents.nrows),
                           int(mat_ptr.contents.ncols))


######################################################################

DetectionBase = collections.namedtuple(
    'DetectionBase',
    'tag_family, tag_id, hamming, goodness, decision_margin, '
    'homography, center, corners')

class Detection(DetectionBase):

    '''Pythonic wrapper for apriltag_detection which derives from named
tuple class.

    '''

    _print_fields = [
        'Family', 'ID', 'Hamming error', 'Goodness',
        'Decision margin', 'Homography', 'Center', 'Corners'
    ]

    _max_len = max(len(field) for field in _print_fields)

    def tostring(self, values=None, indent=0):

        '''Converts this object to a string with the given level of indentation.'''

        rval = []
        indent_str = ' '*(self._max_len+2+indent)

        if not values:
            values = collections.OrderedDict(zip(self._print_fields, self))

        for label in values:

            value_str = str(values[label])

            if value_str.find('\n') > 0:
                value_str = value_str.split('\n')
                value_str = [value_str[0]] + [indent_str+v for v in value_str[1:]]
                value_str = '\n'.join(value_str)

            rval.append('{:>{}s}: {}'.format(
                label, self._max_len+indent, value_str))

        return '\n'.join(rval)

    def __str__(self):
        return self.tostring().encode('ascii')

######################################################################


class DetectorOptions(object):

    '''Convience wrapper for object to pass into Detector
initializer. You can also pass in the output of an
argparse.ArgumentParser on which you have called add_arguments.

    '''

# pylint: disable=R0902
# pylint: disable=R0913

    def __init__(self,
                 families='tag36h11',
                 border=1,
                 nthreads=4,
                 quad_decimate=1.0,
                 quad_blur=0.0,
                 refine_edges=True,
                 refine_decode=False,
                 refine_pose=False,
                 debug=False,
                 quad_contours=True):

        self.families = families
        self.border = int(border)

        self.nthreads = int(nthreads)
        self.quad_decimate = float(quad_decimate)
        self.quad_sigma = float(quad_blur)
        self.refine_edges = int(refine_edges)
        self.refine_decode = int(refine_decode)
        self.refine_pose = int(refine_pose)
        self.debug = int(debug)
        self.quad_contours = quad_contours

######################################################################

def add_arguments(parser):

    '''Add arguments to the given argparse.ArgumentParser object to enable
passing in the resulting parsed arguments into the initializer for
Detector.

    '''

    defaults = DetectorOptions()

    show_default = ' (default %(default)s)'

    parser.add_argument('-f', metavar='FAMILIES',
                        dest='families', default=defaults.families,
                        help='Tag families' + show_default)

    parser.add_argument('-B', metavar='N',
                        dest='border', type=int, default=defaults.border,
                        help='Tag border size in pixels' + show_default)

    parser.add_argument('-t', metavar='N',
                        dest='nthreads', type=int, default=defaults.nthreads,
                        help='Number of threads' + show_default)

    parser.add_argument('-x', metavar='SCALE',
                        dest='quad_decimate', type=float,
                        default=defaults.quad_decimate,
                        help='Quad decimation factor' + show_default)

    parser.add_argument('-b', metavar='SIGMA',
                        dest='quad_sigma', type=float, default=defaults.quad_sigma,
                        help='Apply low-pass blur to input' + show_default)

    parser.add_argument('-0', dest='refine_edges', default=True,
                        action='store_false',
                        help='Spend less time trying to align edges of tags')

    parser.add_argument('-1', dest='refine_decode', default=False,
                        action='store_true',
                        help='Spend more time trying to decode tags')

    parser.add_argument('-2', dest='refine_pose', default=False,
                        action='store_true',
                        help='Spend more time trying to precisely localize tags')

    parser.add_argument('-c', dest='quad_contours', default=False,
                        action='store_true',
                        help='Use new contour-based quad detection')


######################################################################

class Detector(object):

    '''Pythonic wrapper for apriltag_detector. Initialize by passing in
the output of an argparse.ArgumentParser on which you have called
add_arguments; or an instance of the DetectorOptions class.  You can
also optionally pass in a list of paths to search for the C dynamic
library used by ctypes.

    '''

    def __init__(self, options=None, searchpath=[]):

        if options is None:
            options = DetectorOptions()

        self.options = options

        # detect OS to get extension for DLL
        uname0 = os.uname()[0]
        if uname0 == 'Darwin':
            extension = '.dylib'
        else:
            extension = '.so' # TODO test on windows?

        filename = 'libapriltag'+extension

        self.libc = None
        self.tag_detector = None

        for path in searchpath:
            relpath = os.path.join(path, filename)
            if os.path.exists(relpath):
                self.libc = ctypes.CDLL(relpath)
                break

        # if full path not found just try opening the raw filename;
        # this should search whatever paths dlopen is supposed to
        # search.
        if self.libc is None:
            self.libc = ctypes.CDLL(filename)

        if self.libc is None:
            raise RuntimeError('could not find DLL named ' + filename)

        # declare return types of libc function
        self._declare_return_types()

        # create the c-_apriltag_detector object
        self.tag_detector = self.libc.apriltag_detector_create()
        self.tag_detector.contents.nthreads = int(options.nthreads)
        self.tag_detector.contents.quad_decimate = float(options.quad_decimate)
        self.tag_detector.contents.quad_sigma = float(options.quad_sigma)
        self.tag_detector.refine_edges = int(options.refine_edges)
        self.tag_detector.refine_decode = int(options.refine_decode)
        self.tag_detector.refine_pose = int(options.refine_pose)

        if options.quad_contours:
            self.libc.apriltag_detector_enable_quad_contours(self.tag_detector, 1)

        self.families = []

        flist = self.libc.apriltag_family_list()

        for i in range(flist.contents.size):
            ptr = ctypes.c_char_p()
            self.libc.zarray_get(flist, i, ctypes.byref(ptr))
            self.families.append(ctypes.string_at(ptr))

        self.libc.apriltag_family_list_destroy(flist)

        if options.families == 'all':
            families_list = self.families
        elif isinstance(options.families, list):
            families_list = options.families
        else:
            families_list = [n for n in re.split(r'\W+', options.families) if n]

        # add tags
        for family in families_list:
            self.add_tag_family(family)

    def __del__(self):
        if self.tag_detector is not None:
            self.libc.apriltag_detector_destroy(self.tag_detector)

    def detect(self, img, return_image=False):

        '''Run detectons on the provided image. The image must be a grayscale
image of type numpy.uint8.'''

        assert len(img.shape) == 2
        assert img.dtype == numpy.uint8

        c_img = self._convert_image(img)

        return_info = []

        #detect apriltags in the image
        detections = self.libc.apriltag_detector_detect(self.tag_detector, c_img)

        apriltag = ctypes.POINTER(_ApriltagDetection)()
        
        for i in range(0, detections.contents.size):

            #extract the data for each apriltag that was identified
            self.libc.zarray_get(detections, i, ctypes.byref(apriltag))

            tag = apriltag.contents

            homography = _matd_get_array(tag.H).copy()
            center = numpy.ctypeslib.as_array(tag.c, shape=(2,)).copy()
            corners = numpy.ctypeslib.as_array(tag.p, shape=(4, 2)).copy()

            detection = Detection(
                ctypes.string_at(tag.family.contents.name),
                tag.id,
                tag.hamming,
                tag.goodness,
                tag.decision_margin,
                homography,
                center,
                corners)

            #Append this dict to the tag data array
            return_info.append(detection)

        self.libc.image_u8_destroy(c_img)

        if return_image:

            dimg = self._vis_detections(img.shape, detections)
            rval = return_info, dimg

        else:

            rval = return_info

        self.libc.apriltag_detections_destroy(detections)

        return rval


    def add_tag_family(self, name):

        '''Add a single tag family to this detector.'''

        family = self.libc.apriltag_family_create(name.encode('ascii'))

        if family:
            family.contents.border = self.options.border
            self.libc.apriltag_detector_add_family(self.tag_detector, family)
        else:
            print('Unrecognized tag family name. Try e.g. tag36h11')

    def detection_pose(self, detection, camera_params, tag_size=1, z_sign=1):

        fx, fy, cx, cy = [ ctypes.c_double(c) for c in camera_params ]
        
        H = self.libc.matd_create(3, 3)
        arr = _matd_get_array(H)
        arr[:] = detection.homography
        corners = detection.corners.flatten().astype(numpy.float64)

        dptr = ctypes.POINTER(ctypes.c_double)

        corners = corners.ctypes.data_as(dptr)

        init_error = ctypes.c_double(0)
        final_error = ctypes.c_double(0)
        
        Mptr = self.libc.pose_from_homography(H, fx, fy, cx, cy,
                                              ctypes.c_double(tag_size),
                                              ctypes.c_double(z_sign),
                                              corners,
                                              dptr(init_error),
                                              dptr(final_error))

        M = _matd_get_array(Mptr).copy()
        self.libc.matd_destroy(H)
        self.libc.matd_destroy(Mptr)

        return M, init_error.value, final_error.value

    def _vis_detections(self, shape, detections):

        height, width = shape
        c_dimg = self.libc.image_u8_create(width, height)
        self.libc.apriltag_vis_detections(detections, c_dimg)
        tmp = _image_u8_get_array(c_dimg)

        rval = tmp[:, :width].copy()

        self.libc.image_u8_destroy(c_dimg)

        return rval

    def _declare_return_types(self):

        self.libc.apriltag_detector_create.restype = ctypes.POINTER(_ApriltagDetector)
        self.libc.apriltag_family_create.restype = ctypes.POINTER(_ApriltagFamily)
        self.libc.apriltag_detector_detect.restype = ctypes.POINTER(_ZArray)
        self.libc.image_u8_create.restype = ctypes.POINTER(_ImageU8)
        self.libc.image_u8_write_pnm.restype = ctypes.c_int
        self.libc.apriltag_family_list.restype = ctypes.POINTER(_ZArray)
        self.libc.apriltag_vis_detections.restype = None

        self.libc.pose_from_homography.restype = ctypes.POINTER(_Matd)
        self.libc.matd_create.restype = ctypes.POINTER(_Matd)

    def _convert_image(self, img):

        height = img.shape[0]
        width = img.shape[1]
        c_img = self.libc.image_u8_create(width, height)

        tmp = _image_u8_get_array(c_img)

        # copy the opencv image into the destination array, accounting for the
        # difference between stride & width.
        tmp[:, :width] = img

        # tmp goes out of scope here but we don't care because
        # the underlying data is still in c_img.
        return c_img

######################################################################

def _get_demo_searchpath():
    
    return [
        os.path.join(os.path.dirname(__file__), '../build/lib'),
        os.path.join(os.getcwd(), '../build/lib')
    ]

######################################################################

def _camera_params(pstr):
    
    pstr = pstr.strip()
    
    if pstr[0] == '(' and pstr[-1] == ')':
        pstr = pstr[1:-1]

    params = tuple( [ float(param.strip()) for param in pstr.split(',') ] )

    assert( len(params) ==  4)

    return params

######################################################################

def _draw_pose(overlay, camera_params, tag_size, pose, z_sign=1):

    opoints = numpy.array([
        -1, -1, 0,
         1, -1, 0,
         1,  1, 0,
        -1,  1, 0,
        -1, -1, -2*z_sign,
         1, -1, -2*z_sign,
         1,  1, -2*z_sign,
        -1,  1, -2*z_sign,
    ]).reshape(-1, 1, 3) * 0.5*tag_size

    edges = numpy.array([
        0, 1,
        1, 2,
        2, 3,
        3, 0,
        0, 4,
        1, 5,
        2, 6,
        3, 7,
        4, 5,
        5, 6,
        6, 7,
        7, 4
    ]).reshape(-1, 2)
        
    fx, fy, cx, cy = camera_params

    K = numpy.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

    rvec, _ = cv2.Rodrigues(pose[:3,:3])
    tvec = pose[:3, 3]

    dcoeffs = numpy.zeros(5)

    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

    ipoints = numpy.round(ipoints).astype(int)
    
    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

    for i, j in edges:
        cv2.line(overlay, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)

    

######################################################################

def main():

    '''Test function for this Python wrapper.'''

    from argparse import ArgumentParser

    # for some reason pylint complains about members being undefined :(
    # pylint: disable=E1101

    parser = ArgumentParser(
        description='test apriltag Python bindings')

    parser.add_argument('filenames', metavar='IMAGE', nargs='+',
                        help='files to scan')

    parser.add_argument('-n', '--no-gui', action='store_true',
                        help='suppress OpenCV gui')

    parser.add_argument('-d', '--debug-images', action='store_true',
                        help='output debug detection image')

    parser.add_argument('-k', '--camera-params', type=_camera_params,
                        default=None,
                        help='intrinsic parameters for camera (in the form fx,fy,cx,cy)')

    parser.add_argument('-s', '--tag-size', type=float,
                        default=1.0,
                        help='tag size in user-specified units (default=1.0)')

    add_arguments(parser)

    options = parser.parse_args()

    # set up a reasonable search path for the apriltag DLL inside the
    # github repo this file lives in;
    #
    # for "real" deployments, either install the DLL in the appropriate
    # system-wide library directory, or specify your own search paths
    # as needed.
    
    det = Detector(options, searchpath=_get_demo_searchpath())

    use_gui = not options.no_gui

    if use_gui and not _HAVE_CV2:
        use_gui = False
        print('suppressing GUI because cv2 module not found')

    if not _HAVE_CV2:
        from PIL import Image

    for filename in options.filenames:

        if _HAVE_CV2:
            orig = cv2.imread(filename)
            if len(orig.shape) == 3:
                gray = cv2.cvtColor(orig, cv2.COLOR_RGB2GRAY)
            else:
                gray = orig
        else:
            pil_image = Image.open(filename)
            orig = numpy.array(pil_image)
            gray = numpy.array(pil_image.convert('L'))

        detections, dimg = det.detect(gray, return_image=True)

        if len(orig.shape) == 3:
            overlay = orig // 2 + dimg[:, :, None] // 2
        else:
            overlay = gray // 2 + dimg // 2

        num_detections = len(detections)
        print('Detected {} tags in {}\n'.format(
            num_detections, os.path.split(filename)[1]))

        for i, detection in enumerate(detections):
            print( 'Detection {} of {}:'.format(i+1, num_detections))
            print()
            print(detection.tostring(indent=2))

            if options.camera_params is not None:
                
                pose, e0, e1 = det.detection_pose(detection,
                                                  options.camera_params,
                                                  options.tag_size)

                if _HAVE_CV2:
                    _draw_pose(overlay,
                               options.camera_params,
                               options.tag_size,
                               pose)
                
                print(detection.tostring(
                    collections.OrderedDict([('Pose',pose),
                                             ('InitError', e0),
                                             ('FinalError', e1)]),
                    indent=2))
                
            print()


        if options.debug_images:
            if _HAVE_CV2:
                cv2.imwrite('detections.png', overlay)
            else:
                output = Image.fromarray(overlay)
                output.save('detections.png')
            
        if use_gui:
            cv2.imshow('win', overlay)
            while cv2.waitKey(5) < 0:
                pass


if __name__ == '__main__':
    main()

