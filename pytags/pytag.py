#!/usr/bin/env python
"""Python wrapper for C version of apriltags. This program creates two
classes that are used to detect apriltags and extract information from
them. Using this module, you can identify all apriltags visible in an
image, and get information about the location and orientation of the
tags.

Original author: Isaac Dulin, Spring 2016
Updates: Matt Zucker, Fall 2016

"""

import ctypes
import collections
import os
import re
import numpy
from PIL import Image

######################################################################

class _image_u8(ctypes.Structure):
  _fields_ = [
    ('width', ctypes.c_int),
    ('height', ctypes.c_int),
    ('stride', ctypes.c_int),
    ('buf', ctypes.POINTER(ctypes.c_uint8))
  ]

class _image_u32(ctypes.Structure):
  _fields_ = [
    ('width', ctypes.c_int),
    ('height', ctypes.c_int),
    ('stride', ctypes.c_int),
    ('buf', ctypes.POINTER(ctypes.c_uint32))
  ]
  
class _apriltag_family(ctypes.Structure):
  _fields_ = [
    ('ncodes', ctypes.c_int32),
    ('codes', ctypes.POINTER(ctypes.c_int64)),
    ('black_border', ctypes.c_int32),
    ('d', ctypes.c_int32),
    ('h', ctypes.c_int32),
    ('name', ctypes.c_char_p),
  ]

class _matd(ctypes.Structure):
  _fields_ = [
    ('nrows', ctypes.c_int),
    ('ncols', ctypes.c_int),
    ('data', ctypes.c_double*1),
  ]  

class _apriltag_detection(ctypes.Structure):
  _fields_ = [
    ('family', ctypes.POINTER(_apriltag_family)),
    ('id', ctypes.c_int),
    ('hamming', ctypes.c_int),
    ('goodness', ctypes.c_float),
    ('decision_margin', ctypes.c_float),
    ('H', ctypes.POINTER(_matd)),
    ('c', ctypes.c_double*2),
    ('p', (ctypes.c_double*2)*4)
  ]

class _zarray(ctypes.Structure):
  _fields_ = [
    ('el_sz', ctypes.c_size_t),
    ('size', ctypes.c_int),
    ('alloc', ctypes.c_int),
    ('data', ctypes.c_void_p)
  ]
  
class _apriltag_detector(ctypes.Structure):
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
  return numpy.ctypeslib.as_array(array_buf, shape=(rows,cols))

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
  'raw, tag_family, tag_id, hamming, goodness, decision_margin, '
  'homography, center, corners')

class Detection(DetectionBase):

  _print_fields = [
    'Family', 'ID', 'Hamming error', 'Goodness',
    'Decision margin', 'Homography', 'Center', 'Corners'
  ]

  _max_len = max(len(field) for field in _print_fields)

  def tostring(self, indent=0):

    rval = []
    indent_str = ' '*(self._max_len+2+indent)

    for i, label in enumerate(self._print_fields):
      
      value = str(self[i+1])
      
      if value.find('\n') > 0:
        value = value.split('\n')
        value = [value[0]] + [indent_str+v for v in value[1:]]
        value = '\n'.join(value)
      
      rval.append('{:>{}s}: {}'.format(
        label, self._max_len+indent, value))

    return '\n'.join(rval)

  def __str__(self):
    return self.tostring()

######################################################################
  
class DetectorOptions:

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
               
class Detector:
  
  def __init__(self, options=None):

    if options is None:
      options = DetectorOptions()

    self.options = options
      

    # detect OS to get extension for DLL
    uname0 = os.uname()[0]
    if uname0 == 'Darwin':
      extension = '.dylib'
    else:
      extension = '.so' # TODO test on windows?

    # load the C library and store it as a class variable
    self.libc = ctypes.CDLL('./../build/lib/libapriltag'+extension)

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

    if options.families == 'all':
      families = self.families
    elif isinstance(options.families, list):
      families = options.families
    else:
      families = [n for n in re.split(r'\W+', options.families) if n]

    # add tags
    for family in families:
        self.add_tag_family(family)

  def detect(self, img):
    
    c_img = self._convert_image(img)
    
    #detect apriltags in the image
    detections = self.libc.apriltag_detector_detect(self.tag_detector, c_img)
    
    #create a pytags_info object
    return_info = []

    for i in range(0, detections.contents.size):

      #extract the data for each apriltag that was identified
      apriltag = ctypes.POINTER(_apriltag_detection)()
      self.libc.zarray_get(detections, i, ctypes.byref(apriltag))
      
      tag = apriltag.contents

      homography = _matd_get_array(tag.H).copy()
      center = numpy.ctypeslib.as_array(tag.c, shape=(2,)).copy()
      corners = numpy.ctypeslib.as_array(tag.p, shape=(4,2)).copy()

      d = Detection(
        apriltag,
        tag.family.contents.name,
        tag.id,
        tag.hamming,
        tag.goodness,
        tag.decision_margin,
        homography,
        center,
        corners)
      
      #Append this dict to the tag data array
      return_info.append(d)
      
    return return_info


  def detection_image(self, shape, detections):

    height, width = shape[:2]
    c_img = self.libc.image_u8_create(width, height)

    for d in detections:
      self.libc.apriltag_vis_rasterize(d.raw, c_img)

    tmp = _image_u8_get_array(c_img)

    return tmp[:, :width].copy()

  def add_tag_family(self, name):

    family = self.libc.apriltag_family_create(name)

    if family:
      family.contents.border = self.options.border
      self.libc.apriltag_detector_add_family(self.tag_detector, family)
    else:
      print 'Unrecognized tag family name. Try e.g. tag36h11'

  def _declare_return_types(self):
    #Declare return type for detector constructor
    self.libc.apriltag_detector_create.restype = ctypes.POINTER(_apriltag_detector)

    self.libc.apriltag_family_create.restype = ctypes.POINTER(_apriltag_family)

    #declare return type for detection
    self.libc.apriltag_detector_detect.restype = ctypes.POINTER(_zarray)

    #declare return type for image construction
    self.libc.image_u8_create.restype = ctypes.POINTER(_image_u8)

    self.libc.image_u8_write_pnm.restype = ctypes.c_int

    self.libc.apriltag_family_list.restype = ctypes.POINTER(_zarray)

    self.libc.apriltag_vis_rasterize.restype = None

  def _convert_image(self, img):
    
    assert len(img.shape) == 2
    assert img.dtype == numpy.uint8

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

def main():

  import sys

  try:
    import cv2
    HAVE_CV2 = True
  except ImportError:
    HAVE_CV2 = False

  HAVE_CV2 = False

  if len(sys.argv) > 1:
    if HAVE_CV2:
      orig = cv2.imread(sys.argv[1])
      if len(orig.shape) == 3:
        gray = cv2.cvtColor(orig, cv2.COLOR_RGB2GRAY)
      else:
        gray = orig
    else:
      pil_image = Image.open(sys.argv[1])
      orig = numpy.array(pil_image)
      gray = numpy.array(pil_image.convert('L'))
  else:
    print 'usage: {} IMAGE.png'
    sys.exit(0)

  options = DetectorOptions(families='tag36h11',
                            quad_contours=True)
  
  det = Detector(options)

  detections = det.detect(gray)

  num_detections = len(detections)
  print 'Detected {} tags.\n'.format(num_detections)

  for i, d in enumerate(detections):
    print 'Detection {} of {}:'.format(i+1, num_detections)
    print
    print d.tostring(indent=2)
    print

  dimg = det.detection_image(gray.shape, detections)

  if len(orig.shape) == 3:
    dimg_color = dimg[:, :, None]
    overlay = orig / 2 + dimg_color / 2
  else:
    overlay = gray / 2 + dimg / 2

  if HAVE_CV2:
    cv2.imshow('win', overlay)
    while cv2.waitKey(5) < 0:
      pass
  else:
    output = Image.fromarray(overlay)
    output.save('detections.png')
    
if __name__ == '__main__':

  main()

  
