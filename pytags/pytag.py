from ctypes import *
import ctype_structs as cts
import cv2
import numpy as np
import sys
import os
import copy


class tag_info:
  def __init__(self):
    self.data = []
    self.raw = []
    self.img = None
  
  def print_info(self):
    for i, datum in enumerate(self.data):
      print 'Tag Number', i
      print 'ID:', datum['tag_id']
      print 'Family:', datum['tag_family']
      print 'Center:', datum['center']
      print 'Hamming:', datum['hamming']
      print 'Goodness:', datum['goodness']
      print 'Homography:\n', datum['homog']
      print 'Corners:', datum['corners'], '\n'

    print len(self.data), "tags found"

  def show_tags(self):
    cv2.imshow('Image', self.img)
    cv2.waitKey(0)

    image = self.get_image()

    cv2.imshow('Image', image)
    cv2.waitKey(0)

  def get_image(self):
    image = copy.copy(self.img)
    for datum in self.data:
      lines = np.array(datum['corners'])
      lines.reshape(-1,1,2)
      cv2.polylines(image, [lines], True, (0, 0, 255), 5)

    return image


class detector:
  def __init__(self, tag_name="all"):

    uname0 = os.uname()[0]
    if uname0 == 'Darwin':
      extension = '.dylib'
    else:
      extension = '.so' # TODO test on windows?

    #load the c library and store it as a class variable
    #self.libc = CDLL('./../build/lib/libapriltag'+extension)
    self.libc = CDLL('/home/team2/c-apriltag/build/lib/libapriltag'+extension)

    #Declare return types of libc function
    self._declare_return_types()

    #Create the c-apriltag detector object
    self.tag_detector = self.libc.apriltag_detector_create()
    #self.libc.apriltag_detector_enable_quad_contours(self.tag_detector, 1)

    #Add tags. 
    self.add_tag_family(tag_name)


  def detect(self, img, threshold=1):
    c_img = self._convert_image(img)
    
    #detect apriltags in the image
    detections = self.libc.apriltag_detector_detect(self.tag_detector, c_img)
    
    #create a pytags_info object
    return_info = tag_info()
    return_info.img = img
    for i in range(0, detections.contents.size):
      #extract the data for each apriltag that was identified
      apriltag = POINTER(cts.apriltag_detection)()
      self.libc.zarray_get(detections, i, byref(apriltag))
      tag = apriltag.contents
      if tag.hamming >= threshold:
        continue

      #write the data from the apriltag_detection object to our pytag object
      new_info = {}
      new_info['tag_family'] = tag.family.contents.name #Just the name of the apriltag_family
      new_info['tag_id'] = tag.id #code ID
      new_info['hamming'] = tag.hamming #somethin'
      new_info['goodness'] = tag.goodness #somethin'
      new_info['decision_margin'] = tag.decision_margin #somethin'
      new_info['homog'] = np.array(tag.H.contents.data)
      new_info['center'] = (int(tag.c[0]), int(tag.c[1])) #Center of the tag (tuple)
      new_info['corners'] = [(int(tag.p[0][0]), int(tag.p[0][1])), #Corners of the tag
                             (int(tag.p[1][0]), int(tag.p[1][1])),
                             (int(tag.p[2][0]), int(tag.p[2][1])),
                             (int(tag.p[3][0]), int(tag.p[3][1]))]

      #Append this dict to the tag data array
      return_info.data.append(new_info)
      return_info.raw.append(tag)
    
    return return_info


  def add_tag_family(self, name):
    tag_dict = {'tag16h5' : self.libc.tag16h5_create,
                'tag25h7' : self.libc.tag25h7_create,
                'tag25h9' : self.libc.tag25h9_create,
                'tag36h10': self.libc.tag36h10_create,
                'tag36h11': self.libc.tag36h11_create}

    if name in tag_dict:
      self.libc.apriltag_detector_add_family(self.tag_detector, tag_dict[name]())
    elif name=='all':
      for tag in tag_dict:
        self.libc.apriltag_detector_add_family(self.tag_detector, tag_dict[tag]())
    else:
      print "Unreecognized tag family name. Options are \'all\' or any tag name."
      sys.exit(1)


  def _declare_return_types(self):
    #Declare return type for detector constructor
    self.libc.apriltag_detector_create.restype = POINTER(cts.apriltag_detector)

    #declare return type for tag family constructors
    self.libc.tag36h11_create.restype = POINTER(cts.apriltag_family)
    self.libc.tag16h5_create.restype = POINTER(cts.apriltag_family)
    self.libc.tag25h7_create.restype = POINTER(cts.apriltag_family)
    self.libc.tag25h9_create.restype = POINTER(cts.apriltag_family)
    self.libc.tag36h10_create.restype = POINTER(cts.apriltag_family)

    #declare return type for detection
    self.libc.apriltag_detector_detect.restype = POINTER(cts.zarray)

    #declare return type for image construction
    self.libc.image_u8_create.restype = POINTER(cts.image_u8)

  def _convert_image(self, orig_img):
    self.img = orig_img

    img = cv2.cvtColor(orig_img, cv2.COLOR_RGB2GRAY)

    height = img.shape[0]
    width = img.shape[1]
    c_img = self.libc.image_u8_create(width, height)

    # wrap that pointer with a numpy array -- pointer stays pointing at
    # the same memory, but now numpy can access it directly.
    tmp = np.ctypeslib.as_array(c_img.contents.buf,
                               (c_img.contents.height,
                                c_img.contents.stride))

    # copy the opencv image into the destination array, accounting for the 
    # difference between stride & width.
    tmp[:, :width] = img

    # tmp goes out of scope here but we don't care because
    # the underlying data is still in c_img.
    return c_img



if __name__ == '__main__':
  img = cv2.imread('./diabolic_image_orig.jpg')

  det = detector()
  info = det.detect(img, 2)
  
  info.print_info()
  info.show_tags()
