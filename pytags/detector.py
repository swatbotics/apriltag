from ctypes import *
import ctype_structs as cts
import cv2
import numpy as np
import os

uname0 = os.uname()[0]

if uname0 == 'Darwin':
  extension = '.dylib'
else:
  extension = '.so' # TODO test on windows?


libc = CDLL('./../build/lib/libapriltag'+extension)

class pytag_info:
  def __init__(self):
    self.data = []
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

    image = self.img[:]

    for datum in self.data:
      lines = np.array(datum['corners'])
      lines.reshape(-1,1,2)
      cv2.polylines(image, [lines], True, (0, 0, 255),5)

    cv2.imshow('Image', image)
    cv2.waitKey(0)


class pytags_detector:
  def __init__(self):
    #Declare return types
    self.declareReturnTypes()

    #Create the detector and the empty image variable
    self.tag_detector = libc.apriltag_detector_create()
    self.c_img = None
    self.img = None

  def declareReturnTypes(self):
    #Declare return type for detector constructor
    libc.apriltag_detector_create.restype = POINTER(cts.apriltag_detector)

    #declare return type for tag family constructors
    libc.tag36h11_create.restype = POINTER(cts.apriltag_family)
    libc.tag16h5_create.restype = POINTER(cts.apriltag_family)
    libc.tag25h7_create.restype = POINTER(cts.apriltag_family)
    libc.tag25h9_create.restype = POINTER(cts.apriltag_family)
    libc.tag36h10_create.restype = POINTER(cts.apriltag_family)

    #declare return type for detection
    libc.apriltag_detector_detect.restype = POINTER(cts.zarray)

    #declare return type for image construction
    libc.image_u8_create.restype = POINTER(cts.image_u8)


  def add_tag_family(self, name):
    
    if name == 'tag16h5':
      tag_family = libc.tag16h5_create()
    elif name == 'tag25h7':
      tag_family = libc.tag25h7_create()
    elif name == 'tag25h9':
      tag_family = libc.tag25h9_create()
    elif name == 'tag36h10':
      tag_family = libc.tag35h10_create()
    elif name == 'tag36h11':
      tag_family = libc.tag36h11_create()
    else:
      print "Unrecognized tag family name."
      return 1
    
    #add the family to the detector
    libc.apriltag_detector_add_family(self.tag_detector, tag_family)


  def load_image_from_file(self, filename):
    img = cv2.imread(filename)
    self.load_image(img)
    return 0

  def load_image(self, orig_img):
    self.img = orig_img

    img = cv2.cvtColor(orig_img, cv2.COLOR_RGB2GRAY)

    height = img.shape[0]
    width = img.shape[1]
    c_img = libc.image_u8_create(width, height)

    # wrap that pointer with a numpy array -- pointer stays pointing at
    # the same memory, but now numpy can access it directly.
    tmp = np.ctypeslib.as_array(c_img.contents.buf,
                                (c_img.contents.height,
                                 c_img.contents.stride))

    # copy the opencv image into the destination array (note we still need
    # to deal with difference between stride & width)
    tmp[:, :width] = img

    # tmp goes out of scope here but we don't care because
    # the underlying data is still in c_img.

    self.c_img = c_img

  def process(self):
    
    if not self.c_img:
      raise Exception("The image was not successfully loaded.")

    #detect apriltags in the image
    detections = libc.apriltag_detector_detect(self.tag_detector, self.c_img)
    
    #create a pytags_info object
    tag_info = pytag_info()
    for i in range(0, detections.contents.size):
      #extract the data for each apriltag that was identified
      tag = POINTER(cts.apriltag_detection)()
      libc.zarray_get(detections, i, byref(tag))
      tag = tag.contents
      
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
      tag_info.data.append(new_info)
      tag_info.img = self.img

    return tag_info


if __name__ == '__main__':
  detector = pytags_detector()
  img = cv2.imread('./sample_pic.png')

  detector.load_image(img)
  detector.add_tag_family("tag36h11")
  detector.add_tag_family("tag25h9") 
  tag_info = detector.process()
  
  tag_info.print_info()
  tag_info.show_tags()
