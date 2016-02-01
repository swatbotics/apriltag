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
    image = cv2.imread(self.img)
    cv2.imshow('Image', image)
    cv2.waitKey(0)

    for datum in self.data:
      lines = np.array(datum['corners'])
      lines.reshape(-1,1,2)
      cv2.polylines(image, [lines], True, (0, 0, 255),5)
    cv2.imshow('Image', image)

    cv2.waitKey(0)


class pytags_detector:
  def __init__(self):
    #Declare return types
    libc.apriltag_detector_create.restype = POINTER(cts.apriltag_detector)
    
    #Right now we're hardcoding which tag family we're using.
    libc.tag36h11_create.restype = POINTER(cts.apriltag_family)
    libc.apriltag_detector_detect.restype = POINTER(cts.zarray)
    libc.image_u8_create_from_pnm.restype = POINTER(cts.image_u8)
    libc.tag_create_by_name.restype = POINTER(cts.apriltag_family)
    
    self.tag_detector = libc.apriltag_detector_create()
    self.img = None
    self.img_file = None


  def add_tag_family(self, name):
    #Right now this is hardcoded for tag36h11. Can change easily once I know how 
    #tag families work.
    #tag_family = libc.tag36h11_create()
    tag_family = libc.tag_create_by_name(name)
    libc.apriltag_detector_add_family(self.tag_detector, tag_family)

  def load_pnm_image(self, filename):
    self.img_file = filename
    self.img = libc.image_u8_create_from_pnm(filename)

  def process(self):
    #detect apriltags in the image
    detections = libc.apriltag_detector_detect(self.tag_detector, self.img)
    
    #create a pytags_info object
    tag_info = pytag_info()
    for i in range(0, detections.contents.size):
      tag = POINTER(cts.apriltag_detection)()
      libc.zarray_get(detections, i, byref(tag))
      tag = tag.contents
      
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
      tag_info.img = self.img_file

    return tag_info




if __name__ == '__main__':
  detector = pytags_detector()
  detector.load_pnm_image("./sample_pic.pnm")
  detector.add_tag_family("tag36h11") #Do we want this to be variable? Easy to do.
  detector.add_tag_family("tag25h9") 
  tag_info = detector.process()
  tag_info.print_info()
  tag_info.show_tags()
