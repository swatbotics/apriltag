from ctypes import *
import ctype_structs as cts
import cv2
import numpy as np

libc = CDLL('./../build/lib/libapriltag.so')



class pytag_info:
  def __init__(self):
    self.data = []
    self.img = './sample_pic3.pnm'
  
  def print_info(self):
    for i, datum in enumerate(self.data):
      print 'Tag Number', i
      print 'ID:', datum['tag_id']
      print 'Family:', datum['tag_family']
      print 'Center:', datum['center']
      print 'Hamming:', datum['hamming']
      print 'Goodness:', datum['goodness']
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
    
    self.tag_detector = libc.apriltag_detector_create()
    self.img = None


  def add_tag_family(self):
    #Right now this is hardcoded for tag36h11. Can change easily once I know how 
    #tag families work.
    tag_family = libc.tag36h11_create()
    libc.apriltag_detector_add_family(self.tag_detector, tag_family)

  def load_image(self, filename):
    self.img = libc.image_u8_create_from_pnm(filename)

  def use_numpy_image(self, img):
    self.img = np.ctypes(img)

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
      new_info['homog'] = tag.H #TODO// This is the pointer to the matd_t.
      new_info['center'] = (int(tag.c[0]), int(tag.c[1])) #Center of the tag (tuple)
      new_info['corners'] = [(int(tag.p[0][0]), int(tag.p[0][1])), #Corners of the tag
                             (int(tag.p[1][0]), int(tag.p[1][1])),
                             (int(tag.p[2][0]), int(tag.p[2][1])),
                             (int(tag.p[3][0]), int(tag.p[3][1]))]
      #Append this dict to the tag data array
      tag_info.data.append(new_info)

    return tag_info




if __name__ == '__main__':
  detector = pytags_detector()
  detector.load_image("./sample_pic3.pnm")
  detector.add_tag_family() #Do we want this to be variable? Easy to do.
  tag_info = detector.process()
  tag_info.print_info()
  tag_info.show_tags()


  '''
  #Declare some return types 
  #Declare return types
  libc.apriltag_detector_create.restype = POINTER(cts.apriltag_detector)
  libc.tag36h11_create.restype = POINTER(cts.apriltag_family)
  libc.apriltag_detector_detect.restype = POINTER(cts.zarray_det)
  libc.image_u8_create_from_pnm.restype = POINTER(cts.image_u8)

  td = libc.apriltag_detector_create()
  tf = libc.tag36h11_create()

  libc.apriltag_detector_add_family(td, tf)
  libc.apriltag_detector_add_family(td, tf2)
  img = libc.image_u8_create_from_pnm("./sample_pic3.pnm")

  detection = libc.apriltag_detector_detect(td, img)
  #libc.refine_edges("nothin")
  print "detected"


  for i in range(0, detection.contents.size):
    det = POINTER(cts.apriltag_detection)()
    libc.zarray_get(detection, i, byref(det))
    print "For tag", i 
    print "ID:", det.contents.id
    print "Hamming:", det.contents.hamming
    print "Center:", det.contents.c[0], det.contents.c[1]
    print "family:", det.contents.family.contents.name
    print "points:", det.contents.p[0], det.contents.p[1]
  '''