import numpy as np
import cv2
import pytag
import os
from ctypes import *
import ctype_structs as cts


def findPose1(infotag, img):
  '''
  Takes a homography from the apriltag function and finds 
  the associated pose. What is our best guess for where the robot
  is relative to the apriltag that was found?
  '''
  print infotag.data[0]['homog']
  i = 0 #later we'll actually iterate. for now just testing
  homog = infotag.data[i]['homog']
  corners = infotag.data[i]['corners']

  ideal_tag = np.array([[[-1], [-1], [ 1]],
                        [[ 1], [-1], [ 1]],
                        [[ 1], [ 1], [ 1]],
                        [[-1], [ 1], [ 1]]], dtype=float)

  corners = np.array(infotag.data[i]['corners'], dtype =float).reshape((4,2,1))

  cameraMatrix = np.eye(3)
  distCoeffs = np.zeros((5,1))


  ret, rot, trans = cv2.solvePnP(ideal_tag, corners, cameraMatrix, distCoeffs)
  mat, _ = cv2.Rodrigues(rot)
  #print np.hstack([mat, trans])
  print cv2.findHomography(corners, ideal_tag)

def findPose2(infotag, img):

  uname0 = os.uname()[0]
  if uname0 == 'Darwin':
    extension = '.dylib'
  else:
    extension = '.so' 
  libc = CDLL('./../build/lib/libapriltag'+extension)
  libc.homography_to_pose.restype = POINTER(cts.matd_4)
  
  '''
  corners = np.array(infotag.data[i]['corners'], dtype =float).reshape((4,2,1))
  print corners
  H = infotag.data[0]['homog']
  new_corners = []
  new_corners.append()
  '''

  mat = libc.homography_to_pose(infotag.raw[0].H, c_double(1), c_double(1), c_double(0), c_double(0))

  mat_arr = np.array(mat.contents.data)
  print mat_arr


if __name__ == '__main__':
  img = cv2.imread('sample_pic.png')
  det = pytag.detector()
  infotag = det.detect(img)
  #infotag.print_info()

  findPose1(infotag, img)
  findPose2(infotag, img)
