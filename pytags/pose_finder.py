import numpy as np
import cv2
import pytag
import os
from ctypes import *
import ctype_structs as cts


def findPose1(intrinsics, distCoeffs, infotag, img):
  '''
  Takes a homography from the apriltag function and finds 
  the associated pose. What is our best guess for where the robot
  is relative to the apriltag that was found?
  '''
  #print infotag.data[0]['homog']
  i = 0 #later we'll actually iterate. for now just testing
  homog = infotag.data[i]['homog']
  corners = infotag.data[i]['corners']

  ideal_tag = np.array([[[-6.125], [-6.125], [ 1]],
                        [[ 6.125], [-6.125], [ 1]],
                        [[ 6.125], [ 6.125], [ 1]],
                        [[-6.125], [ 6.125], [ 1]]], dtype=float)

  corners = np.array(infotag.data[i]['corners'], dtype =float).reshape((4,2,1))

  

  ret, rot, trans = cv2.solvePnP(ideal_tag, corners, intrinsics, distCoeffs)
  mat, _ = cv2.Rodrigues(rot)
  return rot, trans
 

def findPose2(intrinsics, dist, infotag, img):

  uname0 = os.uname()[0]
  if uname0 == 'Darwin':
    extension = '.dylib'
  else:
    extension = '.so' 
  #libc = CDLL('./../build/lib/libapriltag'+extension)
  libc = CDLL('/home/team2/c-apriltag/build/lib/libapriltag'+extension)
  libc.homography_to_pose.restype = POINTER(cts.matd_4)
  
  '''
  corners = np.array(infotag.data[i]['corners'], dtype =float).reshape((4,2,1))
  print corners
  H = infotag.data[0]['homog']
  new_corners = []
  new_corners.append()
  '''

  mat = libc.homography_to_pose(infotag.raw[0].H,
				c_double(intrinsics[0,0]), 
				c_double(intrinsics[1,1]), 
				c_double(intrinsics[0,2]), 
				c_double(intrinsics[1,2]))

  mat_arr = np.array(mat.contents.data,dtype=float)
  mat_arr = 6.125*np.linalg.inv(mat_arr)
  rot, _ = cv2.Rodrigues(mat_arr[0:3,0:3])
  trans = mat_arr[0:3,3]
  return rot, trans


if __name__ == '__main__':
  img = cv2.imread('sample_pic.png')
  det = pytag.detector()
  infotag = det.detect(img)
  #infotag.print_info()

  findPose1(infotag, img)
  findPose2(infotag, img)
