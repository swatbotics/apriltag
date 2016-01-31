from ctypes import *
import cv2
import numpy as np


class image_u8(Structure):
  _fields_ = [("width", c_int),
              ("height", c_int),
              ("stride", c_int),
              ("buf", c_int)]


class apriltag_family(Structure):
  _fields_ = [("ncodes", c_int32),
              ("codes", POINTER(c_int64)),
              ("black_border", c_int32),
              ("d", c_int32),
              ("h", c_int32),
              ("name", c_char_p),
              ("impl", c_void_p)
             ]

class matd_t(Structure):
  _fields_ = [("rows", c_int),
              ("cols", c_int),
              ("data", (c_double*3)*3)]


class apriltag_detection(Structure):
  _fields_ = [("family", POINTER(apriltag_family)),
              ("id", c_int),
              ("hamming", c_int),
              ("goodness", c_float),
              ("decision_margin", c_float),
              ("H", POINTER(matd_t)),
              ("c", c_double*2),
              ("p", (c_double*2)*4)
              ]

class zarray(Structure):
  _fields_ = [("el_sz", c_size_t),
              ("size", c_int),
              ("alloc", c_int),
              ("data", c_void_p)]


#The class zarray is unspecific in defining what it
#is storing. Python needs to know every sublevel stored
#datatype, so zarray_det is a zarray storing detection 
#objects and zarray_fam is a zarray storing tag_families
class zarray_det(Structure):
  _fields_ = [("el_sz", c_size_t),
              ("size", c_int),
              ("alloc", c_int),
              ("data", c_char_p)]


class zarray_fam(Structure):
  _fields_ = [("el_sz", c_size_t),
              ("size", c_int),
              ("alloc", c_int),
              ("data", c_char_p)]



class apriltag_detector(Structure):
  _fields_ = [("nthreads", c_int),
              ("quad_decimate", c_float),
              ("quad_sigma", c_float),
              ("refine_edges", c_int),
              ("refine_decode", c_int),
              ("refine_pose", c_int),
              ("debug", c_int),
              ("quad_contours", c_int),
              #union
              #timeprofile_t
              ("nedges", c_int),
              ("nsegments", c_int),
              ("nquads", c_int),
              ("tag_families", POINTER(zarray))
              #workerpool
              #pthread mutex
              ]
