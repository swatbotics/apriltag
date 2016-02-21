from ctypes import *
import cv2
import numpy as np


class image_u8(Structure):
  _fields_ = [("width", c_int),
              ("height", c_int),
              ("stride", c_int),
              ("buf", POINTER(c_uint8))]


class apriltag_family(Structure):
  _fields_ = [("ncodes", c_int32),
              ("codes", POINTER(c_int64)),
              ("black_border", c_int32),
              ("d", c_int32),
              ("h", c_int32),
              ("name", c_char_p),
              ("impl", c_void_p)]

class matd_3(Structure):
  _fields_ = [("rows", c_int),
              ("cols", c_int),
              ("data", (c_double*3)*3)]

class matd_4(Structure):
  _fields_ = [("rows", c_int),
              ("cols", c_int),
              ("data", (c_double*4)*4)]


class apriltag_detection(Structure):
  _fields_ = [("family", POINTER(apriltag_family)),
              ("id", c_int),
              ("hamming", c_int),
              ("goodness", c_float),
              ("decision_margin", c_float),
              ("H", POINTER(matd_3)),
              ("c", c_double*2),
              ("p", (c_double*2)*4)]

class zarray(Structure):
  _fields_ = [("el_sz", c_size_t),
              ("size", c_int),
              ("alloc", c_int),
              ("data", c_void_p)]


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
