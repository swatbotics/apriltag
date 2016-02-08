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

img = cv2.imread('sample_pic.png')
img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

height, width = img.shape

libc.image_u8_create.restype = POINTER(cts.image_u8)
c_img = libc.image_u8_create(width, height)

buf = c_img.contents.buf

tmp = np.ctypeslib.as_array(buf, (c_img.contents.height,
                                c_img.contents.stride))

tmp[:, :width] = img

libc.image_u8_write_pnm(c_img, "foo.pnm")




