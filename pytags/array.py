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

# get the uint8 pointer from ctypes
buf = c_img.contents.buf

# wrap that pointer with a numpy array -- pointer stays pointing at
# the same memory, but now numpy can access it directly.
tmp = np.ctypeslib.as_array(buf, (c_img.contents.height,
                                  c_img.contents.stride))

# copy the opencv image into the destination array (note we still need
# to deal with difference between stride & width)
tmp[:, :width] = img

# make sure the image_u8 stuff works now
libc.image_u8_write_pnm(c_img, "foo.pnm")




