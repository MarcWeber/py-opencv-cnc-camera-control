import math
import scipy
from random import randrange
import sys
import time
import numpy as np
from numpy.linalg import eig

import imreg_dft
import cv2 as cv
import cv2 as cv2
from lib.center_utils import center_by_homography, center_by_regression, rotation_deg
from lib.numpyutils import rotation_matrix_2d

from lib.opencvutils import cross, resize

img1 = cv.imread("images-10/img1.jpg")
img2 = cv.imread("images-10/img2.jpg")

print("this is fine, x/y/scaling")
c1= center_by_regression(img1, img2)
print("%.2f %.2f" % (c1[0], c1[1]))

print("center_by_homography 300px wrong x direction 1082 vs 1416")
c2 = center_by_homography(img1, img2)
print("%.2f %.2f" % (c2[0], c2[1]))