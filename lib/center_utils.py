import cv2 as cv
import scipy
import math
import numpy as np
from lib.numpyutils import rotation_matrix_2d, rotation_matrix_2d_x, vector_intersect, vectors_angle
from lib.opencvutils import CameraThread, ImageDR, cross, line_rounded

from machines.DummyMachine import Machine

# https://towardsdatascience.com/3-d-reconstruction-with-vision-ef0f80cbb299 
#    3d from motionn SfM(Structure from Motion) and SLAM(Simultaneous Localisation and Mapping) a
#    https://www.stereolabs.com/zed-2/ (camear simlutanious mapping and location) 670 EUR
#  slam https://de.wikipedia.org/wiki/Simultaneous_Localization_and_Mapping
#   http://kos.informatik.uni-osnabrueck.de/3Dscans/ (also demo data)
# https://www.youtube.com/watch?v=N451VeA8XRA (opyncv python odometry (robot path / closing loop)
# https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html
# https://stackoverflow.com/questions/7388893/extract-transform-and-rotation-matrices-from-homography
# https://pypi.org/project/StereoVision/

# homography
# https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html

# https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html
# https://www.robots.ox.ac.uk/~vgg/hzbook/
# https://cs.gmu.edu/~kosecka/cs685/VisionBookHandout.pdf
# https://szeliski.org/Book/
# https://hal.inria.fr/inria-00174036

def get_matches(img1, img2, MIN_MATCHES):

    feat = cv.ORB_create(nfeatures=500)

    # First create the detector
    # feat = cv.SIFT_create()

    kp1, des1 = feat.detectAndCompute(img1, None)

    # kp_img1 = cv.drawKeypoints(img1, kp1, None, color=(0, 255, 0), flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # cv.imshow('kp1', kp_img1)

    kp2, des2 = feat.detectAndCompute(img2, None)

    kp_img2 = cv.drawKeypoints(img2, kp2, None, color=(0, 255, 0), flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    index_params = dict(algorithm=6,
                        table_number=6,
                        key_size=12,
                        multi_probe_level=2)
    search_params = {}
    flann = cv.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1, des2, k=2)

    print("matches found %s" % len(matches))

    # As per Lowe's ratio test to filter good matches
    good_matches = []
    for m, n in matches:
        if m.distance < 0.75 * n.distance:

            print(str(kp1[m.queryIdx].pt))
            pt1 = np.array(list(kp1[m.queryIdx].pt))
            pt2 = np.array(list(kp2[m.trainIdx].pt))

            line_rounded(kp_img2, pt1, pt2)
            movement = np.linalg.norm(pt1-pt2)
                
            if movement > 30:
                good_matches.append((pt1, pt2))
                line_rounded(kp_img2, pt1, pt2, color=(255,0,0))


    print("good matches found %s" % len(good_matches))

    if len(good_matches) < MIN_MATCHES:
        raise Exception("not enough matches expected %s got %s" % (MIN_MATCHES, len(good_matches)))
    return {"matches": matches, "good_matches": good_matches, "kp1": kp1, "kp2": kp2, "des1" : des1, "des2": des2, "kp_img2": kp_img2}


def keypoints(img1, img2):
    feat = cv.SIFT_create()
    # feat = cv2.ORB_create()

    kp1, des1 = feat.detectAndCompute(img1, None)
    kp2, des2 = feat.detectAndCompute(img2, None)

    # index_params = dict(algorithm=6,
    #                     table_number=6,
    #                     key_size=12,
    #                     multi_probe_level=2)
    # search_params = {}
    # flann = cv.FlannBasedMatcher(index_params, search_params)
    # matches = flann.knnMatch(des1, des2, k=2)

    bf = cv.BFMatcher()
    matches = bf.knnMatch(des1, des2, k=2)
    # Even in the top 2 descriptors, we may have obtained some trivial descriptors. We eliminate those with ratio test.

    good = []
    for m in matches:
        if (m[0].distance < 0.5*m[1].distance):
            good.append(m)
    print(["matches" ,len(good)])
    matches = np.asarray(good)

    return dict(kp1 = kp1, kp2 = kp2, des1 = des1, des2 = des2, matches = matches)

def center_by_homography(img1, img2):
    """ doesn't work well 50px off or so """
    kp = keypoints(img1, img2)
    matches = kp["matches"]
    kp1 = kp["kp1"]
    kp2 = kp["kp2"]
    kp_img1 = cv.drawKeypoints(img1, [kp1[m.queryIdx] for m in matches[:,0] ], None, color=(0, 255, 0), flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    kp_img2 = cv.drawKeypoints(img2, [kp2[m.trainIdx] for m in matches[:,0] ], None, color=(0, 255, 0), flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    if (len(matches[:,0]) >= 4):
        src_points = np.float32([ kp1[m.queryIdx].pt for m in matches[:,0] ])
        dst_points =np.float32([ kp2[m.trainIdx].pt for m in matches[:,0] ])
        src = src_points.reshape(-1,1,2)
        dst = dst_points.reshape(-1,1,2)
        H, masked = cv.findHomography(src, dst, cv.RANSAC, 5.0)
    else:
        raise AssertionError('Can’t find enough keypoints.')

    # print(H)
    
    A,B,C = list(H[0])
    D,E,F = list(H[1])
    # print(["ABC", A, B, C])
    # print(["DEF", D, E, F])

    # print("-")
    x = -(B*F+C*(1-E))/ (E+A*(1-E)+B*D-1)
    y = (A*F-F-C*D) / (E+A*(1-E)+B*D-1)
    return (x,y )

#     opts = ()

#     def err(vec, *args):
#         # print(vec)
#         xy = vec[0:2]
#         # print(["xy", xy])
#         t = vec[2]
#         val = np.sum(np.square((src_points - xy) * t + xy - dst_points))
#         # print(val)
#         return val

#     options = {'gtol': 1e-6, 'disp': True, 'eps': 0.000001, 'maxiter': 2000}
#     optimized = scipy.optimize.minimize(err, start , args=opts, options={}) # , method=None, jac=None, hess=None, hessp=None, bounds=None, constraints=(), tol=None, callback=None, options=None)
#     print (optimized["x"])

# def get_center_by_blur(ct: CameraThread, machine: Machine):
#     """ bad idea didnt work """
#     img1 = ct.fresh()
#     # move z up
#     machine.move_xyz(z=50, relative=True)
#     # find points on new image
#     img2 = ct.fresh()

#     #imgf1 = cv.convertTo(img1, cv.32FC3, 1/255.0)
#     #imgf2 = cv.convertTo(img2, cv.32FC3, 1/255.0)

#     blur1 = cv.blur(img1,(30,30))
#     blur2 = cv.blur(img2,(30,30))

# def get_center_by_images(img1, img2):
#     matched = get_matches(img1, img2, MIN_MATCHES = 3)
#     good_matches = matched["good_matches"]
#     kp_img2 = matched["kp_img2"]

#     intersections = []     
#     l = len(good_matches)
#     for i in range(0, l):
#         m1 = good_matches[i]

#         pi025 = 0.25 * math.pi
#         pi075 = 0.75 * math.pi

#         for j in range(1, l):
#             m2 = good_matches[(i+j) % l]
#             a = vectors_angle(m1[1]-m1[0], m2[1]-m2[0])
#             print("angle")
#             print(a)
#             if a > pi025  and a < pi075:
#                 print("angle ok")
#                 break
#             m2 = None

#         if (m2 == None): continue

#         inters = vector_intersect(m1[0], m1[1], m2[0], m2[1])
#         if (math.isinf(inters[0])):
#             print("parallel lines found, ignoring")
#             continue
#         else:
#             print("intersection:")
#             print(inters)
#             cross(kp_img2, inters)
#             intersections.append( inters )


#     cv.imshow('kp2', kp_img2)
#     return intersections 

#     # print("intersections left %s" % len(intersections)) 
#     # print("intersections")
#     # print(intersections)

#     # # TODO : sanity check (standard deviation / error something like that)

#     # return np.mean(np.array(intersections), axis=0)


# TODO new module

def with_images(ct: CameraThread, machine: Machine, f, move):
    img1 = ct.fresh()
    # move z up
    machine.move_xyz(relative=True, **move)
    try:
        # find points on new image
        img2 = ct.fresh()

        cv.imwrite("img1.jpg", img1)
        cv.imwrite("img2.jpg", img2)

        r = f(img1, img2) 

        for k in move.keys():
            move[k] = - move[k]

    finally: 
        machine.move_xyz(relative=True, wait=False, **move)

    return r

def center_by_regression(img1, img2):
    """ doesn't work well 50px off or so """

    height, width = [int(x) for x in img1.shape][:2]

    kp = keypoints(img1, img2)
    matches = kp["matches"]
    kp1 = kp["kp1"]
    kp2 = kp["kp2"]

    if (len(matches[:,0]) >= 4):
        src_points = np.float32([ kp1[m.queryIdx].pt for m in matches[:,0] ])
        dst_points =np.float32([ kp2[m.trainIdx].pt for m in matches[:,0] ])
    else:
        raise AssertionError('Can’t find enough keypoints.')

    ranges = ((0, width), (0, height), (0.2, 0.999))

    def err(vec, *args):
        xy = vec[0:2]
        t = vec[2]
        return np.sum(np.square((src_points - xy) * t + xy - dst_points))

    r = scipy.optimize.differential_evolution(err, bounds= ranges, args={}) # , method=None, jac=None, hess=None, hessp=None, bounds=None, constraints=(), tol=None, callback=None, options=None)
    return r["x"]

def get_center(ct: CameraThread, machine: Machine, dz=4):
    return list(with_images(ct = ct, machine=machine, f=center_by_regression, move = dict(z=40) )[:2])

    # https://blog.francium.tech/feature-detection-and-matching-with-opencv-5fd2394a590
    # calculate center by amount
    # average/error

def rotation_deg(img1, img2):
    height, width = [int(x) for x in img1.shape][:2]

    kp = keypoints(img1, img2)
    matches = kp["matches"]
    kp1 = kp["kp1"]
    kp2 = kp["kp2"]

    if (len(matches[:,0]) >= 4):
        src_points = np.float32([ kp1[m.queryIdx].pt for m in matches[:,0] ])
        dst_points = np.float32([ kp2[m.trainIdx].pt for m in matches[:,0] ])
    else:
        raise AssertionError('Can’t find enough keypoints.')

    ranges = ((-width, width), (-89, 89))

    def err(vec, *args):
        rm = rotation_matrix_2d_x(vec[1])
        xy = rm.dot(vec[0] * np.array([0,1]))
        # print(["xy", xy])
        r = np.sum(np.square((src_points + xy - dst_points)))
        print(r)
        return r

    # return scipy.optimize.brute(err, ranges= ranges) # , method=None, jac=None, hess=None, hessp=None, bounds=None, constraints=(), tol=None, callback=None, options=None)
    return scipy.optimize.differential_evolution(err, bounds= ranges) # , method=None, jac=None, hess=None, hessp=None, bounds=None, constraints=(), tol=None, callback=None, options=None)

# TODO new module
def get_rotation_deg( ct: CameraThread, machine: Machine):
    return with_images(ct=ct, machine=machine, f = rotation_deg, move = dict(x=40) )["x"][1]