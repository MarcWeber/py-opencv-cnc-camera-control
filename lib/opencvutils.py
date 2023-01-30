import threading
import time
import cv2 as cv

from functools import lru_cache


def resize(img, scale = 0.25):
    if scale == 1:
        return img
    scale_percent = 25 # percent of original size
    width  = int(img.shape[1] * scale)
    height = int(img.shape[0] * scale)
    dim = (width, height)

    return cv.resize(img, dim, interpolation = cv.INTER_AREA)

def line_rounded(img, pt1, pt2, color=(0,0,0), **kwargs):
    pt1 = [round(x) for x in list(pt1)]
    pt2 = [round(x) for x in list(pt2)]
    cv.line(img, pt1, pt2, color=color, **{"thickness": 1, **kwargs})


def circle_rounded(img, center, r, *args, **kwargs):
    center = [round(x) for x in center]
    r = round(r)
    cv.circle(img, center, r, *args, **kwargs)

def cross(frame, c, color=(0,0,0), thickness=1, dt = 5):
    x=round(c[0])
    y=round(c[1])
    cv.line(frame, [x-dt, y-dt], [x+dt, y+dt], color=color, thickness=thickness)
    cv.line(frame, [x+dt, y-dt], [x-dt, y+dt], color=color, thickness=thickness)

class ImageDR:
    """ stuff derived from image"""

    def __init__(self, image): 
        self.image = image

    @lru_cache(maxsize=None)
    def gray(self):
        return cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)    

    @lru_cache(maxsize=None)
    def sift(self):
        sift = cv.SIFT()
        kp = sift.detect(self.gray(), None)
        return kp

# class Cam:

#     def __init__(Self, camera): 
#         self.camera = camera

#     def image():

class CameraThread(threading.Thread):

    def __init__(self, cap, sleep, dropcount=3, name='camera-buffer-cleaner-thread'):
        self.sleep = sleep
        self.dropcount = dropcount
        self.cap = cap
        self.last_frame = None
        self.next_last = -1
        self.i = 0
        self.dropping = 0
        self.stope = threading.Event()
        self.fresh_event = None
        self.next_event = None
        self.starting_thread = threading.currentThread()     
        self.start_time = time.time()
        self.sleeping_done  = threading.Event()
        super(CameraThread, self).__init__(name=name)
        self.start()

    def stop(self):
        self.stope.set()

    def run(self):
        while not self.stope.is_set() and self.starting_thread.is_alive():
            ret, self.last_frame = self.cap.read()

            if (self.start_time != None and time.time() - self.start_time > self.sleep):
                self.sleeping_done.set()
                self.start_time = None

            if not ret:
                print("getting frame failed")
                self.last_frame = "error"
                print("ending CR thread")     
                return
            else:
                pass
                # print("frame ok")
            self.i += 1
            if self.fresh_event != None:
                self.dropping -= 1
                if self.dropping < 0:
                    # if waitnig for fresh image, notiify that its there
                   self.fresh_event.set() 
                   self.fresh_event = None

            if self.next_event != None:
                # waiting for next image with new id notify its there
                self.next_event.set()
                self.next_event = None
        print("ending CR thread")     


    def check_error(self): 
        if type(self.last_frame) == type(""):
           raise "error reading webcam" 

    def next(self, wait):
        if wait:
            self.sleeping_done.wait()
        self.check_error()
        if self.i == self.next_last or self.last_frame is None:
            self.next_event = next_event =  threading.Event()
            next_event.wait()
        self.next_last = self.i
        return self.last_frame

    def fresh(self, wait = True):
        if wait:
            self.sleeping_done.wait()
        self.check_error()
        self.dropping = self.dropcount
        self.fresh_event = fresh_event = threading.Event()
        fresh_event.wait()
        return self.last_frame