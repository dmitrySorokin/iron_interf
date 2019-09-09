from .pyueye_example_camera import Camera
from .pyueye_example_utils import FrameThread
import threading
import time

from pyueye import ueye

import cv2
import numpy as np

# TODO: run me as root once to start a daemon
# /usr/bin/ueyeusbd


class _ImageHandle(object):
    def __init__(self, capacity=16):
        self.images = []
        self.last_image = None
        self.is_started = False
        self.capacity = capacity

    def handle(self, image_data):
        self.last_image = image_data.as_1d_image()
        self.last_image = self.last_image[:, 128: 1024 - 128]
        if self.is_started and len(self.images) < self.capacity:
            self.images.append(self.last_image)
        image_data.unlock()

    def reset(self):
        self.images = []
        self.is_started = False

    def is_ready(self):
        threading.Lock()
        return len(self.images) >= self.capacity

    def start(self):
        self.is_started = True

    def image(self):
        threading.Lock()
        while self.last_image is None:
            pass
        return self.last_image


class IDSCamera(object):
    def __init__(self, n_frames, h_points, w_points):
        self.image_handle = _ImageHandle(n_frames)
        self.camera = Camera()
        self.camera.init()
        self.camera.set_colormode(ueye.IS_CM_SENSOR_RAW8)
        #self.camera.set_aoi(0, 0, h_points, w_points)
        self.camera.alloc()
        self.camera.capture_video()
        self.thread = FrameThread(self.camera, self.image_handle)
        self.thread.timeout = 200
        self.thread.start()
        self.camera.set_exposure(0.1)


    def stop(self):
        self.thread.stop()
        self.thread.join()
        self.camera.stop_video()
        self.camera.exit()

    def calc_state(self):
        self.image_handle.start()
        while not self.image_handle.is_ready():
            pass
        images = np.array(self.image_handle.images)
        self.image_handle.reset()

        tot_intens = [np.sum(image) for image in images]

        return images, tot_intens

    def image(self):
        #aoi = self.camera.get_aoi()
        #print(aoi.x, aoi.y, aoi.height, aoi.width, type(aoi))
        return self.image_handle.image()
