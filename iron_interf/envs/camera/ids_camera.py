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
        self.is_started = False
        self.capacity = capacity

    def handle(self, image_data):
        if self.is_started and len(self.images) < self.capacity:
            image = image_data.as_1d_image()
            self.images.append(image)
        image_data.unlock()

    def reset(self):
        self.images = []
        self.is_started = False

    def is_ready(self):
        threading.Lock()
        return len(self.images) >= self.capacity

    def start(self):
        self.is_started = True


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
        self.thread.timeout = 100
        self.thread.start()

    def stop(self):
        self.thread.stop()
        self.thread.join()
        self.camera.stop_video()
        self.camera.exit()

    def calc_state(self):
        state_calc_start = time.clock()
        self.image_handle.start()
        while not self.image_handle.is_ready():
            pass
        images = np.array(self.image_handle.images)
        self.image_handle.reset()
        state_calc_end = time.clock()

        tot_intens = [np.sum(image) for image in images]

        print('state_calc_time = ', state_calc_end - state_calc_start)
        return images, tot_intens
