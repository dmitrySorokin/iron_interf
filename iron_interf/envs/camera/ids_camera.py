from .pyueye_example_camera import Camera
from .pyueye_example_utils import FrameThread
import threading
import time
from .trigger import CameraTrigger

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
        #print('handle image time', time.time())
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
        self.camera.set_exposure(0.5)
        self.trigger = CameraTrigger()
        self.trigger.init()

        self.height = h_points
        self.width = w_points

    def set_exposure(self, value):
        self.camera.set_exposure(value)

    def stop(self):
        self.thread.stop()
        self.thread.join()
        self.camera.stop_video()
        self.camera.exit()

    def calc_state(self, verbose=True):
        begin_sync = time.time()
        self.wait_for_start()
        end_sync = time.time()
        if verbose:
            print('SYNC_TIME', end_sync - begin_sync)

        self.trigger.start()
        begin_film = time.time()
        self.image_handle.start()
        while not self.image_handle.is_ready():
            pass
        images = np.array(self.image_handle.images)
        self.image_handle.reset()
        end_film = time.time()
        if verbose:
            print('CAMERA_TIME', end_film - begin_film)

        tot_intens = [np.sum(image) for image in images]

        tot_intens_device = self.trigger.get_intens()
        self.trigger.stop()

        return self._resize(images), tot_intens, tot_intens_device

    def image(self):
        return self.image_handle.image()

    def wait_for_start(self):
        self.trigger.start()
        for res in self.trigger.is_generator_max():
            if res:
                break
        self.trigger.stop()

    def _resize(self, images):
        n_imgs = images.shape[0]
        result = np.zeros(shape=(n_imgs, self.height, self.width), dtype=np.uint8)
        for i, img in enumerate(images):
            img = cv2.cv2.resize(img, (self.height, self.width))
            result[i] = img
        return result
