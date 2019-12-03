import os
import serial
import matplotlib.pyplot as plt
import time


class SerialPortReader(object):
    def __init__(self, name='/dev/ttyACM0'):
        self.dev = serial.Serial()
        self.dev.setPort(name)

    def get_data(self, verbose=False):
        """
        arduino convention: "value_generator, value_detector"
        """
        while True:
            while self.dev.inWaiting() > 0:
                line = self.dev.readline()
                try:
                    decoded = line.decode().strip()
                    decoded = decoded.split(',')
                    if len(decoded) == 2:
                        try:
                            yield float(decoded[0]), float(decoded[1])
                        except ValueError:
                            if verbose:
                                print('could not convert "{}" to float!'.format(decoded))
                except UnicodeDecodeError:
                    if verbose:
                        print('can not decode {}'.format(line))

    def open(self):
        self.dev.open()

    def close(self):
        self.dev.close()

    def flush(self):
        while self.dev.inWaiting() > 0:
            self.dev.read_all()


class CameraTrigger(object):
    def __init__(self, buff_size=2, threshold_coeff=0.9):
        self.reader = SerialPortReader()
        self.buffer = [0.0] * buff_size
        self.threshold = threshold_coeff
        self.triggered = False

    def init(self):
        self.reader.open()
        begin = time.time()
        max_value = -1
        for value, value_detector in self.reader.get_data():
            print(value, 'time', time.time() - begin)
            # something strange appears
            if value > max_value and value < 1000:
                max_value = value
            if time.time() - begin > 5:
                break

        print('max_value', max_value)
        self.threshold *= max_value
        self.reader.close()

    def is_generator_max(self):
        for value, value_detector in self.reader.get_data():
            self.buffer = self.buffer[1:] + [value]
            if value > self.threshold and self.buffer[0] - self.buffer[-1] > 0:
                if not self.triggered:
                    self.triggered = True
                    yield True
            else:
                self.triggered = False
                yield False

    def get_intens(self):
        res = []
        for value_generator, value in self.reader.get_data():
            res.append(value)
            if len(res) == 120:
                break

        return res

    def start(self):
        self.reader.open()
        self.reader.flush()

    def stop(self):
        self.reader.close()
