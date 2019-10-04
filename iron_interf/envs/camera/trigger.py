import os
import serial
import matplotlib.pyplot as plt
import time


class SerialPortReader(object):
    def __init__(self, name='/dev/ttyACM0'):
        self.dev = serial.Serial()
        self.dev.setPort(name)

    def get_data(self):
        while True:
            while self.dev.inWaiting() > 0:
                line = self.dev.readline()
                try:
                    decoded = line.decode()
                    decoded = decoded.strip()
                    try:
                        yield float(decoded)
                    except ValueError:
                        print('could not convert {} to float!'.format(decoded))
                except UnicodeDecodeError:
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
        for value in self.reader.get_data():
            print(value, 'time', time.time() - begin)
            max_value = max(max_value, value)
            if time.time() - begin > 5:
                break

        print('max_value', max_value)
        self.threshold *= max_value
        self.reader.close()

    def can_start(self):
        self.reader.open()
        self.reader.flush()

        for value in self.reader.get_data():
            #print(value)

            self.buffer = self.buffer[1:] + [value]
            if value > self.threshold and self.buffer[0] - self.buffer[-1] > 0:
                if not self.triggered:
                    self.triggered = True
                    yield True
            else:
                self.triggered = False
                yield False

    def stop(self):
        self.reader.close()
