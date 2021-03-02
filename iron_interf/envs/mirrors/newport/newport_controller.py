from .newport_api import Controller


class NewportController(object):
    def __init__(self):
        # run lsusb to get the address
        self._mirror_controller = Controller(idProduct=0x4000, idVendor=0x104d, address=13)

    def wait_for_motor(self, motor_id):
        motor_done_cmd = '{}MD?'.format(motor_id)
        is_done = False
        while not is_done:
            resp = self._mirror_controller.command(motor_done_cmd)
            is_done = int(resp[2])

    def move_relative(self, motor_id, value):
        move_motor_cmd = '{}PR{}'.format(motor_id, value)
        self._mirror_controller.command(move_motor_cmd)

    def move_absolute(self, motor_id, value):
        move_motor_cmd = '{}PA{}'.format(motor_id, value)
        self._mirror_controller.command(move_motor_cmd)

    def get_home_position(self, motor_id):
        return int(self._mirror_controller.command('{}DH?'.format(motor_id))[2:])

    def get_position(self, motor_id):
        return int(self._mirror_controller.command('{}TP?'.format(motor_id))[2:])

    def set_home_position(self, motor_id, value):
        self._mirror_controller.command('{}DH{}'.format(motor_id, value))

    def get_target(self, motor_id):
        return int(self._mirror_controller.command('{}PA?'.format(motor_id))[2:])
