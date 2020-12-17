from enum import Enum
from .newport import Controller

# run lsusb to get the address
_mirror_controller = Controller(idProduct=0x4000, idVendor=0x104d, address=13)
_lens_controller = Controller(idProduct=0x4000, idVendor=0x104d, address=24)


class ControllerType(Enum):
    MIRROR = 0
    LENS = 1


def _controller(controller_type):
    if controller_type == ControllerType.MIRROR:
        return _mirror_controller
    elif controller_type == ControllerType.LENS:
        return _lens_controller
    else:
        assert False, f'wrong controller type = {controller_type}'


def wait_for_motor(controller_type, motor_id):
    motor_done_cmd = '{}MD?'.format(motor_id)
    is_done = False
    while not is_done:
        resp = _controller(controller_type).command(motor_done_cmd)
        is_done = int(resp[2])


def move_relative(controller_type, motor_id, value):
    move_motor_cmd = '{}PR{}'.format(motor_id, value)
    #print(move_motor_cmd)
    _controller(controller_type).command(move_motor_cmd)


def move_absolute(controller_type, motor_id, value):
    move_motor_cmd = '{}PA{}'.format(motor_id, value)
    _controller(controller_type).command(move_motor_cmd)


def get_home_position(controller_type, motor_id):
    return int(_controller(controller_type).command('{}DH?'.format(motor_id))[2:])


def get_position(controller_type, motor_id):
    return int(_controller(controller_type).command('{}TP?'.format(motor_id))[2:])


def set_home_position(controller_type, motor_id, value):
    _controller(controller_type).command('{}DH{}'.format(motor_id, value))


def get_target(controller_type, motor_id):
    return int(_controller(controller_type).command('{}PA?'.format(motor_id))[2:])


#_controller.command('RS')
#move_relative(1, 100)
#wait_for_motor(1)



#move_absolute(1, 230)
#wait_for_motor(1)

#print('target = ', get_target(1))
#print('home = ', get_home_position(1))

#print('target = ', get_target(2))
#print('home = ', get_home_position(2))

