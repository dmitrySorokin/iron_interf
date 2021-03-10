from enum import Enum
from .standa.standa_controller import StandaController
from .newport.newport_controller import NewportController


class ControllerType(Enum):
    MIRROR = 0
    LENS = 1


_standa = StandaController()
_newport = NewportController()


def wait_for_motor(controller_type, motor_id):
    if controller_type == ControllerType.MIRROR:
        return _newport.wait_for_motor(motor_id)
    elif controller_type == ControllerType.LENS:
        return _standa.wait_for_stop()
    else:
        raise ValueError(f'unknown controller_type {controller_type}')


def move_relative(controller_type, motor_id, value):
    if controller_type == ControllerType.MIRROR:
        return _newport.move_relative(motor_id, value)
    elif controller_type == ControllerType.LENS:
        return _standa.move_relative(value, 0)
    else:
        raise ValueError(f'unknown controller_type {controller_type}')


def get_home_position(controller_type, motor_id):
    if controller_type == ControllerType.MIRROR:
        return _newport.get_home_position(motor_id)
    elif controller_type == ControllerType.LENS:
        return 0
    else:
        raise ValueError(f'unknown controller_type {controller_type}')


def get_position(controller_type, motor_id):
    if controller_type == ControllerType.MIRROR:
        return _newport.get_position(motor_id)
    elif controller_type == ControllerType.LENS:
        return _standa.get_position()
    else:
        raise ValueError(f'unknown controller_type {controller_type}')


def close():
    _standa.close()
