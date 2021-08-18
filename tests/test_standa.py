
from iron_interf.envs.mirrors.motor_controller import *


if __name__ == '__main__':
    # lens_max_screw_value = 1'000'000
    print(get_position(ControllerType.LENS, motor_id=0))
    print(get_home_position(ControllerType.LENS, motor_id=0))
    move_relative(ControllerType.LENS, motor_id=0, value=3000)
    move_relative(ControllerType.LENS, motor_id=0, value=-6000)
    move_relative(ControllerType.LENS, motor_id=0, value=3000)

    print(get_position(ControllerType.LENS, motor_id=1))
    print(get_home_position(ControllerType.LENS, motor_id=1))
    move_relative(ControllerType.LENS, motor_id=1, value=100000)
    move_relative(ControllerType.LENS, motor_id=1, value=-200000)
    move_relative(ControllerType.LENS, motor_id=1, value=100000)

    close()
