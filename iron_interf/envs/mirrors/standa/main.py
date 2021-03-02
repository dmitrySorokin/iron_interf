from pyximc import lib
from utils import *


if __name__ == '__main__':
    print("Library version: ", get_lib_version(lib))

    device_id = open_device(lib)

    # info = get_info(lib, device_id)
    # print(info)

    # status = get_status(lib, device_id)
    # print(status)

    set_microstep_mode_256(lib, device_id)

    print(get_position(lib, device_id))

    current_speed = get_speed(lib, device_id)
    print('current_speed', current_speed)
    set_speed(lib, device_id, 1024)

    move_relative(lib, device_id, -6000, 0)
    move_relative(lib, device_id, 6000, 0)
    # move_absolute(lib, device_id, 0, 0)
    close_device(lib, device_id)
