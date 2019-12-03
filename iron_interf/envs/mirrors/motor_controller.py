from .newport import Controller


_controller = Controller(idProduct=0x4000, idVendor=0x104d)


def wait_for_motor(motor_id):
    motor_done_cmd = '{}MD?'.format(motor_id)
    is_done = False
    while not is_done:
        resp = _controller.command(motor_done_cmd)
        is_done = int(resp[2])


def move_relative(motor_id, value):
    move_motor_cmd = '{}PR{}'.format(motor_id, value)
    #print(move_motor_cmd)
    _controller.command(move_motor_cmd)


def move_absolute(motor_id, value):
    move_motor_cmd = '{}PA{}'.format(motor_id, value)
    _controller.command(move_motor_cmd)


def get_home_position(motor_id):
    return int(_controller.command('{}DH?'.format(motor_id))[2:])


def get_position(motor_id):
    return int(_controller.command('{}TP?'.format(motor_id))[2:])


def set_home_position(motor_id, value):
    _controller.command('{}DH{}'.format(motor_id, value))


def get_target(motor_id):
    return int(_controller.command('{}PA?'.format(motor_id))[2:])


#_controller.command('RS')
#move_relative(1, 100)
#wait_for_motor(1)



#move_absolute(1, 230)
#wait_for_motor(1)

#print('target = ', get_target(1))
#print('home = ', get_home_position(1))

#print('target = ', get_target(2))
#print('home = ', get_home_position(2))

