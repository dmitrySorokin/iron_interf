import gym
from matplotlib import pyplot as plt
import numpy as np
import time as tm

from .mirrors.motor_controller import get_position, get_home_position, set_home_position, wait_for_motor, move_absolute
from .camera.ids_camera import IDSCamera


class IronInterfEnv(gym.Env):
    n_points = 64
    n_frames = 16
    n_actions = 4

    # mirror screw step l / L, (ratio of delta screw length to vertical distance)
    far_mirror_max_screw_value = 5000
    near_mirror_max_screw_value = 2500

    metadata = {'render.modes': ['human', 'rgb_array', 'last_state']}
    reward_range = (0, 1)

    observation_space = gym.spaces.Box(low=0, high=255, shape=(n_frames, n_points, n_points), dtype=np.uint8)
    action_space = gym.spaces.Box(low=-1, high=1, shape=(n_actions,), dtype=np.float64)

    done_visibility = 0.9999

    max_steps = 200

    mirror2motor = {
        'mirror1_screw_x': 1,
        'mirror1_screw_y': 2,
        'mirror2_screw_x': 3,
        'mirror2_screw_y': 4
    }

    def __init__(self):
        self.init_mirror1_screw_x = get_position(IronInterfEnv.mirror2motor['mirror1_screw_x'])
        self.init_mirror1_screw_y = get_position(IronInterfEnv.mirror2motor['mirror1_screw_y'])
        self.init_mirror2_screw_x = get_position(IronInterfEnv.mirror2motor['mirror2_screw_x'])
        self.init_mirror2_screw_y = get_position(IronInterfEnv.mirror2motor['mirror2_screw_y'])

        for key in IronInterfEnv.mirror2motor:
            motor_id = IronInterfEnv.mirror2motor[key]
            print('{}, home {}; current {}'.format(key, get_home_position(motor_id), get_position(motor_id)))

        self.mirror1_screw_x = 0
        self.mirror1_screw_y = 0
        self.mirror2_screw_x = 0
        self.mirror2_screw_y = 0

        self.camera = IDSCamera(IronInterfEnv.n_frames, IronInterfEnv.n_points, IronInterfEnv.n_points)

        self.state = None
        self.n_steps = None
        self.info = None
        self.visib = None
        self.camera_enabled = True

        self._calc_reward = self._calc_reward_delta_visib

    def enable_camera(self, enabled):
        self.camera_enabled = enabled

    def set_calc_reward(self, method):
        if method == 'visib_minus_1':
            self._calc_reward = self._calc_reward_visib_minus_1
        elif method == 'delta_visib':
            self._calc_reward = self._calc_reward_delta_visib
        else:
            assert False, 'unknown reward_calc == {} optnions are "visib_minus1", "delta_visib"'.format(method)

    def get_keys_to_action(self):
        return {
            (ord('w'),): 0,
            (ord('s'),): 1,
            (ord('a'),): 2,
            (ord('d'),): 3,
            (ord('i'),): 4,
            (ord('k'),): 5,
            (ord('j'),): 6,
            (ord('l'),): 7
        }

    def seed(self, seed=None):
        self.action_space.seed(seed)

    def step(self, actions):
        #print('step ', actions)
        self.n_steps += 1

        print('actions', actions)

        for action_id, action_value in enumerate(actions):
            self._take_action(action_id, action_value)
            self._wait_for_motors()

        print('relative m1_x = {}, m1_y = {}, m2_x = {}, m2_y = {}'.format(
            self.mirror1_screw_x, self.mirror1_screw_y, self.mirror2_screw_x, self.mirror2_screw_y)
        )
        print('absolute init_m1_x = {}, init_m1_y = {}, init_m2_x = {}, init_m2_y = {}'.format(
            self.init_mirror1_screw_x, self.init_mirror1_screw_y, self.init_mirror2_screw_x, self.init_mirror2_screw_y)
        )

        start = tm.time()
        self.state, tot_intens = self.calc_state()
        end = tm.time()
        self.info['state_calc_time'] = end - start
        reward = self._calc_reward(tot_intens)

        return self.state, reward, self.game_over(), self.info

    def reset(self, actions=None):
        '''
        reset to absolute position
        :param actions: absolute position to reset
        :return: state
        '''
        self.n_steps = 0
        self.info = {}

        self.mirror1_screw_x = 0
        self.mirror1_screw_y = 0
        self.mirror2_screw_x = 0
        self.mirror2_screw_y = 0

        if actions is None:
            actions = IronInterfEnv.action_space.sample()

        print('reset actions: ', actions)
        for action_id, action_value in enumerate(actions):
            self._take_action(action_id, action_value)
            # wait until actions are done
            self._wait_for_motors()

        self.state, tot_intens = self.calc_state()
        self.visib = self._calc_visib(tot_intens)

        return self.state

    def render(self, mode='human', close=False):
        if mode == 'rgb_array':
            img = self.camera.image()
            return np.array([img])
        elif mode == 'last_state':
            return self.state
        elif mode == 'human':
            plt.imshow(self.state[0], vmin=0, vmax=4)
            plt.ion()
            plt.pause(1)
            plt.show()
        else:
            return None

    def _take_action(self, action_id, normalized_step_length):
        """
        :param action:
        :return:
        """

        if action_id == 0:
            self.mirror1_screw_y = np.clip(self.mirror1_screw_y + normalized_step_length, -1, 1)
            value = self.init_mirror1_screw_y + int(self.mirror1_screw_y * self.far_mirror_max_screw_value)
            print('move mirror1y to ', value)
            move_absolute(
                motor_id=IronInterfEnv.mirror2motor['mirror1_screw_y'],
                value=int(value)
            )
        elif action_id == 1:
            self.mirror1_screw_x = np.clip(self.mirror1_screw_x + normalized_step_length, -1, 1)
            value = self.init_mirror1_screw_x + int(self.mirror1_screw_x * self.far_mirror_max_screw_value)
            print('move mirror1x to ', value)
            move_absolute(
                motor_id=IronInterfEnv.mirror2motor['mirror1_screw_x'],
                value=int(value)
            )
        elif action_id == 2:
            self.mirror2_screw_y = np.clip(self.mirror2_screw_y + normalized_step_length, -1, 1)
            value = self.init_mirror2_screw_y + int(self.mirror2_screw_y * self.near_mirror_max_screw_value)
            print('move mirror2y to ', value)
            move_absolute(
                motor_id=IronInterfEnv.mirror2motor['mirror2_screw_y'],
                value=int(value)
            )
        elif action_id == 3:
            self.mirror2_screw_x = np.clip(self.mirror2_screw_x + normalized_step_length, -1, 1)
            value = self.init_mirror2_screw_x + int(self.mirror2_screw_x * self.near_mirror_max_screw_value)
            print('move mirror2x to ', value)
            move_absolute(
                motor_id=IronInterfEnv.mirror2motor['mirror2_screw_x'],
                value=int(value)
            )
        else:
            assert False, 'unknown action = {}'.format(action_id)

    def _wait_for_motors(self):
        for motor_id in IronInterfEnv.mirror2motor.values():
            wait_for_motor(motor_id)

    def _calc_reward_visib_minus_1(self, tot_intens):
        self.visib = self._calc_visib(tot_intens)
        self.info['visib'] = self.visib
        return self.visib - 1.

    def _calc_reward_delta_visib(self, tot_intens):
        prev_visib = self.visib
        self.visib = self._calc_visib(tot_intens)
        self.info['visib'] = self.visib
        return self.visib - prev_visib

    def _calc_visib(self, tot_intens):
        def visib(vmin, vmax):
            return (vmax - vmin) / (vmax + vmin)

        imin, imax = min(tot_intens), max(tot_intens)
        self.info['imin'] = imin
        self.info['imax'] = imax

        return visib(float(min(tot_intens)), float(max(tot_intens)))

    def game_over(self):
        return self.visib > IronInterfEnv.done_visibility or \
               self.n_steps >= IronInterfEnv.max_steps

    def close(self):
        self.camera.stop()

    def save_state(self, path):
        assert self.state is not None, 'IronInterf: can\'t save, state is None'
        np.save(path, self.state)
        np.save(path + '_handles', np.array([
            self.mirror1_screw_y * self.far_mirror_max_screw_value,
            self.mirror1_screw_x * self.far_mirror_max_screw_value,
            self.mirror2_screw_y * self.near_mirror_max_screw_value,
            self.mirror2_screw_x * self.near_mirror_max_screw_value
        ]))

    def calc_state(self):
        if self.camera_enabled:
            return self.camera.calc_state()
        return None, [1] * IronInterfEnv.n_frames
