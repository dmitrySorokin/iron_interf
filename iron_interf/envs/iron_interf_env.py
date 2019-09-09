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
    max_mirror_screw_value = 5000

    metadata = {'render.modes': ['human', 'rgb_array']}
    reward_range = (0, 1)

    observation_space = gym.spaces.Box(low=0, high=4, shape=(n_frames, n_points, n_points), dtype=np.float64)
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

        self._calc_reward = self._calc_reward_delta_visib

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


        start = tm.clock()
        self.state, tot_intens = self.camera.calc_state()
        end = tm.clock()
        self.info['state_calc_time'] = end - start
        reward = self._calc_reward(tot_intens)

        return self.state, reward, self.game_over(), self.info

    def reset(self, actions=None):
        #print('reset ', actions)
        self.n_steps = 0
        self.info = {}

        self.mirror1_screw_x = 0
        self.mirror1_screw_y = 0
        self.mirror2_screw_x = 0
        self.mirror2_screw_y = 0

        if actions is None:
            actions = IronInterfEnv.action_space.sample()
        for action_id, action_value in enumerate(actions):
            self._take_action(action_id, action_value)
        # wait until actions are done
        self._wait_for_motors()

        # TODO: calc state and visib
        self.state, tot_intens = self.camera.calc_state()
        self.visib = self._calc_visib(tot_intens)

        return self.state

    def render(self, mode='human', close=False):
        if mode == 'rgb_array':
            img = self.camera.image()
            return np.array([img])
            return self.state
        elif mode == 'human':
            plt.imshow(self.state[0], vmin=0, vmax=4)
            plt.ion()
            plt.pause(1)
            plt.show()
        else:
            return None

    def _take_action(self, action, normalized_step_length):
        """
        0 - do nothing
        [1, 2, 3, 4] - mirror1
        [5, 6, 7, 8] - mirror2
        :param action:
        :return:
        """

        if action == 0:
            self.mirror1_screw_y = np.clip(self.mirror1_screw_y + normalized_step_length, -1, 1)
            value = np.clip(
                self.mirror1_screw_y * self.max_mirror_screw_value + self.init_mirror1_screw_y,
                self.init_mirror1_screw_y - self.max_mirror_screw_value,
                self.init_mirror1_screw_y + self.max_mirror_screw_value)
            move_absolute(
                motor_id=IronInterfEnv.mirror2motor['mirror1_screw_y'],
                value=int(value)
            )
        elif action == 1:
            self.mirror1_screw_x = np.clip(self.mirror1_screw_x + normalized_step_length, -1, 1)
            value = np.clip(
                self.mirror1_screw_x * self.max_mirror_screw_value + self.init_mirror1_screw_x,
                self.init_mirror1_screw_x - self.max_mirror_screw_value,
                self.init_mirror1_screw_x + self.max_mirror_screw_value)
            move_absolute(
                motor_id=IronInterfEnv.mirror2motor['mirror1_screw_x'],
                value=int(value)
            )
        elif action == 2:
            self.mirror2_screw_y = np.clip(self.mirror2_screw_y + normalized_step_length, -1, 1)
            value = np.clip(
                self.mirror2_screw_y * self.max_mirror_screw_value + self.init_mirror2_screw_y,
                self.init_mirror2_screw_y - self.max_mirror_screw_value,
                self.init_mirror2_screw_y + self.max_mirror_screw_value)
            move_absolute(
                motor_id=IronInterfEnv.mirror2motor['mirror2_screw_y'],
                value=int(value)
            )
        elif action == 3:
            self.mirror2_screw_x = np.clip(self.mirror2_screw_x + normalized_step_length, -1, 1)
            value = np.clip(
                self.mirror2_screw_x * self.max_mirror_screw_value + self.init_mirror2_screw_x,
                self.init_mirror2_screw_x - self.max_mirror_screw_value,
                self.init_mirror2_screw_x + self.max_mirror_screw_value)
            move_absolute(
                motor_id=IronInterfEnv.mirror2motor['mirror2_screw_x'],
                value=int(value)
            )
        else:
            assert False, 'unknown action = {}'.format(action)

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
