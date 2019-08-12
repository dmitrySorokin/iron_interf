from gym.envs.registration import register
from .envs import IronInterfEnv

register(id='iron_interf-v1',
         entry_point='iron_interf.envs:IronInterfEnv')
