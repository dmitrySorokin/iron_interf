from gym.envs.registration import register
from .envs import IronInterfEnv

register(id='iron_interf-v2',
         entry_point='iron_interf.envs:IronInterfEnv')
