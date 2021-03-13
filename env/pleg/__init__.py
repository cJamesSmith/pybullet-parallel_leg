import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id='pleg-v0',
    entry_point='pleg.envs:PlegEnv',
)
