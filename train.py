import pleg
import gym
from stable_baselines3 import DDPG
from stable_baselines3.common.vec_env import DummyVecEnv

# define some hyper parameters
params = {'learning_rate': 1e-5,
          'gamma': 0.99,
          'batch_size': 1024,
          'verbose': 1,
          'tensorboard_log': './log/',
          'policy_kwargs': dict(net_arch=[256, 256, 256])}

# initialise the environment
env = gym.make('pleg-v0', render=True)

# create the agent
agent = DDPG('MlpPolicy', DummyVecEnv([lambda: env]), **params)

# reset the environment
obs, state, dones = env.reset(), None, [False]

# train the agent for x amount of timesteps
agent.learn(total_timesteps=200000)
agent.save('sb-pleg')
env.close()

# once the environment is trained, run a while loop
# agent = DDPG.load(r'sb-pleg.zip')
# while True:
#     actions, state = agent.predict(obs, state=state, mask=dones)
#     obs, rew, done, info = env.step(actions)
#     if done:
#         env.reset()
#         # break
# env.close()
