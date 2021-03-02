import gym
from gym import spaces
import pybullet as p
import pybullet_data
import numpy as np


class Parallel(gym.Env):
    metadata = {'render.modes': [
        'human', 'rgb_array'], 'video.frames_per_second': 50}

    def __init__(self,
                 urdfRoot=pybullet_data.getDataPath(),
                 actionRepeat=1,
                 renders=False,
                 isDiscrete=False,
                 maxSteps=1000):
        self._isDiscrete = isDiscrete
        self._timeStep = 1. / 240.
        self._urdfRoot = urdfRoot
        self._actionRepeat = actionRepeat
        self._renders = renders
        self._maxSteps = maxSteps
        self._observation = []
        self._envStepCounter = 0
        self.terminated = 0
        self._cam_dist = 1.3
        self._cam_yaw = 180
        self._cam_pitch = -40
        self.largeValObservation = 100

        if self._renders:
            cid = p.connect(p.SHARED_MEMORY)
            if (cid < 0):
                cid = p.connect(p.GUI)
            p.resetDebugVisualizerCamera(cameraDistance=1.30, cameraYaw=50.0, cameraPitch=-23.80,
                                         cameraTargetPosition=[-0.65, 0.49, -0.25], physicsClientId=cid)
        else:
            p.connect(p.DIRECT)
        p.setRealTimeSimulation(0) # TODO
        plane = p.createCollisionShape(p.GEOM_PLANE)
        p.createMultiBody(0, plane)

        self.seed() # TODO
        self.reset() # TODO

        observationDim = len(self.getExtendedObservation())

        observation_high = np.array([self.largeValObservation] * observationDim)
        if (self._isDiscrete):
            self.action_space = spaces.Discrete(7)
        else:
            action_dim = 3
            self._action_bound = 1
            action_high = np.array([self._action_bound] * action_dim)
            self.action_space = spaces.Box(-action_high, action_high)
        self.observation_space = spaces.Box(-observation_high,
                                            observation_high)
        self.viewer = None

    def reset(self):
        self.terminated = 0

    def getExtendedObservation(self):
        pass