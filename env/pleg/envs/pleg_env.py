import math

import gym
import numpy as np
import pybullet as p
import pybullet_data
from gym import spaces
from gym.utils import seeding


class PlegEnv(gym.Env):

    def __init__(self, render=False):
        self.robotPos_1 = [0, 0, 0.35]
        self.robotPos_2 = [0, -0.07, 0]
        self.robotPos_3 = [0, 0.07, 0]
        self.jointNameToID_1 = {}
        self.linkNameToID_1 = {}
        self.revoluteID_1 = []
        self.jointNameToID_2 = {}
        self.linkNameToID_2 = {}
        self.revoluteID_2 = []
        self.jointNameToID_3 = {}
        self.linkNameToID_3 = {}
        self.revoluteID_3 = []
        self._observation = []

        self._envStepCounter = 0
        self.time_step = 0.01

        action_dim = 12
        self._action_bound = 1
        action_high = np.array([self._action_bound] * action_dim)
        self.action_space = spaces.Box(-action_high, action_high)

        observation_dim = 36
        self._observation_bound = math.pi
        observation_high = np.array([self._observation_bound] * observation_dim)
        self.observation_space = spaces.Box(-observation_high, observation_high)

        if render:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)  # non-graphical version

        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF

        self._seed()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        self._assign_throttle(action)
        p.stepSimulation()
        self._observation = self._compute_observation()
        reward = self._compute_reward()
        done = self._compute_done()

        self._envStepCounter += 1

        return np.array(self._observation), reward, done, {}

    def reset(self):
        self._envStepCounter = 0
        p.resetSimulation()
        p.setGravity(0, 0, -10)  # m/s^2
        p.setTimeStep(self.time_step)  # sec
        p.loadURDF("plane.urdf")
        self.robot_1 = self._load_robot_1()
        self.robot_2 = self._load_robot_2()
        self.robot_3 = self._load_robot_3()
        self._add_constraint()
        self._observation = self._compute_observation()
        return np.array(self._observation)

    def _assign_throttle(self, action):
        action = action * math.pi
        print('action', action)
        p.setJointMotorControl2(bodyUniqueId=self.robot_1,
                                jointIndex=self.jointNameToID_1['L_R_J_0'],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=action[0],
                                # maxVelocity=2
                                )
        p.setJointMotorControl2(bodyUniqueId=self.robot_1,
                                jointIndex=self.jointNameToID_1['L_R_J_1'],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=action[1],
                                # maxVelocity=2
                                )
        p.setJointMotorControl2(bodyUniqueId=self.robot_1,
                                jointIndex=self.jointNameToID_1['L_R_J_2'],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=action[2],
                                # maxVelocity=2
                                )
        p.setJointMotorControl2(bodyUniqueId=self.robot_1,
                                jointIndex=self.jointNameToID_1['L_L_J_1'],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=action[3],
                                # maxVelocity=2
                                )
        p.setJointMotorControl2(bodyUniqueId=self.robot_1,
                                jointIndex=self.jointNameToID_1['L_L_J_2'],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=action[4],
                                # maxVelocity=2
                                )
        p.setJointMotorControl2(bodyUniqueId=self.robot_1,
                                jointIndex=self.jointNameToID_1['R_R_J_0'],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=action[5],
                                # maxVelocity=2
                                )
        p.setJointMotorControl2(bodyUniqueId=self.robot_1,
                                jointIndex=self.jointNameToID_1['R_R_J_1'],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=action[7],
                                # maxVelocity=2
                                )
        p.setJointMotorControl2(bodyUniqueId=self.robot_1,
                                jointIndex=self.jointNameToID_1['R_R_J_2'],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=action[8],
                                # maxVelocity=2
                                )
        p.setJointMotorControl2(bodyUniqueId=self.robot_1,
                                jointIndex=self.jointNameToID_1['R_L_J_1'],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=action[9],
                                # maxVelocity=2
                                )
        p.setJointMotorControl2(bodyUniqueId=self.robot_1,
                                jointIndex=self.jointNameToID_1['R_L_J_2'],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=action[10],
                                # maxVelocity=2
                                )
        p.setJointMotorControl2(bodyUniqueId=self.robot_2,
                                jointIndex=self.jointNameToID_2['J_R_0'],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=action[6],
                                # maxVelocity=2
                                )
        p.setJointMotorControl2(bodyUniqueId=self.robot_3,
                                jointIndex=self.jointNameToID_3['J_L_0'],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=action[11],
                                # maxVelocity=2
                                )

    def _compute_observation(self):
        states = [p.getJointState(self.robot_1, self.jointNameToID_1['L_R_J_0']),
                  p.getJointState(self.robot_1, self.jointNameToID_1['L_R_J_1']),
                  p.getJointState(self.robot_1, self.jointNameToID_1['L_R_J_2']),
                  p.getJointState(self.robot_1, self.jointNameToID_1['L_L_J_1']),
                  p.getJointState(self.robot_1, self.jointNameToID_1['L_L_J_2']),
                  p.getJointState(self.robot_1, self.jointNameToID_1['R_R_J_0']),
                  p.getJointState(self.robot_1, self.jointNameToID_1['R_R_J_1']),
                  p.getJointState(self.robot_1, self.jointNameToID_1['R_R_J_2']),
                  p.getJointState(self.robot_1, self.jointNameToID_1['R_L_J_1']),
                  p.getJointState(self.robot_1, self.jointNameToID_1['R_L_J_2']),
                  p.getJointState(self.robot_2, self.jointNameToID_2['J_R_0']),
                  p.getJointState(self.robot_3, self.jointNameToID_3['J_R_0'])]
        obs = []
        for state in states:
            obs.append(state[0])
            obs.append(state[1])  # 0 for pos, 1 for vel
        cube_pos, cube_orn = p.getBasePositionAndOrientation(self.robot_1)
        cube_euler = p.getEulerFromQuaternion(cube_orn)
        print('cube_euler', cube_euler)
        print('cube_pos', cube_pos)
        linear, angular = p.getBaseVelocity(self.robot_1)
        obs.append(cube_pos[0])
        obs.append(cube_pos[1])
        obs.append(cube_pos[2])
        obs.append(cube_euler[0])
        obs.append(cube_euler[1])
        obs.append(cube_euler[2])
        obs.append(linear[0])
        obs.append(linear[1])
        obs.append(linear[2])
        obs.append(angular[0])
        obs.append(angular[1])
        obs.append(angular[2])
        return obs

    def _compute_reward(self):
        robot1_pos, robot1_orn = p.getBasePositionAndOrientation(self.robot_1)
        robot1_euler = p.getEulerFromQuaternion(robot1_orn)
        robot2_pos, robot2_orn = p.getBasePositionAndOrientation(self.robot_2)
        robot2_euler = p.getEulerFromQuaternion(robot2_orn)
        robot3_pos, robot3_orn = p.getBasePositionAndOrientation(self.robot_3)
        robot3_euler = p.getEulerFromQuaternion(robot3_orn)
        reward = robot1_pos[0] * 10 + robot2_pos[0] * 10 + robot3_pos[0] * 10 + robot2_pos[2] * 10 + robot3_pos[2] * 10 + self._envStepCounter * self.time_step
        print('reward0', reward)
        # if math.fabs(robot_pos[2]) < 0.2:
        #     reward /= 2
        # print('reward1', reward)
        return reward

    def _compute_done(self):
        robot_pos, _ = p.getBasePositionAndOrientation(self.robot_1)
        return self._envStepCounter >= 1500 or math.fabs(robot_pos[2]) < 0.25

    def _render(self, mode='human', close=False):
        pass

    def _load_robot_1(self):

        robot = p.loadURDF(r'leg-1.SLDASM/urdf/leg-1.SLDASM.urdf',
                           self.robotPos_1,
                           useFixedBase=0,
                           )
        for j in range(p.getNumJoints(robot)):
            info = p.getJointInfo(robot, j)
            # print(info)
            joint_name = info[1].decode('utf8')
            joint_type = info[2]
            if joint_type == p.JOINT_REVOLUTE:
                self.jointNameToID_1[joint_name] = info[0]
                self.linkNameToID_1[info[12].decode('UTF-8')] = info[0]
                self.revoluteID_1.append(j)
        return robot

    def _load_robot_2(self):
        robot = p.loadURDF(r'leg-2.SLDASM/urdf/leg-2.SLDASM.urdf',
                           self.robotPos_2,
                           useFixedBase=0,
                           )

        for j in range(p.getNumJoints(robot)):
            info = p.getJointInfo(robot, j)
            joint_name = info[1].decode('utf8')
            joint_type = info[2]
            if joint_type == p.JOINT_REVOLUTE:
                self.jointNameToID_2[joint_name] = info[0]
                self.linkNameToID_2[info[12].decode('UTF-8')] = info[0]
                self.revoluteID_2.append(j)
        return robot

    def _load_robot_3(self):
        robot = p.loadURDF(r'leg-2.SLDASM/urdf/leg-2.SLDASM.urdf',
                           self.robotPos_3,
                           useFixedBase=0,
                           )

        for j in range(p.getNumJoints(robot)):
            info = p.getJointInfo(robot, j)
            joint_name = info[1].decode('utf8')
            joint_type = info[2]
            if joint_type == p.JOINT_REVOLUTE:
                self.jointNameToID_3[joint_name] = info[0]
                self.linkNameToID_3[info[12].decode('UTF-8')] = info[0]
                self.revoluteID_3.append(j)
        return robot

    def _add_constraint(self):

        # collisions between robot_1 and robot_2
        for i in range(len(self.linkNameToID_1)):
            for j in range(len(self.linkNameToID_2)):
                p.setCollisionFilterPair(self.robot_1, self.robot_2, i, j, 0)

        # collisions between robot_1 and robot_3
        for i in range(len(self.linkNameToID_1)):
            for j in range(len(self.linkNameToID_2)):
                p.setCollisionFilterPair(self.robot_1, self.robot_3, i, j, 0)

        constraintNum = 20
        # constraint between robot_1 and robot_2
        for i in range(constraintNum):
            p.createConstraint(parentBodyUniqueId=self.robot_1,
                               parentLinkIndex=self.linkNameToID_1["L_L_L_1"],
                               childBodyUniqueId=self.robot_2,
                               childLinkIndex=self.linkNameToID_2['L_L_2'],
                               jointType=p.JOINT_POINT2POINT,
                               jointAxis=[0, 0, 0],
                               parentFramePosition=[
                                   -0.005 + (0.01 / constraintNum) * i, -0.06, 0],
                               childFramePosition=[
                                   -0.00722, -0.005 + (0.01 / constraintNum) * i, 0.05211]
                               )
        for i in range(constraintNum):
            p.createConstraint(parentBodyUniqueId=self.robot_1,
                               parentLinkIndex=self.linkNameToID_1["L_L_L_3"],
                               childBodyUniqueId=self.robot_2,
                               childLinkIndex=self.linkNameToID_2["L_L_2"],
                               jointType=p.JOINT_POINT2POINT,
                               jointAxis=[0, 0, 0],
                               parentFramePosition=[
                                   -0.005 + (0.01 / constraintNum) * i, -0.055, 0],
                               childFramePosition=[
                                   0.04278, -0.005 + (0.01 / constraintNum) * i, 0.102113]
                               )
        for i in range(constraintNum):
            p.createConstraint(parentBodyUniqueId=self.robot_1,
                               parentLinkIndex=self.linkNameToID_1["L_R_L_1"],
                               childBodyUniqueId=self.robot_2,
                               childLinkIndex=self.linkNameToID_2['L_R_2'],
                               jointType=p.JOINT_POINT2POINT,
                               jointAxis=[0, 0, 0],
                               parentFramePosition=[
                                   -0.005 + (0.01 / constraintNum) * i, -0.06, 0],
                               childFramePosition=[
                                   -0.005 + (0.01 / constraintNum) * i, 0.05211, -0.00722]
                               )
        for i in range(constraintNum):
            p.createConstraint(parentBodyUniqueId=self.robot_1,
                               parentLinkIndex=self.linkNameToID_1["L_R_L_3"],
                               childBodyUniqueId=self.robot_2,
                               childLinkIndex=self.linkNameToID_2["L_R_2"],
                               jointType=p.JOINT_POINT2POINT,
                               jointAxis=[0, 0, 0],
                               parentFramePosition=[
                                   -0.005 + (0.01 / constraintNum) * i, -0.055, 0],
                               childFramePosition=[
                                   -0.005 + (0.01 / constraintNum) * i, 0.102113, 0.04278]
                               )

        # constraint between robot_1 and robot_3
        for i in range(constraintNum):
            p.createConstraint(parentBodyUniqueId=self.robot_1,
                               parentLinkIndex=self.linkNameToID_1["R_L_L_1"],
                               childBodyUniqueId=self.robot_3,
                               childLinkIndex=self.linkNameToID_3['L_L_2'],
                               jointType=p.JOINT_POINT2POINT,
                               jointAxis=[0, 0, 0],
                               parentFramePosition=[
                                   -0.005 + (0.01 / constraintNum) * i, -0.06, 0],
                               childFramePosition=[
                                   -0.00722, -0.005 + (0.01 / constraintNum) * i, 0.05211]
                               )
        for i in range(constraintNum):
            p.createConstraint(parentBodyUniqueId=self.robot_1,
                               parentLinkIndex=self.linkNameToID_1["R_L_L_3"],
                               childBodyUniqueId=self.robot_3,
                               childLinkIndex=self.linkNameToID_3["L_L_2"],
                               jointType=p.JOINT_POINT2POINT,
                               jointAxis=[0, 0, 0],
                               parentFramePosition=[
                                   -0.005 + (0.01 / constraintNum) * i, -0.055, 0],
                               childFramePosition=[
                                   0.04278, -0.005 + (0.01 / constraintNum) * i, 0.102113]
                               )
        for i in range(constraintNum):
            p.createConstraint(parentBodyUniqueId=self.robot_1,
                               parentLinkIndex=self.linkNameToID_1["R_R_L_1"],
                               childBodyUniqueId=self.robot_3,
                               childLinkIndex=self.linkNameToID_3['L_R_2'],
                               jointType=p.JOINT_POINT2POINT,
                               jointAxis=[0, 0, 0],
                               parentFramePosition=[
                                   -0.005 + (0.01 / constraintNum) * i, -0.06, 0],
                               childFramePosition=[
                                   -0.005 + (0.01 / constraintNum) * i, 0.05211, -0.00722]
                               )
        for i in range(constraintNum):
            p.createConstraint(parentBodyUniqueId=self.robot_1,
                               parentLinkIndex=self.linkNameToID_1["R_R_L_3"],
                               childBodyUniqueId=self.robot_3,
                               childLinkIndex=self.linkNameToID_3["L_R_2"],
                               jointType=p.JOINT_POINT2POINT,
                               jointAxis=[0, 0, 0],
                               parentFramePosition=[
                                   -0.005 + (0.01 / constraintNum) * i, -0.055, 0],
                               childFramePosition=[
                                   -0.005 + (0.01 / constraintNum) * i, 0.102113, 0.042785]
                               )
