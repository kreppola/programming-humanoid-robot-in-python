'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity

import numpy as np

from scipy.optimize import fmin, fmin_tnc

import random

from math import sqrt

class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []

        # YOUR CODE HERE
        
        # define the function
        func = lambda joint_values: self.error_function(transform, joint_values, joint_names)
        # initialize joint_names and joint_angles
        joint_names = self.chains[effector_name]
        joint_angles = [0 for j in range(0, len(joint_names))]
        # load the ranges of joint
        bounds = [self.ranges[j] for j in joint_names]
        # optimize the function
        return fmin_tnc(func, joint_angles, bounds = bounds, approx_grad = True, maxfun=1000)[0]


    def endEffectorPos(self, joint_values, joint_names):
        """
        Determines pos of Effector
        """
        Ts = [identity(4)]
        for joint in range(len(joint_values)):
            angle = joint_values[joint]
            Tl = self.local_trans(joint_names[joint], angle)
            T = np.dot(Ts[-1], Tl)
            Ts.append(T)
        return Ts[-1]

    def error_function(self, target, joint_values, joint_names):

        # Get Positon of Eff
        Te = self.endEffectorPos(joint_values, joint_names)

        # Error
        error = target - Te
        error = np.linalg.norm(error)

        return error  

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.keyframes = ([], [], [])  # the result joint angles have to fill in
        jvalues = self.inverse_kinematics(effector_name, transform)

        for m in range(0, len(jvalues)):
            self.keyframes[0].append(self.chains[effector_name][m])
            self.keyframes[1].append([5, 10])
            self.keyframes[2].append([[jvalues[m], [3, 0, 0], [3, 0, 0]],
                                    [0, [3, 0, 0], [3, 0, 0]]])

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    T[-1, 0] = 0.2
    agent.set_transforms('LLeg', T)
    agent.run()
