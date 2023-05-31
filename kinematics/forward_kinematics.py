'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity

import autograd.numpy as numpy

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                        'RShoulder': ['RShoulderPitch', 'RShoulderRoll', 'RShoulderPitch', 'RElbowRoll', 'RElbowYaw'],
                        'LShoulder': ['LShoulderPitch', 'LShoulderRoll', 'LShoulderPitch', 'LElbowRoll', 'LElbowYaw'],
                        'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                        'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       }
        self.ranges = {'HeadYaw': (-2.0857, 2.0857),
                       'HeadPitch': (-0.6720, 0.5149),
                       'LShoulderPitch': (-2.0857, 2.0857),
                       'RShoulderPitch': (-2.0857, 2.0857),
                       'LShoulderRoll': (-0.3142, 1.3265),
                       'RShoulderRoll': (-0.3142, 1.3265),
                       'LElbowYaw': (-2.0857, 2.0857),
                       'RElbowYaw': (-2.0857, 2.0857),
                       'LElbowRoll': (-1.5446, 0.0349),
                       'RElbowRoll': (-1.5446, 0.0349),
                       'LHipYawPitch': (-1.145303, 0.740810),
                       'RHipYawPitch': (-1.145303, 0.740810),
                       'LHipRoll': (-0.379472, 0.790477),
                       'RHipRoll': (-0.379472, 0.790477),
                       'LHipPitch': (-1.535889, 0.484090),
                       'RHipPitch': (-1.535889, 0.484090),
                       'LKneePitch': (-0.092346, 2.112528),
                       'RKneePitch': (-0.092346, 2.112528),
                       'LAnklePitch': (-1.189516, 0.922747),
                       'RAnklePitch': (-1.189516, 0.922747),
                       'LAnkleRoll': (-0.397880, 0.769001),
                       'RAnkleRoll': (-0.397880, 0.769001),}
        self.translation = {'HeadYaw': [0, 0, 0.1265],
                            'HeadPitch': [0, 0, 0],
                            'LShoulderPitch': [0, 0.098, 0.1],
                            'RShoulderPitch': [0, 0.098, 0.1],
                            'LShoulderRoll': [0, 0, 0],
                            'RShoulderRoll': [0, 0, 0],
                            'LElbowYaw': [0.105, 0.015, 0],
                            'RElbowYaw': [0.105, 0.015, 0],
                            'LElbowRoll': [0, 0, 0],
                            'RElbowRoll': [0, 0, 0],
                            'LWristYaw': [0.05595, 0, 0],
                            'RWristYaw': [0.05595, 0, 0],
                            'LHipYawPitch': [0, 0.050, -0.085],
                            'RHipYawPitch': [0, 0.050, -0.085],
                            'LHipRoll': [0, 0, 0],
                            'RHipRoll': [0, 0, 0],
                            'LHipPitch': [0, 0, 0],
                            'RHipPitch': [0, 0, 0],
                            'LKneePitch': [0, 0, -0.1],
                            'RKneePitch': [0, 0, -0.1],
                            'LAnklePitch': [0, 0, -0.10290],
                            'RAnklePitch': [0, 0, -0.10290],
                            'LAnkleRoll': [0, 0, 0],
                            'RAnkleRoll': [0, 0, 0],}

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE

        sin = numpy.sin(joint_angle)
        cos = numpy.cos(joint_angle)

        # joint Roll represents x axis Rotation
        if 'Roll' in joint_name:
            T = numpy.dot(T, numpy.matrix([[1, 0, 0, 0],
                            [0, cos, -sin, 0],
                            [0, sin, cos, 0],
                            [0, 0, 0, 1]]))
        # joint Pitch Corresponds to y axis Rotation
        elif 'Pitch' in joint_name:
            T = numpy.dot(T, numpy.matrix([[cos, 0, sin, 0],
                                    [0, 1, 0, 0],
                                    [-sin, 0, cos, 0],
                                    [0, 0, 0, 1]]))
        # z axis Rotation
        elif 'Yaw' in joint_name:
            T = numpy.dot(T, numpy.matrix([[cos, sin, 0, 0],
                            [-sin, cos, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]]))

        # now we need to add the translation
        for j in range(0,3):
            T[3, j] = self.translation[joint_name][j]

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = numpy.dot(T, Tl)
                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
