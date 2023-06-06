'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent

from xmlrpc.server import SimpleXMLRPCServer

import pickle

import time

import threading

from numpy.matlib import identity, matrix

import numpy as np

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''

    # YOUR CODE HERE

    def __init__(self, ip, port, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):

        #super(ServerAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        super(ServerAgent, self).__init__()
        self.start_server(ip, port)

    def start_server(self, ip, port):

        self.server = SimpleXMLRPCServer((ip, port), allow_none=True)
        self.server.register_instance(self)
        t1 = threading.Thread(target=self.server.serve_forever, daemon=True)
        t1.start()
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.perception.joint[joint_name] = angle
        return 2

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.posture


    def execute_keyframes_nonblocking(self, keyframes):

        t1 = threading.Thread(target=self.execute_keyframes, args=[keyframes])
        t1.start()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE

        self.keyframes = keyframes
        self.startTime = self.perception.time
        while self.keyframes != ([], [], []):
            time.sleep(0.5)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transforms[name].tolist()

    def set_transform_nonblocking(self, effector_name, transform):

        t1 = threading.Thread(target=self.set_transform, args=[effector_name, transform])
        t1.start()


    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        transform = matrix(transform)
        self.set_transforms(effector_name, transform)
        self.execute_keyframes(self.keyframes)


if __name__ == '__main__':
    agent = ServerAgent("localhost", 3000)
    agent.run()