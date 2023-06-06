'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref

from xmlrpc.client import ServerProxy

import threading

from keyframes import rightBackToStand

import pickle

import time

from numpy.matlib import identity, matrix

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        #call nonblocking
        t1 = threading.Thread(target=self.proxy.proxy.execute_keyframes_nonblocking, args=[keyframes], daemon=True)
        t1.start()
        #wait a short moment
        time.sleep(0.05)
        return t1

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        t1 = threading.Thread(target=self.proxy.proxy.set_transform_nonblocking, args=[effector_name, transform], daemon=True)
        t1.start()
        time.sleep(0.05)
        return t1



class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    proxy = None

    def __init__(self):
        self.post = PostHandler(self)
        self.proxy = ServerProxy("http://localhost:3000")

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.proxy.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.proxy.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.proxy.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        return self.proxy.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        return self.proxy.get_transform(name)
        # YOUR CODE HERE

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE

        self.proxy.set_transform(effector_name, transform)


if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE


    agent.set_angle('RElbowRoll', -0.5)

    T = T = [[1,0,0,0],
        [0,1,0,0],
        [0,0,1,0],
        [0,0,0,1]]
    T[-1][1] = 0.05
    T[-1][2] = -0.26
    T[-1][0] = 0.2

    agent.set_transform("LLeg", T)
    agent.execute_keyframes(rightBackToStand())
    print(agent.set_angle('RElbowRoll', 0.5))
    print(agent.get_angle("RElbowRoll"))

    agent.post.set_transform('LLeg', T)
    print(agent.get_posture())
    time.sleep(15)
    agent.post.execute_keyframes(rightBackToStand())
    print(agent.get_angle("LShoulderRoll"))


