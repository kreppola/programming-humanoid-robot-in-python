'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello, leftBackToStand, fallDownBack, stretch


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

        self.startTime = self.perception.time

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        times = self.keyframes[1]
        keys = self.keyframes[2]

        # duration of the movement
        stopTime = max([times[i][-1] for i in range(len(keyframes[0]))], default=-1.0)

        # if movement is finished, restart movement
        if perception.time - self.startTime > stopTime:
            #self.startTime = perception.time
            self.keyframes = ([], [], [])

        # current time of the movement
        time = perception.time - self.startTime

        # initialize joints
        for joint in perception.joint:
            target_joints[joint] = 0.0

        # for each joint
        for joint in range(len(keyframes[0])):
            # get the current step
            current_step = len(times[joint])
            for step in range(0, len(times[joint])):
                if time < times[joint][step]:
                    current_step = step
                    break

            # when last step
            if current_step == len(times[joint]):
                break

            # get the points for the bezier
            if current_step == 0:
                p_0 = (0,0)
                p_1 = (0,0)
                p_3 = (keys[joint][current_step][0], times[joint][current_step])
                p_2 = (p_3[0] + keys[joint][current_step][1][2], p_3[1] + keys[joint][current_step][1][1])
            else:
                p_0 = (keys[joint][current_step - 1][0], times[joint][current_step - 1])
                p_1 = (p_0[0] + keys[joint][current_step - 1][2][2], p_0[1] + keys[joint][current_step - 1][2][1])
                p_3 = (keys[joint][current_step][0], times[joint][current_step])
                p_2 = (p_3[0] + keys[joint][current_step][1][2], p_3[1] + keys[joint][current_step][1][1])

            # find the value for i
            i = 0.0
            bt = 0.0
            while i < 1.0 and bt < time:
                bt = (1-i)**3 * p_0[1] + 3 * (1-i)**2 * i * p_1[1] + 3 * (1-i) * i**2 * p_2[1] + i**3 * p_3[1]
                i = i + 0.01

            # find value joint
            bv = (1-i)**3 * p_0[0] + 3* (1-i)**2 * i * p_1[0] + 3 * (1-i) * i**2 * p_2[0] + i**3 * p_3[0]
            target_joints[self.keyframes[0][joint]] = bv

        return target_joints

    def restart_keyframe(perception):
        self.startTime = self.perception

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes =  stretch() # CHANGE DIFFERENT KEYFRAMES
    agent.run()
