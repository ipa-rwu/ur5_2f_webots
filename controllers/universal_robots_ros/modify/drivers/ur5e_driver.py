import copy
import math
import rospy
import yaml

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class UR5e_driver(object):

    def __init__(self, robot):
        self.jointPrefix = []

    def get_joint(self):
        # Read YAML file
        with open("config/ur5e_joints.yaml", 'r') as stream:
            data_loaded = yaml.safe_load(stream)
            self.jointPrefix = data_loaded['ur5e_joints']
            jointPrefix = self.jointPrefix
            return jointPrefix

    def get_current_joint_position(self):


    def interp_cubic(self, p0, p1, t_abs):
        """Perform a cubic interpolation between two trajectory points."""
        T = (p1.time_from_start - p0.time_from_start).to_sec()
        t = t_abs - p0.time_from_start.to_sec()
        q = [0] * 6
        qdot = [0] * 6
        qddot = [0] * 6
        for i in range(len(p0.positions)):
            a = p0.positions[i]
            b = p0.velocities[i]
            c = (-3 * p0.positions[i] + 3 * p1.positions[i] - 2 * T * p0.velocities[i] - T * p1.velocities[i]) / T**2
            d = (2 * p0.positions[i] - 2 * p1.positions[i] + T * p0.velocities[i] + T * p1.velocities[i]) / T**3

            q[i] = a + b * t + c * t**2 + d * t**3
            qdot[i] = b + 2 * c * t + 3 * d * t**2
            qddot[i] = 2 * c + 6 * d * t
        return JointTrajectoryPoint(positions=q, velocities=qdot, accelerations=qddot, time_from_start=rospy.Duration(t_abs))

