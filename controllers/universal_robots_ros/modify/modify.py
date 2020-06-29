# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""universal_robot_ros controller."""

import rospy
from controller import Robot
from joint_state_publisher import JointStatePublisher
from trajectory_follower import TrajectoryFollower
from rosgraph_msgs.msg import Clock
###
from gripper_trajectory_follower_2joints import TrajectoryFollowerGripper

#from drivers.ur5e_driver import UR5e_driver




rospy.init_node('ur_driver', disable_signals=True)

jointPrefix = rospy.get_param('prefix', '')
gripperjointPrefix = rospy.get_param('prefix', '')
if jointPrefix:
    print('Setting prefix to %s' % jointPrefix)

robot = Robot()
#ur5e_driver = UR5e_driver()


jointStatePublisher = JointStatePublisher(robot, jointPrefix)


trajectoryFollower = TrajectoryFollower(robot, jointStatePublisher, jointPrefix)
trajectoryFollower.start()
####
trajectoryFollowergripper = TrajectoryFollowerGripper(robot, jointStatePublisher, jointPrefix)
trajectoryFollowergripper.start()

# we want to use simulation time for ROS
clockPublisher = rospy.Publisher('clock', Clock, queue_size=1)
if not rospy.get_param('use_sim_time', False):
    rospy.logwarn('use_sim_time is not set!')

timestep = int(robot.getBasicTimeStep())

while robot.step(timestep) != -1 and not rospy.is_shutdown():
    jointStatePublisher.publish()
    trajectoryFollower.update()
    trajectoryFollowergripper.update()

    # pulish simulation clock
    msg = Clock()
    time = robot.getTime()
    msg.clock.secs = int(time)
    # round prevents precision issues that can cause problems with ROS timers
    msg.clock.nsecs = round(1000 * (time - msg.clock.secs)) * 1.0e+6
    clockPublisher.publish(msg)
