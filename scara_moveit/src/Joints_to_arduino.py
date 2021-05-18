#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import JointState
from rospy_tutorials.msg import Floats


class JointPositionsSender(object):
    def __init__(self, pub):
        self._pub = pub
 
    def call_back(self, msg):
        '''
        Subcriber call back method. Takes joints position from Moveit. 
        In the case of revolute joints, convert radians values to degree.
        Publish joints position to Arduino topic.
        '''
        positions = Floats()

        positions.data.append(msg.position[0])
        positions.data.append(math.degrees(msg.position[1]))
        positions.data.append(math.degrees(msg.position[2]))

        #rospy.sleep(1/2)
        self._pub.publish(positions)
    

def main():
    rospy.init_node('Joints_to_aurdino')

    pub = rospy.Publisher('/joints_to_aurdino', Floats, queue_size=1)
    sender = JointPositionsSender(pub)

    rospy.Subscriber('/joint_states', JointState, sender.call_back, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
    

