#!/usr/bin/env python3
# license removed for brevity

import sys
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

class Node():
    def __init__(self,linear_scaling,angular_scaling,name):
        self.linear_scaling = linear_scaling
        self.angular_scaling = angular_scaling
        self.name = name
        self.g_set_state = None

    def callback(self, data):
        rospy.logdebug("RX: Twist "+rospy.get_caller_id())
        rospy.logdebug("\tlinear:")
        rospy.logdebug("\t\tx:%f,y:%f,z:%f"%(data.linear.x,
                                            data.linear.y,
                                            data.linear.z))
        rospy.logdebug("\tangular:")
        rospy.logdebug("\t\tx:%f,y:%f,z:%f"%(data.angular.x,
                                            data.angular.y,
                                            data.angular.z))
        # scaling factors
        linfac = self.linear_scaling
        angfac = self.angular_scaling

        # model state
        motion_state_msg = ModelState()
        motion_state_msg.model_name = self.name
        motion_state_msg.twist.linear.x = data.linear.x
        motion_state_msg.twist.angular.z = data.angular.z
        motion_state_msg.reference_frame = self.name

        print(f"control: {motion_state_msg.twist.linear.x}, {motion_state_msg.twist.angular.z}")

        try:
            resp = self.g_set_state(motion_state_msg)
        except rospy.ServiceException as e:
            print(f"[{self.name}] Service call failed: {e}")



if __name__ == '__main__':

    rospy.init_node('twist2modelstate')

    # ROS Parameters
    # Scaling from Twist.linear.x to (left+right)
    linear_scaling = rospy.get_param('~linear_scaling',0.2)
    # Scaling from Twist.angular.z to (right-left)
    angular_scaling = rospy.get_param('~angular_scaling',0.05)

    rospy.loginfo("Linear scaling=%f, Angular scaling=%f"%(linear_scaling,angular_scaling))

    key = '--keyboard' in sys.argv
    name = "wamv"
    for i,key in enumerate(sys.argv):
      if key=="--name" and i+1 < len(sys.argv):
        name = sys.argv[i+1]
        break
    print(f"name: {name}")

    node=Node(linear_scaling, angular_scaling, name)

    # Service, must before Subscriber
    rospy.wait_for_service('/gazebo/set_model_state')
    node.g_set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    print("start to control!")

    # Subscriber
    rospy.Subscriber("cmd_vel", Twist, node.callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
