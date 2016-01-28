#!/usr/bin/env python
import rospy
import roslib
import tf
from math import copysign, sqrt, pow
from geometry_msgs.msg import Twist, Point


class linearMove(object):
    def __init__(self):
        rospy.init_node('forword_move', anonymous=False)
        rospy.loginfo('the linear srv move init ok')
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('please input you want the robot to translation distance?')
        self.test_distance = float(raw_input())
        rospy.loginfo('the distance you want to translation is %s' % self.test_distance)
        self.rate = 100
        r = rospy.Rate(self.rate)
        self.speed = rospy.get_param('~speed', 0.10)
        self.tolerance = rospy.get_param('~tolerance', 0.01)
        self.cmd_vel = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=5)
        self.base_frame = rospy.get_param('~base_frame', '/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1)
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
        rospy.loginfo("start test!!!!")
        # define a bianliang
        self.flag = rospy.get_param('~flag', True)
        # tf get position
        self.position = Point()
        self.position = self.get_position()
        y_start = self.position.y
        x_start = self.position.x
        # publish cmd_vel
        move_cmd = Twist()
        while not rospy.is_shutdown():
            move_cmd = Twist()
            if self.flag:
                self.position = self.get_position()
                distance = float(sqrt(pow((self.position.x - x_start), 2) +
                                      pow((self.position.y - y_start), 2)))
                if self.test_distance > 0:
                    self.error = float(distance - self.test_distance)
                else:
                    self.error = -float(distance + self.test_distance)
                if not self.flag or abs(self.error) < self.tolerance:
                    self.flag = False
                else:
                    move_cmd.linear.x = copysign(self.speed, -1 * self.error)
            else:
                self.position = self.get_position()
                x_start = self.position.x
                y_start = self.position.y
            self.cmd_vel.publish(move_cmd)
            if move_cmd.linear.x != 0.0:
                self.cmd_vel.publish(move_cmd)
            else:
                break
            r.sleep()
        self.cmd_vel.publish(Twist())

    def get_position(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.lookupException):
            rospy.loginfo("TF Exception")
            return

        return Point(*trans)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    rospy.loginfo("we must be the winnner!!!")
    linearMove()
