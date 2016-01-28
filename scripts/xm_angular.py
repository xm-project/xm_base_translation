#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import tf
from math import radians, copysign
import math
import PyKDL
from tf.transformations import *

class angularPose(object):
    def __init__(self):
        rospy.init_node('angular_node', anonymous=False)
        rospy.loginfo('the angular pose init ok!!!')
        self.speed = rospy.get_param('~speed', 0.20)
        self.secretindigal = 1.0
        self.tolerance = rospy.get_param('tolerance', math.radians(5))
        self.start = True
        self.cmd_vel = rospy.Publisher('mobile_base_controller/smooth_cmd_vel', Twist, queue_size=5)
        self.base_frame = rospy.get_param('~base_frame', '/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        rospy.on_shutdown(self.shutdown)
        self.rate = 30
        self.start_test = True
        r = rospy.Rate(self.rate)
        rospy.loginfo("please input you want ratation angular")
        self.angle = raw_input()
        self.tf_listener = tf.TransformListener()
        rospy.sleep(0.5)
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60))
        rospy.loginfo("start run")
        move_cmd = Twist()
        while not rospy.is_shutdown():
            move_cmd = Twist()
            self.odom_angle = self.get_odom_angle()
            last_angle = self.odom_angle
            turn_angle = 0

            angular_speed = self.speed
            while abs(float(turn_angle)) < abs(float(self.angle)):
                if rospy.is_shutdown():
                    return
                if self.angle < 0:
                    move_cmd.angular.z = -angular_speed
                else:
                    move_cmd.angular.z = angular_speed
                self.cmd_vel.publish(move_cmd)
                r.sleep()
                self.odom_angle = self.get_odom_angle()
                delta_angle = self.secretindigal * self.normalize_angle(self.odom_angle - last_angle)
                turn_angle += delta_angle
                last_angle = self.odom_angle
            rospy.sleep(0.5)
            if move_cmd.angular.z != 0.0:
                self.cmd_vel.publish(move_cmd)
            else:
                break
            self.cmd_vel.publish(Twist())

    def get_odom_angle(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return self.quat_to_angle(Quaternion(*rot))

    def quat_to_angle(self,quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def normalize_angle(self,angle):
        res = angle
        while res > math.pi:
            res -= 2.0 * math.pi
        while res < -math.pi:
            res += 2.0 * math.pi
        return res

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    angularPose()