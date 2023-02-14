import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, String, Bool, Float32
from collections import *
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point
import  math


class Controller:
    def __init__(self):
        args_lateral_dict = {}
        args_lateral_dict['K_P'] = 0.8
        args_lateral_dict['K_I'] = 0.0
        args_lateral_dict['K_D'] = 0.1

        args_longitudinal_dict = {}
        args_longitudinal_dict['K_P'] = 0.206
        args_longitudinal_dict['K_I'] = 0
        args_longitudinal_dict['K_D'] = 0.515

        self._twist_msg = Twist()
        self._current_pose = None
        self._next_goal = PoseStamped()
        self._target_speed = 0.0
        self._current_speed = 0.0

        #SUBCRIBERS
        # self._odometry_subscriber = rospy.Subscriber("uwb_odom_low_pass", Odometry, self.odometry_cb)
        self._odometry_subscriber = rospy.Subscriber("uwb_odom", Odometry, self.odometry_cb)

        self._waypoint_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.waypoint_cb)

        #PUBLISHERS
        self._steering_command_publisher = rospy.Publisher("/steer", Twist, queue_size=10)

        self._vehicle_controller = VehiclePIDController(
            self, args_lateral=args_lateral_dict, args_longitudinal=args_longitudinal_dict)

    def odometry_cb(self, odometry_msg):
        # with self.data_lock:
        self._current_pose = odometry_msg.pose.pose
        
        # self._current_speed = math.sqrt(odometry_msg.twist.twist.linear.x ** 2 + odometry_msg.twist.twist.linear.y ** 2 + odometry_msg.twist.twist.linear.z ** 2) * 3.6
        self.run_step()
        # rospy.loginfo(self._current_speed)
        # print(odometry_msg.twist.twist.linear.x )
    def waypoint_cb(self, goal):
        self._next_goal = goal
        rospy.loginfo("Recieved NEXT GOAL : {}".format(goal))

    def run_step(self):
        if not self._current_pose:
            rospy.loginfo("Waiting for goal")
            return

        target_pose = self._next_goal 
        rospy.loginfo("test")
        
        if self._next_goal.header.frame_id != 'Now stop':
            steering = self._vehicle_controller.run_step(
                self._target_speed, self._current_speed, self._current_pose, target_pose.pose)
        else:
            steering = 0
        self._twist_msg.angular.z = steering
        self._steering_command_publisher.publish(self._twist_msg)
        