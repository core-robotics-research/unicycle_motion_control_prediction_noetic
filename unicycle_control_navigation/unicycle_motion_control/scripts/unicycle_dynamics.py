#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from unicycle_motion_control import unicycle_discrete_update


class UnicycleDynamics:
    def __init__(self):

        # Initialize state variables
        self.position = [0.0, 0.0]
        self.orientation = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0 
        self.rate = 10.0
        self.frame_id = 'world'
        
    def initial_pose_callback(self, msg):
        # Get initial pose information from message
        self.position = [msg.pose.position.x, msg.pose.position.y]
        quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.orientation = euler_from_quaternion(quat)[2]
        self.frame_id = msg.header.frame_id

    def cmd_vel_callback(self, msg):
        # Get twist components from message
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def start(self):

        # Create and register the ROS node with the master
        rospy.init_node('unicycle_dynamics', anonymous=True)

        # Set up subscriber to get Twist messages for velocity control
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

        # Set up subscriber to get initial pose information
        rospy.Subscriber('initial_pose', PoseStamped, self.initial_pose_callback)

        # Set up publisher to publish PoseStamped messages for new pose information
        self.pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=1)

        # Set up control loop to run at 10 Hz
        self.rate = rospy.get_param("~rate", default=10.0) 
        self.velocity_decay = rospy.get_param("~velocity_decay", default=0.1)

        # Publish new pose as PoseStamped message
        pose_msg = PoseStamped()

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():

            # Integrate unicycle dynamics to get new pose
            self.position, self.orientation = unicycle_discrete_update(self.position, self.orientation, self.linear_velocity, self.angular_velocity, 1/self.rate)

            # Apply velocity decay
            self.linear_velocity = self.linear_velocity * self.velocity_decay
            self.angular_velocity = self.angular_velocity * self.velocity_decay

            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = self.frame_id
            pose_msg.pose.position.x = self.position[0]
            pose_msg.pose.position.y = self.position[1]
            pose_msg.pose.position.z = 0.0
            quat = quaternion_from_euler(0.0, 0.0, self.orientation)
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            self.pose_pub.publish(pose_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        unicycle_dynamics = UnicycleDynamics().start()
    except rospy.ROSInterruptException:
        pass
