#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, PoseStamped, PolygonStamped
from tf.transformations import euler_from_quaternion

from unicycle_motion_control import dgc_truncatedicecreamcone_motion_prediction


class TruncatedIcecreamConeMotionPrediction:
    def __init__(self):

        # Initialize state variables
        self.position = [0.0, 0.0]
        self.orientation = 0.0
        self.goal_position = [0.0, 0.0]
        self.rate = 10
        self.resolution = 60
        self.frame_id = 'world'
        
    def pose_callback(self, msg):
        # Get pose information from message
        self.position = [msg.pose.position.x, msg.pose.position.y]
        quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.orientation = euler_from_quaternion(quat)[2]
        self.frame_id = msg.header.frame_id

    def goal_pose_callback(self, msg):
        # This function will be called when a new PoseStamped message is received
        self.goal_position = [msg.pose.position.x, msg.pose.position.y]


    def start(self):

        # Create and register the ROS node with the master
        rospy.init_node('truncatedicecreamcone_motion_prediction_node', anonymous=True)

        # Set up subscriber to get initial pose information
        rospy.Subscriber('pose', PoseStamped, self.pose_callback, queue_size=1)

        # Create a subscriber to the "goal" topic
        rospy.Subscriber("goal", PoseStamped, self.goal_pose_callback, queue_size=1)

        # Set up control loop to run at 10 Hz
        self.rate = rospy.get_param("~rate", default=10.0) 

        # Set up resolution of the motion prediction
        self.resolution = rospy.get_param("~resolution", default=60)

        # Set up motion direction
        self.motion_direction = rospy.get_param("~motion_direction", default="forward")

        # Set up a publisher to publish motion prediction as a polygon. We need PolygonStamped to visualize in rviz
        self.motion_prediction_pub = rospy.Publisher('motion_prediction', PolygonStamped, queue_size=1)

        # Set up the motion prediction PolygonStamped message
        motion_prediction_msg = PolygonStamped()
        motion_prediction_msg.header.frame_id = self.frame_id

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():


            # Get the motion prediction for the unicycle under adaptive headway control
            motion_prediction_polygon = dgc_truncatedicecreamcone_motion_prediction(self.position, self.orientation, self.goal_position, self.motion_direction, self.resolution)

            # Update the motion prediction message
            # Clear the message
            motion_prediction_msg.polygon.points = []
            # Add the points to the message
            for p in motion_prediction_polygon:

                motion_prediction_msg.polygon.points.append(Point())
                motion_prediction_msg.polygon.points[-1].x = p[0]
                motion_prediction_msg.polygon.points[-1].y = p[1]
                motion_prediction_msg.polygon.points[-1].z = 0.0

            # Publish the motion prediction as a PoseArray message
            self.motion_prediction_pub.publish(motion_prediction_msg)

            # Sleep for the remainder of the loop
            rate.sleep()
            

if __name__ == '__main__':
    try:
        truncatedicecreamcone_motion_prediction = TruncatedIcecreamConeMotionPrediction().start()
    except rospy.ROSInterruptException:
        pass