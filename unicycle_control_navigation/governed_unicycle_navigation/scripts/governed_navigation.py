#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point
from nav_msgs.msg import OccupancyGrid, Path

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

from map_tools import OccupancyGridMap2D
from unicycle_motion_control import distance_gradient_ctrl, dgc_circle_motion_prediction, dgc_boundedcone_motion_prediction, dgc_icecreamcone_motion_prediction, dgc_truncatedicecreamcone_motion_prediction
from governed_unicycle_navigation import robot_governor_discrete_update_ode45


class ReferenceGovernor:
    def __init__(self):

        self.rate = 10
        self.frame_id = 'world'

        # Initialize state variables        
        self.governor_position = None
        self.robot_position = [0.0, 0.0]
        self.robot_orientation = 0.0

        self.map_copy = None
        self.isvalid_distance_field = False
        self.path = None


    def robot_pose_callback(self, msg):
        # Get initial pose information from message
        self.robot_position = [msg.pose.position.x, msg.pose.position.y]
        quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.robot_orientation = euler_from_quaternion(quat)[2]
        self.robot_frame_id = msg.header.frame_id

    def map_callback(self, msg):
        map_data = np.array(msg.data)/100
        map_matrix = np.reshape(map_data, (msg.info.height, msg.info.width))
        map_resolution = msg.info.resolution


        map_width = round(msg.info.width*map_resolution, 2)
        map_height = round(msg.info.height*map_resolution, 2)

        map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]

        occ_grid = OccupancyGridMap2D(map_width, map_height, map_resolution, map_origin)

        xy = np.indices(map_matrix.shape).reshape(2, -1).T
        occ_grid.set_occupancy(xy, map_data, frame='grid')

        self.map_copy = occ_grid

    def path_callback(self, msg):
        path_points = msg.poses

        num_points = len(path_points)
        path = np.zeros((num_points, 2))
        for i, pose in enumerate(path_points):
            path[i, 0] = pose.pose.position.x
            path[i, 1] = pose.pose.position.y

        self.path = path



    def start(self):

        # Create and register the ROS node with the master
        rospy.init_node('reference_governor', anonymous=True)

        # Create a subscriber to the "map" topic to compute safety level later on
        rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)

        # Create a subscriber to the "path" topic
        rospy.Subscriber("path", Path, self.path_callback, queue_size=1)

        # Create a subscriber to the "robot_pose" topic
        rospy.Subscriber("robot_pose", PoseStamped, self.robot_pose_callback, queue_size=1)

        # Wait for the first map to be published
        while not rospy.is_shutdown() and self.map_copy is None:
            rospy.sleep(0.1)

        # Wait for the first path to be published
        while not rospy.is_shutdown() and self.path is None:
            rospy.sleep(0.1)
        self.governor_position = self.path[0,:]

        # Set up publisher to publish PoseStamped messages for governor pose information
        self.governor_pose_pub = rospy.Publisher('governor_pose', PoseStamped, queue_size=1)

        # Set up publisher to publish PolygonStamped messages for motion polygon information
        self.motion_polygon_pub = rospy.Publisher('motion_polygon', PolygonStamped, queue_size=1)

        # Set up control loop to run at 10 Hz
        self.rate = rospy.get_param("~rate", default=10.0)
        
        # Set up reference governor gain
        governor_gain = rospy.get_param("~governor_gain", default=1.0)

        # Set up robot radius
        robot_radius = rospy.get_param("~robot_radius", default=0.5)

        # Set up convergence tolerance
        tol = rospy.get_param("~convergence_tolerance", default=1e-3)
        
        # Set up motion direction
        motion_direction = rospy.get_param("~motion_direction", default='Bidirectional')

        # Set up integration step
        max_integration_step = rospy.get_param("~max_integration_step", default=5e-2)

        # Set up control method
        control_method = rospy.get_param("~control_method", default='distance_gradient_control')

        # Set up motion prediction method and resolution
        motion_prediction_method = rospy.get_param("~motion_prediction_method", default='circle')
        motion_prediction_resolution = rospy.get_param("~motion_prediction_resolution", default=60)

        motion_prediction_args = {'res': motion_prediction_resolution, 'motion_direction': motion_direction}

        if control_method == 'distance_gradient_control':
            control_fnc = distance_gradient_ctrl
            lingain = rospy.get_param("~linear_gain", default=1.0)
            anggain = rospy.get_param("~angular_gain", default=1.0)
            control_args = {'lingain': lingain, 'anggain': anggain, 'tol': tol, 'motion_direction': motion_direction}

            if motion_prediction_method == 'circle':
                motion_prediction_fnc = dgc_circle_motion_prediction
            elif motion_prediction_method == 'bounded_cone':
                motion_prediction_fnc = dgc_boundedcone_motion_prediction
            elif motion_prediction_method == 'icecream_cone':
                motion_prediction_fnc = dgc_icecreamcone_motion_prediction
            elif motion_prediction_method == 'truncatedicecream_cone':
                motion_prediction_fnc = dgc_truncatedicecreamcone_motion_prediction
            else:
                rospy.logerr("Motion prediction method not recognized. Using circular motion prediction.")
                motion_prediction_fnc = dgc_circle_motion_prediction

        else:
            rospy.logerr("Control method not recognized. Using distance gradient control with circuler motion prediction.")
            control_fnc = distance_gradient_ctrl
            control_args = {'lingain': 1.0, 'anggain': 1.0, 'tol': 1e-3, 'motion_direction': 'Bidirectional'}
            motion_prediction_fnc = dgc_circle_motion_prediction
            motion_prediction_args = {'res': 60, 'motion_direction': 'Bidirectional'}

        # Set up governor pose message type
        governor_msg = PoseStamped()

        # Set up motion polygon message type
        motion_polygon_msg = PolygonStamped()

        rate = rospy.Rate(self.rate)
        
        # Wait for actual map and path to be published
        rospy.sleep(1)

        while not rospy.is_shutdown():


            next_governor_position = robot_governor_discrete_update_ode45(self.robot_position, self.robot_orientation, self.governor_position, governor_gain, robot_radius, 
                                    self.map_copy, self.path, 1/self.rate, max_step = max_integration_step, 
                                    control = control_fnc, control_args = control_args, 
                                    motion_prediction = motion_prediction_fnc, motion_prediction_args = motion_prediction_args)  
        
            motion_ploygon = motion_prediction_fnc(self.robot_position, self.robot_orientation, self.governor_position,  motion_prediction_args)

            rospy.loginfo("Governor position: (%.2f, %.2f)", next_governor_position[0], next_governor_position[1])
            # Publish the next governor pose
            governor_msg.header.stamp = rospy.Time.now()
            governor_msg.header.frame_id = self.frame_id
            governor_msg.pose.position.x = next_governor_position[0]
            governor_msg.pose.position.y = next_governor_position[1]
            governor_msg.pose.position.z = 0.0
            quat = quaternion_from_euler(0.0, 0.0, 0.0)
            governor_msg.pose.orientation.x = quat[0]
            governor_msg.pose.orientation.y = quat[1]
            governor_msg.pose.orientation.z = quat[2]
            governor_msg.pose.orientation.w = quat[3]
            self.governor_pose_pub.publish(governor_msg)
            self.governor_position = next_governor_position

            # Update the motion prediction message            
            # Clear the message
            motion_polygon_msg.polygon.points = []
            motion_polygon_msg.header.stamp = rospy.Time.now()
            motion_polygon_msg.header.frame_id = self.frame_id
            # Add the points to the message
            for point in motion_ploygon:
                motion_polygon_msg.polygon.points.append(Point())
                motion_polygon_msg.polygon.points[-1].x = point[0]
                motion_polygon_msg.polygon.points[-1].y = point[1]
                motion_polygon_msg.polygon.points[-1].z = 0.0
            self.motion_polygon_pub.publish(motion_polygon_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        unicycle_control = ReferenceGovernor().start()
    except rospy.ROSInterruptException:
        pass