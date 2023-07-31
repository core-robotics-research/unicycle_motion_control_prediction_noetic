#!/usr/bin/env python3

from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PolygonStamped
import rospy
from tf.transformations import euler_from_quaternion

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.patches as patches
from skimage import measure

from map_tools import OccupancyGridMap2D
from governed_unicycle_navigation import is_in_domain, governor_vector_field

class PlotReferenceGovernedNavigation:
    """ 
    Robot-Governor Navigation Visualization using matplotlib
    """

    def __init__(self):
        self.map_copy = None
        self.path_copy = None
        self.robot_position = None
        self.robot_orientation = None
        self.governor_position = None
        self.motionpolygon = None
        
        self.frame_id = 'world' # Visualization frame id
        self.rate = 10 # Plot update rate
        self.figure_options = {'figwidth': 6.4, 'figheight': 6.4} # Refer to **kwargs of matplotlib.figure.Figure
        self.axes_options = {'aspect': 'equal'} # Refer to **kwargs of matplotlib.axes.Axes
        self.grid_options = {'visible': True, 'linewidth': 0.5} # Refer to **kwargs of matplotlib.axes.Axes.grid
        self.plot_options = {'color': 'r', 'marker': 'o', 'linestyle': '', 'markersize': 2.0} # Refer to **kwargs of matplotlib.pyplot.plot
        self.patch_options = {'facecolor':'b', 'edgecolor': None, 'alpha': 0.3} # Refer to **kwargs of matplotlib.patches.Patch
        self.domain_resolution = 200
        self.field_resolution = 40

    def robot_pose_callback(self, msg):
        # Get initial pose information from message
        self.robot_position = [msg.pose.position.x, msg.pose.position.y]
        quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.robot_orientation = euler_from_quaternion(quat)[2]
        self.robot_frame_id = msg.header.frame_id
        
    def governor_pose_callback(self, msg):
        # Get initial pose information from message
        self.governor_position = [msg.pose.position.x, msg.pose.position.y]
        quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.governor_orientation = euler_from_quaternion(quat)[2]
        self.governor_frame_id = msg.header.frame_id

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

        self.path_copy = path

    def motionpolygon_callback(self, msg):
        # This function will be called when a new PolygonStamped message is received
        
        points = msg.polygon.points
        points = np.array([[point.x, point.y] for point in points])
        self.motionpolygon = points

    def plot_loop(self):  
        """
        Visualization Loop
        """

        # Get Occupancy Grid Map Data 
        resolution = self.map_copy.resolution()
        map = self.map_copy.occupancy_matrix()
        map_size = np.shape(map)[0] * resolution

        # Get Domain
        XLim = self.map_copy.xworldlimits()
        YLim = self.map_copy.yworldlimits()
        X, Y = np.meshgrid(np.linspace(XLim[0], XLim[1], self.domain_resolution), np.linspace(YLim[0], YLim[1], self.domain_resolution))
        XY = np.column_stack((X.flatten(), Y.flatten()))
        IN = is_in_domain(self.map_copy, self.path_copy, XY, frame='world')
        IN = IN.reshape(X.shape)
        contours = measure.find_contours(IN, 0.5)
        I = np.ravel_multi_index((contours[0][:, 0].astype(int), contours[0][:, 1].astype(int)), X.shape)

        # Get Vector Field
        Xfield, Yfield = np.meshgrid(np.linspace(XLim[0], XLim[1], self.field_resolution), np.linspace(YLim[0], YLim[1], self.field_resolution))
        XYfield = np.column_stack((Xfield.flatten(), Yfield.flatten()))
        INfield = is_in_domain(self.map_copy, self.path_copy, XYfield, frame='world')        
        XYfield = XYfield[INfield.flatten(),:]
        collision_distances, coordinates = self.map_copy.distance_to_collision(XYfield)
        UV = governor_vector_field(XYfield, 1, collision_distances, self.path_copy)
        UV = UV / np.sqrt(np.sum(UV**2, axis=1)[:, np.newaxis])

        # Create Figure for Visualization
        plt.ion()
        fig = plt.figure()
        fig.set(**self.figure_options)
        ax = fig.add_subplot(1,1,1)
        ax.set(**self.axes_options)
        ax.grid(**self.grid_options)
        ax.axis('equal')
        ax.set_xlim([0, map_size])
        ax.set_ylim([0, map_size])

        # Add visualization Elements
        plt.imshow(map, extent=[0, map_size, 0, map_size], cmap='binary', origin="lower")

        domain_plot = plt.fill(X.flatten()[I], Y.flatten()[I], color = [1 , 0, 0], alpha=0.20)
        field_plot = plt.quiver(XYfield[:,0], XYfield[:,1], UV[:,0], UV[:,1])

        path = ax.plot(self.path_copy[:,0], self.path_copy[:,1], 'r', linewidth=2)
        start = patches.Circle((self.path_copy[0,0], self.path_copy[0,1]), radius=self.robot_radius, facecolor=[0, 0.6, 0])
        goal = patches.Circle((self.path_copy[-1,0], self.path_copy[-1,1]), radius=self.robot_radius, facecolor=[1, 0, 0])
        ax.add_patch(start)
        ax.add_patch(goal)

        # save robot and governor body position
        robot_position_data = np.array([self.robot_position[0], self.robot_position[1]])
        governor_position_data = np.array([self.governor_position[0], self.governor_position[1]])
        robot_path_plot = ax.plot(robot_position_data[0], robot_position_data[1], 'b', linewidth=2)
        governor_path_plot = ax.plot(governor_position_data[0], governor_position_data[1], color=[0, 0.6, 0], linewidth=2.5)

        # draw governor
        governor_body = patches.Circle((self.governor_position[0], self.governor_position[1]), radius=self.robot_radius, facecolor=[0, 0.6, 0])
        ax.add_patch(governor_body)

        # draw robot
        R = self.robot_radius
        thwl = np.linspace(5 * np.pi / 6, np.pi / 6, 60)  # Angle samples for the visualization of the left robot wheel
        thwr = np.linspace(7 * np.pi / 6, 11 * np.pi / 6, 60)  # Angle sample for the visualization of the right robot wheel

        robot_body_plot = patches.Circle((self.robot_position[0], self.robot_position[1]), radius=R, facecolor=[0, 0.75, 0.75])
        robot_left_wheel_plot = patches.Polygon(np.array([self.robot_position[0] + R * np.cos(thwl + self.robot_orientation), self.robot_position[1] + R * np.sin(thwl + self.robot_orientation)]).T, facecolor='k')
        robot_right_wheel_plot = patches.Polygon(np.array([self.robot_position[0] + R * np.cos(thwr + self.robot_orientation), self.robot_position[1] + R * np.sin(thwr + self.robot_orientation)]).T, facecolor='k')
        ax.add_patch(robot_body_plot)
        ax.add_patch(robot_left_wheel_plot)
        ax.add_patch(robot_right_wheel_plot)    

        thick = 0.05
        thdir = np.concatenate((np.array([np.pi / 2, -np.pi / 2]), np.linspace(-np.arcsin(thick / R), np.arcsin(thick / R), 60)))
        mdir = np.concatenate((np.array([thick, thick]), np.tile(R, 60)))
        robot_direction_plot = patches.Polygon(np.array([self.robot_position[0] + mdir * np.cos(thdir + self.robot_orientation), self.robot_position[1] + mdir * np.sin(thdir + self.robot_orientation)]).T, facecolor='k')
        ax.add_patch(robot_direction_plot)

        # draw motion polygon
        motion_polygon_plot = patches.Polygon(self.motionpolygon, facecolor='y', alpha=0.75)
        ax.add_patch(motion_polygon_plot)

        # get robot and governor on top z-order
        robot_body_plot.set_zorder(10)
        robot_left_wheel_plot.set_zorder(10)
        robot_right_wheel_plot.set_zorder(10)
        robot_direction_plot.set_zorder(10)
        governor_body.set_zorder(10)
        robot_path_plot[0].set_zorder(10)
        governor_path_plot[0].set_zorder(10)


        fig.canvas.draw_idle()
        fig.canvas.flush_events()

        # Update Visualization in a Loop
        rate = rospy.Rate(self.rate) 
        while not rospy.is_shutdown():

            # update robot and governor body positions
            governor_body.center = (self.governor_position[0], self.governor_position[1])
            robot_body_plot.center = (self.robot_position[0], self.robot_position[1])

            # append the robot and governor positions
            governor_position_data = np.vstack((governor_position_data, np.array([self.governor_position[0], self.governor_position[1]])))
            robot_position_data = np.vstack((robot_position_data, np.array([self.robot_position[0], self.robot_position[1]])))
            # update robot and governor paths
            governor_path_plot[0].set_xdata(governor_position_data[:,0])
            governor_path_plot[0].set_ydata(governor_position_data[:,1])
            robot_path_plot[0].set_xdata(robot_position_data[:,0])
            robot_path_plot[0].set_ydata(robot_position_data[:,1])
            

            # update robot wheel positions
            robot_left_wheel_plot.set_xy(np.array([self.robot_position[0] + R * np.cos(thwl + self.robot_orientation), self.robot_position[1] + R * np.sin(thwl + self.robot_orientation)]).T)
            robot_right_wheel_plot.set_xy(np.array([self.robot_position[0] + R * np.cos(thwr + self.robot_orientation), self.robot_position[1] + R * np.sin(thwr + self.robot_orientation)]).T)

            # update robot direction
            robot_direction_plot.set_xy(np.array([self.robot_position[0] + mdir * np.cos(thdir + self.robot_orientation), self.robot_position[1] + mdir * np.sin(thdir + self.robot_orientation)]).T)

            # update motion polygon
            motion_polygon_plot.set_xy(self.motionpolygon)


            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            
            self.pause_pub.publish(self.pause_cmd)
            
            rate.sleep()

    def start(self):
        """
        Start ROS Node
        """

        # Initialize the ROS node
        rospy.init_node('plot_navigation', anonymous=True)
        
        # ROS Node Parameters
        self.frame_id = rospy.get_param('~frame_id', default=self.frame_id)
        self.rate = rospy.get_param('~rate', default=self.rate)
        self.figure_options = rospy.get_param('~figure_options', default=self.figure_options)        
        self.axes_options = rospy.get_param('~axes_options', default=self.axes_options)
        self.grid_options = rospy.get_param('~grid_options', default=self.grid_options)
        self.plot_options = rospy.get_param('~plot_options', default=self.plot_options)
        self.domain_options = rospy.get_param('~domain_resolution', default=self.domain_resolution)
        self.field_options = rospy.get_param('~field_resolution', default=self.field_resolution)

        # Set up robot radius
        self.robot_radius = rospy.get_param("~robot_radius", default=0.5)  

        # Create a subscriber to the "map" topic to compute safety level later on
        rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)

        # Create a subscriber to the "path" topic
        rospy.Subscriber("path", Path, self.path_callback, queue_size=1)

        # Create a subscriber to the "robot_pose" topic
        rospy.Subscriber("robot_pose", PoseStamped, self.robot_pose_callback, queue_size=1)

        # Create a subscriber to the "governor_pose" topic
        rospy.Subscriber("governor_pose", PoseStamped, self.governor_pose_callback, queue_size=1)

        # Create a subscriber to the "motion_prediction" topic
        rospy.Subscriber("motion_polygon", PolygonStamped, self.motionpolygon_callback, queue_size=1)

        # Create and register pause_motion publisher with the master
        self.pause_pub = rospy.Publisher('/pause_motion', Bool, queue_size=1);
        self.pause_cmd = Bool()
        self.pause_motion = False
        self.pause_cmd.data = self.pause_motion

        # Wait for the map to be published
        while not rospy.is_shutdown() and self.map_copy is None:
            rospy.sleep(0.1)

        # Wait for the path to be published
        while not rospy.is_shutdown() and self.path_copy is None:
            rospy.sleep(0.1)

        
        rospy.loginfo("--------------------WAITING FOR THE TOPICS TO BE READY FOR VISUALIZATION-------------------")
        rospy.sleep(3)
        rospy.loginfo("--------------------VISUALIZATION STARTED-------------------")


        # Start Visualization Loop
        self.plot_loop()


if __name__ == '__main__':
    try:
        plot_domain = PlotReferenceGovernedNavigation().start()    
    except rospy.ROSInterruptException:
        pass
