#!/usr/bin/env python3

import rospy
import yaml
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathServer:
    """
    ROS Path Server to publish a nav_msgs/Path from a yaml file
    """
    def __init__(self):
        """
        """
        self.path_file = ""

    def read_path(self):
        """
        Read and create a nav_msgs/Path message from a path yaml file 
        """

        with open(self.path_file, 'r') as f:
            path_yaml = yaml.safe_load(f)

        path = Path()
        path.header.stamp = rospy.Time.now()
        try:
            path.header.frame_id = rospy.get_param('~frame_id')
        except KeyError:
            path.header.frame_id = path_yaml['frame_id']

        for point in path_yaml['points']:
            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = point[2]
            path.poses.append(pose_stamped)
        return path    
    
    def start(self):
        """
        Start the path server node to published a nav_msgs/Path from a yaml file 
        """

        rospy.init_node('path_server', anonymous=True)
        self.path_file = rospy.get_param('~path_file', default='temp.yaml')
        self.path_pub = rospy.Publisher('path', Path, queue_size=1, latch=True)

        path_msg = self.read_path()
        self.path_pub.publish(path_msg)
        
        rospy.loginfo('Path server started!')
        rospy.spin()
           
if __name__ == '__main__':
    try:
        path_server = PathServer().start()
    except rospy.ROSInterruptException:
        pass
