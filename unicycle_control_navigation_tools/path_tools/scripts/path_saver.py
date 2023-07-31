#!/usr/bin/env python3

import rospy
import yaml
from nav_msgs.msg import Path

class PathSaver:
    def __init__(self):
        self.path_file = ""

    def path_callback(self, path):

        frame_id = path.header.frame_id    
        
        points = []
        for pose_stamped in path.poses:
            points.append([pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z])

        with open(self.path_file, 'w') as file:
            yaml.dump({'frame_id': frame_id, 'points': points}, file, default_flow_style=None)
        
        rospy.loginfo('Path is saved!')
        rospy.signal_shutdown("Path is saved!")    
    
    def start(self):
        rospy.init_node('path_saver', anonymous=True)
        self.path_file = rospy.get_param('~path_file', default='temp.yaml')
        rospy.Subscriber('path', Path, self.path_callback)
        rospy.loginfo('Path saver started!')
        rospy.spin()
           
if __name__ == '__main__':
    try:
        path_saver = PathSaver().start()
    except rospy.ROSInterruptException:
        pass
