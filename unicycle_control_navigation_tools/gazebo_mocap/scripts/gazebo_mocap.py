#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from gazebo_msgs.msg import ModelStates

class GazeboMocap:
    """
    Gazebo Mocap Node Class
    """
    def __init__(self):
        """
        Initialization
        """
        self.rate = [] # Mocap Update Rate
        self.posepub = {} # Object pose publishers
        self.twistpub = {} # Object twist publishers
        self.posemsg = PoseStamped()
        self.twistmsg = TwistStamped()

    def callback_modelstates(self, data):
        """
        Callback for gazebo model_states updates in the gazebo_msgs/ModelStates format
        """

        for k in range(len(data.name)):
            if data.name[k] in self.posepub:
                self.posemsg.pose = data.pose[k]
                self.twistmsg.twist = data.twist[k]
                self.posepub[data.name[k]].publish(self.posemsg)
                self.twistpub[data.name[k]].publish(self.twistmsg)
            else:
                self.posepub[data.name[k]] = rospy.Publisher('mocap/' + data.name[k] +'/pose', PoseStamped, queue_size=1)
                self.twistpub[data.name[k]] = rospy.Publisher('mocap/' + data.name[k] +'/twist', TwistStamped, queue_size=1)
                rospy.loginfo("Gazebo mocap - New object:  %s", data.name[k])
                self.posemsg.pose = data.pose[k]
                self.twistmsg.twist = data.twist[k]
                self.posepub[data.name[k]].publish(self.posemsg)
                self.twistpub[data.name[k]].publish(self.twistmsg)
        self.rate.sleep()

    def start(self):

        # Register node with the master
        rospy.init_node('gazebo_mocap', anonymous=True)
        mocap_rate = rospy.get_param('mocap/rate', 50.0)
        self.rate = rospy.Rate(mocap_rate) # Mocap Update Rate
        mocap_frame = rospy.get_param('mocap/frame', 'world')
        self.posemsg.header.frame_id = mocap_frame
        self.twistmsg.header.frame_id = mocap_frame

        # Register the gazebo/model_states subscriber with the master
        rospy.Subscriber('gazebo/model_states', ModelStates,  callback=self.callback_modelstates, queue_size=1)
        rospy.loginfo('Gazebo mocap is started!')

        # Keep node running until it is stopped
        rospy.spin()

if __name__ == '__main__':
    try:
        mocap = GazeboMocap().start()
    except rospy.ROSInterruptException:
        pass
