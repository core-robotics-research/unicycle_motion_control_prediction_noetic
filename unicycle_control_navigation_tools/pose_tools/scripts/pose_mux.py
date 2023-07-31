#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class PoseMux():
    def __init__(self):
        """
        Initialization
        """
        self.topics = None
        self.locks = None
        self.locks_time = {}
        self.topics_time = {}
        self.topic_priority = 0.0
        self.lock_priority = 255
        self.last_topic_time = 0.0
        self.last_topic_timeout = 0.0
    
    def callback_topic(self, data, topic):
        """
        Callback for pose mux topics
        """
        # Determine maximum active lock priority
        self.lock_priority = 0.0
        for lock in self.locks:
            if (rospy.get_time() - self.locks_time[lock["topic"]]) > lock["timeout"]:
                self.lock_priority = max(self.lock_priority, lock["priority"])
                default_lock_enable = False

        # Determine maximum active topic priority
        self.topic_priority = self.lock_priority
        for mytopic in self.topics:
            if (rospy.get_time() - self.topics_time[mytopic["topic"]]) < mytopic["timeout"]:
                self.topic_priority = max(self.topic_priority, mytopic["priority"])
        
        # Publish topic message if it satisfies the maximum topic priority
        if topic["priority"] >= self.topic_priority:
            self.topics_time[topic["topic"]] = rospy.get_time()
            self.pose_pub.publish(data)
            self.last_topic_time = self.topics_time[topic["topic"]]
            self.last_topic_timeout = topic["timeout"]

    def callback_lock(self, data, lock):
        """
        Callback for pose mux locks
        """
        self.locks_time[lock["topic"]] = rospy.get_time()

    def start(self):
        """
        Start a pose_mux ROS node
        """
        # Create and register control command publisher with the master
        self.pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=1)

        # Register node with the master
        rospy.init_node('pose_mux', anonymous=True)
        
        # Load node parameters
        self.topics = rospy.get_param('~topics', None)
        self.locks = rospy.get_param('~locks', None)

        for topic in self.topics:
            # Topic time initialization
            self.topics_time[topic["topic"]] = rospy.get_time()
            # Create and register joy topic subscriber with the master
            rospy.Subscriber(topic["topic"], PoseStamped, self.callback_topic, topic)

        for lock in self.locks:
            # Lock time initialization
            self.locks_time[lock["topic"]] = rospy.get_time()
            # Create and register joy topic subscriber with the master
            rospy.Subscriber(lock["topic"], Bool, self.callback_lock, lock)


        rospy.spin()


if __name__ == '__main__':
    try:
        pose_mux = PoseMux().start()
    except rospy.ROSInterruptException:
        pass

