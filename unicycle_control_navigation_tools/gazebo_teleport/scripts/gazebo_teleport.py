#!/usr/bin/env python3

import rospy
import tf2_ros
import time
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, GetModelState
from geometry_msgs.msg import PoseStamped, TransformStamped

class GazeboTeleport:

    def __init__(self):
        self.model_name = ""

    def update_transform_frame(self):
        """
        Updates the transform message and send it with the TF broadcaster
        """
        current_position = self.get_model_state(self.model_name, 'world')

        self.tf_msg.transform.translation.x = current_position.pose.position.x
        self.tf_msg.transform.translation.y = current_position.pose.position.y
        self.tf_msg.transform.translation.z = current_position.pose.position.z
        self.tf_msg.transform.rotation.x = current_position.pose.orientation.x
        self.tf_msg.transform.rotation.y = current_position.pose.orientation.y
        self.tf_msg.transform.rotation.z = current_position.pose.orientation.z
        self.tf_msg.transform.rotation.w = current_position.pose.orientation.w

        self.tf_msg.header.stamp = rospy.Time.now()
        self.tf_pub.sendTransform(self.tf_msg)


    def pose_callback(self, data):
        try:
            # Update the SetModelStateRequest message instance to teleport simplerobot
            self.model_state_request.model_state.pose = data.pose
            self.model_state_request.model_state.reference_frame = data.header.frame_id
            # Call the service to teleport simplerobot
            self.set_model_state(self.model_state_request)
            #self.update_transform_frame()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def start(self):

        # Register the ROS node with the master
        rospy.init_node('gazebo_teleport', anonymous=True)

        # Register the model pose subscriber with the master
        rospy.Subscriber("pose", PoseStamped, self.pose_callback, queue_size=1)

        # Wait until the /gazebo/get_model_state service is available
        rospy.loginfo('Wait for the /gazebo/get_model_state service')
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.loginfo('Connected to /gazebo/get_model_state service')

        # Wait until the /gazebo/set_model_state service is available
        rospy.loginfo('Wait for the /gazebo/set_model_state service')
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.loginfo('Connected to /gazebo/set_model_state service')

        # ROS Node Parameters
        self.model_name = rospy.get_param('~name')
        self.base_frame = rospy.get_param('~tf_frame', f'{self.model_name}/base_footprint')
        map_frame = rospy.get_param('~tf_child_frame', f'{self.model_name}/map')
        self.tf_rate = rospy.get_param('~tf_rate', 10)

        self.tf_pub = tf2_ros.TransformBroadcaster()
        self.tf_msg = TransformStamped()
        self.tf_msg.child_frame_id = self.base_frame
        self.tf_msg.header.frame_id = map_frame
        self.tf_msg.transform.rotation.w = 1 #  So it is a valid quaternion

        self.model_state_request = SetModelStateRequest()
        self.model_state_request.model_state.model_name = self.model_name

        # Create a handle for calling the gazebo set_model_state service
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        rate = rospy.Rate(self.tf_rate)
        while not rospy.is_shutdown():
            self.update_transform_frame()
            current_position = self.get_model_state(self.model_name, 'world')
            self.tf_msg.header.stamp = rospy.Time.now()
            self.tf_msg.transform.translation.x = current_position.pose.position.x
            self.tf_msg.transform.translation.y = current_position.pose.position.y
            self.tf_msg.transform.translation.z = current_position.pose.position.z
            self.tf_msg.transform.rotation.x = current_position.pose.orientation.x
            self.tf_msg.transform.rotation.y = current_position.pose.orientation.y
            self.tf_msg.transform.rotation.z = current_position.pose.orientation.z
            self.tf_msg.transform.rotation.w = current_position.pose.orientation.w

            self.tf_pub.sendTransform(self.tf_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        time.sleep(1)  # Give the model a bit of time to spawn in
        GazeboTeleport().start()
    except rospy.ROSInterruptException:
        pass
