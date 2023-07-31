#!/usr/bin/env python3

import rospy
import xml.etree.ElementTree as ET
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf
import numpy as np


class ModelStaticTransformBroadcaster:
    """
    Model Static Transform Broadcaster using a sdf-model specification
    """

    def __init__(self):
        self.namespace = ""
        self.model_description = ""
        

    def start(self):    

        # Register ROS node with the master
        rospy.init_node('model_static_tf_broadcaster')

        # Get the model description from the parameter server
        self.model_description = rospy.get_param('model_description')
        self.namespace = rospy.get_namespace()

        # Construct model element tree
        model_etree = ET.fromstring(self.model_description)
        
        tf_pub = StaticTransformBroadcaster()
        tf_msg_list = []

        for joint in model_etree.iter('joint'):

            if joint.attrib['type'] == 'fixed':
                
                # Parent and child link names and poses where a pose array consists of (x, y, z, roll, pitch, yaw)
 
                parent_name = joint.find('./parent').text
                child_name = joint.find('./child').text
                parent_pose = model_etree.find(f".//*/link/[@name='{parent_name}']/pose")
                child_pose = model_etree.find(f".//*/link/[@name='{child_name}']/pose")
                
                if parent_pose is None:
                    parent_position = [0.0, 0.0, 0.0]
                    parent_quaternion = tf.transformations.quaternion_from_euler(0, 0, 0, axes='sxyz')
                else:
                    parent_pose_vector = [float(x) for x in parent_pose.text.strip().split()]
                    parent_position = parent_pose_vector[0:3]
                    parent_quaternion = tf.transformations.quaternion_from_euler(parent_pose_vector[3], parent_pose_vector[4], parent_pose_vector[5], axes='sxyz') 

                if child_pose is None:
                    child_position = [0.0, 0.0, 0.0]
                    child_quaternion = tf.transformations.quaternion_from_euler(0, 0, 0, axes='sxyz')
                else:
                    child_pose_vector = [float(x) for x in child_pose.text.strip().split()]
                    child_position = child_pose_vector[0:3]
                    child_quaternion = tf.transformations.quaternion_from_euler(child_pose_vector[3], child_pose_vector[4], child_pose_vector[5], axes='sxyz')
                
                # Relative Translation and Rotation from Parent to Child
                position_parent2child = np.array(child_position) - np.array(parent_position)
                quaternion_parent2child= tf.transformations.quaternion_multiply(child_quaternion, tf.transformations.quaternion_inverse(parent_quaternion))
                
                tf_msg = TransformStamped()
                tf_msg.header.stamp = rospy.Time.now()
                tf_msg.header.frame_id = self.namespace + parent_name
                tf_msg.child_frame_id = self.namespace + child_name    
                tf_msg.transform.translation.x = position_parent2child[0]
                tf_msg.transform.translation.y = position_parent2child[1]
                tf_msg.transform.translation.z = position_parent2child[2]
                tf_msg.transform.rotation.x = quaternion_parent2child[0]
                tf_msg.transform.rotation.y = quaternion_parent2child[1]
                tf_msg.transform.rotation.z = quaternion_parent2child[2]
                tf_msg.transform.rotation.w = quaternion_parent2child[3]

                tf_msg_list.append(tf_msg)
        
        tf_pub.sendTransform(tf_msg_list)
        rospy.spin()

if __name__ == '__main__':
    try:
        model_tf_pub = ModelStaticTransformBroadcaster().start()
    except rospy.ROSInterruptException:
        pass

