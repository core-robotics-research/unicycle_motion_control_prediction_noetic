#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import ast 

import sys
import select
import termios
import tty
from threading import Thread


class PositionTeleopKey():
    def __init__(self):
        """
        Initialization
        """
        self.position = [0.0, 0.0, 0.0]  # [x, y, z]
        self.position_update = [0.0, 0.0, 0.0] # [x, y, z]
        self.pause_motion = True 
        self.mode = 'pause'
        self.ispressed = False
        self.exit = False
        self.keySettings = termios.tcgetattr(sys.stdin)
        self.step_size = 0.1
        self.initialization_time = rospy.Time()
        self.initialization_delay = 0.5 # Delay in second

    def print_instruction(self):
        """
        Print Control Instructions
        """

        print('***********************************************************')
        print('Position keyboard control to change 3D position\n')
        print('     w    e            a/d : increase/descrease x position')
        print('  a     d              w/x : increase/descrease y position')
        print('     x    c            e/c : increase/descrease z position')
        print('                       p/m : pause/move')
        print(' ')
        print('Press <ctrl-c> or <q> to exit')
        print('***********************************************************')

    def print_settings(self):
        """
        Print Current Control Settings
        """
        print('Current Settings --- Pause: {}, Position: {:.2f} {:.2f} {:.2f}'.format(self.pause_motion, *self.position))

    def getkey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.keySettings)
        return key 
    
    def key_input(self):
        """
        Keyboard Teleoperation
        """
        self.print_instruction()
        self.print_settings()

        kInput = 0
        while not(rospy.is_shutdown() or self.exit):
            key = self.getkey()
            is_valid_input = False # Flag variable if the steering input is changed
            is_position_updated = False

            # -- Position Update --
            if key == 'w':
                self.position_update[1] = self.position_update[1] + self.step_size
                is_valid_input = True
                is_position_updated = True
            if key == 'x':
                self.position_update[1] = self.position_update[1] - self.step_size
                is_valid_input = True
                is_position_updated = True
            if key == 'a':
                self.position_update[0] = self.position_update[0] - self.step_size
                is_valid_input = True
                is_position_updated = True
            if key == 'd':
                self.position_update[0] = self.position_update[0] + self.step_size
                is_valid_input = True
                is_position_updated = True
            if key == 'e':
                self.position_update[2] = self.position_update[2] + self.step_size
                is_valid_input = True
                is_position_updated = True
            if key == 'c':
                self.position_update[2] = self.position_update[2] - self.step_size
                is_valid_input = True
                is_position_updated = True

            # -- Additional function keys --
            if key == 'p':
                self.pause_motion= True
                self.mode = 'pause'
                is_valid_input = True
            if key == 'm':
                self.pause_motion = False
                self.mode = 'move '
                is_valid_input = True
            if (key == 'q') or (key == '\x03'):
                self.exit = True 

            # Publish updated pose
            if is_position_updated:
                self.publish_pose()  

            # Publish control instruction and settings
            if is_valid_input:
                kInput = kInput + 1
                if kInput > 10:
                    kInput = 0
                    self.print_instruction()    
                self.print_settings()
   
    def initial_pose_callback(self, msg):
        """
        Update position
        """
        self.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def publish_pose(self):
        """
        Publish pose 
        """

        if not(self.pause_motion):
            next_position = [self.position[k] + self.position_update[k] for k in range(3)]
            self.pose_msg.header.stamp = rospy.Time.now()
            self.pose_msg.pose.position.x = next_position[0]
            self.pose_msg.pose.position.y = next_position[1]
            self.pose_msg.pose.position.z = next_position[2]
            self.pose_pub.publish(self.pose_msg)        
            self.position = next_position
            self.position_update = [0.0, 0.0, 0.0]
        
    def start(self):
        """
    	Start a position_teleop_key node
    	"""

        # Create and register a subscriber for initial pose with the master
        rospy.Subscriber('initial_pose', PoseStamped, self.initial_pose_callback, queue_size=1)

        # Create and register control command publisher with the master
        self.pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=1, latch=True)
        self.pose_msg = PoseStamped()

        # Create and register pause_motion publisher with the master
        pause_pub = rospy.Publisher('/pause_motion', Bool, queue_size=1);
        pause_cmd = Bool()

        # Register node with the master
        rospy.init_node('position_teleop_key', anonymous=True)
        # Load parameters
        self.step_size = rospy.get_param('~step_size', default=0.1)  # Step Size
        self.position = ast.literal_eval(rospy.get_param('~initial_position', default=[0.0, 0.0, 0.0]))  # Initial Position
        self.rate = rospy.get_param('~rate', default=10) # Publication Rate

        rospy.sleep(1)  # Wait for one second

        # Start a thread for keyboard inputs and pose updates
        key_thread = Thread(target=self.key_input, daemon=True)   
        key_thread.start()

        # Command publication loop
        rate = rospy.Rate(self.rate) # Publication Rate
        while not(rospy.is_shutdown() or self.exit): 
            if not(self.pause_motion):
                pause_cmd.data = self.pause_motion
                pause_pub.publish(pause_cmd)
      
            # Loop delay
            rate.sleep()    

if __name__ == '__main__':
    try:
        position_teleop_key = PositionTeleopKey().start()
    except rospy.ROSInterruptException:
        pass
