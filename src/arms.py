import roslib; roslib.load_manifest("rose_joystick")
import rospy

import actionlib
import arm_controller.msg

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from arm_controller.msg import manipulateAction, manipulateGoal

from interpreter import JoystickInterpreter, Submode
import threading
MOVE_GRIPPER=2
SET_VELOCITY=4

class ArmsControlInterpreter(JoystickInterpreter):
    def __init__(self, settings, linear_scaling_factor=0.1, angular_scaling_factor=0.1):
        super(ArmsControlInterpreter, self).__init__()

        self.settings = settings

        self.linear_scaling_factor = linear_scaling_factor
        self.angular_scaling_factor = angular_scaling_factor

        self.arm_client = actionlib.SimpleActionClient('/arms', arm_controller.msg.manipulateAction)

        self.left_arm_velocity_publisher = rospy.Publisher("/left_arm/cmd_vel", Twist)
        self.right_arm_velocity_publisher = rospy.Publisher("/right_arm/cmd_vel", Twist)

        self.selected_arm_publisher = self.left_arm_velocity_publisher
        self.selected_arm = 0

        self.stopper = None
        self.publisher_thread = None
        self.twist = Twist()

        self.previous_button_state = []

    def start(self):
        self.timer = rospy.Timer(rospy.Duration(0.1), self.repeat_message, oneshot=False)

        rospy.loginfo("Arm is waiting for server...")
        connected = self.arm_client.wait_for_server(timeout=rospy.Duration(1.0))
        if connected: 
            rospy.loginfo("Arm server found")
        else:
            rospy.logerr("Arm server not (yet) found. Commands may be delayed or not arrive at all.")

    def repeat_message(self, *args, **kwargs):
        self.selected_arm_publisher.publish(self.twist)

    def stop(self):
        self.timer.shutdown()
        self.twist = Twist() #Empty twist, everything is zero
        self.left_arm_velocity_publisher.publish(self.twist)
        self.right_arm_velocity_publisher.publish(self.twist)

    def process(self, joystick_msg):
        if not self.previous_button_state:
            self.previous_button_state = joystick_msg.buttons

        if self.settings.has_key("switch_left_arm") and \
           joystick_msg.buttons[self.settings["switch_left_arm"]] != self.previous_button_state[self.settings["switch_left_arm"]] and \
           not joystick_msg.buttons[self.settings["switch_left_arm"]]:
            #the buttons is pressed and released
            self.selected_arm_publisher = self.left_arm_velocity_publisher
            self.selected_arm = 1
            rospy.loginfo("Controlling LEFT arm")
        
        if self.settings.has_key("switch_right_arm") and \
           joystick_msg.buttons[self.settings["switch_right_arm"]] != self.previous_button_state[self.settings["switch_right_arm"]] and \
           not joystick_msg.buttons[self.settings["switch_right_arm"]]:
            #the buttons is pressed and released
            self.selected_arm_publisher = self.right_arm_velocity_publisher
            self.selected_arm = 0
            rospy.loginfo("Controlling RIGHT arm")        

        if self.settings.has_key("switch_other_arm") and \
           joystick_msg.buttons[self.settings["switch_other_arm"]] != self.previous_button_state[self.settings["switch_other_arm"]] and \
           not joystick_msg.buttons[self.settings["switch_other_arm"]]:
            #the buttons is pressed and released
            text = "right" if self.selected_arm_publisher == self.left_arm_velocity_publisher else "left"
            self.selected_arm_publisher = self.right_arm_velocity_publisher if self.selected_arm_publisher == self.left_arm_velocity_publisher else self.left_arm_velocity_publisher
            self.selected_arm = 1 if self.selected_arm_publisher == self.left_arm_velocity_publisher else 0 
            rospy.loginfo("Controlling other arm: {0}".format(text))
        
        self.twist = Twist()
        self.twist.linear.x = joystick_msg.axes[self.settings["linear_x"]["axis"]] * self.settings["linear_x"]["scale"]
        self.twist.linear.y = joystick_msg.axes[self.settings["linear_y"]["axis"]] * self.settings["linear_y"]["scale"]
        self.twist.linear.z = joystick_msg.axes[self.settings["linear_z"]["axis"]] * self.settings["linear_z"]["scale"]

        self.twist.angular.x = joystick_msg.axes[self.settings["angular_x"]["axis"]] * self.settings["angular_x"]["scale"]
        self.twist.angular.y = joystick_msg.axes[self.settings["angular_y"]["axis"]] * self.settings["angular_y"]["scale"]
        self.twist.angular.z = joystick_msg.axes[self.settings["angular_z"]["axis"]] * self.settings["angular_z"]["scale"]

        goal = manipulateGoal()
        goal.required_action = 3
        goal.arm             = self.selected_arm
        goal.required_velocity = self.twist

        self.arm_client.send_goal(goal)

        self.previous_button_state = joystick_msg.buttons


    def __str__(self):
        return "Arms"