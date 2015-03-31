import roslib; roslib.load_manifest("rose_joystick")
import rospy

import actionlib
import arm_controller.msg

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from arm_controller.msg import manipulateAction, manipulateGoal

from interpreter import LatchingJoystickInterpreter, Submode, twist_is_zero
import threading
MOVE_GRIPPER=2
SET_VELOCITY=4

class ArmControlInterpreter(LatchingJoystickInterpreter):
    arm_parameters = {"left"    :{"index":1, "topic":"/left_arm/cmd_vel"}, 
                      "right"   :{"index":0, "topic":"/right_arm/cmd_vel"}}

    gripper_open = 100
    gripper_closed = 10

    def __init__(self, settings, side, linear_scaling_factor=0.1, angular_scaling_factor=0.1):
        """
        Instantiate a new ArmControlInterpreter
        @param side side of the arm, i.e. left or right. Some parameters are determined based on this.
        """
        super(ArmControlInterpreter, self).__init__(rate=0.4)

        self.settings = settings

        self.linear_scaling_factor = linear_scaling_factor
        self.angular_scaling_factor = angular_scaling_factor

        self.arm_client = actionlib.SimpleActionClient('/arms', arm_controller.msg.manipulateAction)

        self.side = side
        self.arm_index = ArmControlInterpreter.arm_parameters[side]["index"]

        self.goal = manipulateGoal()
        self.twist = Twist()

        self.previous_twist = None

        self.gripper_width = ArmControlInterpreter.gripper_open
        self.open_close_toggle = self.settings["open_close"]

    def when_active(self):
        self.arm_client.send_goal_and_wait(self.goal)

    def become_inactive(self):
        self.goal = manipulateGoal()
        self.goal.required_action = SET_VELOCITY
        self.goal.arm = self.arm_index
        self.goal.required_velocity = Twist()
        self.goal.required_gripper_width = self.gripper_width
        self.arm_client.send_goal_and_wait(self.goal)

    def define_width(self):
        if self.gripper_width == ArmControlInterpreter.gripper_closed: #Open it!
            rospy.loginfo("Opening gripper")
            self.gripper_width      = ArmControlInterpreter.gripper_open
        else: #Close it!
            rospy.loginfo("Closing gripper")
            self.gripper_width      = ArmControlInterpreter.gripper_closed
        return self.gripper_width

    def process(self, joystick_msg, down, released, downed):        
        self.goal       = manipulateGoal()
        self.goal.arm   = self.arm_index

        if self.open_close_toggle in released:
            self.define_width()
            self.goal.required_action = MOVE_GRIPPER
        else:
            self.goal.required_action = SET_VELOCITY
            self.twist = Twist()
            self.twist.linear.x = joystick_msg.axes[self.settings["linear_x"]["axis"]] * self.settings["linear_x"]["scale"]
            self.twist.linear.y = joystick_msg.axes[self.settings["linear_y"]["axis"]] * self.settings["linear_y"]["scale"]
            self.twist.linear.z = joystick_msg.axes[self.settings["linear_z"]["axis"]] * self.settings["linear_z"]["scale"]

            self.twist.angular.x = joystick_msg.axes[self.settings["angular_x"]["axis"]] * self.settings["angular_x"]["scale"] if "angular_x" in self.settings else 0.0
            self.twist.angular.y = joystick_msg.axes[self.settings["angular_y"]["axis"]] * self.settings["angular_y"]["scale"] if "angular_y" in self.settings else 0.0
            self.twist.angular.z = joystick_msg.axes[self.settings["angular_z"]["axis"]] * self.settings["angular_z"]["scale"] if "angular_z" in self.settings else 0.0

            self.goal.required_velocity = self.twist

        self.active = not twist_is_zero(self.twist)
        
        self.goal.required_gripper_width = self.gripper_width
        if self.goal.required_action == MOVE_GRIPPER:
            rospy.loginfo("Waiting for gripper to be closed/opened...")
            success = self.arm_client.send_goal_and_wait(self.goal, rospy.Duration.from_sec(2.0))
            rospy.loginfo("Gripper is closed/opened")


    def __str__(self):
        return "{0} arm".format(self.side).capitalize()

