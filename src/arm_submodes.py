import roslib; roslib.load_manifest("rose_joystick")
import rospy

import actionlib
import rose_arm_controller_msgs.msg

from geometry_msgs.msg import Twist, TwistStamped
from rose_arm_controller_msgs.msg import set_gripper_widthAction, set_gripper_widthGoal, set_velocityAction, set_velocityGoal

from interpreter import Submode
from arm import ArmControlInterpreter
import threading

class ArmLinearMode(Submode):
    def __init__(self, settings, parent):
        super(ArmLinearMode, self).__init__(settings, parent)

    def sub_process(self, joystick_msg):
        twist = Twist()
        twist.linear.x = joystick_msg.axes[self.settings["linear_x"]["axis"]] * self.settings["linear_x"]["scale"]
        twist.linear.y = joystick_msg.axes[self.settings["linear_y"]["axis"]] * self.settings["linear_y"]["scale"]
        twist.linear.z = joystick_msg.axes[self.settings["linear_z"]["axis"]] * self.settings["linear_z"]["scale"]
        return twist

    def __str__(self):
        return "Linear"

class ArmAngularMode(Submode):
    def __init__(self, settings, parent):
        super(ArmAngularMode, self).__init__(settings, parent)

    def sub_process(self, joystick_msg):
        twist = Twist()
        twist.angular.x = joystick_msg.axes[self.settings["angular_x"]["axis"]] * self.settings["angular_x"]["scale"]
        twist.angular.y = joystick_msg.axes[self.settings["angular_y"]["axis"]] * self.settings["angular_y"]["scale"]
        twist.angular.z = joystick_msg.axes[self.settings["angular_z"]["axis"]] * self.settings["angular_z"]["scale"]
        return twist
        
    def __str__(self):
        return "Rotation"

class ArmControlInterpreterWithSubmodes(ArmControlInterpreter):

    def __init__(self, settings, name, linear_scaling_factor=0.1, angular_scaling_factor=0.1):
        """
        Instantiate a new ArmControlInterpreterWithSubmodes
        @param name name of the arm.
        """
        super(ArmControlInterpreterWithSubmodes, self).__init__(settings, name, linear_scaling_factor, angular_scaling_factor)

        self.submodes = dict()
        try:
            linear = ArmLinearMode(settings['submodes']['linear'], self)
            self.submodes[tuple(sorted(linear.enable_buttons))] = linear
        except KeyError, ke:
            rospy.logwarn("Could not find settings for linear submode of arms")

        try:
            angular = ArmAngularMode(settings['submodes']['angular'], self)
            self.submodes[tuple(sorted(angular.enable_buttons))] = angular
        except KeyError, ke:
            rospy.logwarn("Could not find settings for angular submode of arms")

    def process(self, joystick_msg, down, released, downed):        
        self.velocity_goal      = set_velocityGoal()
        self.velocity_goal.arm  = self.arm_name
        self.velocity_goal.required_velocity = TwistStamped()
        self.velocity_goal.required_velocity.header.stamp     = rospy.Time.now()
        self.velocity_goal.required_velocity.header.frame_id  = "base_link"

        if self.open_close_toggle in released:
            self.define_width()
            self.gripper_goal.arm            = self.arm_name
            self.gripper_goal.required_width = self.gripper_width

            rospy.loginfo("Waiting for gripper to be closed/opened...")
            success = self.arm_gripper_client.send_goal_and_wait(self.gripper_goal, rospy.Duration.from_sec(3.5))
            rospy.loginfo("Gripper is closed/opened")
        else:
            active_mode = self.submodes.get(tuple(sorted(down)), None) #Get the mode that is activated by the button that are pressed down now
            twist = Twist()
            if active_mode:
                rospy.loginfo("Submode: {0}_{1}".format(self, active_mode))
                twist = active_mode.sub_process(joystick_msg)
                self.active = True
            else:
                rospy.loginfo("No submode activated by {1}. {0}".format([mode.usage() for mode in self.submodes.values()], down))
                twist = Twist()
                self.active = False

            self.velocity_goal.required_velocity.twist = twist


    def __str__(self):
        return "{0} arm".format(self.arm_name).capitalize()