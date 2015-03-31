import roslib; roslib.load_manifest("rose_joystick")
import rospy

import actionlib
import arm_controller.msg

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from arm_controller.msg import manipulateAction, manipulateGoal

from interpreter import Submode
from arm import ArmControlInterpreter, MOVE_GRIPPER, SET_VELOCITY
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

    def __init__(self, settings, side, linear_scaling_factor=0.1, angular_scaling_factor=0.1):
        """
        Instantiate a new ArmControlInterpreterWithSubmodes
        @param side side of the arm, i.e. left or right. Some parameters are determined based on this.
        """
        super(ArmControlInterpreterWithSubmodes, self).__init__(settings, side, linear_scaling_factor, angular_scaling_factor)

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

    def process(self, joystick_msg):
        down, released, downed = self.button_state.derive_button_events(joystick_msg.buttons)
        
        goal = manipulateGoal()
        goal.arm             = self.arm_index

        if self.open_close_toggle in released:
            pass
        else:
            goal.required_action = SET_VELOCITY

            active_mode = self.submodes.get(tuple(sorted(down)), None) #Get the mode that is activated by the button that are pressed down now
            if active_mode:
                rospy.loginfo("Submode: {0}_{1}".format(self, active_mode))
                self.twist = active_mode.sub_process(joystick_msg)
            else:
                rospy.loginfo("No submode activated by {1}. {0}".format([mode.usage() for mode in self.submodes.values()], down))
                self.twist = Twist()

            goal.required_velocity = self.twist

        goal.required_gripper_width = self.gripper_width

        self.arm_client.send_goal(goal) #And wait... brings the rate down!

    def __str__(self):
        return "{0} arm".format(self.side).capitalize()