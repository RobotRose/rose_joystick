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

def twist_is_zero(twist):
    zeros = [   twist.linear.x  == 0.0,
                twist.linear.y  == 0.0,
                twist.linear.y  == 0.0,
                twist.angular.x == 0.0,
                twist.angular.y == 0.0,
                twist.angular.y == 0.0]
    all_zero = all(zeros)
    return all_zero

class ArmControlInterpreter(JoystickInterpreter):
    arm_parameters = {"left"    :{"index":1, "topic":"/left_arm/cmd_vel"}, 
                      "right"   :{"index":0, "topic":"/right_arm/cmd_vel"}}

    gripper_open = 100
    gripper_closed = 10

    def __init__(self, settings, side, linear_scaling_factor=0.1, angular_scaling_factor=0.1):
        """
        Instantiate a new ArmControlInterpreter
        @param side side of the arm, i.e. left or right. Some parameters are determined based on this.
        """
        super(ArmControlInterpreter, self).__init__()

        self.settings = settings

        self.linear_scaling_factor = linear_scaling_factor
        self.angular_scaling_factor = angular_scaling_factor

        self.arm_client = actionlib.SimpleActionClient('/arms', arm_controller.msg.manipulateAction)

        self.side = side
        self.arm_index = ArmControlInterpreter.arm_parameters[side]["index"]

        self.twist = Twist()

        self.previous_button_state = []
        self.previous_twist = None

        self.gripper_width = ArmControlInterpreter.gripper_open
        self.open_close_toggle = self.settings["open_close"]

    def process(self, joystick_msg):
        if not self.previous_button_state:
            self.previous_button_state = joystick_msg.buttons
        
        goal = manipulateGoal()
        goal.arm             = self.arm_index

        if (joystick_msg.buttons[self.open_close_toggle] != self.previous_button_state[self.open_close_toggle] and not joystick_msg.buttons[self.open_close_toggle]):
            pass
            # goal.required_action = MOVE_GRIPPER
            # if self.gripper_width == ArmControlInterpreter.gripper_closed: #Open it!
            #     rospy.loginfo("Opening gripper")
            #     self.gripper_width      = ArmControlInterpreter.gripper_open
            # else: #Close it!
            #     rospy.loginfo("Closing gripper")
            #     self.gripper_width      = ArmControlInterpreter.gripper_closed
        else:
            goal.required_action = SET_VELOCITY
            self.twist = Twist()
            self.twist.linear.x = joystick_msg.axes[self.settings["linear_x"]["axis"]] * self.settings["linear_x"]["scale"]
            self.twist.linear.y = joystick_msg.axes[self.settings["linear_y"]["axis"]] * self.settings["linear_y"]["scale"]
            self.twist.linear.z = joystick_msg.axes[self.settings["linear_z"]["axis"]] * self.settings["linear_z"]["scale"]

            self.twist.angular.x = joystick_msg.axes[self.settings["angular_x"]["axis"]] * self.settings["angular_x"]["scale"] if "angular_x" in self.settings else 0.0
            self.twist.angular.y = joystick_msg.axes[self.settings["angular_y"]["axis"]] * self.settings["angular_y"]["scale"] if "angular_y" in self.settings else 0.0
            self.twist.angular.z = joystick_msg.axes[self.settings["angular_z"]["axis"]] * self.settings["angular_z"]["scale"] if "angular_z" in self.settings else 0.0

            goal.required_velocity = self.twist

        goal.required_gripper_width = self.gripper_width

        #When the user leaves the joystick at rest, we don't want to send 0 speeds all the time, but let other send goal as well.
        #But after letting the joystick go to rest, send a 0 speed-goal at least once so that the arms correctly stop
        #
        #So: When the signal goes to 0, one step later, the previous goes to 0. 
        #If we only send the signal when the previous != 0, then there has been 1 step where the signal was 0 and we did send it.
        #See visual explanation at http://10.201.11.11/redmine/attachments/download/57/2014-10-02%2014.53.28.jpg
        if self.previous_twist and not twist_is_zero(self.previous_twist):
            print "sending goal"
            self.arm_client.send_goal(goal)
        
        # if goal.required_action == MOVE_GRIPPER:
        #     rospy.loginfo("Waiting for gripper to be closed/opened...")
        #     success = self.arm_client.send_goal_and_wait(goal, rospy.Duration.from_sec(2.0))
        #     rospy.loginfo("Gripper is closed/opened")

        self.previous_button_state = joystick_msg.buttons
        self.previous_twist = self.twist


    def __str__(self):
        return "{0} arm".format(self.side).capitalize()

