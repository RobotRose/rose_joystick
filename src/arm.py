import roslib; roslib.load_manifest("rose_joystick")
import rospy

import actionlib
import rose_arm_controller_msgs.msg

from geometry_msgs.msg import Twist, TwistStamped
from rose_arm_controller_msgs.msg import set_gripper_widthAction, set_gripper_widthGoal, set_velocityAction, set_velocityGoal

from interpreter import LatchingJoystickInterpreter, Submode, twist_is_zero
import threading

class ArmControlInterpreter(LatchingJoystickInterpreter):
    gripper_open   = 0.15
    gripper_closed = 0.0

    def __init__(self, settings, name, linear_scaling_factor=0.1, angular_scaling_factor=0.1):
        """
        Instantiate a new ArmControlInterpreter
        @param side side of the arm.
        """
        super(ArmControlInterpreter, self).__init__(rate=0.033)

        self.settings = settings

        self.linear_scaling_factor = linear_scaling_factor
        self.angular_scaling_factor = angular_scaling_factor

        self.arm_velocity_client = actionlib.SimpleActionClient('/arm_controller/velocity', rose_arm_controller_msgs.msg.set_velocityAction)
        self.arm_gripper_client  = actionlib.SimpleActionClient('/arm_controller/gripper_width', rose_arm_controller_msgs.msg.set_gripper_widthAction)

        self.arm_name = name

        self.velocity_goal = set_velocityGoal()
        self.gripper_goal  = set_gripper_widthGoal()

        self.gripper_width = ArmControlInterpreter.gripper_open
        self.open_close_toggle = self.settings["open_close"]

    def when_active(self):
        self.arm_velocity_client.send_goal(self.velocity_goal)

    def become_inactive(self):
        self.velocity_goal     = set_velocityGoal()
        self.velocity_goal.arm = self.arm_name
        self.velocity_goal.required_velocity = TwistStamped()
        self.velocity_goal.required_velocity.header.stamp     = rospy.Time.now()
        self.velocity_goal.required_velocity.header.frame_id  = "base_link"

        self.arm_velocity_client.send_goal(self.velocity_goal)

    def define_width(self):
        if self.gripper_width == ArmControlInterpreter.gripper_closed: #Open it!
            rospy.loginfo("Opening gripper")
            self.gripper_width      = ArmControlInterpreter.gripper_open
        else: #Close it!
            rospy.loginfo("Closing gripper")
            self.gripper_width      = ArmControlInterpreter.gripper_closed
        return self.gripper_width

    def process(self, joystick_msg, down, released, downed):        
        self.velocity_goal     = set_velocityGoal()
        self.velocity_goal.arm = self.arm_name
        self.velocity_goal.required_velocity = TwistStamped()
        self.velocity_goal.required_velocity.header.stamp     = rospy.Time.now()
        self.velocity_goal.required_velocity.header.frame_id  = "base_link"

        twist = Twist()
        if self.open_close_toggle in released:
            self.define_width()
            self.gripper_goal.arm            = self.arm_name
            self.gripper_goal.required_width = self.gripper_width

            rospy.loginfo("Waiting for gripper to be closed/opened...")
            success = self.arm_gripper_client.send_goal_and_wait(self.gripper_goal, rospy.Duration.from_sec(3.5))
            rospy.loginfo("Gripper is closed/opened")
        else:
            twist.linear.x = joystick_msg.axes[self.settings["linear_x"]["axis"]] * self.settings["linear_x"]["scale"]
            twist.linear.y = joystick_msg.axes[self.settings["linear_y"]["axis"]] * self.settings["linear_y"]["scale"]
            twist.linear.z = joystick_msg.axes[self.settings["linear_z"]["axis"]] * self.settings["linear_z"]["scale"]

            twist.angular.x = joystick_msg.axes[self.settings["angular_x"]["axis"]] * self.settings["angular_x"]["scale"] if "angular_x" in self.settings else 0.0
            twist.angular.y = joystick_msg.axes[self.settings["angular_y"]["axis"]] * self.settings["angular_y"]["scale"] if "angular_y" in self.settings else 0.0
            twist.angular.z = joystick_msg.axes[self.settings["angular_z"]["axis"]] * self.settings["angular_z"]["scale"] if "angular_z" in self.settings else 0.0

            self.velocity_goal.required_velocity.twist = twist

            self.active = not twist_is_zero(twist)

    def __str__(self):
        return "{0} arm".format(self.arm_name).capitalize()

