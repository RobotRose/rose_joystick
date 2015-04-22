import roslib; roslib.load_manifest("rose_joystick")
import rospy

from geometry_msgs.msg import Twist, TwistStamped

from interpreter import LatchingJoystickInterpreter, Submode, twist_is_small
import threading

class BaseControlInterpreter(LatchingJoystickInterpreter):
    """Convert joystick values to a Twist"""

    def __init__(self, settings, driving_scale=0.1, steering_scale=0.2): #max value for a Joy-axis value is 1.0. Max speed is 0.1m/s
        super(BaseControlInterpreter, self).__init__()
        self.cmd_vel = rospy.Publisher("/manual_cmd_vel", TwistStamped)

        self.settings = settings

        self.stopper = None
        self.publisher_thread = None
        self.twist = Twist()

    def when_active(self):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp  = rospy.get_rostime()
        twist_stamped.twist         = self.twist

        self.cmd_vel.publish(twist_stamped)
        
    def become_inactive(self):
        self.twist = Twist() #Empty twist, everything is zero
        
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp  = rospy.get_rostime()
        twist_stamped.twist         = self.twist

        self.cmd_vel.publish(twist_stamped)

    def process(self, joystick_msg, down, released, downed):
        self.twist = Twist()

        if self.settings.has_key("throttle"):
            throttle_scale = joystick_msg.axes[self.settings["throttle"]["axis"]]+1.0 * self.settings["throttle"]["scale"] #+1 because min=-1. min shouyld be 0
        else:
            throttle_scale = 1.0

        self.twist.linear.x = joystick_msg.axes[self.settings["linear_x"]["axis"]] * self.settings["linear_x"]["scale"] * throttle_scale

        #If the config requires a button to enable strafing
        if self.settings.has_key("press_to_strafe"):
            if not self.settings["press_to_strafe"] in down: 
                inversion_multiplier = 1 #no inversion by default
                if joystick_msg.axes[self.settings["linear_x"]["axis"]] < -0.1: 
                    inversion_multiplier = -1 #Somehow, this makes more sense...
                self.twist.angular.z = joystick_msg.axes[self.settings["angular_z"]["axis"]] * self.settings["angular_z"]["scale"] * inversion_multiplier
            else:
                self.twist.linear.y = joystick_msg.axes[self.settings["linear_y"]["axis"]] * self.settings["linear_y"]["scale"] * throttle_scale
        else: #or is it always on?
            self.twist.angular.z = joystick_msg.axes[self.settings["angular_z"]["axis"]] * self.settings["angular_z"]["scale"]
            self.twist.linear.y = joystick_msg.axes[self.settings["linear_y"]["axis"]] * self.settings["linear_y"]["scale"] * throttle_scale

        self.active = not twist_is_small(self.twist)

    def __str__(self):
        return "Base"

class BaseSteeringMode(Submode):
    def __init__(self, settings, parent):
        super(BaseSteeringMode, self).__init__(settings, parent)

    def sub_process(self, joystick_msg):
        self.twist = Twist()

        throttle_scale = joystick_msg.axes[self.settings["throttle"]["axis"]]+1.0 * self.settings["throttle"]["scale"] #+1 because min=-1. min shouyld be 0

        self.twist.linear.x = joystick_msg.axes[self.settings["linear_x"]["axis"]] * self.settings["linear_x"]["scale"] * throttle_scale
        inversion_multiplier = 1 #no inversion by default
        
        if joystick_msg.axes[self.settings["linear_x"]["axis"]] < -0.1: 
            inversion_multiplier = -1 #Somehow, this makes more sense...
        self.twist.angular.z = joystick_msg.axes[self.settings["angular_z"]["axis"]] * self.settings["angular_z"]["scale"] * inversion_multiplier * throttle_scale

        return self.twist

    def __str__(self):
        return "Steering"

class BaseStrafingMode(Submode):
    def __init__(self, settings, parent):
        super(BaseStrafingMode, self).__init__(settings, parent)

    def sub_process(self, joystick_msg):
        self.twist = Twist()

        throttle_scale = joystick_msg.axes[self.settings["throttle"]["axis"]]+1.0 * self.settings["throttle"]["scale"] #+1 because min=-1. min shouyld be 0

        self.twist.linear.x = joystick_msg.axes[self.settings["linear_x"]["axis"]] * self.settings["linear_x"]["scale"] * throttle_scale
        self.twist.linear.y = joystick_msg.axes[self.settings["linear_y"]["axis"]] * self.settings["linear_y"]["scale"] * throttle_scale

        return self.twist

    def __str__(self):
        return "Strafing"

class BaseControlInterpreterWithSubmodes(LatchingJoystickInterpreter):

    def __init__(self, settings):
        super(BaseControlInterpreterWithSubmodes, self).__init__()
        self.cmd_vel = rospy.Publisher("/manual_cmd_vel", TwistStamped)

        self.settings = settings

        self.twist = Twist()

        steering = BaseSteeringMode(settings['submodes']['steering'], self)
        strafing = BaseStrafingMode(settings['submodes']['strafing'], self)
        self.submodes = {tuple(sorted(steering.enable_buttons)):steering, 
                         tuple(sorted(strafing.enable_buttons)):strafing}

    def start(self):
        rospy.loginfo("Starting {0}".format(self))
        super(BaseControlInterpreterWithSubmodes, self).start()

    def when_active(self):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp  = rospy.get_rostime()
        twist_stamped.twist         = self.twist

        self.cmd_vel.publish(twist_stamped)
        
        
    def become_inactive(self):
        self.twist = Twist() #Empty twist, everything is zero
        
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp  = rospy.get_rostime()
        twist_stamped.twist         = self.twist

        self.cmd_vel.publish(twist_stamped)

    def process(self, joystick_msg, down, released, downed):
        active_mode = self.submodes.get(tuple(sorted(down)), None) 
        if active_mode:
            self.active = True
            rospy.loginfo("Submode: {0}_{1}".format(self, active_mode))
            self.twist = active_mode.sub_process(joystick_msg)
        else:
            self.active = False
            rospy.loginfo("No submode activated by {1}. {0}".format([mode.usage() for mode in self.submodes.values()], down))
            self.twist = Twist()

    def __str__(self):
        return "Base"