import roslib; roslib.load_manifest("rose_joystick")
import rospy

from geometry_msgs.msg import Twist

from interpreter import JoystickInterpreter, Submode
import threading

class BaseControlInterpreter(JoystickInterpreter):
    """Convert joystick values to a Twist"""

    def __init__(self, settings, driving_scale=0.1, steering_scale=0.2): #max value for a Joy-axis value is 1.0. Max speed is 0.1m/s
        super(BaseControlInterpreter, self).__init__()
        self.cmd_vel = rospy.Publisher("/manual_cmd_vel", Twist)

        self.settings = settings

        self.stopper = None
        self.publisher_thread = None
        self.active = False
        self.previously_active = False
        self.twist = Twist()

    def start(self):
        self.timer = rospy.Timer(rospy.Duration(0.2), self.repeat_messsage, oneshot=False)

    def repeat_messsage(self, *args, **kwargs):
        if self.active:
            self.cmd_vel.publish(self.twist)
        
        if self.previously_active and not self.active:
            self.cmd_vel.publish(Twist())

        self.previously_active = self.active

    def process(self, joystick_msg):
        self.twist = Twist()

        if self.settings.has_key("throttle"):
            throttle_scale = joystick_msg.axes[self.settings["throttle"]["axis"]]+1.0 * self.settings["throttle"]["scale"] #+1 because min=-1. min shouyld be 0
        else:
            throttle_scale = 1.0

        self.twist.linear.x = joystick_msg.axes[self.settings["linear_x"]["axis"]] * self.settings["linear_x"]["scale"] * throttle_scale

        #If the config requires a button to enable strafing
        if self.settings.has_key("press_to_strafe"):
            if not joystick_msg.buttons[self.settings["press_to_strafe"]]: 
                inversion_multiplier = 1 #no inversion by default
                if joystick_msg.axes[self.settings["linear_x"]["axis"]] < -0.1: 
                    inversion_multiplier = -1 #Somehow, this makes more sense...
                self.twist.angular.z = joystick_msg.axes[self.settings["angular_z"]["axis"]] * self.settings["angular_z"]["scale"] * inversion_multiplier
            else:
                self.twist.linear.y = joystick_msg.axes[self.settings["linear_y"]["axis"]] * self.settings["linear_y"]["scale"] * throttle_scale
        else: #or is it always on?
            self.twist.angular.z = joystick_msg.axes[self.settings["angular_z"]["axis"]] * self.settings["angular_z"]["scale"]
            self.twist.linear.y = joystick_msg.axes[self.settings["linear_y"]["axis"]] * self.settings["linear_y"]["scale"] * throttle_scale

        self.active = ( abs(self.twist.linear.x) > 0.001 or 
                        abs(self.twist.linear.y) > 0.001 or 
                        abs(self.twist.linear.z) > 0.001 or 
                        abs(self.twist.angular.x) > 0.001 or 
                        abs(self.twist.angular.y) > 0.001 or 
                        abs(self.twist.angular.z) > 0.001)

    def stop(self):
        rospy.loginfo("Stopping {0}".format(self))
        self.twist = Twist() #Empty twist, everything is zero
        self.timer.shutdown()
        
        self.cmd_vel.publish(self.twist)

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

class BaseControlInterpreterWithSubmodes(JoystickInterpreter):

    def __init__(self, settings):
        super(BaseControlInterpreterWithSubmodes, self).__init__()
        self.cmd_vel = rospy.Publisher("/manual_cmd_vel", Twist)

        self.settings = settings

        self.twist = Twist()
        self.active = False
        self.previously_active = False

        steering = BaseSteeringMode(settings['submodes']['steering'], self)
        strafing = BaseStrafingMode(settings['submodes']['strafing'], self)
        self.submodes = {tuple(sorted(steering.enable_buttons)):steering, 
                         tuple(sorted(strafing.enable_buttons)):strafing}

    def start(self):
        rospy.loginfo("Starting {0}".format(self))
        self.timer = rospy.Timer(rospy.Duration(0.2), self.repeat_messsage, oneshot=False)

    def repeat_messsage(self, *args, **kwargs):
        if self.active:
            self.cmd_vel.publish(self.twist)
        
        if self.previously_active and not self.active:
            self.cmd_vel.publish(Twist())

        self.previously_active = self.active

    def process(self, joystick_msg):
        pushed_buttons = [index for index, pressed in enumerate(joystick_msg.buttons) if pressed]
        active_mode = self.submodes.get(tuple(sorted(pushed_buttons)), None)
        if active_mode:
            self.active = True
            rospy.loginfo("Submode: {0}_{1}".format(self, active_mode))
            self.twist = active_mode.sub_process(joystick_msg)
        else:
            self.active = False
            rospy.loginfo("No submode activated by {1}. {0}".format([mode.usage() for mode in self.submodes.values()], pushed_buttons))
            self.twist = Twist()

    def stop(self):
        rospy.loginfo("Stopping {0}".format(self))
        self.twist = Twist() #Empty twist, everything is zero
        self.timer.shutdown()
        
        self.cmd_vel.publish(self.twist)

    def __str__(self):
        return "Base"