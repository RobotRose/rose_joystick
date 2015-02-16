from rose_joystick.srv import switch_joystick_mode, switch_joystick_modeResponse
from rose_joystick.msg import available_modes

class JoystickInterpreter(object):
    """Converts joystick values to some other message type and publishes that"""

    def __init__(self):
        pass

    def start(self):
        """Start the interpreting after is was stopped"""
        pass

    def process(self, joystick_msg):
        """Take a sensor_msgs/Joy message and interpret it for this mode (e.g. base, arms, neck...)"""
        pass

    def stop(self):
        """Publish a message to stop the current mode. For the base, this means sending a zero speed to /cmd_vel, so the robot does not run away"""
        pass

    def __str__(self):
        return "Undefined JoystickInterpreter"

class Submode(JoystickInterpreter):
    """A Submode is a Joystick interpreter that interprets signal slighlty different while a collection of buttons(s) is pushed"""

    def __init__(self, settings, parent):
        super(Submode, self).__init__()
        self.settings = settings
        self.enable_buttons = settings['enable']

        self.parent = parent

    def is_enabled(self, joystick_msg):
        return all(joystick_msg.buttons[enable_button] for enable_button in self.enable_buttons)

    def sub_process(self, joystick_msg):
        """Process a joystick_msg according to a submode. Return the message it wants to send so the parent mode can handle it further"""
        raise NotImplementedError("Implement sub_process in a proper Submode.")
        return None

    def usage(self):
        return "{0}_{1} is enabled by button indices {2}".format(self.parent, self, self.enable_buttons)

class CombinedInterpreter(JoystickInterpreter):

    def __init__(self, *subinterpreters):
        self.subinterpreters = subinterpreters

    def start(self):
        for interpreter in self.subinterpreters:
            interpreter.start()
    
    def stop(self):
        for interpreter in self.subinterpreters:
            interpreter.stop()

    def process(self, joystick_msg):
        for interpreter in self.subinterpreters:
            interpreter.process(joystick_msg)

    def __str__(self):
        return "+".join([str(i) for i in self.subinterpreters])