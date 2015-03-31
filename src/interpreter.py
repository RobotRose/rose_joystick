#! /usr/bin/env python
from rose_joystick.srv import switch_joystick_mode, switch_joystick_modeResponse
from rose_joystick.msg import available_modes

class JoystickInterpreter(object):
    """Converts joystick values to some other message type and publishes that"""

    def __init__(self):
        self.timer = None
        self.button_state = ButtonState()

    def start(self):
        """Start the interpreting after is was stopped"""
        pass

    def process(self, joystick_msg, down, released, downed):
        """Take a sensor_msgs/Joy message and interpret it for this mode (e.g. base, arms, neck...)
        @param joystick_msgs the raw joystick message with all the joystick's axes and buttons
        @param down the indices of the buttons on the joystick that are down/pressed
        @param released the indices of the buttons that are released (have a flank/edge from down to up)
        @param downed buttons that are just pressed (have a flank from up to down)"""
        pass

    def stop(self):
        """Publish a message to stop the current mode. For the base, this means sending a zero speed to /cmd_vel, so the robot does not run away"""
        pass

    def repeat_message(self):
        """Publish a predefined message again, e.g. to keep publishing at a certain rate even though the joystick does not update"""
        raise NotImplementedError("Subclasses must implement this function")

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

    def process(self, joystick_msg, down, released, downed):
        for interpreter in self.subinterpreters:
            interpreter.process(joystick_msg, down, released, downed)

    def __str__(self):
        return "+".join([str(i) for i in self.subinterpreters])

class ButtonState(object):
    def __init__(self):
        self.previous_button_states = []

    def derive_button_events(self, button_states):
        """Detect and return indices of buttons that are down, were (just) released and (just) pressed
        @param buttons a sequence of [Bool] indicating which buttons are down/pressed (True) and up (False)
        @returns three tuples: (indices of down buttons), (indices of buttons that are just released), (indices of buttons that are just pressed).
        >>> state = ButtonState()
        >>> state.derive_button_events([False, False, False]) #First event, so nothing happens
        ((), (), ())
        >>> state.derive_button_events([False, True, False])
        ((1,), (), (1,))
        >>> state.derive_button_events([False, True, False]) #button at index 1 is still pressed, so not *just* pressed
        ((1,), (), ())
        >>> state.derive_button_events([False, False, False]) #Button 1 is no longer pressed, so released
        ((), (1,), ())
        >>> state.derive_button_events([False, False, False]) #No changes, no buttons are pressed
        ((), (), ())
        >>> state.derive_button_events([True, True, False])
        ((0, 1), (), (0, 1))
        >>> state.derive_button_events([True, True, False])
        ((0, 1), (), ())
        >>> state.derive_button_events([False, True, True])
        ((1, 2), (0,), (2,))
        """
        if not self.previous_button_states:
            self.previous_button_states = button_states

        down = tuple([index for index, state in enumerate(button_states) if state])
        released = tuple([index for index, (prev, curr) in enumerate(zip(self.previous_button_states, button_states)) if prev and not curr]) #If the button was pressed previously but not currently, it has been released
        pressed = tuple([index for index, (prev, curr) in enumerate(zip(self.previous_button_states, button_states)) if not prev and curr]) #If the button was pressed previously but not currently, it has been released

        self.previous_button_states = button_states

        return down, released, pressed


class LatchingJoystickInterpreter(JoystickInterpreter):
    def __init__(self):
        super(LatchingJoystickInterpreter, self).__init__()
        self.active = False
        self.previously_active = False

    def repeat_messsage(self, *args, **kwargs):
        if self.active:
            self.when_active()
        
        if self.previously_active and not self.active:
            self.become_inactive()

        self.previously_active = self.active

    def when_active(self):
        raise NotImplementedError("Subclasses must implement this function")

    def become_inactive(self):
        raise NotImplementedError("Subclasses must implement this function")


if __name__ == "__main__":
    import doctest
    doctest.testmod() #Run the doctests, i.e. the """ >>> """ stuff that looks like a Python interpreter