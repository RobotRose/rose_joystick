#!/usr/bin/env python

"""teleop.py is the entry point for joystick control.
When a a Joy-message is received, it is passed to an Interpreter, 
    that handles most of the conversion of a joystick-state to movement commands. 
The JoystickTeleop-class allows to switch between interpreters by a service call or via a button. 
Each interpreter is a mode that the joystick can be in, 
    e.g. base-controlling-mode or arm-controlling mode etc.

The JoystickTeleop is initialized with a nested dictionary (from a yaml-file) 
    that indicates which joystick-axes map to which motion axis and which button does what."""

import roslib; roslib.load_manifest("rose_joystick")
import rospy

from rose_joystick.srv import switch_joystick_mode, switch_joystick_modeResponse
from rose_joystick.msg import available_modes
from rose_arm_controller_msgs.srv import get_arms

from sensor_msgs.msg import Joy
from std_msgs.msg import String
from roscomm.msg import stringlist


import sys
import yaml

import interpreter
import base
import neck
import arm, arm_submodes
import lift

class CallIfChanged(object):
    '''Decorator that only executes the function if the input is different than the previous time it was called'''
    def __init__(self, func):
        self.func = func
        self.cache = None
    def __call__(self, *args):
        if self.cache != args:
            self.func(*args)
        self.cache = args

rospy.loginfo = CallIfChanged(rospy.loginfo) #Decorate loginfo to only print if the thing to print changed since the last call

class JoystickTeleop(object):
    def __init__(self, settings):
        """Initialize a new JoystickTeleop. 
        @param settings is a parsed yaml file, i.e. a nested dictionary"""
        self.mode_publisher = rospy.Publisher("~mode", String, latch=True)
        self.switch_joystick_mode_service = rospy.Service("~set_mode", switch_joystick_mode, self.handle_set_joystick_mode)
        self.available_modes_publisher = rospy.Publisher("~available_modes", stringlist, latch=True)

        self.deadzone = settings.get("deadzone", 0.05)
    
        self.interpreters = []

        base_mode = None

        if settings.has_key('base'):
            if settings['base'].has_key('submodes'):
                base_mode = base.BaseControlInterpreterWithSubmodes(settings['base'])
            else:
                base_mode = base.BaseControlInterpreter(settings['base'])

        if settings.has_key('neck'):
            if base_mode:
                if settings['neck'].has_key("tilt_simple"):
                    neck_simple = neck.SimpleNeckController(settings['neck'])
                    neck_predef = neck.NeckPredefinedController(settings['neck'])
                    self.interpreters += [interpreter.CombinedInterpreter(base_mode, neck_simple, neck_predef)]
                else:
                    self.interpreters += [base_mode]
                    self.interpreters += [neck.NeckControlInterpreter(settings['neck'])]
        else:
            self.interpreters += [base_mode]

        if settings.has_key('arms'):
            available_arms = []
            try:
                rospy.wait_for_service('/arm_controller/get_arms', 10) # Wait at most 10 seconds
                get_available_arms = rospy.ServiceProxy('/arm_controller/get_arms', get_arms)
                try:
                    response = get_available_arms()
                    available_arms = response.arms
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

                for arm in available_arms:
                    if settings['arms'].has_key('submodes'):
                        left = arm_submodes.ArmControlInterpreterWithSubmodes(settings['arms'], name=arm)
                        if settings['neck'].has_key("tilt_simple"):
                            neck_simple = neck.SimpleNeckController(settings['neck'])
                            neck_predef = neck.NeckPredefinedController(settings['neck'])
                            self.interpreters += [interpreter.CombinedInterpreter(left, neck_simple, neck_predef)]
                            # self.interpreters += [interpreter.CombinedInterpreter(right, neck_simple)]
                    else:
                        self.interpreters += [arm.ArmControlInterpreter(settings['arms'], name=arm)]
            except rospy.ROSException, e:
                print("Could not connect to get available arms via service request: " + str(e))

        if settings.has_key('lift'):
            self.interpreters += [lift.LiftControlInterpreter(settings['lift'])]

        self.interpreter_names = [str(inter) for inter in self.interpreters]
        self._interpreter = None

        self.next_btn = settings.get('next_mode', None)
        self.previous_btn = settings.get('previous_mode', -1)
        
        self.button_state = interpreter.ButtonState()

        joystick_topic = settings["topic"]
        self.joystick_subscriber = rospy.Subscriber(joystick_topic, Joy, self.process_joystick)

        self.interpreter = self.interpreters[0]


    def publish_current_mode(self):
        """Publish what mode the teleop is currently in"""
        rospy.logwarn("Current mode controls {0}".format(self.interpreter))
        self.mode_publisher.publish(str(self.interpreter))


    def publish_available_modes(self):
        """Announce the list of interpreters available. 
        Elements from this list can be received again via handle_set_joystick_mode"""
        modes = stringlist(self.interpreter_names)
        self.available_modes_publisher.publish(modes)


    @property
    def interpreter(self):
        """Get the current interpreter"""
        return self._interpreter

    @interpreter.setter
    def interpreter(self, value):
        """Stop the current interpreter, set the new current interpreter, start it and announce the change"""
        #Stop the previous one
        if self._interpreter:
            self._interpreter.stop()

        self._interpreter = value

        #Start the current one
        if self._interpreter:
            self._interpreter.start()

        self.publish_current_mode() 
        self.publish_available_modes()

    def handle_set_joystick_mode(self, request):
        """Check whether the requested mode is available, 
        sets the mode if so and replies with a confirmation"""
        rospy.loginfo("Mode switch requested: {0}".format(request.switch_to_mode))
        
        if request.switch_to_mode in self.interpreter_names:
            interpreter_index = self.interpreter_names.index(request.switch_to_mode)
            self.interpreter = self.interpreters[interpreter_index]
            return switch_joystick_modeResponse(True)
        else:
            rospy.logerr("Mode switch to {0} requested but no such mode exists".format(request.switch_to_mode))
            return switch_joystick_modeResponse(False)

    def _switch_mode_with_button(self, switch_interpreter):
        """Take the next mode from the list of modes"""
        current_interpreter_index = 0
        try:
            current_interpreter_index = self.interpreters.index(self.interpreter)
        except ValueError:
            pass #then just start from 0

        interpreter_index = current_interpreter_index + switch_interpreter
        self.interpreter = self.interpreters[interpreter_index % len(self.interpreters)]

        if not self.interpreter and self.next_btn:
            rospy.logerr("No joystick interpreter selected, press button {0} to switch to the next mode".format(self.next_btn+1)) #Real-life starts counting at 1
            self.mode_publisher.publish(str(self.interpreter)) #Real-life starts counting at 1

    def process_joystick(self, joystick_msg):
        """Take a joystick message and let be interpreted by the interpreter that is currently selected."""
        def apply_deadzone(signal):
            negativity_multiplier = 1 if signal > 0 else -1
            signal = abs(signal) #multiply final result with negativity_multiplier later on
            no_deadzone = signal - self.deadzone
            if no_deadzone < 0.0:
                return 0.0
            output = no_deadzone * negativity_multiplier
            return output

        joystick_msg.axes = [apply_deadzone(value) for value in joystick_msg.axes]

        down, released, downed = self.button_state.derive_button_events(joystick_msg.buttons)

        switch_interpreter = 0
        if self.next_btn != None and self.next_btn in released: 
            switch_interpreter = 1
        if self.previous_btn != -1 and self.previous_btn in released:
            switch_interpreter = -1

        if switch_interpreter:
            self._switch_mode_with_button(switch_interpreter)

        if self.interpreter:
            self.mode_publisher.publish(str(self.interpreter))
            try:
                self.interpreter.process(joystick_msg, down, released, downed)
            except TypeError, te:
                rospy.logerr("Interpreter {0} cannot process {1}: {2}".format(self.interpreter, (joystick_msg, down, released, downed), te))
        else:
            self.mode_publisher.publish(str(self.interpreter)) #Real-life starts counting at 1

if __name__ == "__main__":
    rospy.init_node("joystick_teleop")

    settings = {'arms': [
                            {'linear_x': {'axis': 0, 'scale': 0.1}}, 
                            {'linear_y': {'axis': 1, 'scale': 0.1}},
                            {'linear_z': {'axis': 3, 'scale': 0.1}},
                            {'angular_x': {'axis': 5, 'scale': 0.1}},
                            {'angular_y': {'axis': 4, 'scale': 0.1}},
                            {'angular_z': {'axis': 2, 'scale': 0.1}}
                        ],
                'base': [
                            {'linear_x': {'axis': 1, 'scale': 0.1}},
                            {'angular_z': {'axis': 2, 'scale': 0.2}},
                            {'maxspeed': {'axis': 3}}
                        ],
                'lift': [{'pose': {'axis': 3}}],
                'neck': [
                            {'pan': {'axis': 0, 'scale': 3.0}},
                            {'tilt': {'axis': 1, 'scale': -3.0}}
                        ],
                'modeswitch': 3
                }
    try:
        args = rospy.myargv(argv=sys.argv)
        settings = yaml.load(open(args[1]))
    except IndexError:
        print "Optionally, specify a mapping"

    teleop = JoystickTeleop(settings)
    teleop.publish_available_modes()
    teleop.publish_current_mode()

    rospy.spin()
