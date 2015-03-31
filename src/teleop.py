#!/usr/bin/env python

import roslib; roslib.load_manifest("rose_joystick")
import rospy

from rose_joystick.srv import switch_joystick_mode, switch_joystick_modeResponse
from rose_joystick.msg import available_modes

from sensor_msgs.msg import Joy
from std_msgs.msg import String
from roscomm.msg import stringlist

from collections import OrderedDict

import sys
import yaml

import interpreter
import base
import neck
import arm, arm_submodes, arms
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
        @param settings is a parsed yaml file"""
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
            if settings['arms'].has_key('submodes'):
                left = arm_submodes.ArmControlInterpreterWithSubmodes(settings['arms'], side="left")
                # right += [arm_submodes.ArmControlInterpreterWithSubmodes(settings['arms'], side="right")]
                if settings['neck'].has_key("tilt_simple"):
                    neck_simple = neck.SimpleNeckController(settings['neck'])
                    neck_predef = neck.NeckPredefinedController(settings['neck'])
                    self.interpreters += [interpreter.CombinedInterpreter(left, neck_simple, neck_predef)]
                    # self.interpreters += [interpreter.CombinedInterpreter(right, neck_simple)]
            else:
                self.interpreters += [arm.ArmControlInterpreter(settings['arms'], side="left")]
                # self.interpreters += [arm.ArmControlInterpreter(settings['arms'], side="right")]

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


    @property
    def interpreter(self):
        return self._interpreter

    def publish_current_mode(self):
        rospy.logwarn("Current mode controls {0}".format(self.interpreter))
        self.mode_publisher.publish(str(self.interpreter))


    def publish_available_modes(self):
        modes = stringlist(self.interpreter_names)
        self.available_modes_publisher.publish(modes)

    @interpreter.setter
    def interpreter(self, value):
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
        rospy.loginfo("Mode switch requested: {0}".format(request.switch_to_mode))
        
        if request.switch_to_mode in self.interpreter_names:
            interpreter_index = self.interpreter_names.index(request.switch_to_mode)
            self.interpreter = self.interpreters[interpreter_index]
            return switch_joystick_modeResponse(True)
        else:
            rospy.logerr("Mode switch to {0} requested but no such mode exists".format(request.switch_to_mode))
            return switch_joystick_modeResponse(False)

    def _switch_mode_with_button(self, switch_interpreter):
        #or (joystick_msg.buttons[self.previous_btn] != self.previous_button_state[self.previous_btn] and not joystick_msg.buttons[self.previous_btn]):
        #The modeswitcher is was on and is now released: clicked and thus switch to the next mode

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
