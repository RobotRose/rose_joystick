import roslib; roslib.load_manifest("rose_joystick")
import rospy

from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

from interpreter import JoystickInterpreter

class NeckControlInterpreter(JoystickInterpreter):
    def __init__(self, settings, scaling_factor=10/3):
        super(NeckControlInterpreter, self).__init__()
        self.pan_publisher = rospy.Publisher("/neck_pan_controller/command", Float64)
        self.tilt_publisher = rospy.Publisher("/neck_tilt_controller/command", Float64)

        self.pan_subscriber = rospy.Subscriber("/neck_pan_controller/state", JointState, self.update_pan)
        self.tilt_subscriber = rospy.Subscriber("/neck_tilt_controller/state", JointState, self.update_tilt)

        if settings.has_key("submodes"):
            self.settings = settings['submodes']['control']
        else:
            self.settings = settings

        self.pan = 0
        self.tilt = 0

    def update_pan(self, msg):
        self.pan = msg.current_pos

    def update_tilt(self, msg):
        self.tilt = msg.current_pos

    def process(self, joystick_msg):
        rospy.logdebug("Neck processing joystick_msg")
        #import ipdb;ipdb.set_trace()
        new_pan = self.pan + joystick_msg.axes[self.settings["pan"]["axis"]] * self.settings["pan"]["scale"]
        new_tilt = self.tilt + joystick_msg.axes[self.settings["tilt"]["axis"]] * self.settings["tilt"]["scale"]
        
        #rospy.loginfo("new_pan={0} = self.pan={1} + (joystick_msg.axes[5]={2} / self.scaling_factor={3})".format(new_pan, self.pan, joystick_msg.axes[5], self.scaling_factor))

        self.pan_publisher.publish(new_pan)
        self.tilt_publisher.publish(new_tilt)
        rospy.logdebug("Neck process done")

    def __str__(self):
        return "Neck"

class NeckPredefinedController(JoystickInterpreter):
    def __init__(self, settings):
        super(NeckPredefinedController, self).__init__()
        self.settings = settings
        
        self.pan_publisher = rospy.Publisher("/neck_pan_controller/command", Float64)
        self.tilt_publisher = rospy.Publisher("/neck_tilt_controller/command", Float64)
        
        self.predefined_pose_stepper = self.settings["predefined_pose_stepper"]
        self.selected_pose = 0

        self.previous_pose = (0,0)

        self.previous_button_state = []

    def update_pan(self, msg):
        self.pan = msg.current_pos

    def update_tilt(self, msg):
        self.tilt = msg.current_pos

    def process(self, joystick_msg):
        def apply_pose(pose_dict):
            new_pan, new_tilt = pose_dict["pan"], pose_dict["tilt"]
            rospy.loginfo("Neck going to pose (pan: {0}, tilt: {1})".format(new_pan, new_tilt))
         
            self.pan_publisher.publish(new_pan)
            self.tilt_publisher.publish(new_tilt)

        #import ipdb;ipdb.set_trace()
        if not self.previous_button_state:
            self.previous_button_state = joystick_msg.buttons

        if joystick_msg.buttons[self.predefined_pose_stepper] != self.previous_button_state[self.predefined_pose_stepper] and not joystick_msg.buttons[self.predefined_pose_stepper]:
            
            self.selected_pose += 1
            poses = self.settings["predefined_poses"]
            pose_keys = poses.keys()
            selected_pose_name = pose_keys[self.selected_pose % len(pose_keys)]
            pose = poses[selected_pose_name]
            apply_pose(pose)

        changed_buttons = [curr != prev for (curr, prev) in zip(joystick_msg.buttons, self.previous_button_state)]
        # print "Changed buttons: {0}".format(changed_buttons)
        pushed_buttons  = [index for index, changed in enumerate(changed_buttons) if changed and joystick_msg.buttons[index]]
        if any(pushed_buttons):
            first_pushed_button = pushed_buttons[0]
            # print "Pushed buttons: {0}. first_pushed_button = {1}".format(pushed_buttons, first_pushed_button)
            poses = self.settings["predefined_poses"]
            try:
                # import ipdb; ipdb.set_trace()
                pushed_pose = [pose for pose in poses.values() if int(pose["button"]) == first_pushed_button][0]
                apply_pose(pushed_pose)
            except IndexError, e:
                pass
                # rospy.logwarn("No pose connected to button index {0}".format(first_pushed_button))

        self.previous_button_state = joystick_msg.buttons

    def __str__(self):
        return "Neck with predefined poses"

class SimpleNeckController(JoystickInterpreter):
    def __init__(self, settings):
        super(SimpleNeckController, self).__init__()
        self.tilt_publisher = rospy.Publisher("/neck_tilt_controller/command", Float64)
        self.tilt_subscriber = rospy.Subscriber("/neck_tilt_controller/state", JointState, self.update_tilt)

        self.pan_publisher = rospy.Publisher("/neck_pan_controller/command", Float64)
        self.pan_subscriber = rospy.Subscriber("/neck_pan_controller/state", JointState, self.update_pan)

        self.settings = settings

        self.pan = 0
        self.tilt = 0

        self.previous_pose = (self.pan,self.tilt)

    def update_pan(self, msg):
        self.pan = msg.current_pos

    def update_tilt(self, msg):
        self.tilt = msg.current_pos

    def process(self, joystick_msg):
        #import ipdb;ipdb.set_trace()
        if self.settings.has_key("pan_simple"):
            cmd_tilt = joystick_msg.axes[self.settings["tilt_simple"]["axis"]]
            scale = self.settings["tilt_simple"]["scale"]
            new_tilt = self.tilt + cmd_tilt * scale

            if abs(new_tilt - self.tilt) > 0.1 and abs(cmd_tilt) > 0.1:
                self.tilt_publisher.publish(new_tilt)
                #rospy.loginfo("new_tilt={0:.3f} = self.tilt={1:.3f} + (cmd_tilt={2:.3f} * scale={3:.3f})".format(new_tilt, self.tilt, cmd_tilt, scale))

        if self.settings.has_key("pan_simple"):
            cmd_pan = joystick_msg.axes[self.settings["pan_simple"]["axis"]]
            scale = self.settings["pan_simple"]["scale"]
            new_pan = self.pan + cmd_pan * scale

            if abs(new_pan - self.pan) > 0.1 and abs(cmd_pan) > 0.1:
                self.pan_publisher.publish(new_pan)
        
    def __str__(self):
        return "Neck (only tilt)"