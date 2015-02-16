import roslib; roslib.load_manifest("rose_joystick")
import rospy

from rose20_platform.msg import lift

from interpreter import JoystickInterpreter

class LiftControlInterpreter(JoystickInterpreter):
    def __init__(self, settings):
        super(LiftControlInterpreter, self).__init__()
        self.settings = settings
        self.lift_pose_requester = rospy.Publisher('/lift/pose_request', lift)

        #Maps parts of the range of a joystick axis' range to a predefined lift pose
        self.mapping = {(-1, -0.5):0, (-0.5, 0.5):1, (0.5, 1):2}

    def process(self, joystick_msg):
        #import ipdb; ipdb.set_trace()
        joyvalue = joystick_msg.axes[self.settings["pose"]["axis"]]
        for (_min, _max), pose in self.mapping.iteritems():
            if _min <= joyvalue < _max:
                request = lift()
                request.enabled = True
                request.pose = pose
                self.lift_pose_requester.publish(request)
                break

    def __str__(self):
        return "Lift"