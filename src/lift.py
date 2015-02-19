import rospy

from rose_base_msgs.msg import lift_command

from interpreter import JoystickInterpreter

class LiftControlInterpreter(JoystickInterpreter):
   def __init__(self, settings):
       super(LiftControlInterpreter, self).__init__()
       self.settings = settings
       self.lift_pose_requester = rospy.Publisher('lift_controller/lift/command', lift_command)

   def process(self, joystick_msg):
       #import ipdb; ipdb.set_trace()
       joyvalue = joystick_msg.axes[self.settings["pose"]["axis"]]
       request = lift_command()
       request.speed_percentage = 100.0
       request.position_percentage = abs(joyvalue*100.0)
       self.lift_pose_requester.publish(request)  


   def __str__(self):
       return "Lift"