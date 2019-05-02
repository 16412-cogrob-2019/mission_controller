import rospy
import actionlib

from mission_controller.msg import GoToAction, GoToGoal, ActivityStart, ActivityEnd

class TurtlebotClient:

    def __init__(self):
        self.client = actionlib.SimpleActionClient('mission_controller', GoToAction)
        self.client.wait_for_server()

        self.dispatch_sub = rospy.Subscriber('/mission_activities', )

        self.activity_pub_start = rospy.Publisher('/mission_activities', ActivityStart)
        self.activity_pub_end = rospy.Publisher('/mission_activities', ActivityEnd)

    def connect_client(self, msg):

        pass


