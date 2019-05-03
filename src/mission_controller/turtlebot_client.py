import rospy
import actionlib

from mission_controller.msg import GoToAction, GoToGoal


class TurtlebotClient: #This will likely have to spawn multiple clients

    def __init__(self, results):
        self.client = actionlib.SimpleActionClient('mission_controller', GoToAction)
        self.client.wait_for_server()
        self.results = results

        # self.dispatch_sub = rospy.Subscriber('/mission_activities', )

        # self.activity_pub_start = rospy.Publisher('/mission_activities', ActivityStart)
        # self.activity_pub_end = rospy.Publisher('/mission_activities', ActivityEnd)

    def connect_client(self, data):
        """
        Data - waypoint tuple (x, y, vel) TODO update for sequence
        """
        goal = GoToAction(x=data[0], y=data[1], vel=data[2])

        self.client.send_goal(goal, done_cb=self.post_result)

        print("Goal {} sent.".format(goal))

        return True

    def post_goal(self):

        print("Goal completed.")

        res = self.client.get_result()

        #add to list of results
        self.results.append(res)

        return




