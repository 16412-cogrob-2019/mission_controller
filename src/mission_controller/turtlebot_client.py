#! /usr/bin/env python

import rospy
import actionlib

from mission_controller.msg import GoToAction, GoToGoal, FollowPlanAction, FollowPlanGoal


class TurtlebotClient: #This will likely have to spawn multiple clients

    def __init__(self, results, plan=True):
        self.plan = True
        if plan:
            self.client = actionlib.SimpleActionClient('mission_controller', FollowPlanAction)
        else:               
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

        if self.plan:
            goal = FollowPlanGoal(data.wypts)
        else:
            goal = GoToGoal(data[0], data[1], data[2])

        self.client.send_goal(goal, done_cb=self.post_result)

        #print("Goal {} sent.".format(goal))

        return True

    def post_result(self, status, result):

        print("Goal completed.")

        #add to list of results
        self.results.append(result)

        return




