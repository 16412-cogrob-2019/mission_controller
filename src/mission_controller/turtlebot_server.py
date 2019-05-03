#! /usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalID

from mission_controller.msg import GoToAction, GoToResult

class TurtlebotServer:
    _result_goto = GoToResult()
    

    def __init__(self):
        self.server = actionlib.SimpleActionServer('mission_controller', GoToAction, self.go_to, False)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.goal_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.goal_cancel_client = rospy.Publisher('/move_base/cancel', GoalID,queue_size=10)


        self.server.start()

    def go_to(self, goal):
        "args: (x, y, vel)"

        print("Going to point {}, {} with speed {}".format(goal.x, goal.y, goal.vel))

        rospy.sleep(3)
        # #cancel any activities that were currently being acheived by robot
        # self.goal_cancel_client.publish(GoalID())

        # #set the velocity parameters
        # rospy.set_param('/move_base/DWAPlannerROS/min_trans_vel', goal.vel)
        # rospy.set_param('/move_base/DWAPlannerROS/max_trans_vel', goal.vel)

        # #create move_base action
        # action_goal = MoveBaseGoal()
        # action_goal.target_pose.header.frame_id = "map"
        # action_goal.target_pose.header.stamp = rospy.Time.now()
        # action_goal.target_pose.pose.position.x = goal.x
        # action_goal.target_pose.pose.position.y = goal.y
        # action_goal.target_pose.pose.position.z = 0
        # quaternion = tf.transformations.quaternion_from_euler(0, 0, 0) #TODO needs to be something useful
        # action_goal.target_pose.pose.orientation.x = quaternion[0]
        # action_goal.target_pose.pose.orientation.y = quaternion[1]
        # action_goal.target_pose.pose.orientation.z = quaternion[2]
        # action_goal.target_pose.pose.orientation.w = quaternion[3]
        # self.goal_client.send_goal(action_goal) # send goal to turtlebot

        # self.goal_client.wait_for_result()
        print("Arrived!")

        self._result_goto.arrived = True
        self.server.set_succeeded(self._result_goto)
        

        


