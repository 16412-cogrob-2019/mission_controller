#! /usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalID
import dynamic_reconfigure.client
import numpy as np
import matplotlib.mlab as mlab

from mission_controller.msg import GoToAction, GoToResult, FollowPlanAction, FollowPlanResult


MAX_SPEED = 0.22

def simple_sample():
    return float('%.4f'%np.random.rand())

def caldera_sim_function(x, y):
    x, y = x / 10.0, y / 10.0
    z0 = mlab.bivariate_normal(x, y, 10.0, 5.0, 5.0, 0.0)
    z1 = mlab.bivariate_normal(x, y, 1.0, 2.0, 2.0, 5.0)
    z2 = mlab.bivariate_normal(x, y, 1.7, 1.7, 8.0, 8.0)
    return 50000.0 * z0 + 2500.0 * z1 + 5000.0 * z2


class TurtlebotServer:
    _result_goto = GoToResult()
    _result_plan = FollowPlanResult()
    

    def __init__(self, ns="", plan=True, sampling=True):

        self.sampling = sampling
        self.ns = ns
        self.ns_print = self.ns_print = self.ns +": " if self.ns != "" else ""

        if plan:
            self.server = actionlib.SimpleActionServer(self.ns+'/mission_controller', FollowPlanAction, self.follow_plan, False)
        else:
            self.server = actionlib.SimpleActionServer(self.ns+'/mission_controller', GoToAction, self.go_to, False)

        #self.reconfig_node = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS")
        self.vel_pub = rospy.Publisher(self.ns+'/cmd_vel', Twist, queue_size=10)
        self.goal_client = actionlib.SimpleActionClient(self.ns+"/move_base", MoveBaseAction)
        self.goal_cancel_client = rospy.Publisher(self.ns+'/move_base/cancel', GoalID, queue_size=10)


        self.server.start()

    def send_nav_goal(self, goal):

        try:
            self.goal_client.wait_for_server()

            #cancel any activities that were currently being acheived by robot
            #self.goal_cancel_client.publish(GoalID())

            #set the velocity parameters
            #rospy.set_param('/move_base/DWAPlannerROS/min_trans_vel', goal.vel)
            #rospy.set_param('/move_base/DWAPlannerROS/max_trans_vel', goal.vel)

            #vel_params = {'/move_base/DWAPlannerROS/min_trans_vel': goal.vel, '/move_base/DWAPlannerROS/max_trans_vel': goal.vel}
            #config = self.reconfig_node.update_configuration(vel_params)


            #create move_base action
            action_goal = MoveBaseGoal()
            action_goal.target_pose.header.frame_id = "map"
            action_goal.target_pose.header.stamp = rospy.Time.now()
            action_goal.target_pose.pose.position.x = goal.x
            action_goal.target_pose.pose.position.y = goal.y
            action_goal.target_pose.pose.position.z = 0
            #quaternion = tf.transformations.quaternion_from_euler(0, 0, 0) #TODO needs to be something useful
            action_goal.target_pose.pose.orientation.x = 0.0
            action_goal.target_pose.pose.orientation.y = 0.0
            action_goal.target_pose.pose.orientation.z = 0.0
            action_goal.target_pose.pose.orientation.w = 1.0
            self.goal_client.send_goal(action_goal) # send goal to turtlebot
            self.goal_client.wait_for_result()

            return True
            
        
        except KeyboardInterrupt:
            print(self.ns_print + "Planning interrupted! Stopping robot...")
            self.goal_cancel_client.publish(GoalID())
            return False
        except Exception as e:
            raise 



    def follow_plan(self, plan):
        "args: list of waypoints"

        if not plan.plan:
            print(self.ns_print + "No plan received. Doing nothing.")
            nav_goal = True

        for waypt in plan.plan:
            print(self.ns_print + "Going to point {}, {} with speed {}".format(waypt.x, waypt.y, min(MAX_SPEED, waypt.vel))) #TODO change to easier messages

            ##DEBUGGING
            #rospy.sleep(2)
            #nav_goal = True

            nav_goal = self.send_nav_goal(waypt)

            if nav_goal:
                continue
            else:
                break

        
        if self.sampling and plan.plan:
            samp_loc = (plan.plan[-1].x, plan.plan[-1].y)
            print(self.ns_print + "Arrived at sample location. Sampling...")
            rospy.sleep(3)
            samp_val = caldera_sim_function(*samp_loc)
            print(self.ns_print + "Sampled value {}.".format(samp_val))
            self._result_plan.sample = samp_val
        else:
            self._result_plan.sample = 0.0

        if plan.plan:   
            print(self.ns_print + "Done executing plan!")
        self._result_plan.arrived = nav_goal
        self.server.set_succeeded(self._result_plan)



    def go_to(self, goal):
        "args: (x, y, vel)"

        print(self.ns_print + "Going to point {}, {} with speed {}".format(goal.x, goal.y, min(MAX_SPEED, waypt.vel))) #TODO change to easier messages

        rospy.sleep(3) #USE ME ALONE TO TEST PIPELINE

        #nav_goal = self.send_nav_goal(goal)

        print(self.ns_print + "Arrived!")

        self._result_goto.arrived = True
        self.server.set_succeeded(self._result_goto)
        

        


