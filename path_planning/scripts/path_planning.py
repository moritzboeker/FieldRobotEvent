#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler


import numpy as np
import matplotlib.pyplot as plt

def scan2cart(scan, max_range=30.0):
    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    ignore_ranges = np.array(scan.ranges) > max_range
    x = scan.ranges*np.cos(angles)
    y = scan.ranges*np.sin(angles)
    scan_cart = np.array([x, y])
    scan_cart[:, ignore_ranges] = None
    return scan_cart

# def trim_scan(scan, max_range=30.0):
#     ignore_ranges = np.array(scan.ranges) > max_range
#     scan = np.array(scan.ranges)
#     scan[ignore_ranges] = None
#     return scan


def laser_callback(scan):
    if state == "state_in_row":
        global angle
        global row_width

        angle_increment = scan.angle_increment
        angle_outer_limit_curr = scan.angle_max
        angle_outer_limit_targ = np.radians(130)
        angle_inner_limit_targ = np.radians(20)
        idx_ranges_outer = int(np.round((angle_outer_limit_curr - angle_outer_limit_targ) / angle_increment))
        idx_ranges_inner = int(np.round((angle_outer_limit_curr - angle_inner_limit_targ) / angle_increment))
        
        scan_cart = scan2cart(scan, max_range=row_width)
        scan_left = scan_cart[:, -idx_ranges_inner:-idx_ranges_outer]
        scan_right = scan_cart[:, idx_ranges_outer:idx_ranges_inner]

        # # use euclidean distance
        # mean_left = np.nanmean(scan_left, axis=1)
        # mean_right = np.nanmean(scan_right, axis=1)
        # mean_left = np.sqrt(mean_left[0]**2+mean_left[1]**2)
        # mean_right = np.sqrt(mean_right[0]**2+mean_right[1]**2)
        mean_left = np.nanmean(scan_left[1, :])
        mean_right = np.nanmean(scan_right[1, :])

        offset = mean_right + mean_left
        angle = - offset / (mean_left + mean_left)    

        # # 2. solution
        # ranges = trim_scan(scan, max_range=row_width)
        # ranges_left = ranges[idx_ranges_outer:idx_ranges_inner]
        # ranges_right = ranges[-idx_ranges_inner:-idx_ranges_outer]
        # mean_left = np.nanmean(ranges_left)
        # mean_right = np.nanmean(ranges_right)

        # # plot xy-graph
        # scan_extract = np.concatenate((scan_right, scan_left), axis=1)
        # print("scan_extract", scan_extract.shape)
        # plt.figure()
        # plt.plot(scan_extract[0,:], scan_extract[1,:], "ob")
        # plt.show()
    else:
        pass

def state_in_row(pub_vel):
    setpoint_alpha = 0;                         # [rad]
    # setpoint_ydist = 0;                       # [m]
    error_alpha = setpoint_alpha - angle
    # error_ydist = setpoint_ydist - y

    max_angular_z = np.pi/2;                    # [rad/s]
    max_alpha = np.pi/4;                        # [rad]
    # max_ydist = 0.75/2;                       # [m]
    p_gain_alpha = max_angular_z/max_alpha;     # = 2.0
    # p_gain_ydist = max_angular_z/max_ydist;   # = 4.19

    act_alpha = p_gain_alpha*error_alpha
    # act_ydist = -p_gain_ydist*error_ydist
    
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.2
    cmd_vel.angular.z = act_alpha
    # cmd_vel.angular.z = act_alpha/2 + act_ydist/2

    pub_vel.publish(cmd_vel)        

    if np.isnan(angle):
        return "state_headland"
    else:
        return "state_in_row"

def state_headland():
    global row_width
    global lin_row_enter
    global lin_row_exit
    global turn_l
    global turn_r

    path_pattern = rospy.get_param('~path_pattern')
    path_pattern = path_pattern.replace("-", "")
    path_pattern = path_pattern[1:-1]

    if len(path_pattern) > 0:
        which_row = path_pattern[0]
        which_turn = path_pattern[1]
        if which_turn == 'L':
            turn = turn_l
        elif which_turn == 'R':
            turn = turn_r
        else:
            rospy.logerr("Path pattern syntax error: undefined parameter '%s' for turn specification!", which_turn)
            return "error"

        result_straight = movebase_client(lin_row_exit/2, 0.0, 0.0)
        if result_straight:
            rospy.logwarn("1. Straight goal execution done!")
        else:
            rospy.logerr("1. Straight goal not reached!")
            return "error"
        
        result_turn = movebase_client(lin_row_exit, row_width/2, turn)
        if result_turn:
            rospy.logwarn("2. Turn goal execution done!")
        else:
            rospy.logerr("2. Turn goal not reached!")
            return "error"

        try:
            which_row = int(which_row)
        except ValueError:
            rospy.logerr("Path pattern syntax error: non-integer value for row specification!")
            return "error"

        if which_row > 1:
            result_straight = movebase_client((which_row-1)*row_width, 0.0, 0.0)
            if result_straight:
                rospy.logwarn("3.1 Straight goal execution done!")
            else:
                rospy.logerr("3.1 Straight goal not reached!")
                return "error"

        result_turn = movebase_client(0.0, 0.0, turn/2)
        if result_turn:
            rospy.logwarn("3.2 Turn goal execution done!")
        else:
            rospy.logerr("3.2 Turn goal not reached!")
            return "error"

        result_turn = movebase_client(lin_row_enter, row_width/2-0.1, turn/2)
        if result_turn:
            rospy.logwarn("4. Turn goal execution done!")
        else:
            rospy.logerr("4. Turn goal not reached!")
            return "error"

        result_straight = movebase_client(lin_row_enter/2, 0.0, 0.0)
        if result_straight:
            rospy.logwarn("5. Straight goal execution done!")
        else:
            rospy.logerr("5. Straight goal not reached!")
            return "error"

        return "state_in_row"
    else:
        return "done"

def movebase_client(lin_x, lin_y, ang_z):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = lin_x
    goal.target_pose.pose.position.y = lin_y
    q_rot = quaternion_from_euler(0.0, 0.0, ang_z)
    goal.target_pose.pose.orientation.x = q_rot[0]
    goal.target_pose.pose.orientation.y = q_rot[1]
    goal.target_pose.pose.orientation.z = q_rot[2]
    goal.target_pose.pose.orientation.w = q_rot[3]

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
        

def state_idle():
    pass
    return "state_idle"

def state_error():
    pass
    return "state_done"

            
angle = 0.0
row_width = 0.75
turn_l = np.pi/2
turn_r = -np.pi/2
lin_row_exit = 1.0
lin_row_enter = lin_row_exit
state = "state_in_row" 

if __name__ == '__main__':
    rospy.init_node('path_planning', anonymous=True)
    sub_laser = rospy.Subscriber("front/scan", LaserScan, laser_callback)
    pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown() and state != "done":
        if state == "state_in_row":
            state = state_in_row(pub_vel)
        elif state == "state_headland":
            state = state_headland()
        elif state == "state_idle":
            state = state_idle()
        elif state == "error":
            state = state_error()
        else:
            state = "done"

        print(state)
        
        rate.sleep()