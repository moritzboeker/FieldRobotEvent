#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler


import numpy as np
import matplotlib.pyplot as plt
# from scipy.cluster.hierarchy import fclusterdata

def scan2cart(scan, max_range=30.0):
    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    x = scan.ranges*np.cos(angles)
    y = scan.ranges*np.sin(angles)
    scan_cart = np.array([x, y])
    scan_cart = np.transpose(scan_cart)
    # ignore_ranges = np.array(scan.ranges) > max_range
    # scan_cart[ignore_ranges, :] = None
    allow_ranges = np.array(scan.ranges) <= max_range
    scan_cart = scan_cart[allow_ranges, :]
    return scan_cart




def laser_callback(scan):
    global alpha_old
    global alpha_new
    if state == "state_in_row":
        # convert scan from polar into cartesian coordinates
        scan_cart = scan2cart(scan, max_range=30.0)

        # determine euclidean distance between each subsequent scan point
        x_dist_scan = np.diff(scan_cart[:, 0])
        y_dist_scan = np.diff(scan_cart[:, 1])
        eucl_dist = np.sqrt(x_dist_scan**2 + y_dist_scan**2)
        eucl_dist = np.insert(eucl_dist, 0, 0, axis=0)
        
        # # plot histogram of x, y and euclidean distance
        # n_bins = 10
        # fig, axs = plt.subplots(1, 3, sharey=True, tight_layout=True)
        # axs[0].hist(x_dist_scan, bins=n_bins)
        # axs[1].hist(y_dist_scan, bins=n_bins)
        # axs[2].hist(eucl_dist, bins=n_bins)
        # plt.show(block=True)

        # determine which scan points belong to one and the same plant and split scan accordingly
        idx_next_plant = np.where(eucl_dist > 0.05)
        split_plants = np.vsplit(scan_cart, idx_next_plant[0])

        # determine mean position of all plants having at least n_points of scan points
        n_points = 3
        plants_all = []
        for plant in split_plants:
            if plant.shape[0] >= n_points:
                mean_position = np.nanmean(plant, axis=0)
                plants_all.append(mean_position)
        plants_all = np.array(plants_all)

        # # plot xy-graph of cartesian scan and mean position of detected plants
        # plt.figure()
        # plt.plot(scan_cart[:,0], scan_cart[:,1], "ob", label="cartesian laser scan")
        # plt.plot(plants_all[:,0], plants_all[:,1], "xr", label="averaged plant position with at least 3 laser points")
        # plt.legend(loc='lower left')
        # plt.show(block=True)

        # calculate line angles from one plant to another
        x_dist_plant = np.diff(plants_all[:, 0])
        y_dist_plant = np.diff(plants_all[:, 1])
        alphas = np.arctan2(y_dist_plant, x_dist_plant)
        # change perspective plants to robot --> robot to plants
        alphas = -alphas
        # Modify angles so that 
        # e.g. [   9  125   17   -2   75 -176 -175   57 -164] --> [   9  125   17   -2   75    3    4   57 -164]
        alphas = np.where(alphas < np.radians(-170), alphas + np.pi, alphas)
        alphas = np.where(alphas > np.radians(170), alphas - np.pi, alphas)

        # determine the histogram of line angles
        n_bins = 72
        range_min = -np.pi
        range_max = np.pi
        hist_res = (range_max - range_min) / n_bins # [degrees/bin] resolution of histogram
        bin_counts, edges = np.histogram(alphas, bins=n_bins, range=(range_min, range_max))
        # # plot histogram of line angles
        # fig, axs = plt.subplots()
        # axs.hist(np.degrees(alphas), bins=n_bins, range=(np.degrees(range_min), np.degrees(range_max)))
        # plt.show(block=True)

        # evaluate histogram around previous angle of robot to the center line of corn row
        tol_degrees = 10 # [degrees] when determining alpha, we will look for angles +- this angle around alpha_old
        tol_idx = np.round(np.radians(tol_degrees) / hist_res)
        alpha_old = alpha_new
        alpha_old_hist_idx = np.round((alpha_old + np.pi) / hist_res)
        if alpha_old_hist_idx + tol_idx >= n_bins:
            indices = np.r_[(alpha_old_hist_idx - tol_idx):, 0:(alpha_old_hist_idx + tol_idx - n_bins - 1)]
        elif alpha_old_hist_idx - tol_idx < 0:
            indices = np.r_[(alpha_old_hist_idx - tol_idx + n_bins):, 0:(alpha_old_hist_idx + tol_idx + 1)]
        else:
            indices = np.r_[(alpha_old_hist_idx - tol_idx):(alpha_old_hist_idx + tol_idx + 1)]
        indices = indices.astype(dtype=int)

        # determine current angle of robot to the center line of corn row
        bin_counts_peak = bin_counts[indices]
        edges_peak = edges[indices] + hist_res / 2
        alpha_new = np.sum(bin_counts_peak * edges_peak) / np.sum(bin_counts_peak)

        np.set_printoptions(precision=1, suppress=True)
        print("number of plants:", plants_all.shape[0], "angle:", np.degrees(alpha_new))
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
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.0
    # cmd_vel.angular.z = act_alpha
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
alpha_old = 0.0
alpha_new = 0.0

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
