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
    global theta
    global ydist
    global theta_valid
    global ydist_valid
    global row_width
    if state == "state_in_row":
        # convert scan from polar into cartesian coordinates
        scan_cart = scan2cart(scan, max_range=30.0)

        # determine euclidean distance between each subsequent scan point
        x_dist_scan = np.diff(scan_cart[:, 0])
        y_dist_scan = np.diff(scan_cart[:, 1])
        eucl_dist = np.sqrt(x_dist_scan**2 + y_dist_scan**2)
        eucl_dist = np.insert(eucl_dist, 0, 0, axis=0)

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
        # # PLOT XY-SCATTER
        # # of cartesian scan and mean position of detected plants
        # plt.figure()
        # plt.plot(scan_cart[:,0], scan_cart[:,1], "ob", label="cartesian laser scan")
        # plt.plot(plants_all[:,0], plants_all[:,1], "xr", label="averaged plant position with at least 3 laser points")
        # plt.legend(loc='lower left')
        # plt.show(block=True)

        # calculate line angles from one plant to another
        x_dist_plant = np.diff(plants_all[:, 0])
        y_dist_plant = np.diff(plants_all[:, 1])
        thetas = np.arctan2(y_dist_plant, x_dist_plant)
        # change perspective plants to robot --> robot to plants
        thetas = -thetas
        # Modify angles so that 
        # e.g. [   9  125   17   -2   75 -176 -175   57 -164] --> [   9  125   17   -2   75    4    5   57 -164]
        thetas = np.where(thetas < np.radians(-170), thetas + np.pi, thetas)
        thetas = np.where(thetas > np.radians(170), thetas - np.pi, thetas)

        # apply a histogram to line angles
        range_min_theta = -np.pi
        range_max_theta = np.pi
        bin_res = np.radians(5) # [radians/bin] resolution of histogram
        n_bins_theta = int(np.round((range_max_theta - range_min_theta) / bin_res))
        bin_counts, bin_edges = np.histogram(thetas, bins=n_bins_theta, range=(range_min_theta, range_max_theta))
        # # PLOT HISTOGRAM
        # # of line angles
        # fig, axs = plt.subplots()
        # axs.hist(np.degrees(thetas), bins=n_bins_theta, range=(np.degrees(range_min_theta), np.degrees(range_max_theta)))
        # plt.show(block=True)

        # evaluate histogram around previous angle of robot to the center line of corn row
        if not np.isnan(theta):
            # theta will only be updated if the previous theta has been a valid angle
            # at the end of the row, 'nan' may be assigned to theta
            # in such a case the previous theta will not be updated
            theta_valid = theta
        else:
            theta_valid = 0
        tol_degrees = 10 # [degrees] when determining theta, we will look for angles +- this angle around theta_old
        tol_degrees_bins = np.round(np.radians(tol_degrees) / bin_res)
        theta_hist_idx = np.round((theta_valid + np.pi) / bin_res)
        indices_peak_theta = indices_peak(theta_hist_idx, tol_degrees_bins, n_bins_theta)
        # determine current angle of robot to the center line of corn row
        theta = mean_peak(indices_peak_theta, bin_counts, bin_edges, bin_res)

        # rotate scan around z by theta_new
        c, s = np.cos(-theta), np.sin(-theta)
        rot_z = np.array(((c, -s), (s, c)))
        scan_cart_rot = np.matmul(scan_cart, rot_z)
        # # PLOT XY-SCATTER
        # # of rotated cartesian scan
        # plt.figure()
        # plt.plot(scan_cart_rot[:,0], scan_cart_rot[:,1], "ob", label="rotated cartesian laser scan")
        # plt.legend(loc='lower left')
        # plt.show(block=True)

        # apply a histogram to y-distances of rotated scan
        range_min_ydist = -row_width # [m]
        range_max_ydist = row_width # [m]
        bin_res = 0.01 # [m]
        n_bins_ydist = int(np.round((range_max_ydist - range_min_ydist) / bin_res)) # number of bins for y dist histogram
        y_dists = scan_cart_rot[:, 1]
        bin_counts, bin_edges = np.histogram(y_dists, bins=n_bins_ydist, range=(range_min_ydist, range_max_ydist))
        # # PLOT HISTOGRAM
        # # of y distances from robot to plants
        # fig, axs = plt.subplots()
        # axs.hist(y_dists, bins=n_bins_ydist, range=(range_min_ydist, range_max_ydist))
        # plt.show(block=True)  

        # evaluate histogram around y-distance of robot to the center line of corn row
        if not np.isnan(ydist):
            # ydist will only be updated if the previous ydist has been a valid angle
            # at the end of the row, 'nan' may be assigned to ydist
            # in such a case the previous ydist will not be updated
            ydist_valid = ydist
        else:
            ydist_valid = 0
        tol_ydist = 0.03 # [m]
        tol_ydist_bins = np.round(tol_ydist / bin_res) # [bins]
        ydist_max_idx = np.argmax(bin_counts)
        indices_peak_ydist = indices_peak(ydist_max_idx, tol_ydist_bins, n_bins_ydist)
        # determine current y-distance of robot to the center line of corn row
        ydist = mean_peak(indices_peak_ydist, bin_counts, bin_edges, bin_res)
        if ydist < 0:
            ydist += row_width/2
        else:
            ydist -= row_width/2      
    else:
        pass

def indices_peak(peak_idx, tol_bins, n_bins):
    """ Module returning the indices before, at and after a peak for
    evaluating a histogram
    param1 peak_idx:    [bins] the index of the peak within the histogram
    param2 tol_bins:    [bins] according to this tolerance indices will be returned
                        before and after the peak
    param3 n_bins:      [bins] the number of bins in the histogram
    """
    if peak_idx + tol_bins >= n_bins:
        indices = np.r_[(peak_idx - tol_bins):n_bins, 0:(peak_idx + tol_bins - n_bins - 1)]
    elif peak_idx - tol_bins < 0:
        indices = np.r_[(peak_idx - tol_bins + n_bins):n_bins, 0:(peak_idx + tol_bins + 1)]
    else:
        indices = np.r_[(peak_idx - tol_bins):(peak_idx + tol_bins + 1)]
    return indices.astype(dtype=int)

def mean_peak(indices_peak, bin_counts, bin_edges, bin_res):
    """ Module returning the mean of a peak in a histogram
    param1 indices_peak:    [bins] the indices before, at and after a peak
    param2 bin_counts:      [1] counts of each bin of histogram
    param3 bin_edges:       [*] edges of each bin of histogram
    param4 bin_res:         [*/bin] resolution of a bin of histogram 
    """
    bin_counts_peak = bin_counts[indices_peak]
    if sum(bin_counts_peak) <= 1e-16:
        # if all bins before, at and after the peak are empty,
        # we pretend the bin in the middle has one hit:
        # e.g. [0, 0, 0, 0, 0] --> [0, 0, 1, 0, 0]
        bin_counts_peak = np.zeros_like(bin_counts_peak)
        bin_counts_peak[bin_counts_peak.shape[0]//2] = 1
    bin_edges_peak = bin_edges[indices_peak] + bin_res / 2
    return np.sum(bin_counts_peak * bin_edges_peak) / np.sum(bin_counts_peak)
    
def state_in_row(pub_vel):
    global row_width
    global p_gain_theta_factor
    global p_gain_ydist_factor
    global velz_control

    setpoint_theta = 0;                         # [rad]
    setpoint_ydist = 0;                         # [m]
    error_theta = setpoint_theta - theta_valid
    error_ydist = setpoint_ydist - ydist_valid

    max_angular_z = np.pi/2;                    # [rad/s]
    max_theta = np.pi/4;                        # [rad]
    max_ydist = row_width/2;                    # [m]
    p_gain_theta = max_angular_z/max_theta*p_gain_theta_factor;     # = 2.0
    p_gain_ydist = max_angular_z/max_ydist*p_gain_ydist_factor;     # = 4.19

    act_theta = p_gain_theta*error_theta
    act_ydist = -p_gain_ydist*error_ydist
    
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.5
    if velz_control == "theta":
        cmd_vel.angular.z = act_theta
    elif velz_control == "ydist":
        cmd_vel.angular.z = act_ydist
    else:
        cmd_vel.angular.z = act_theta/2 + act_ydist/2

    pub_vel.publish(cmd_vel)        
    # print(ydist_valid, act_ydist)
    # print(theta_valid, act_theta)

    if np.isnan(theta) and np.isnan(ydist):
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

row_width = 0.75
turn_l = np.pi/2
turn_r = -np.pi/2
lin_row_exit = 1.0
lin_row_enter = lin_row_exit
state = "state_in_row"
theta = 0.0
ydist = 0.0
theta_valid = 0.0
ydist_valid = 0.0
p_gain_theta_factor = 0.0
p_gain_ydist_factor = 0.0
velz_control = 0.0

if __name__ == '__main__':
    rospy.init_node('path_planning', anonymous=True)
    sub_laser = rospy.Subscriber("front/scan", LaserScan, laser_callback)
    pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)

    p_gain_theta_factor = rospy.get_param('~p_gain_theta_factor')
    p_gain_ydist_factor = rospy.get_param('~p_gain_ydist_factor')
    velz_control = rospy.get_param('~velz_control')
    
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
