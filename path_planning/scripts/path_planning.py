#!/usr/bin/env python
from sys import path
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


import numpy as np
import matplotlib.pyplot as plt
import collections


class MoveRobotPathPattern:
    def __init__(self):
        self.sub_laser = rospy.Subscriber("front/scan", LaserScan, self.laser_callback, queue_size=3)   # [Laserscan] subscriber on /front/scan
        self.pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size=3)                                  # [Twist] publisher on /cmd_vel
        self.p_gain_tuning_factor_in_row = rospy.get_param('~p_gain_tuning_factor_in_row')              # [1.0] factor used in the gain of the p-controller applied to the mid-row offset for driving within a row
        self.p_gain_tuning_factor_in_headland = rospy.get_param('~p_gain_tuning_factor_in_headland')    # [1.0] factor used in the gain of the p-controller applied to the mid-row offset for driving within the headland
        self.max_lin_vel_in_row = rospy.get_param('~max_lin_vel_in_row')                                # [m/s] maximum linear velocity for driving within a row
        self.max_lin_vel_in_headland= rospy.get_param('~max_lin_vel_in_headland')                       # [m/s] maximum linear velocity for driving within the headland
        self.max_ang_vel_robot= rospy.get_param('~max_ang_vel_robot')                                   # [rad/s] maximum angluar velocity of robot
        self.path_pattern = rospy.get_param('~path_pattern')                                            # [str] path pattern describing the robots trajectory through the field e.g. S-1L-2L-1L-0R-F
        self.time_for_quater_turn = rospy.get_param('~time_for_quater_turn')                            # [s] the time for a +-pi/2 turn used to exit or enter a row. It determines the angular velocity, but not the stop criterion
        self.path_pattern = self.path_pattern.replace(" ", "")                                          # remove spaces: S 1L 2L 1L 0R F --> S1L2L1L0RF
        self.path_pattern = self.path_pattern[1:-1]                                                     # remove S (Start) and F (Finish): S1L2L1L1RF --> 1L2L1L0R

        self.scan = LaserScan()                                                                         # [Laserscan] saving the current laser scan
        self.x_front_laser_in_base_link = 0.12                                                          # [m] x-coordinate of front_laser frame in base_link frame 
        self.sums_ranges = collections.deque(maxlen=10)                                                 # circular buffer for the sum of ranges used in detect_row_end
        self.x_means = collections.deque(maxlen=5)                                                      # circular buffer for the means of x-coordinates used in state_turn_exit and state_turn_enter
        self.y_means = collections.deque(maxlen=5)                                                      # circular buffer for the means of y-coordinates used in state_turn_exit and state_turn_enter
        self.x_mean = 0.0                                                                               # [m] mean of means of x-coordinates
        self.y_mean = 0.0                                                                               # [m] mean of means of y-coordinates
        self.x_mean_old = 0.0                                                                           # [m] previous mean of means of x-coordinates
        self.y_mean_old = 0.0                                                                           # [m] previous mean of means of y-coordinates
        self.row_width = 0.75                                                                           # [m] row width
        self.turn_l = np.pi/2                                                                           # [rad] angle defining a left turn
        self.turn_r = -np.pi/2                                                                          # [rad] angle defining a right turn
        self.state = "state_wait_at_start"                                                              # [str] state that the state machine starts with
        self.angle_valid = 0.0                                                                          # [rad] valid mid-row angle applicable for robot control
        self.offset_valid = 0.0                                                                         # [m] valid mid-row offset applicable for robot control
        self.time_start = rospy.Time.now()                                                              # [rospy.Time] timestamp used in state_headlands
        self.laser_box_drive_headland = np.zeros((10,2))
        self.laser_box_detect_row = np.zeros((10,2))
        self.xy_scan_raw = np.zeros((10,2))
        self.there_was_row = False
        self.there_was_no_row = False
        self.trans_norow2row = False
        self.trans_row2norow = True
        self.ctr_trans_row2norow = 0
        self.ctr_trans_norow2row = 0
        self.range_factor = 1.0                                                                         # [1] factor modifiying the used range of the laser scan when the robot can't see something 
        self.sum_scan_dots = 0                                                                          # [1] stores the scan dots seen in a small laser box right in front of the robot

    #########################################
    ######### Miscellaneous Methods #########
    #########################################

    def scan2cart_w_ign(self, scan, max_range=30.0):
        """
        Module converting a ROS LaserScan into cartesian coordinates.
        param1 scan:        [LaserScan] raw laser scan
        param2 max_range:   [m] ranges greater than this distances are omitted
        return:             x- and y-coordinates of laser scan
        """
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        x = scan.ranges*np.cos(angles)
        y = scan.ranges*np.sin(angles)
        scan_cart = np.array([x, y])
        ignore_ranges = np.array(scan.ranges) > max_range
        scan_cart[:, ignore_ranges] = None
        return scan_cart

    def scan2cart_wo_ign(self, scan):
        """
        Module converting a ROS LaserScan into cartesian coordinates.
        param1 scan:        [LaserScan] raw laser scan
        param2 max_range:   [m] ranges greater than this distances are omitted
        return:             x- and y-coordinates of laser scan
        """
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        x = scan.ranges*np.cos(angles)
        y = scan.ranges*np.sin(angles)
        scan_cart = np.array([x, y])
        return scan_cart

    def detect_row_end(self, scan, angle_min, angle_max, range_max):
        """
        Module that uses the laser scan to detect whether the robot has reached 
        the end of the row. It does so by summing up the ranges in a cropped
        laser scan. If the sum is below a threshold for several times in a row,
        the end of the field is most probably reached.
        param1 scan:        [LaserScan] raw laser scan
        param2 angle_min:   [rad] min angle for cropping laser scan in order to detect the end of the row
        param3 angle_max:   [rad] max angle for cropping laser scan in order to detect the end of the row
        param4 range_max:   [m] max range allowed for ranges in cropped laser scan
        param5 ctr_max:     [1] how many times the sum of ranges has to fall below the threshold
                            in order to say the end of the row is reached.
        return:             [False, True] end of row reached (True) or not (False)
        """
        thresh_sums_ranges = 1.0 # [m] threshold the sum of ranges is evaluated with
        # crop the laser scan
        if scan.angle_min > angle_min:
            angle_min = scan.angle_min
        if scan.angle_max < angle_max:
            angle_max = scan.angle_max
        num_ranges = len(scan.ranges)
        idx_lower = int(num_ranges/2 + np.round(angle_min / scan.angle_increment))
        idx_upper = int(num_ranges/2 + np.round(angle_max / scan.angle_increment))
        scan_cropped = np.array(scan.ranges[idx_lower:idx_upper])
        # filter ranges above range_max
        allow_ranges = scan_cropped <= range_max
        scan_cropped = scan_cropped[allow_ranges]
        # sum up all ranges and append them to the circular deque buffer
        self.sums_ranges.append(scan_cropped.sum())
        # if the median of the whole deque is below the defined threshold,
        # this will be considered as the end of the row.
        end_of_row = np.median(self.sums_ranges) < thresh_sums_ranges
        return end_of_row

    def detect_robot_running_crazy(self, scan):
        thresh_sum_scan_dots = 30
        nose_box_width = 0.10
        nose_box_height = 0.05
        x_min = 0.23 - nose_box_height/2
        x_max = 0.23 + nose_box_height/2
        y_min = -nose_box_width/2
        y_max = nose_box_width/2
        nose_box = self.laser_box(scan, x_min, x_max, y_min, y_max)
        num_scan_dots = nose_box.shape[1]
        self.sum_scan_dots += num_scan_dots
        print("scan points", num_scan_dots, "self.sum_scan_dots", self.sum_scan_dots)
        if self.sum_scan_dots > thresh_sum_scan_dots:
            # This does not work here
            print("entered state error")
            self.state == "state_error"

    
    def laser_box(self, scan, x_min, x_max, y_min, y_max):
        xy_all = self.scan2cart_wo_ign(scan)
        allow_x1 = xy_all[0,:] > x_min
        allow_x2 = xy_all[0,:] < x_max
        allow_x = np.logical_and(allow_x1, allow_x2)

        allow_y1 = xy_all[1,:] > y_min
        allow_y2 = xy_all[1,:] < y_max
        allow_y = np.logical_and(allow_y1, allow_y2)

        allow = np.logical_and(allow_x, allow_y)
        return xy_all[:, allow]

    def move_robot(self, pub, distance, angle, period):
        """
        Module that translates the robot according to a distance and rotates it
        according to an angle in a given period of time.
        param1 pub:         [publisher] ROS publisher on topic /cmd_vel
        param2 distance:    [m] distance the robot is to move in x-direction
        param3 angle:       [rad] yaw angle the robot is to turn around z-axis
        param4 period:      [s] time period available for the movement
        return:             nothing
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = distance / period
        cmd_vel.angular.z = angle / period
        pub.publish(cmd_vel)
        return None

    def clip(self, val, max_val, min_val):
        if val > max_val:
            return max_val
        elif val < min_val:
            return min_val
        else:
            return val    

    def laser_callback(self, scan):
        self.scan = scan
        return None

    ##########################################
    ######### States of Statemachine #########
    ##########################################
    def state_wait_at_start(self):
        # This is because the robot falls down when launching the simulation.
        # Therefore we need to wait until the robot has come down.
        t = 5.0 # [s] period of time that the robot waits before entering the first row
        if rospy.Time.now() - self.time_start > rospy.Duration.from_sec(t):                
            return "state_in_row"
        else:
            return "state_wait_at_start"

    def state_in_row(self, pub_vel):
        # mach das auch wenn nur eine reihe
        angle_increment = self.scan.angle_increment
        angle_outer_limit_curr = self.scan.angle_max
        angle_outer_limit_targ = np.radians(130)
        angle_inner_limit_targ = np.radians(20)
        idx_ranges_outer = int(np.round((angle_outer_limit_curr - angle_outer_limit_targ) / angle_increment))
        idx_ranges_inner = int(np.round((angle_outer_limit_curr - angle_inner_limit_targ) / angle_increment))
        
        scan_cart = self.scan2cart_w_ign(self.scan, max_range=self.row_width*self.range_factor)
        scan_left = scan_cart[:, -idx_ranges_inner:-idx_ranges_outer]
        scan_right = scan_cart[:, idx_ranges_outer:idx_ranges_inner]

        mean_left = np.nanmean(scan_left[1, :])
        mean_right = np.nanmean(scan_right[1, :])

        offset = mean_right + mean_left
        if not np.isnan(offset):
            # If the determined offset is a number, the controller
            # will get an update with the current offset value. 
            # Additionally we will reset the used range of the laser scanner.  
            self.range_factor = 1.0         
            self.offset_valid = offset
        else:            
            # If the determined offset is not a number (nan) a warning is raised and
            # the controller will not get an update but uses the last valid offset.
            # Additionally, we will increase the used range of the laser scanner 
            # each time if the warning persists. This will eventually solve the warning.
            self.range_factor += 0.001

        # control variables concerning the mid-row-offset
        setpoint_offset = 0                                                                     # [m] setpoint for offset
        error_offset = setpoint_offset - self.offset_valid                                      # [m] control deviation
        max_offset = self.row_width/2                                                           # [m] maximum mid-row-offset possible
        p_gain_offset = self.max_ang_vel_robot/max_offset*self.p_gain_tuning_factor_in_row      # [(rad*m)/s] gain for p controller
        act_offset = -p_gain_offset*error_offset                                                # [rad/s] actuating variable
        
        normed_offset_abs = np.abs(self.offset_valid / max_offset)
        normed_offset_abs = self.clip(normed_offset_abs, 1.0, 0.0)
        cmd_vel = Twist()
        cmd_vel.linear.x = self.max_lin_vel_in_row * (1 - normed_offset_abs)
        cmd_vel.angular.z = act_offset
        pub_vel.publish(cmd_vel)       

        end_of_row = self.detect_row_end(self.scan, np.radians(-85), np.radians(85), 5.0)

        if end_of_row:
            # Check if path pattern has already been completed
            if len(self.path_pattern) > 0:
                return "state_turn_exit"
            else:
                "state_done"
        else:
            return "state_in_row"

    def state_turn_exit(self, pub_vel):
        # TODO: nimm die lineargeschwqindigkeit von ende der reihe
        
        # Extract scan points out of a rectangular box. This box is 
        # placed around the robot with itself lying in the center.
        # The scan points falling in this box are evaluated:
        # use the mean of x-coordinates to turn out of the row correctly

        self.xy_scan_raw = self.scan2cart_w_ign(self.scan, max_range=30.0)
        # the robot has always to see an uneven number of rows
        x_min = -1.5*self.row_width
        x_max = 1.5*self.row_width
        y_min = -1.5*self.row_width
        y_max = 1.5*self.row_width
        self.laser_box_drive_headland = self.laser_box(self.scan, x_min, x_max, y_min, y_max)
        self.x_means.append(np.mean(self.laser_box_drive_headland[0,:]))
        self.x_mean = np.mean(self.x_means) 

        # extract next turn from path pattern
        # and check direction ('L' or 'R' ?)
        which_turn = self.path_pattern[1]
        if which_turn == 'L':
            turn = self.turn_l
        elif which_turn == 'R':
            turn = self.turn_r

        ang_z = turn                            # [rad]
        dist_x = self.row_width/2 * abs(ang_z)  # [m]
        t = self.time_for_quater_turn           # [s]

        # Check if the same row is to be entered again (-> 0)
        # If this is the case, we want the robot to turn in place
        # and therefore have no linear velocity but only
        # angular velocity. Thus, we set the distance in x direction
        # to zero.
        which_row = int(self.path_pattern[0])
        if which_row == 0:
            dist_x = 0.0

        x_close_to_zero = abs(self.x_mean) < 5e-2
        x_zero_crossing = self.x_mean*self.x_mean_old < 0.0
        if x_close_to_zero or x_zero_crossing:
            self.time_start = rospy.Time.now()
            # reset variable
            self.x_mean_old = 0.0
            return "state_headlands"
        else:
            self.move_robot(pub_vel, dist_x, ang_z, t)
            self.x_mean_old = self.x_mean
            return "state_turn_exit"

    def state_headlands(self, pub_vel):
        # TODO: 
        # - vel in headland close to previous velocity
        # - increase width of box so that even shortened row ends
        #   will be detected

        # Extract scan points out of a rectangular box. This box is 
        # placed around the robot with itself lying in the center.
        # The scan points falling in this box are evaluated:
        # use the mean of x coordinates to control the robot to pass by

        self.xy_scan_raw = self.scan2cart_w_ign(self.scan, max_range=30.0)
        # the robot has always to see an uneven number of rows
        x_min_drive_headland = -1.5*self.row_width
        x_max_drive_headland = 1.5*self.row_width
        which_turn = self.path_pattern[1]
        # TODO: increase box width in y diretion if no scans in headland
        if which_turn == 'L':
            y_min_drive_headland = 0.0
            y_max_drive_headland = 1.5*self.row_width
        elif which_turn == 'R':
            y_min_drive_headland = -1.5*self.row_width
            y_max_drive_headland = 0.0
        self.laser_box_drive_headland = self.laser_box(self.scan, x_min_drive_headland, x_max_drive_headland, y_min_drive_headland, y_max_drive_headland)
        self.x_means.append(np.mean(self.laser_box_drive_headland[0,:]))
        self.y_means.append(np.mean(self.laser_box_drive_headland[1,:]))
        self.x_mean = np.mean(self.x_means)
        self.y_mean = np.mean(self.y_means)     
        
        # Extract scan points out of rectangular box.
        # This box is placed in front of the robot so that
        # it sees the end of a row when passing by in headland.
        # It is dependent on the direction of the turn defined
        # by the path pattern: If the robot exited a row via
        # a left turn, the box is set to the left. If the robot
        # exited a row via a right turn, the box is set to the right.
        # This prevents the robot from accidently seeing the qr-code 
        # tower as a corn row.
        box_height = 0.35
        box_width = 2.0
        x_min_detect_row = self.row_width - self.x_front_laser_in_base_link - box_height/2
        x_max_detect_row = self.row_width - self.x_front_laser_in_base_link + box_height/2
        which_turn = self.path_pattern[1]
        if which_turn == 'L':
            y_min_detect_row = 0.0
            y_max_detect_row = box_width
        elif which_turn == 'R':
            y_min_detect_row = -box_width
            y_max_detect_row = 0.0
        self.laser_box_detect_row = self.laser_box(self.scan, x_min_detect_row, x_max_detect_row, y_min_detect_row, y_max_detect_row)

        # Count the scan points within the defined box and
        # identify whether a row is seen or the space in between.
        upper_thresh_scan_points = 20
        lower_thresh_scan_points = 10
        num_scan_dots = self.laser_box_detect_row.shape[1]
        there_is_row = num_scan_dots > upper_thresh_scan_points
        there_is_no_row = num_scan_dots < lower_thresh_scan_points

        # Count the transitions 
        # from seeing a row to seeing the space in between or 
        # from seeing the space in between to seeing a row.        
        if (there_is_no_row and self.trans_row2norow) or self.there_was_no_row:
            self.there_was_no_row = True
            if there_is_row:
                self.ctr_trans_norow2row += 1
                print("no row -> row")
                self.there_was_no_row = False
                self.trans_norow2row = True
                self.trans_row2norow = False

        if (there_is_row and self.trans_norow2row) or self.there_was_row:
            self.there_was_row = True
            if there_is_no_row:
                self.ctr_trans_row2norow += 1
                print("row -> no row")
                self.there_was_row = False
                self.trans_norow2row = False
                self.trans_row2norow = True
        
        # If the robot has go to e.g. the 3rd row on the left side,
        # the following transitions must be detected in order to
        # be able to turn into this row:
        # row --> no row (this one is skipped)
        # no row --> row
        # row --> no row
        # no row --> row
        # Thus, we have 2 * 3rd row - 3 = 3 transitions
        sum_transitions = self.ctr_trans_row2norow + self.ctr_trans_norow2row
        which_row = int(self.path_pattern[0])
        set_transitions = 2 * which_row - 3

        print(sum_transitions, "/", set_transitions, "points", num_scan_dots)
        target_row_reached = sum_transitions >= set_transitions
        if target_row_reached:
            self.there_was_row = False
            self.there_was_no_row = False
            self.trans_norow2row = False
            self.trans_row2norow = True
            self.ctr_trans_row2norow = 0
            self.ctr_trans_norow2row = 0
            return "state_turn_enter"
        else:
            # The robot is passing by some rows.
            # When the current section of path pattern is e.g. 3L,
            # it passes by the first and second row on the left
            # in order to turn into the third one.
            # While passing by the robot is not ought to drive blindly
            # but controlled. For doing so we evaluate the mean
            # of the x-coordinates of all scan points that are
            # in our scan box which is placed around the robot
            # (see laser_callback). As long as the x-mean is around
            # zero, the robot is passing by the desired rows
            # orthogonally.

            # control variables concerning the x-mean
            setpoint_offset = 0                                                                         # [m] setpoint for x_mean
            error_offset = setpoint_offset - self.x_mean                                                # [m] control deviation
            max_offset = 1.5                                                                            # [m] maximum mid-row-offset possible
            p_gain_offset = self.max_ang_vel_robot/max_offset*self.p_gain_tuning_factor_in_headland     # [(rad*m)/s] gain for p controller
            act_offset = p_gain_offset*error_offset                                                     # [rad/s] actuating variable
                    
            normed_offset_abs = np.abs(self.x_mean / max_offset)
            normed_offset_abs = self.clip(normed_offset_abs, 1.0, 0.0)

            cmd_vel = Twist()
            cmd_vel.linear.x = self.max_lin_vel_in_headland * (1 - normed_offset_abs)
            cmd_vel.angular.z = act_offset
            pub_vel.publish(cmd_vel)
            return "state_headlands"

    def state_turn_enter(self, pub_vel):
        # Extract scan points out of a rectangular box. This box is 
        # placed around the robot with itself lying in the center.
        # The scan points falling in this box are evaluated:
        # use the mean of y-coordinates to turn out of the row correctly

        self.xy_scan_raw = self.scan2cart_w_ign(self.scan, max_range=30.0)
        # the robot has always to see an uneven number of rows
        x_min = -1.5*self.row_width
        x_max = 1.5*self.row_width
        y_min = -1.5*self.row_width
        y_max = 1.5*self.row_width
        self.laser_box_drive_headland = self.laser_box(self.scan, x_min, x_max, y_min, y_max)
        self.y_means.append(np.mean(self.laser_box_drive_headland[0,:]))
        self.y_mean = np.mean(self.y_means) 

        # extract next turn from path pattern
        # and check direction ('L' or 'R' ?)
        which_turn = self.path_pattern[1]
        if which_turn == 'L':
            turn = self.turn_l
        elif which_turn == 'R':
            turn = self.turn_r

        ang_z = turn                            # [rad]
        dist_x = self.row_width/2 * abs(ang_z)  # [m]
        t = self.time_for_quater_turn           # [s]

        # Check if the same row is to be entered again (-> 0)
        # If this is the case, we want the robot to turn in place
        # and therefore have no linear velocity but only
        # angular velocity. Thus, we set the distance in x direction
        # to zero.
        which_row = int(self.path_pattern[0])
        if which_row == 0:
            dist_x = 0.0

        y_close_to_zero = abs(self.y_mean) < 5e-2
        y_zero_crossing = self.y_mean*self.y_mean_old < 0.0
        if y_close_to_zero or y_zero_crossing:
            # reset variable
            self.y_mean_old = 0.0
            return "state_crop_path_pattern"
        else:
            self.move_robot(pub_vel, dist_x, ang_z, t)
            self.y_mean_old = self.y_mean
            return "state_turn_enter"

    def state_crop_path_pattern(self):
        # remove executed turn from path pattern
        self.path_pattern = self.path_pattern[2::]
        return "state_in_row" 

    def state_idle(self, pub_vel):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z = 0.0
        pub_vel.publish(cmd_vel)
        self.detect_robot_running_crazy(self.scan)
        return "state_idle"

    def state_error(self):
        print("An error has occured and the robot is in safeguard stop.")
        return "state_done"

    ###########################################
    ######### The Statemachine itself #########
    ###########################################

    def launch_state_machine(self):
        rate = rospy.Rate(10)
        fig = plt.figure(figsize=(7,20))
        while not rospy.is_shutdown() and self.state != "state_done":
            if self.state == "state_wait_at_start":
                self.state = self.state_wait_at_start()
            elif self.state == "state_in_row":
                self.state = self.state_in_row(self.pub_vel)
            elif self.state == "state_turn_exit":
                self.state = self.state_turn_exit(self.pub_vel)
            elif self.state == "state_headlands":
                self.state = self.state_headlands(self.pub_vel)
            elif self.state == "state_turn_enter":
                self.state = self.state_turn_enter(self.pub_vel)    
            elif self.state == "state_crop_path_pattern":
                self.state = self.state_crop_path_pattern()
            elif self.state == "state_idle":
                self.state = self.state_idle(self.pub_vel)
            elif self.state == "state_error":
                self.state = self.state_error()
            else:
                self.state = "state_done"
            print(self.state)
            rate.sleep()

            resolution = 0.10
            hist_min = -1.5*self.row_width
            hist_max = 1.5*self.row_width
            bins = int(round((hist_max - hist_min) / resolution))

            plt.subplot(311)
            plt.hist(self.laser_box_drive_headland[0,:], bins, label='x', range=[hist_min, hist_max], density=True)
            plt.axvline(self.x_mean, color='r', linestyle='dashed', linewidth=2)
            plt.legend(loc='upper left')

            plt.subplot(312)
            plt.hist(self.laser_box_drive_headland[1,:], bins, label='y', range=[hist_min, hist_max], density=True)
            plt.axvline(self.y_mean, color='r', linestyle='dashed', linewidth=2)
            plt.legend(loc='upper left')

            ax1 = plt.subplot(313, aspect='equal')
            ax1.set_xlim([-2, 2])
            ax1.set_ylim([-2, 2])
            plt.grid(color='k', alpha=0.5, linestyle='dashed', linewidth=0.5)
            plt.plot(self.xy_scan_raw[0,:],self.xy_scan_raw[1,:], "ob")
            plt.plot(self.laser_box_detect_row[0,:],self.laser_box_detect_row[1,:], "oy")
            plt.plot(self.x_mean, self.y_mean, "or")
            plt.draw()
            plt.pause(0.05)
            fig.clear()      
        print("Moving robot according to path pattern completed.")
        return None

if __name__ == '__main__':
    rospy.init_node('path_planning', anonymous=True)

    move_robot = MoveRobotPathPattern()
    move_robot.launch_state_machine()
