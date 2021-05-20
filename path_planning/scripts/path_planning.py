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
    """
    Module converting a ROS LaserScan into cartesian coordinates.
    param1 scan:        [LaserScan] raw laser scan
    param2 max_range:   [m] ranges greater than this distances are omitted
    return:             x- and y-coordinates of laser scan
    """
    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    ignore_ranges = np.array(scan.ranges) > max_range
    x = scan.ranges*np.cos(angles)
    y = scan.ranges*np.sin(angles)
    scan_cart = np.array([x, y])
    scan_cart[:, ignore_ranges] = None
    return scan_cart

def check_end_of_row(scan, angle_min, angle_max, range_max, ctr_max):
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
    global end_row_ctr
    end_of_row = False
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
    # sum up all ranges
    sum_ranges = scan_cropped.sum()
    # if sum of ranges falls below threshold for several times in a row,
    # the end of the field is most probably reached 
    if sum_ranges < 1.0:
        end_row_ctr += 1
        print("sum_ranges:{:06.2f}, end_row_ctr: {:d}".format(sum_ranges, end_row_ctr))
        if end_row_ctr > ctr_max:
            end_of_row = True
    # else the robot is still in the field
    else:
        end_row_ctr = 0
    # # Debugging purposes: plot cropped scan
    # angles = np.linspace(angle_min, angle_max, idx_upper-idx_lower)
    # x = scan_cropped*np.cos(angles)
    # y = scan_cropped*np.sin(angles)
    # scan_cart = np.array([x, y])
    # scan_cart = np.transpose(scan_cart)
    # plt.plot(scan_cart[:,0], scan_cart[:,1], "ob", label="end of row scan")
    # plt.legend(loc='lower left')
    # plt.show(block=True)
    return end_of_row

def laser_callback(scan):
    if state == "state_in_row":
        global angle_valid
        global offset_valid
        global row_width
        global end_of_row

        angle_increment = scan.angle_increment
        angle_outer_limit_curr = scan.angle_max
        angle_outer_limit_targ = np.radians(130)
        angle_inner_limit_targ = np.radians(20)
        idx_ranges_outer = int(np.round((angle_outer_limit_curr - angle_outer_limit_targ) / angle_increment))
        idx_ranges_inner = int(np.round((angle_outer_limit_curr - angle_inner_limit_targ) / angle_increment))
        
        scan_cart = scan2cart(scan, max_range=row_width)
        scan_left = scan_cart[:, -idx_ranges_inner:-idx_ranges_outer]
        scan_right = scan_cart[:, idx_ranges_outer:idx_ranges_inner]

        mean_left = np.nanmean(scan_left[1, :])
        mean_right = np.nanmean(scan_right[1, :])

        offset = mean_right + mean_left
        if not np.isnan(offset):
            offset_valid = offset
        else:
            offset = offset_valid
        angle = - offset / (mean_left - mean_right)
        if not np.isnan(angle):
            angle_valid = angle

        end_of_row = check_end_of_row(scan, -np.pi/2, np.pi/2, 3.0, 20)     
    else:
        pass

def state_in_row(pub_vel):
    global time_start
    global end_of_row
    global angle_valid
    global offset_valid
    global p_gain_angle_factor
    global p_gain_offset_factor
    global ctrl_by_angle
    global ctrl_by_offset
    global max_lin_vel

    setpoint_angle = 0;                         # [rad]
    setpoint_offset = 0;                       # [m]
    error_angle= setpoint_angle - angle_valid
    error_offset = setpoint_offset - offset_valid

    max_angular_z = np.pi/2;                    # [rad/s]
    max_angle = np.pi/4;                        # [rad]
    max_offset = 0.75/2;                         # [m]
    p_gain_angle = max_angular_z/max_angle * p_gain_angle_factor;     # = 2.0
    p_gain_offset = max_angular_z/max_offset * p_gain_offset_factor;   # = 4.19

    act_angle = p_gain_angle*error_angle
    act_offset = -p_gain_offset*error_offset
    
    normed_angle_abs = np.abs(angle_valid / max_angle)
    normed_offset_abs = np.abs(offset_valid / max_offset)
    normed_angle_abs = clip(normed_angle_abs, 1.0, 0.0)
    normed_offset_abs = clip(normed_offset_abs, 1.0, 0.0)
    normed_deviation = (normed_angle_abs + normed_offset_abs) / 2
    cmd_vel = Twist()    

    if ctrl_by_angle and not ctrl_by_offset:
        cmd_vel.linear.x = max_lin_vel * (1 - normed_angle_abs)
        cmd_vel.angular.z = act_angle
    elif ctrl_by_offset and not ctrl_by_angle:
        cmd_vel.linear.x = max_lin_vel * (1 - normed_offset_abs)
        cmd_vel.angular.z = act_offset
    else:
        cmd_vel.linear.x = max_lin_vel * (1 - normed_deviation)
        cmd_vel.angular.z = act_angle/2 + act_offset/2

    pub_vel.publish(cmd_vel)        

    if end_of_row:
        time_start = rospy.Time.now()
        return "state_turn_exit_row"
    else:
        return "state_in_row"

def clip(val, max_val, min_val):
    if val > max_val:
        return max_val
    elif val < min_val:
        return min_val
    else:
        return val

def state_turn_exit_row(pub_vel):
    global path_pattern
    global row_width
    global turn_l
    global turn_r  
    global time_start
    global angle_valid
    global offset_valid

    if len(path_pattern) > 0:
        # extract next turn from path pattern
        # and check direction ('L' or 'R' ?)
        which_turn = path_pattern[1]
        if which_turn == 'L':
            turn = turn_l
        elif which_turn == 'R':
            turn = turn_r
        else:
            rospy.logerr("Path pattern syntax error: undefined parameter '%s' for turn specification!", which_turn)
            return "state_error"
        
        t = 5.0; # [s]
        radius = (row_width - offset_valid) / 2 # [m]
        angle = turn - angle_valid # [rad]
        radius = row_width / 2
        angle = turn
        cmd_vel = Twist()
        cmd_vel.linear.x = radius * abs(angle) / t
        cmd_vel.angular.z = angle / t
        pub_vel.publish(cmd_vel)
        print(np.degrees(angle), radius, rospy.Time.now() - time_start, rospy.Duration.from_sec(t), rospy.Time.now() - time_start > rospy.Duration.from_sec(t))
        if rospy.Time.now() - time_start > rospy.Duration.from_sec(t):            
            # if the robot is to enter one of the neighbouring rows
            # it skips the state of moving a straight distance
            which_row = int(path_pattern[0])
            if which_row == 1:
                time_start = rospy.Time.now()
                return "state_turn_enter_row"
            else:
                return "state_go_straight"
        else:
            return "state_turn_exit_row"
    else:
        return "state_done"

def state_go_straight():
    global path_pattern
    global row_width
    global time_start

    # extract straight distance from path pattern
    which_row = int(path_pattern[0])
    distance_x = (which_row-1)*row_width
    distance_y = 0.0
    angle = 0.0
    
    # but if the robot is to enter another row,
    # it firstly needs to drive to that row before turning into it
    result_straight = movebase_client(distance_x, distance_y, angle)
    if result_straight:
        rospy.logwarn("Straight goal execution done!")
        time_start = rospy.Time.now()
        return "state_turn_enter_row"
    else:
        rospy.logerr("Straight goal not reached!")
        return "state_error"

def state_turn_enter_row(pub_vel):
    global path_pattern
    global row_width
    global turn_l
    global turn_r  
    global time_start

    if len(path_pattern) > 0:
        # extract next turn from path pattern
        # and check direction ('L' or 'R' ?)
        which_turn = path_pattern[1]
        if which_turn == 'L':
            turn = turn_l
        elif which_turn == 'R':
            turn = turn_r
        else:
            rospy.logerr("Path pattern syntax error: undefined parameter '%s' for turn specification!", which_turn)
            return "state_error"
        
        t = 5.0; # [s]
        radius = row_width/2 # [m]
        angle = turn # [rad]
        cmd_vel = Twist()
        cmd_vel.linear.x = radius * abs(angle) / t
        cmd_vel.angular.z = angle / t
        pub_vel.publish(cmd_vel)
        print(np.degrees(angle), radius, rospy.Time.now() - time_start, rospy.Duration.from_sec(t), rospy.Time.now() - time_start > rospy.Duration.from_sec(t))
        if rospy.Time.now() - time_start > rospy.Duration.from_sec(t):                
                return "state_crop_path_pattern"
        else:
            return "state_turn_enter_row"
    else:
        return "state_done"

def state_crop_path_pattern():
    global path_pattern
    # remove executed turn from path pattern
    path_pattern = path_pattern[2::]
    return "state_in_row"

def tf_point_to_center_line(angle, offset, x, y, gamma):
    """
    The Robot has finished a row and the last angle and distance
    to the center line has been angle and offset respectively.
    This function transforms the position of a point lying in 
    a frame with its x-axis at the center line to the robot
    base_link frame.
    inputs:
    param1 angle:   the angle of center line to robot frame x-axis
    param2 offset:   the perpendicular distance between center line and robot frame origin
    param3 x:       the desired distance in x-direction (on center line)
    param4 y:       the desired distance in y-direction (perpedicular to center line)
    param5 gamma:   the desired orientation counter clockwise (from center line)
    outputs:
    x_new:          the x-coordinate in robot frame
    y_new:          the y-coordinate in robot frame
    gamma_new:      the orientation in robot frame
    """
    x_new = x*np.cos(angle) + y*np.sin(angle) - offset*np.sin(angle)
    y_new = -x*np.sin(angle) + y*np.cos(angle) - offset*np.cos(angle)
    gamma_new = gamma - angle
    return (x_new, y_new, gamma_new)

def state_turn_to_next_row():
    global path_pattern
    global row_width
    global angle_valid
    global offset_valid
    lin_row_enter = 1.0     # [m]
    lin_row_exit = 1.0      # [m]
    turn_l = np.pi/2        # [rad]
    turn_r = -np.pi/2       # [rad]

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

        try:
            which_row = int(which_row)
        except ValueError:
            rospy.logerr("Path pattern syntax error: non-integer value for row specification!")
            return "state_error"

        point = tf_point_to_center_line(angle_valid, offset_valid, lin_row_exit, 0.0, 0.0)
        result_straight = movebase_client(point[0], point[1], point[2])
        if result_straight:
            rospy.logwarn("1. Straight goal execution done!")
        else:
            rospy.logerr("1. Straight goal not reached!")
            return "state_error"
        
        result_turn = movebase_client(0, 0, turn)
        if result_turn:
            rospy.logwarn("2. In-place-turn goal execution done!")
        else:
            rospy.logerr("2. In-place-turn goal not reached!")
            return "state_error"

        result_straight = movebase_client(which_row*row_width, 0.0, 0.0)
        if result_straight:
            rospy.logwarn("3. Straight goal execution done!")
        else:
            rospy.logerr("3. Straight goal not reached!")
            return "state_error"

        result_turn = movebase_client(0.0, 0.0, turn)
        if result_turn:
            rospy.logwarn("4. In-place-turn goal execution done!")
        else:
            rospy.logerr("4. In-place-turn goal not reached!")
            return "state_error"

        result_straight = movebase_client(lin_row_enter, 0, 0)
        if result_straight:
            rospy.logwarn("5. Straight goal execution done!")
        else:
            rospy.logerr("5. Straight goal not reached!")
            return "state_error"

        return "state_crop_path_pattern"
    else:
        return "state_done"

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

row_width = 0.75
turn_l = np.pi/2
turn_r = -np.pi/2
state = "state_in_row"
# angle = 0.0
# offset = 0.0
angle_valid = 0.0
offset_valid = 0.0
p_gain_angle_factor = 0.0
p_gain_offset_factor = 0.0
ctrl_by_angle = True
ctrl_by_offset = True
max_lin_vel = 0.0
end_of_row = False
end_row_ctr = 0
path_pattern = ""
time_start = 0.0
xy_coords = np.zeros((10,2))

if __name__ == '__main__':
    rospy.init_node('path_planning', anonymous=True)
    sub_laser = rospy.Subscriber("front/scan", LaserScan, laser_callback)
    pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)

    path_pattern = rospy.get_param('~path_pattern')
    path_pattern = path_pattern.replace("-", "")    # remove hyphens: S-1L-2L-1L-1R-F --> S1L2L1L1RF
    path_pattern = path_pattern[1:-1]               # remove S (Start) and F (Finish): S1L2L1L1RF --> 1L2L1L1R
    # TODO: Check row pattern if correct
    p_gain_angle_factor = rospy.get_param('~p_gain_angle_factor')
    p_gain_offset_factor = rospy.get_param('~p_gain_offset_factor')
    ctrl_by_angle = rospy.get_param('~ctrl_by_angle')
    ctrl_by_offset = rospy.get_param('~ctrl_by_offset')
    max_lin_vel = rospy.get_param('~max_lin_vel')

    # fig = plt.figure()
    while not rospy.is_shutdown() and state != "state_done":
        if state == "state_in_row":
            state = state_in_row(pub_vel)
        elif state == "state_turn_exit_row":
            state = state_turn_exit_row(pub_vel)
        elif state == "state_go_straight":
            state = state_go_straight()
        elif state == "state_turn_enter_row":
            state = state_turn_enter_row(pub_vel)
        elif state == "state_turn_to_next_row":
            state = state_turn_to_next_row()
        elif state == "state_crop_path_pattern":
            state = state_crop_path_pattern()
        elif state == "state_idle":
            state = state_idle()
        elif state == "state_error":
            state = state_error()
        else:
            state = "state_done"
        print(state)

        # plt.plot(xy_coords[:,0], xy_coords[:,1], "ob", label="scatter")
        # plt.draw()
        # plt.pause(0.05)
        # fig.clear()        
        
        rate.sleep()