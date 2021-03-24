#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdlib.h>
// #include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
move_base_msgs::MoveBaseGoal make_goal(double lin_x, double lin_y,double ang_z);
bool reach_goal(MoveBaseClient &ac_client, move_base_msgs::MoveBaseGoal goal);

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  const std::string PATH_PATTERN_NAME = "~path_pattern";
  std::string path_pattern;
  bool path_pattern_ok = ros::param::get(PATH_PATTERN_NAME, path_pattern);
  if(!path_pattern_ok){
    ROS_FATAL_STREAM("Could not get parameter " << PATH_PATTERN_NAME);
    exit(1);
  }

  //remove "-" from path pattern
  for(char c: "-"){
    path_pattern.erase(std::remove(path_pattern.begin(), path_pattern.end(), c), path_pattern.end());
  }
  //remove "S" and "F" from path pattern
  path_pattern.erase(path_pattern.begin());
  path_pattern.pop_back();
  ROS_WARN_STREAM("The path pattern is: " << path_pattern);

  // size_t right_turns = std::count(path_pattern.begin(), path_pattern.end(), 'R');
  // size_t left_turns = std::count(path_pattern.begin(), path_pattern.end(), 'L');
  // size_t turns = right_turns + left_turns;

  const double ROW_GAP = 0.75;
  const double ROW_LEN = 15.0;
  const double HEADLAND = 1.0;
  const double ROW_ENTER = 3.0;
  const double ROW_EXIT = 3.0;
  const double TURN_L = 1.5707;
  const double TURN_R = -1.5707;

  // drive to beginning of first row
  reach_goal(ac, make_goal(2.0, 0.0, 0.0));
  move_base_msgs::MoveBaseGoal goal_straight, goal_turn;

  for(int i=0; i<path_pattern.length(); i+=2){
    int next_row_num = path_pattern[i] - '0';
    char next_turn_dir = path_pattern[i+1];

    // enter row
    goal_straight = make_goal(ROW_ENTER, 0.0, 0.0);
    reach_goal(ac, goal_straight);    
    
    // drive through row
    goal_straight = make_goal(ROW_LEN-ROW_ENTER-ROW_EXIT, 0.0, 0.0);
    reach_goal(ac, goal_straight);

    // exit row
    goal_straight = make_goal(ROW_EXIT, 0.0, 0.0);
    reach_goal(ac, goal_straight);

    // turn according given direction
    if(next_turn_dir == 'L')
      goal_turn = make_goal(HEADLAND, ROW_GAP / 2, TURN_L);
    else
      goal_turn = make_goal(HEADLAND, ROW_GAP / 2, TURN_R);
    reach_goal(ac, goal_turn);

    // drive straight to next row
    goal_straight = make_goal(ROW_GAP*(next_row_num-1), 0.0, 0.0);
    reach_goal(ac, goal_straight);

    // turn again
    reach_goal(ac, goal_turn);
  }
  
  return 0;
}

move_base_msgs::MoveBaseGoal make_goal(double lin_x, double lin_y,double ang_z){
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  // add linear movement to goal
  goal.target_pose.pose.position.x = lin_x;
  goal.target_pose.pose.position.y = lin_y;

  // add angular movement to goal
  tf2::Quaternion q_rot;
  double roll=0, pitch=0, yaw=ang_z;
  q_rot.setRPY(roll, pitch, yaw);
  q_rot.normalize();
  tf2::convert(q_rot, goal.target_pose.pose.orientation);

  return goal;
}

bool reach_goal(MoveBaseClient &ac_client, move_base_msgs::MoveBaseGoal goal){
  ac_client.sendGoal(goal);
  ac_client.waitForResult();
  if(ac_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_WARN("Success: reached goal!");
    return true;
  }
  else{
    ROS_WARN("Error: could not reach goal!");
    return false;
  }
}
