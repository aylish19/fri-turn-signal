/**
 * Node that monitors the robot's current global path and determines if there is an upcoming change in direction. 
 * If there is a turn coming up, the node publishes a message which turns the appropriate turn signal on. If no turn is coming up,
 * the node publishes a message which returns the lights to the default setting. 
 */
 
#include "ros/ros.h"
#include "std_msgs/Char.h"

#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <tf/tf.h>

ros::Publisher signal_pub; 

ros::Subscriber global_path;  
ros::Subscriber robot_pose; 

nav_msgs::Path current_path; 
geometry_msgs::Pose current_pose; 

bool heard_path = false;  
bool heard_pose = false; 

// Updates the current path
void path_cb(const nav_msgs::Path::ConstPtr& msg) {
  current_path = *msg; 
  heard_path = true; 
}

// Updates the current pose
void pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
	geometry_msgs::PoseWithCovarianceStamped new_pose = *msg; 
	current_pose = new_pose.pose.pose; 
	heard_pose = true; 
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "turn_monitor");
  ros::NodeHandle n; 
  
  ros::Rate loop_rate(30);
  
  // Sets up publisher and subscribers
  signal_pub = n.advertise<std_msgs::Char>("/led_strip/turn_signal", 100); 
  global_path = n.subscribe("/move_base/EBandPlannerROS/global_plan", 1, path_cb);
  robot_pose = n.subscribe("/amcl_pose", 1, pose_cb); 
  
  // Waits for current path and pose to update 
  while(!heard_path || !heard_pose) {
	  ros::spinOnce(); 
  }
  
  double stop_yaw = 0; 
  bool turnSignal = false; 
  std_msgs::Char msg;
  msg.data = '0';  
  
  while(ros::ok()) {
	// Updates current path and pose
	ros::spinOnce(); 
	double current_yaw = tf::getYaw(current_pose.orientation); 
	
	// Turns turn signal off if turn is complete  
	if(turnSignal && (abs(current_yaw - stop_yaw) < .1)) {
		turnSignal = false; 
		msg.data = '0'; 
		signal_pub.publish(msg);  
	}
	
	// Iterates through the first half of points in the global path and determines if any major
	// changes in orientation are coming. 
	for(int i = 0; i < current_path.poses.size() / 2; i++) {
	  double yaw = tf::getYaw(current_path.poses[i].pose.orientation); 
	  if(abs(current_yaw - yaw) > 0.5) {
		  turnSignal = true; 
		  // Right turn 
		  if(current_yaw - yaw < 0) {
			  msg.data = '2'; 
			  signal_pub.publish(msg);    
		  }
		  // Left turn 
		  else {
			  msg.data = '1'; 
			  signal_pub.publish(msg); 
		  } 
		  stop_yaw = yaw; 
	  }
	}
	loop_rate.sleep(); 
 }  

  return 0;  
}
