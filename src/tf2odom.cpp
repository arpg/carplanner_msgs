#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_odom_publisher");

  ros::NodeHandle node;
  ros::NodeHandle pnode("~");

  std::string parent_frame, child_frame, odom_topic;
  pnode.param("parent_frame", parent_frame, std::string("world"));
  pnode.param("child_frame", child_frame, std::string("base_link"));
  pnode.param("odom_topic", odom_topic, std::string("odometry"));
  double rate_;
<<<<<<< HEAD
  pnode.param("rate", rate_, (double)20);
=======
  pnode.param("rate", rate_, (double)50);
>>>>>>> fccd723d5095c8435f92f41ead520a1f768c4614

  ros::Publisher robot_odom_pub =
    node.advertise<nav_msgs::Odometry>(odom_topic, 10);

  tf::TransformListener listener;
  nav_msgs::Odometry odom_msg;
  geometry_msgs::TransformStamped tf_msg;
  geometry_msgs::Quaternion quat_msg;

  geometry_msgs::PointStamped point_msg;

  odom_msg.header.frame_id = parent_frame;
  ros::Rate rate(rate_);
  while (node.ok()){
    rate.sleep();
    ros::spinOnce();
    
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(parent_frame.c_str(), child_frame.c_str(),
                               ros::Time(0), transform);
      odom_msg.header.stamp = ros::Time::now();
      odom_msg.pose.pose.position.x = transform.getOrigin().x();
      odom_msg.pose.pose.position.y = transform.getOrigin().y();
      odom_msg.pose.pose.position.z = transform.getOrigin().z();
      tf::quaternionTFToMsg(transform.getRotation().normalize(), odom_msg.pose.pose.orientation);
      robot_odom_pub.publish(odom_msg);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      // ros::Duration(0.1).sleep();
    }
/*
    point_msg.header.stamp = ros::Time::now();
    point_msg.point.x = odom_msg.pose.pose.position.x;
    point_msg.point.y = odom_msg.pose.pose.position.y;
    point_msg.point.z = odom_msg.pose.pose.position.z;

    robot_point_pub.publish(point_msg);
*/
  }
  return 0;
};
