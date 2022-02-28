#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf2odom");

  ros::NodeHandle node;
  ros::NodeHandle pnode("~");

  std::string parent_frame, child_frame, odom_topic;
  pnode.param("parent_frame", parent_frame, std::string("world"));
  pnode.param("child_frame", child_frame, std::string("base_link"));
  pnode.param("odom_topic", odom_topic, std::string("odometry"));
  double rate_;
  pnode.param("rate", rate_, (double)50);
  double cache_len_;
  pnode.param("cache_len", cache_len_, (double)0.1);

  ros::Publisher robot_odom_pub = node.advertise<nav_msgs::Odometry>(odom_topic, 10);

  ROS_INFO("Initialized.");

  tf::TransformListener listener;
  nav_msgs::Odometry odom_msg;
  geometry_msgs::TransformStamped tf_msg;
  geometry_msgs::Quaternion quat_msg;

  geometry_msgs::PointStamped point_msg;

  std::vector<nav_msgs::Odometry> odom_msg_cache;

  odom_msg.header.frame_id = parent_frame;
  odom_msg.child_frame_id = child_frame;
  ros::Rate rate(rate_);
  while (node.ok()){
    rate.sleep();
    ros::spinOnce();
    
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(parent_frame.c_str(), child_frame.c_str(),
                               ros::Time(0), transform);
      odom_msg.header.stamp = transform.stamp_;
      odom_msg.pose.pose.position.x = transform.getOrigin().x();
      odom_msg.pose.pose.position.y = transform.getOrigin().y();
      odom_msg.pose.pose.position.z = transform.getOrigin().z();
      tf::quaternionTFToMsg(transform.getRotation().normalize(), odom_msg.pose.pose.orientation);
    }
    catch (tf::TransformException ex){
      ROS_WARN("%s",ex.what());
      // ros::Duration(0.1).sleep();
      continue;
    }

    if (odom_msg_cache.size()<=0)
    {
      odom_msg_cache.push_back(odom_msg);
      continue;
    }

    double dt = (odom_msg.header.stamp-odom_msg_cache[odom_msg_cache.size()-1].header.stamp).toSec();
    if (dt <= 0.f)
      continue;

    // if (odom_msg_cache.size() > 0)
    {
      odom_msg.twist.twist.linear.x = (odom_msg.pose.pose.position.x - odom_msg_cache[odom_msg_cache.size()-1].pose.pose.position.x)/dt;
      odom_msg.twist.twist.linear.y = (odom_msg.pose.pose.position.y - odom_msg_cache[odom_msg_cache.size()-1].pose.pose.position.y)/dt;
      odom_msg.twist.twist.linear.z = (odom_msg.pose.pose.position.z - odom_msg_cache[odom_msg_cache.size()-1].pose.pose.position.z)/dt;
    }

    odom_msg_cache.push_back(odom_msg);

    geometry_msgs::Vector3 avg_vel;
    for (auto cache_ptr = odom_msg_cache.begin(); cache_ptr != odom_msg_cache.end(); cache_ptr++)
    {
      if ((odom_msg.header.stamp-cache_ptr->header.stamp).toSec() > cache_len_)
      {
        odom_msg_cache.erase(cache_ptr--);
        continue;
      }
      avg_vel.x += cache_ptr->twist.twist.linear.x;
      avg_vel.y += cache_ptr->twist.twist.linear.y;
      avg_vel.z += cache_ptr->twist.twist.linear.z;
    }
    if (odom_msg_cache.size()>0)
    {
      avg_vel.x /= odom_msg_cache.size();
      avg_vel.y /= odom_msg_cache.size();
      avg_vel.z /= odom_msg_cache.size();
    } else {
      ROS_WARN_THROTTLE(5.f, "odom_msg_cache of size 0 detected");
    }
    odom_msg.twist.twist.linear = avg_vel;
    
    robot_odom_pub.publish(odom_msg);


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
