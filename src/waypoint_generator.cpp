#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <carplanner_msgs/OdometryArray.h>
#include <tf2_ros/transform_listener.h>

ros::Publisher pub;
std::string goal_topic, waypoints_topic, map_frame, base_frame;
tf2_ros::Buffer tf_buffer;

void cb(const geometry_msgs::PoseStamped::ConstPtr msg)
{
  ROS_INFO("Got goal: %f %f %f %f %f %f %f",
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z, 
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w
  );

  carplanner_msgs::OdometryArray odom_arr;

  {
    geometry_msgs::TransformStamped tform;
    try
    {
      tform = tf_buffer.lookupTransform(map_frame, base_frame, ros::Time::now());
    }
    catch(tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }

    ROS_INFO("Looked up start: %f %f %f %f %f %f %f",
      tform.transform.translation.x,
      tform.transform.translation.y,
      tform.transform.translation.z, 
      tform.transform.rotation.x,
      tform.transform.rotation.y,
      tform.transform.rotation.z,
      tform.transform.rotation.w
    );

    nav_msgs::Odometry odom;
    odom.header.frame_id = map_frame;
    odom.header.stamp = tform.header.stamp;
    odom.child_frame_id = "start";
    odom.pose.pose.position.x =    tform.transform.translation.x;
    odom.pose.pose.position.y =    tform.transform.translation.y;
    odom.pose.pose.position.z =    tform.transform.translation.z;
    odom.pose.pose.orientation.x = tform.transform.rotation.x;
    odom.pose.pose.orientation.y = tform.transform.rotation.y;
    odom.pose.pose.orientation.z = tform.transform.rotation.z;
    odom.pose.pose.orientation.w = tform.transform.rotation.w;
    odom.twist.twist.linear.x =  1;
    odom.twist.twist.linear.y =  0;
    odom.twist.twist.linear.z =  0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;

    odom_arr.odoms.push_back(odom);
  }

  {
    nav_msgs::Odometry odom;
    odom.header.frame_id = map_frame;
    odom.header.stamp = ros::Time::now();
    odom.child_frame_id = "goal";
    odom.pose.pose.position.x =    msg->pose.position.x;
    odom.pose.pose.position.y =    msg->pose.position.y;
    odom.pose.pose.position.z =    msg->pose.position.z;
    odom.pose.pose.orientation.x = msg->pose.orientation.x;
    odom.pose.pose.orientation.y = msg->pose.orientation.y;
    odom.pose.pose.orientation.z = msg->pose.orientation.z;
    odom.pose.pose.orientation.w = msg->pose.orientation.w;
    odom.twist.twist.linear.x =  1;
    odom.twist.twist.linear.y =  0;
    odom.twist.twist.linear.z =  0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;

    odom_arr.odoms.push_back(odom);
  }

  ROS_INFO("Publishing...");
  pub.publish(odom_arr);
  ros::spinOnce();
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "waypoint_generator");

    ros::NodeHandle nh, pnh("~");
    tf2_ros::TransformListener tf_listener(tf_buffer);

    pnh.param("goal_topic", goal_topic, std::string("goal"));
    pnh.param("waypoints_topic", waypoints_topic, std::string("waypoints"));
    pnh.param("map_frame_id", map_frame, std::string("map"));
    pnh.param("base_frame_id", base_frame, std::string("base_link"));

    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>(goal_topic, 5, cb);
    pub = nh.advertise<carplanner_msgs::OdometryArray>(waypoints_topic, 5);

    ROS_INFO("Initialized.");

    ros::spin();
}
