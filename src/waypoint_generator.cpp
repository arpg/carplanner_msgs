#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <carplanner_msgs/OdometryArray.h>
#include <tf/transform_listener.h>

ros::Publisher pub;
ros::Timer loop;
std::string goal_topic, waypoints_topic, map_frame, base_frame;
geometry_msgs::PoseStamped goal;

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

  goal = *msg;

  // carplanner_msgs::OdometryArray odom_arr;

  // {
  //   static tf::TransformListener tflistener;
  //   static tf::StampedTransform Twc;  
  //   try
  //   {
  //       tflistener.waitForTransform(map_frame, base_frame, ros::Time::now(), ros::Duration(0.5));
  //       tflistener.lookupTransform(map_frame, base_frame, ros::Time(0), Twc);
  //   } 
  //   catch (tf::TransformException ex)
  //   {
  //       ROS_ERROR("%s",ex.what());
  //       return;
  //   }

  //   ROS_INFO("Looked up start: %f %f %f %f %f %f %f",
  //     Twc.getOrigin().getX(),
  //     Twc.getOrigin().getY(),
  //     Twc.getOrigin().getZ(), 
  //     Twc.getRotation().getX(),
  //     Twc.getRotation().getY(),
  //     Twc.getRotation().getZ(),
  //     Twc.getRotation().getW()
  //   );

  //   nav_msgs::Odometry odom;
  //   odom.header.frame_id = map_frame;
  //   odom.header.stamp = Twc.stamp_;
  //   odom.child_frame_id = "start";
  //   odom.pose.pose.position.x =    Twc.getOrigin().getX();
  //   odom.pose.pose.position.y =    Twc.getOrigin().getY();
  //   odom.pose.pose.position.z =    Twc.getOrigin().getZ();
  //   odom.pose.pose.orientation.x = Twc.getRotation().getX();
  //   odom.pose.pose.orientation.y = Twc.getRotation().getY();
  //   odom.pose.pose.orientation.z = Twc.getRotation().getZ();
  //   odom.pose.pose.orientation.w = Twc.getRotation().getW();
  //   odom.twist.twist.linear.x =  1;
  //   odom.twist.twist.linear.y =  0;
  //   odom.twist.twist.linear.z =  0;
  //   odom.twist.twist.angular.x = 0;
  //   odom.twist.twist.angular.y = 0;
  //   odom.twist.twist.angular.z = 0;

  //   odom_arr.odoms.push_back(odom);
  // }

  // {
  //   nav_msgs::Odometry odom;
  //   odom.header.frame_id = map_frame;
  //   odom.header.stamp = ros::Time::now();
  //   odom.child_frame_id = "goal";
  //   odom.pose.pose.position.x =    msg->pose.position.x;
  //   odom.pose.pose.position.y =    msg->pose.position.y;
  //   odom.pose.pose.position.z =    msg->pose.position.z;
  //   odom.pose.pose.orientation.x = msg->pose.orientation.x;
  //   odom.pose.pose.orientation.y = msg->pose.orientation.y;
  //   odom.pose.pose.orientation.z = msg->pose.orientation.z;
  //   odom.pose.pose.orientation.w = msg->pose.orientation.w;
  //   odom.twist.twist.linear.x =  1;
  //   odom.twist.twist.linear.y =  0;
  //   odom.twist.twist.linear.z =  0;
  //   odom.twist.twist.angular.x = 0;
  //   odom.twist.twist.angular.y = 0;
  //   odom.twist.twist.angular.z = 0;

  //   odom_arr.odoms.push_back(odom);
  // }

  // ROS_INFO("Publishing...");
  // pub.publish(odom_arr);
  // ros::spinOnce();
}

void loopFunc(const ros::TimerEvent& event)
{
    if (goal.header.seq==0) return;
    carplanner_msgs::OdometryArray odom_arr;

    {
      static tf::TransformListener tflistener;
      static tf::StampedTransform Twc;  
      try
      {
          tflistener.waitForTransform(map_frame, base_frame, ros::Time::now(), ros::Duration(0.5));
          tflistener.lookupTransform(map_frame, base_frame, ros::Time(0), Twc);
      } 
      catch (tf::TransformException ex)
      {
          ROS_ERROR("%s",ex.what());
          return;
      }

      ROS_INFO("Looked up start: %f %f %f %f %f %f %f",
        Twc.getOrigin().getX(),
        Twc.getOrigin().getY(),
        Twc.getOrigin().getZ(), 
        Twc.getRotation().getX(),
        Twc.getRotation().getY(),
        Twc.getRotation().getZ(),
        Twc.getRotation().getW()
      );

      nav_msgs::Odometry odom;
      odom.header.frame_id = map_frame;
      odom.header.stamp = Twc.stamp_;
      odom.child_frame_id = "start";
      odom.pose.pose.position.x =    Twc.getOrigin().getX();
      odom.pose.pose.position.y =    Twc.getOrigin().getY();
      odom.pose.pose.position.z =    Twc.getOrigin().getZ();
      odom.pose.pose.orientation.x = Twc.getRotation().getX();
      odom.pose.pose.orientation.y = Twc.getRotation().getY();
      odom.pose.pose.orientation.z = Twc.getRotation().getZ();
      odom.pose.pose.orientation.w = Twc.getRotation().getW();
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
      odom.pose.pose.position.x =    goal.pose.position.x;
      odom.pose.pose.position.y =    goal.pose.position.y;
      odom.pose.pose.position.z =    goal.pose.position.z;
      odom.pose.pose.orientation.x = goal.pose.orientation.x;
      odom.pose.pose.orientation.y = goal.pose.orientation.y;
      odom.pose.pose.orientation.z = goal.pose.orientation.z;
      odom.pose.pose.orientation.w = goal.pose.orientation.w;
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

    pnh.param("goal_topic", goal_topic, std::string("goal"));
    pnh.param("waypoints_topic", waypoints_topic, std::string("waypoints"));
    pnh.param("map_frame_id", map_frame, std::string("map"));
    pnh.param("base_frame_id", base_frame, std::string("base_link"));

    ROS_INFO("Params:\n goal=%s\n waypoints=%s\n map_frame=%s\n base_link_frame=%s",std::string(goal_topic).c_str(),std::string(waypoints_topic).c_str(),std::string(map_frame).c_str(),std::string(base_frame).c_str());

    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>(goal_topic, 5, cb);
    pub = nh.advertise<carplanner_msgs::OdometryArray>(waypoints_topic, 5);
    loop = nh.createTimer(ros::Duration(0.5), loopFunc);

    ROS_INFO("Initialized.");

    ros::spin();
}
