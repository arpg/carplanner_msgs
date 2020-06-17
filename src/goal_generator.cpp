#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>

ros::Publisher pub;
std::string goal_topic, path_topic;
double path_len;

void cb(const nav_msgs::Path::ConstPtr msg)
{
    if (msg->poses.size()<2)
    {
      ROS_WARN("Got invalid path");
      return;
    }

    ROS_INFO("Got path");

    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.seq = -1;
    double sum=0;
    Eigen::Vector3d delta;
    for (uint i=0; i<msg->poses.size()-1; i++)
    {
        delta = Eigen::Vector3d(msg->poses[i].pose.position.x - msg->poses[i+1].pose.position.x,
                                msg->poses[i].pose.position.y - msg->poses[i+1].pose.position.y,
                                msg->poses[i].pose.position.z - msg->poses[i+1].pose.position.z );

        if (sum + delta.norm() > path_len)
        {
          double delta_fraction = (path_len - sum)/delta.norm();
          delta *= delta_fraction;

          goal_msg.pose.position.x = msg->poses[i].pose.position.x + delta[0];
          goal_msg.pose.position.y = msg->poses[i].pose.position.y + delta[1];
          goal_msg.pose.position.z = msg->poses[i].pose.position.z + delta[2];
          goal_msg.pose.orientation.x = msg->poses[i+1].pose.orientation.x;
          goal_msg.pose.orientation.y = msg->poses[i+1].pose.orientation.y;
          goal_msg.pose.orientation.z = msg->poses[i+1].pose.orientation.z;
          goal_msg.pose.orientation.w = msg->poses[i+1].pose.orientation.w;
        }
        else
        {
          sum += delta.norm();
        }
        
    }

    if (goal_msg.header.seq == -1)
    {
      goal_msg = *(msg->poses.end());
    }

    ROS_INFO("Publishing...");
    pub.publish(goal_msg);
    ros::spinOnce();
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "goal_generator");

    ros::NodeHandle nh, pnh("~");

    pnh.param("goal_topic", goal_topic, std::string("goal"));
    pnh.param("path_topic", path_topic, std::string("path"));
    pnh.param("path_intercept_length", path_len, 2.0);
    ros::Subscriber sub = nh.subscribe<nav_msgs::Path>(path_topic, 5, cb);
    pub = nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 5);

    ROS_INFO("Initialized.");

    ros::spin();
}
