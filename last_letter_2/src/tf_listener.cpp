#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle node;

  tf2_ros::Buffer tfBuffer(ros::Duration(3.0),false);
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      if (tfBuffer.canTransform("airfoil", "arm",ros::Time(0),ros::Duration(3.0)));
      transformStamped = tfBuffer.lookupTransform("airfoil", "arm",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    printf("tranform.translate.x=%f\n",transformStamped.transform.translation.x);
    rate.sleep();
  }
  return 0;
};