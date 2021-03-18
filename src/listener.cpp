#ifndef POINTCLOUDFILE_H
#define POINTCLOUDFILE_H

#include "beginner_tutorials/Config.h"
#include "beginner_tutorials/LocalDescriptorExtractor.h"
#include "beginner_tutorials/PlaceRecognizer.h"
#include <iomanip>
#include <unistd.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

const std::string configDir = "/home/gys/catkin_ws/src/beginner_tutorials/config/";

Eigen::Affine3f sensorPose;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg, const nav_msgs::OdometryConstPtr& odometryMsg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
  pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

  double x = odometryMsg->pose.pose.position.x;
  double y = odometryMsg->pose.pose.position.y;
  double z = odometryMsg->pose.pose.position.z;

  enc::PlaceRecognizer::addPointCloud(laserCloudIn, std::vector<double>({x, y, z}));
  int placeId;
  double featureDist;
  enc::PlaceRecognizer::recognizePlace(placeId, featureDist);

  std::cout << "place id: " << placeId << " " << "feature dist: " << featureDist << std::endl;
  
  ROS_INFO("I heard: [%f, %f, %f]", x, y ,z);
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  enc::Config::initialize(configDir + "default.yaml");

  enc::PlaceRecognizer::initialize(enc::ENC_ISE, "./FPFH+ISE.txt");

  // char buff[FILENAME_MAX];
  // getcwd( buff, FILENAME_MAX );
  // std::string current_working_dir(buff);
  // std::cout << current_working_dir << std::endl;

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  message_filters::Subscriber<sensor_msgs::PointCloud2> pcSub (n, "/velodyne_points", 2000);

  message_filters::Subscriber<nav_msgs::Odometry> odoSub (n, "/aft_mapped_to_init", 2000);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicies;

  message_filters::Synchronizer<SyncPolicies> sync(SyncPolicies(3000), pcSub, odoSub);

  // message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync(pcSub, odoSub, 1000);

  sync.registerCallback(boost::bind(&chatterCallback, _1, _2));

  // ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1000, chatterCallback);

  ros::spin();

  return 0;
}

#endif