#include "ros/ros.h"
#include "std_msgs/String.h"


// Publish pointcloud
// opt A: pointcloud2
#include <sensor_msgs/PointCloud2.h>
// --> opt B: pcl pointcloud, s. http://wiki.ros.org/pcl_ros
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


#include <sstream>

//struct Point {
//  float x;
//  float y;
//  float z;
//};

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  // (a) ROS
//  ros::Publisher pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>("test/pointcloud2", 1);
  // (b) PCL-> ROS
  ros::Publisher pcl_pub = nh.advertise<PointCloud>("test/points2", 1);
    
  ros::Rate loop_rate(10); // Hz ?

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    // kickoff: create a pointcloud with random values
    int npts = 100;
    PointCloud::Ptr msg_p (new PointCloud);
    msg_p->header.frame_id = "map"; // "velodyne", ...
    msg_p->height = 1;
    msg_p->width = npts;
    for (int i=0;i<npts;i++) {
      float scale = 1.0f/100000000.f;
      float x = -10.0f + rand()*scale;
      float y = -10.0f + rand()*scale;
      float z = -10.0f + rand()*scale;
      msg_p->points.push_back(pcl::PointXYZ(x, y, z));
      ROS_INFO("%d: %f,%f,%f", i,x,y,z);
    }
  
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    pcl_conversions::toPCL(ros::Time::now(), msg_p->header.stamp);
    pcl_pub.publish(*msg_p);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
