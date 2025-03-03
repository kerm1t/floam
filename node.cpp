// (1) ROS
#include "ros/ros.h"

// Publish pointcloud
// opt A: pointcloud2
/// #include <sensor_msgs/PointCloud2.h>
// --> opt B: pcl pointcloud, s. http://wiki.ros.org/pcl_ros
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


// (2) ECAL
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>
// proto
#include "pcl.pb.h"
// unpacker for Pointcloud2
#include "../include/PointCloudHandler.h"

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

// ROS msg is published in eCAL callback now.
// 2do: do this in the main loop using a Mutex
ros::Publisher pcl_pub;

void pointcloud_callback(const pcl::PointCloud2 msg) {

// (A) subscribe <-- eCAL
  std::vector<double> pts;
  std::vector<std::string> fields = {"x","y","z","Processed Intensity"}; // fuckin HRL constraints
  getPointFields(msg, pts, fields);
  if  (pts.size() == 0) return;

  int npts = msg.width();
  ROS_INFO("received %d pts", npts);

// (B) publish --> ROS
  PointCloud::Ptr msg_p(new PointCloud);
  msg_p->header.frame_id = "map"; // "velodyne", ...
  msg_p->height = 1;
  msg_p->width = npts;
  for (int i=0;i<npts;i++) {
    pcl::PointXYZI pt;
    pt.x = pts[i * fields.size()];
    pt.y = pts[i * fields.size() + 1];
    pt.z = pts[i * fields.size() + 2];
    pt.intensity = pts[i * fields.size() + 3];
    msg_p->points.push_back(pt);
  }
  pcl_conversions::toPCL(ros::Time::now(), msg_p->header.stamp);
  pcl_pub.publish(*msg_p);
}

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
  ros::init(argc, argv, "ecal2ros_pointcloud2");

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
   * will be automatically unadvertised.os::Publisher  of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */

  // (a) ROS
//  ros::Publisher pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>("test/pointcloud2", 1);
  // (b) PCL-> ROS
//  pcl_pub = nh.advertise<PointCloud>("test/points2", 1);
  pcl_pub = nh.advertise<PointCloud>("velodyne_points", 1); // tmp ... to satisfy floam node
    

  // Initialize eCAL and create a protobuf subscriber
  eCAL::Initialize(argc, argv, "ecal.PCL2 --> ros.PCL2");
  eCAL::protobuf::CSubscriber<pcl::PointCloud2> subscriber("meta_pcl");//PRe");
  // Set the Callback
  subscriber.AddReceiveCallback(std::bind(&pointcloud_callback, std::placeholders::_2));
  ROS_INFO("started eCAL subscriber for pointcloud2");


  ros::Rate loop_rate(10); // Hz ?

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok() && eCAL::Ok())
  {

#ifdef RAND_POINTCLOUD
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
#endif

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

///    pcl_conversions::toPCL(ros::Time::now(), msg_p->header.stamp);
///    pcl_pub.publish(*msg_p);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
