/**
 *Copyright (c) 2021, Yash Kulkarni
 */
/**
 *  @file    talker.cpp
 *  @author  Yash Kulkarni
 *  
 *
 *  @brief Implementing publisher and subscriber node
 *
 *  @section DESCRIPTION
 *
 *  This program defines the publisher
 *
 */

#include <tf/transform_broadcaster.h>
#include <sstream>
#include "ros/ros.h"
#include "beginner_tutorials/change_output_string.h"
#include "std_msgs/String.h"
extern std::string text = "Before Changing String";
/**
 * @brief changestring is a function to change the string being published.
 * 
 * @param req (request) is the paramter that accepts the request argument.
 * @param res (response) is the parameter that accepts the repsonse argument.
 * 
 * @return true if no errors
 */
bool changeString(beginner_tutorials::change_output_string::Request &req,
                  beginner_tutorials::change_output_string::Response &res) {
  res.output = req.input;
  text = res.output;
  ROS_WARN_STREAM("Changing the String due to Service call");
  return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
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
  ros::NodeHandle n;

  int f = 10;

  /* setting the frequency value to the input/default frequency
   * passed by launch file.
   */

  if (argc > 1) {
    f = atoi(argv[1]);
    ROS_DEBUG_STREAM(" Input entered is " << f);
  }
  // Warning if the frequency is less than 0
  if (f < 0) {
    ROS_ERROR_STREAM("Frequency cannot be negative");
    f = 1;
    ROS_WARN_STREAM("Frequency set to a small value of 1 HZ");
  }
  // Showing Fatal message if frequency is 0
  if (f == 0) {
    ROS_FATAL_STREAM("Frequency cannot be 0."
      "Please launch again with valid inputs");
    system("rosnode kill /listener");
    system("rosnode kill /publisher");
    ros::shutdown();
    return 0;
  }


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
  auto chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  auto server = n.advertiseService("Change_String", changeString);

  ros::Rate loop_rate(f);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  // Creating TransformBroadcaster and Transform objects
  tf::TransformBroadcaster br;
  tf::Transform transform;

  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
    std::stringstream ss;
    ss << text << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    /**
     * Setting the origin for the Transform object. 
     * This sets the translation vector of the transform
     */
    transform.setOrigin(tf::Vector3(5.0, 10.0, 15.0));
    // Defining and Setting a value for the Quaternion
    tf::Quaternion q;
    q.setRPY(1.5, 0.8, 2*count);
    transform.setRotation(q);

    // braoadcasting the transform using Transformbroadcaster
    br.sendTransform(tf::StampedTransform(transform,
    ros::Time::now(), "world", "talk"));

    // Updating all the topics
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

