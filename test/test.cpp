**
 *  @file    test.cpp
 *  @author  Yash Kulkarni

 *  @brief Implementing publisher and subscriber node
 *
 *  @section DESCRIPTION
 *
 *  This program defines the subscriber
 *
 */

#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/change_output_string.h"
/**
 * @brief      testing if the service exist and if its correcty changing the string
 *
 * @param[in]     TESTSuite
 * @param[in]     test_service_correct
 *
 * @return     none
 */
TEST(TESTSuite, testServiceCorrect) {
  // Creating NodeHandle
      ros::NodeHandle nh;

  // Create a srv object for service Change_String
  auto clt =
   nh.serviceClient<beginner_tutorials::change_output_string>("Change_String");

  // creating an object of Change_String
  beginner_tutorials::change_output_string srv;

  // assigning input to srv data
  srv.request.input = "New String";


  // checks if the service call works properly
  bool success = clt.call(srv);
  EXPECT_TRUE(success);

  // compares the input and output string of the service
  EXPECT_STREQ("New String", srv.response.output.c_str());
}
/**
 * @brief      testing if the broadcaster is broadcasting values.
 *
 * @param[in]     TESTSuite
 * @param[in]     testBroadcaster
 *
 * @return     none
 */
 
TEST(TESTSuite, testBroadcaster) {
// Creating NodeHandle

ros::NodeHandle nh;
tf::TransformListener listener;
tf::StampedTransform transform;
listener.waitForTransform("/world", "/talk", ros::Time(0), ros::Duration(10));
listener.lookupTransform("/world", "/talk", ros::Time(0), transform);
auto x = transform.getOrigin().x();
EXPECT_EQ(x, 5);
}
