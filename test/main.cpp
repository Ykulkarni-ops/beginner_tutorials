/**
 *  @file    main.cpp
 *  @author  Yash Kulkarni

 *
 *  @brief Main function for the tests for talker node
 *
 *  @section DESCRIPTION
 *
 *  This section calls the tests for the talker node.
 *
 */

/**
 * @brief      main
 * 
 * @param[in] argc : Number of parmater inputs
 * @param[in] argv : Value of the input param.
 * 
 * @return     none
 */


#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_talker");
  testing::InitGoogleTest(&argc, argv);
  // Creating NodeHandle
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
