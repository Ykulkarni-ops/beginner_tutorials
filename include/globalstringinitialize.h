
/**
 *  @file    globalstringinitilaize.h
 *  @author  Yash Kulkanrni
 *  
 *
 *  @brief Initilaing the Global String text;
 *  which is the default string for the talker node.
 *
 *  @section DESCRIPTION
 *
 *  This is done to addredd the cpplint error of
 *  "Static/global string variables are not permitted."
 *  
 */
#include "std_msgs/String.h"
std::string text = "Before Changing String";
