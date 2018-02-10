#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <sstream>
#include "dynamics_vehicle.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_dynamics");
  ros::NodeHandle n;


   ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

   std_msgs::String msg;
   std::stringstream ss;

   // %Tag(LOOP_RATE)%
     ros::Rate loop_rate(10);
   // %EndTag(LOOP_RATE)%

     /**
      * A count of how many messages we have sent. This is used to create
      * a unique string for each message.
      */
   // %Tag(ROS_OK)%
     int count = 0;
     while (ros::ok())
     {
   // %EndTag(ROS_OK)%
       /**
        * This is a message object. You stuff it with data, and then publish it.
        */
   // %Tag(FILL_MESSAGE)%
       std_msgs::String msg;

       std::stringstream ss;
       ss << "hello world " << count;
       msg.data = ss.str();

       ROS_INFO("%s", msg.data.c_str());


       /**
        * The publish() function is how you send messages. The parameter
        * is the message object. The type of this object must agree with the type
        * given as a template parameter to the advertise<>() call, as was done
        * in the constructor above.
        */
   // %Tag(PUBLISH)%
       chatter_pub.publish(msg);
   // %EndTag(PUBLISH)%

   // %Tag(SPINONCE)%
       ros::spinOnce();
   // %EndTag(SPINONCE)%

   // %Tag(RATE_SLEEP)%
       loop_rate.sleep();
   // %EndTag(RATE_SLEEP)%
       ++count;
     }

     dynamics test;


//  while (ros::ok())
//  {
//    ros::spinOnce();
//    loop_rate.sleep();
//
//  }


  return 0;
}







