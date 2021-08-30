/////////////////////////////////////////////////////////////////////////////////
// This is a simple cpp script to publish the keyboard key that is pressed on the
// topic /key
//
// Date created: 31.05.2019
// Author: Jayadev Madyal
/////////////////////////////////////////////////////////////////////////////////

#include <termios.h>
// #include <iostream>     // belongs to blocking input mode

#include <ros/ros.h>
#include "std_msgs/Int32.h"

/* Function to read terminal inputs non-blocking; i.e., without ENTER being pressed*/
int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new setting
  int ch = getchar();                        // read character (non-blocking
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return ch;
}

// using namespace std;     // belongs to blocking input mode

int main(int argc, char **argv)
{

    ros::init(argc, argv, "keyboard");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::Int32>("/key", 1000);
    ros::Rate loop_rate(10);

    ROS_INFO("keyboard_pub: Node started.");

    while (ros::ok())
    {
        // char input;
        // cin>>input; // blocking input; reads from the terminal only after ENTER is pressed
        // c.data = input;

        std_msgs::Int32 c;
        c.data = getch();  // call your non-blocking input function
        pub.publish(c);
        ROS_INFO_ONCE("keyboard_pub. First time that a key was pressed: %d", c.data);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
