/*
  cc -g test.c -o test -ledit -ltermcap
*/

/* This will include all our libedit functions.  If you use C++ don't
forget to use the C++ extern "C" to get it to compile.
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

/* To print out the prompt you need to use a function.  This could be
made to do something special, but I opt to just have a static prompt.
*/

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "we_console");
    ros::NodeHandle nh, private_nh("~");
    std::string cmd_topic_name_ = "cmd";

    std::string prompt_= "> ";
    private_nh.getParam("command_topic_name", cmd_topic_name_);
    private_nh.getParam("prompt_", prompt_);

    std_msgs::String msg;
    ros::Publisher cmd_pub = nh.advertise<std_msgs::String>(cmd_topic_name_, 1, true);

    std::string line;
    while (ros::ok())
    {
        /* count is the number of characters read.
       line is a const char* of our command line with the tailing \n */
       std::cout << prompt_;
        std::cin>>line;

        /* In order to use our history we have to explicitly add commands
    to the history */
        if (line.size() > 1)
        {
            msg.data = line.substr(0, line.size() - 1);
            cmd_pub.publish(msg);
        }
        ros::spinOnce();
    }

    return 0;
}