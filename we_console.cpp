/*
  cc -g test.c -o test -ledit -ltermcap
*/

/* This will include all our libedit functions.  If you use C++ don't
forget to use the C++ extern "C" to get it to compile.
*/
#include <histedit.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>


/* To print out the prompt you need to use a function.  This could be
made to do something special, but I opt to just have a static prompt.
*/

std::string prompt_name_ = ">";
char * prompt(EditLine *e) {
    return strdup(prompt_name_.c_str());
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "we_console");
  ros::NodeHandle nh, private_nh("~");

  private_nh.getParam("prompt_name", prompt_name_);
  prompt_name_+=" ";

  std_msgs::String msg;
  ros::Publisher cmd_pub = nh.advertise<std_msgs::String>("cmd", 1, true);

  /* This holds all the state for our line editor */
  EditLine *el;

  /* This holds the info for our history */
  History *myhistory;

  /* Temp variables */
  int count;
  const char *line;
  int keepreading = 1;
  HistEvent ev;

  /* Initialize the EditLine state to use our prompt function and
  emacs style editing. */
    
  el = el_init(argv[0], stdin, stdout, stderr);
  el_set(el, EL_PROMPT, &prompt);
  el_set(el, EL_EDITOR, "emacs");

  /* Initialize the history */
  myhistory = history_init();
  if (myhistory == 0) {
    fprintf(stderr, "history could not be initialized\n");
    return 1;
  }

  /* Set the size of the history */
  history(myhistory, &ev, H_SETSIZE, 10);

  /* This sets up the call back functions for history functionality */
  el_set(el, EL_HIST, history, myhistory);


  while (ros::ok()) {
    /* count is the number of characters read.
       line is a const char* of our command line with the tailing \n */
    line = el_gets(el, &count);
    
    /* In order to use our history we have to explicitly add commands
    to the history */
    if (count > 1) {
      history(myhistory, &ev, H_ENTER, line);
      std::string l(line);
      msg.data = l.substr(0,l.size()-1);;
      cmd_pub.publish(msg);
    }
  }
  
  /* Clean up our memory */
  history_end(myhistory);
  el_end(el);
  
  return 0;
}