#!/usr/bin/env python
import readline
import rospy
from std_msgs.msg import String

def make_completer(vocabulary):
    def custom_complete(text, state):
        # None is returned for the end of the completion session.
        results = [x for x in vocabulary if x.startswith(text)] + [None]
        # A space is added to the completion since the Python readline doesn't
        # do this on its own. When a word is fully completed we want to mimic
        # the default readline library behavior of adding a space after it.
        return results[state] + " "
    return custom_complete

def main():
    vocabulary = {}
    readline.parse_and_bind('tab: complete')
    readline.set_completer(make_completer(vocabulary))
    rospy.init_node('cmd_console', anonymous=True)
    pub = rospy.Publisher('cmd', String, queue_size=10)
    prompt = rospy.get_param('~prompt_name', '>')
    prompt += " "

    running = True
    while not rospy.is_shutdown() and running:
        try:
            s = raw_input(prompt).strip()
            pub.publish(s)
        except (EOFError, KeyboardInterrupt) as e:
            running = False
            rospy.signal_shutdown("Shutting down...")
            print('\nShutting down...')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "hello"