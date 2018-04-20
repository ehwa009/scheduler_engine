#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('test_speech_input', anonymous=False)
    pub_msg = rospy.Publisher('/speech_text_input', String, queue_size=10)

    while True:
        input_msg = raw_input('speech_input: ')        
        if 'quit' in input_msg:
            quit()

        pub_msg.publish(input_msg)

if __name__ == '__main__':
    main()
