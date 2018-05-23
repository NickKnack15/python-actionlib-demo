#!/usr/bin/env python
import rospy
from actionlib_demo import DummyActionServer


def start_actionlib_demo():

    rospy.init_node('dummy_action_server', anonymous=False)  # initialize ros node

    dummy_action_server = DummyActionServer()
    dummy_action_server.start()

    def stop_dummy_action_server():
        dummy_action_server.stop()

    rospy.on_shutdown(stop_dummy_action_server)

    rospy.spin()  # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    start_actionlib_demo()
