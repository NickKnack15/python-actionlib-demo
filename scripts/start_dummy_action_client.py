#!/usr/bin/env python
import rospy
from actionlib_demo import DummyActionClient


def start_actionlib_demo():

    rospy.init_node('dummy_action_client', anonymous=False)  # initialize ros node

    dummy_action_client = DummyActionClient()
    dummy_action_client.start()

    def stop_dummy_action_client():
        dummy_action_client.stop()

    rospy.on_shutdown(stop_dummy_action_client)

    rospy.spin()  # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    start_actionlib_demo()
