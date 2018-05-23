import time
import rospy
import actionlib
from std_msgs.msg import String
from actionlib_demo.msg import DoSomethingResult, DoSomethingGoal, DoSomethingFeedback, DoSomethingActionResult, \
    DoSomethingActionGoal, DoSomethingActionFeedback, DoSomethingAction


class DummyActionServer(object):
    def __init__(self):

        self.do_something_server = actionlib.SimpleActionServer('do_something',
                                                                DoSomethingAction,
                                                                self.do_something,
                                                                auto_start=False)

        self.do_something_else_server = actionlib.SimpleActionServer('do_something_else',
                                                                     DoSomethingAction,
                                                                     self.do_something,
                                                                     auto_start=False)

        self.other_topic_subscriber = rospy.Subscriber('other_topic_one', String, self._other_topic_one)
        self.other_topic_two_subscriber = rospy.Subscriber('other_topic_two', String, self._other_topic_two)

    def __del__(self):
        self.do_something_server = None
        self.do_something_else_server = None

    def start(self):
        self._loginfo('Action Server Started')
        self.do_something_server.start()
        self.do_something_else_server.start()

    def stop(self):
        self._loginfo('Action Server Stopped')
        self.do_something_server = None
        self.do_something_else_server = None

    def do_something(self, goal):
        # type: (DoSomethingGoal) -> None
        self._loginfo('Action Server received do_something action request')
        success = True

        r = rospy.Rate(1)

        start_time = time.time()

        while not rospy.is_shutdown():
            if time.time() - start_time > goal.how_long_to_do_something:
                break

            if self.do_something_server.is_preempt_requested():
                self._loginfo('do_something action preempted')

                self.do_something_server.set_preempted()
                success = False
                break

            self._loginfo('Doing something action')
            r.sleep()

        if success:
            self._loginfo('do_something action succeeded')
            result = DoSomethingResult()
            result.did_finish_doing_something = True
            self.do_something_server.set_succeeded(result)

    def do_something_else(self, goal):
        # type: (DoSomethingGoal) -> None
        
        self._loginfo('Action server received do_something_else')
        success = True

        r = rospy.Rate(1)

        start_time = time.time()

        while not rospy.is_shutdown():
            if time.time() - start_time > goal.how_long_to_do_something:
                break

            if self.do_something_else_server.is_preempt_requested():
                self._loginfo('do_something_else action preempted')

                self.do_something_else_server.set_preempted()
                success = False
                break

            self._loginfo('Doing something_else action')
            r.sleep()

        if success:
            self._loginfo('do_something_else action succeeded')
            result = DoSomethingResult()
            result.did_finish_doing_something = True
            self.do_something_else_server.set_succeeded(result)

    def _other_topic_one(self, data):
        self._loginfo('other_topic_one subscriber callback triggered')
        self._loginfo(data)

    def _other_topic_two(self, data):
        self._loginfo('other_topic_two subscriber callback triggered')
        self._loginfo(data)

    @staticmethod
    def _loginfo(message):
        # type: (str) -> None

        rospy.loginfo('DummyActionServer ({}) {}'.format('dummy_server', message))
