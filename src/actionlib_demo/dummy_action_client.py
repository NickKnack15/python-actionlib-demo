import rospy
import actionlib
import traceback
from std_msgs.msg import String
from actionlib_demo.msg import DoSomethingAction, DoSomethingActionFeedback, DoSomethingActionGoal, \
    DoSomethingActionResult, DoSomethingFeedback, DoSomethingGoal, DoSomethingResult


class DummyActionClient(object):
    def __init__(self):

        self.do_something_client = actionlib.SimpleActionClient('do_something', DoSomethingAction)

        self.do_something_else_client = actionlib.SimpleActionClient('do_something_else', DoSomethingAction)

        self.trigger_one_subscriber = rospy.Subscriber('trigger_one', String, self._trigger_one)
        self.trigger_two_subscriber = rospy.Subscriber('trigger_two', String, self._trigger_two)

        self.other_topic_one_subscriber = rospy.Subscriber('other_topic_one', String, self._other_topic_one)
        self.other_topic_two_subscriber = rospy.Subscriber('other_topic_two', String, self._other_topic_two)

    def start(self):
        self._loginfo('Action Client Started')
        self.do_something_client.wait_for_server(rospy.Duration.from_sec(15))
        self.do_something_else_client.wait_for_server(rospy.Duration.from_sec(15))

    def stop(self):
        self._loginfo('Action Client Stopped')
        self.do_something_client = None
        self.do_something_else_client = None

    def send_goal_one(self, length):
        try:
            goal = DoSomethingGoal()
            goal.how_long_to_do_something = length
            self.do_something_client.send_goal(goal,
                                               active_cb=self._goal_one_active,
                                               feedback_cb=self._goal_one_feedback,
                                               done_cb=self._goal_one_done)

            self._loginfo('goal one has been sent')

        except Exception as e:
            print(e.message)
            traceback.print_exc()
            self._loginfo('Error sending goal one')
            self.do_something_client.wait_for_server(rospy.Duration.from_sec(15))
            self.send_goal_one(length)

    def _goal_one_active(self):
        self._loginfo('goal one has transitioned to active state')

    def _goal_one_feedback(self, feedback):
        # type: (DoSomethingFeedback) -> None
        self._loginfo('Goal one feedback received: {}'.format(feedback))

    def _goal_one_done(self, state, result):
        # type: (actionlib.GoalStatus, DoSomethingResult) -> None
        self._loginfo('Goal one done callback triggered')
        self._loginfo(str(state))
        self._loginfo(str(result))
        self._loginfo('Do something result: ' + str(result.did_finish_doing_something))

    def send_goal_two(self, length):
        try:
            goal = DoSomethingGoal()
            goal.how_long_to_do_something = length
            self.do_something_else_client.send_goal(goal,
                                                    active_cb=self._goal_two_active,
                                                    feedback_cb=self._goal_two_feedback,
                                                    done_cb=self._goal_two_done)

        except Exception as e:
            print(e.message)
            traceback.print_exc()
            self._loginfo('Error sending goal two')
            self.do_something_else_client.wait_for_server(rospy.Duration.from_sec(15))
            self.send_goal_one(length)

    def _goal_two_active(self):
        self._loginfo('goal two has transitioned to active state')

    def _goal_two_feedback(self, feedback):
        # type: (DoSomethingFeedback) -> None
        self._loginfo('Goal two feedback received: {}'.format(feedback))

    def _goal_two_done(self, state, result):
        # type: (actionlib.GoalStatus, DoSomethingResult) -> None
        self._loginfo('Goal two done callback triggered')
        self._loginfo(str(state))
        self._loginfo(str(result))
        self._loginfo('Do something_else result: ' + str(result.did_finish_doing_something))

    def _trigger_one(self, data):
        self._loginfo('Trigger one called')
        self._loginfo(data)

        self.send_goal_one(30)

    def _trigger_two(self, data):
        self._loginfo('Trigger two called')
        self._loginfo(data)

        self.send_goal_two(15)

    def _other_topic_one(self, data):
        self._loginfo('other_topic_one called')
        self._loginfo(data)
        self._loginfo('Preempting action do_something goal')
        self.do_something_client.cancel_goal()

    def _other_topic_two(self, data):
        self._loginfo('other_topic_two called')
        self._loginfo(data)
        self._loginfo('Preempting action do_something_else goal')
        self.do_something_else_client.cancel_goal()

    @staticmethod
    def _loginfo(message):
        # type: (str) -> None

        rospy.loginfo('DummyActionClient ({}) {}'.format('dummy_client', message))
