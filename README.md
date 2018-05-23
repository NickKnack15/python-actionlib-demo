# Python Actionlib Demo

## Description

This is a simple demo for showing how to interact with action lib in Python 2.7. Actionlib is a powerfull asynchronous library built in ROS that will allow you to do multithreaded things without dealing with the threading overhead of Python 2.7. This demo will allow you to interact with two different actions independently of each other. You can start them with publishing to a topic, and cancel them by publishing to a different topic. See usage details below.

## Running the Nodes

**Option 1:** Start all nodes with `roslaunch`.

```
$ roslaunch actionlib_demo dummy.launch --screen
```

_*The `--screen` flag logs output from all nodes to the terminal._

**Option 2:** Start nodes individually.

```
# Terminal 1 - The 'actionlib_demo_client' node.

$ rosrun actionlib_demo start_dummy_action_client.py
```

```
# Terminal 2 - The 'actionlib_demo_server' node.

$ rosrun actionlib_demo start_dummy_action_server.py
```
## Triggering an Action

publish a message to topic "trigger_one" from a teminal to start action 1

```
$ rostopic pub /trigger_one std_msgs/String "data: ''"
```

publish a message to topic "trigger_two" from a terminal to start action 2

```
$ rostopic pub /trigger_two std_msgs/String "data: ''"
```

## Cancelling an Action In Progress

publish a mesage to topic "other_topic_one" from a terminal to cancel action 1

```
$ rostopic pub /other_topic_one std_msgs/String "data: ''"
```

publish a message to topic "other_topic_two" from a terminal to cancel action 2

```
$ rostopic pub /other_topic_two std_msgs/String "data: ''"
```
