import numpy as np
import yaml
import matplotlib.pyplot as plt
import rospy
import hardward_controller
import time
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryFeedback, FollowJointTrajectoryActionFeedback
import control_msgs.msg
import datetime
import itertools

a = control_msgs.msg.FollowJointTrajectoryActionFeedback
t = []
JSposition = []

t2 = []
desired_position = []
desired_velocity = []
desired_acceralation = []
actual_position = []
error_position = []

ls_t2 = []
ls_actual_position = []
ls_fileSqe = []

def JScallback(data):
    global t, JSposition
    stamp = data.header.stamp
    t.append(stamp.secs + stamp.nsecs*1e-9)
    JSposition.append(data.position)

def saveJointData(path):
    global t, JSposition
    test_time = datetime.datetime.now().strftime("%y%m%d%H%M%S")
    JSPosition_PATH = path['robotCalibration'] + 'test/' + test_time + '_jsposition.yaml'
    TIME_PATH = path['robotCalibration'] + 'test/' + test_time + '_js_time.yaml'
    ret = np.array(JSposition)
    time = np.array([t[i] - t[0] for i in range(len(t))])

    with open(JSPosition_PATH, 'w') as f:
        yaml.dump(ret.tolist(), f, default_flow_style=False)
        
    with open(TIME_PATH, 'w') as f:
        yaml.dump(time.tolist(), f, default_flow_style=False)

    t = []
    JSposition = []
    print('Data grabbed with series number: %s' % (test_time))

def TFcallback(data):
    global t2, desired_position, desired_velocity, desired_acceralation, actual_position, error_position
    stamp = data.feedback.header.stamp
    desired = data.feedback.desired
    actual = data.feedback.actual
    error = data.feedback.error
    t2.append(stamp.secs + stamp.nsecs*1e-9)
    actual_position.append(actual.positions)
 
    # desired_position.append(desired.positions)
    # desired_velocity.append(desired.velocities)
    # desired_acceralation.append(desired.accelerations)
    # error_position.append(error.positions)

def saveTrajectoryData(path):
    global ls_fileSqe ,ls_t2, ls_actual_position, t2, desired_position, desired_velocity, desired_acceralation, actual_position, error_position
    test_time = datetime.datetime.now().strftime("%y%m%d%H%M%S")

    time2 = np.array([t2[i] - t2[0] for i in range(len(t2))])

    ls_t2.append(time2)
    ls_actual_position.append(np.array(actual_position))
    ls_fileSqe.append(test_time)

    t2 = []
    desired_position = []
    desired_velocity = []
    desired_acceralation = []
    actual_position = []
    error_position = []

    print('Data grabbed with series number: %s' % (test_time))

def saveToYAML(path):
    global ls_fileSqe ,ls_t2, ls_actual_position
    
    for i, test_time in enumerate(ls_fileSqe):
        TIME_PATH = path['robotCalibration'] + 'test/' + test_time + '_tf_time.yaml'
        Actual_Position_PATH = path['robotCalibration'] + 'test/' + test_time + '_actual_position.yaml'

        with open(Actual_Position_PATH, 'w') as f:
            res = ls_actual_position[i].tolist()
            yaml.dump(res, f, default_flow_style=False)
        
        with open(TIME_PATH, 'w') as f:
            res =  ls_t2[i].tolist()
            yaml.dump(res, f, default_flow_style=False)

def move_robot(control_interface, test_joint, joint_goals, speed=0.1):
    '''
    @test_joint:  Joint index, value from 1 to 6
    '''
    goal = [0, 0, np.radians(90), 0, np.radians(90), 0]
    for joint_goal in joint_goals:
        goal[test_joint-1] = np.radians(joint_goal)
        control_interface.go_to_joint_state(goal, speed)
        time.sleep(0.1)
 
if __name__ == "__main__":
    # PATH SETTING
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)


    Robot = hardward_controller.MoveGroupInteface()
    js_sub = rospy.Subscriber('/vs6242/joint_states', JointState, JScallback)
    tf_sub = rospy.Subscriber('/vs6242/arm_controller/follow_joint_trajectory/feedback', FollowJointTrajectoryActionFeedback, TFcallback)
    
    repeat_time = 10
    test_joint = 1 # test for Ji
    joint_goals = [-30, 30, -30]
    speed = 0.1

    for _ in itertools.repeat(None, repeat_time):
        move_robot(Robot, test_joint, joint_goals, speed)
        saveJointData(path)
        # saveTrajectoryData(path)
    # saveToYAML(path)
    print('joint test is done')
    rospy.spin()
    print("============ Calibration process complete!")
    
