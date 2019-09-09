import numpy as np
import numpy.matlib
import quaternion
import rospy
import hardward_controller
import time
import datetime
from sensor_msgs.msg import JointState
import yaml

t = []
JSposition = []

def as_ROSgoal(Homo_mat):
    '''
    ROS goal: goal = (x, y, z, qx, qy, qz, qw)
    '''
    H = np.copy(Homo_mat)
    rot = H[0:3, 0:3]
    trans = H[0:3, 3]
    q = quaternion.from_rotation_matrix(rot)

    return (trans[0], trans[1], trans[2], q.x, q.y, q.z, q.w)

def as_ROSgoals(Homo_mats):
    '''
    ROS goal: goal = (x, y, z, qx, qy, qz, qw)
    '''
    cmd_amount = len(Homo_mats)
    goals = np.zeros((cmd_amount, 7))
    for i, H in enumerate(Homo_mats):
        rot = H[0:3, 0:3]
        trans = H[0:3, 3]
        q = quaternion.from_rotation_matrix(rot)
        goals[i] = (trans[0], trans[1], trans[2],
                    q.x, q.y, q.z, q.w)
    return goals

def as_MatrixGoal(rosGoal):
    '''
    ROS goal: goal = (x, y, z, qx, qy, qz, qw)
    '''
    H = np.matlib.identity(4)
    H[0:3, 3] = np.array(rosGoal[0:3]).reshape(3,1)
    q = np.quaternion(rosGoal[6], rosGoal[3], rosGoal[4], rosGoal[5])
    H[0:3, 0:3] = quaternion.as_rotation_matrix(q)
    return H

def idealHoleRegister(holeID=9, getCenter=False):
    '''
    - origin was defined as the corner which colsed to smallest hole
    - holeID from 0 to 9 which sorted from small to large
    - center was defined as the probe tip
    '''
    H_hole = np.identity(4)
    if getCenter:
        centerCoordinate = np.array([0.075, -0.045, 0.03, 1.0])
        H_hole[:, 3] = centerCoordinate
        return np.matrix(H_hole)

    holesCoordinate = np.array([
                    [0.025, -0.025, 0.0, 1.0],
                    [0.025*2, -0.025, 0.0, 1.0],
                    [0.025*3, -0.025, 0.0, 1.0],
                    [0.025*4, -0.025, 0.0, 1.0],
                    [0.025*5, -0.025, 0.0, 1.0],
                    [0.025, -0.075, 0.0, 1.0],
                    [0.025*2, -0.075, 0.0, 1.0],
                    [0.025*3, -0.075, 0.0, 1.0], 
                    [0.025*4, -0.075, 0.0, 1.0],
                    [0.025*5, -0.075, 0.0, 1.0]])
    H_hole[:, 3] = holesCoordinate[holeID]
    return np.matrix(H_hole)
    # return np.matrix(holesCoordinate[holeID]).reshape(4,1)

def getIdealPiHCommand(B_star):
    holes_number = 10
    idealPiHCommands = np.zeros((holes_number, 4, 4))
    for holeID in range(holes_number):
        dP = idealHoleRegister(holeID) - idealHoleRegister(getCenter=True)
        idealPiHCommands[holeID] = B_star + dP
    return idealPiHCommands

def move_robot(control_interface, B_init, B_tildes, test_holeID, speed=0.01):
    goals = as_ROSgoals(B_tildes)
    goal_init = as_ROSgoal(B_init)

    for holeID in test_holeID:
        goal = goals[holeID]
        # control_interface.go_to_pose_goal(goal_init, speed)
        # time.sleep(0.1)
        control_interface.go_to_pose_goal(goal, speed)
        time.sleep(0.1)

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

if __name__ == "__main__":
    # PATH SETTING
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)

    Robot = hardward_controller.MoveGroupInteface()
    js_sub = rospy.Subscriber('/vs6242/joint_states', JointState, JScallback)

    rosGoal_B_star = [0.227013667126, 0.204820632069, 0.191043148954, -3.02239056684e-05, -0.999999994061, 6.35861126952e-05, 8.31888538146e-05]
    rosGoal_B_init = [0.276983683655, 0.165763650009, 0.239330956866, 0.0861362693318, 0.981339693465, 0.100947450405, 0.139149421103]

    B_init = as_MatrixGoal(rosGoal_B_init)
    B_star = as_MatrixGoal(rosGoal_B_star)
    B_tildes = getIdealPiHCommand(B_star)
    
    Bc = np.array([[[-1.0000   , 0.0008 ,  -0.0007  ,  0.2020],
    [0.0008   , 1.0000 ,   0.0007  ,  0.1747],
    [0.0007   , 0.0007 ,  -1.0000  ,  0.1610],
    [     0   ,      0 ,        0  ,  1.0000]]])

    test_holeID = [0]
    move_robot(Robot, B_init, Bc, test_holeID, speed=0.01)
    saveJointData(path)
    
    print('joint test is done')
    rospy.spin()
    print("============ Calibration process complete!")

    
