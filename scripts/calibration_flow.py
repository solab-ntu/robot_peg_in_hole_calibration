#!/usr/bin/env python

import numpy as np
from math import pi
import rospy
import yaml
import time
import hardward_controller
import subprocess
import os

def AutoCenter(Robot, Camera, path, SIM, DEBUG):
    ###############################
    ### Auto Center
    ###############################
    BASE = path['ACenter'] if DEBUG else path['ROOT']
    Init_GOAL = BASE + 'goal/init_goal.yaml'
    ACenter_GOAL = BASE + 'goal/ac_goal.yaml'

    # Initial shot
    if SIM:
        goal = (0.26, 0.00, 0.25, -9.94506391949e-06, -0.999999997498, 5.22583013222e-06, 6.98376583036e-05)
    else:
        goal = (0.209998087153, 1.11412077042e-06, 0.368260009103, -9.94506391949e-06, -0.999999997498, 5.22583013222e-06, 6.98376583036e-05)

    with open(Init_GOAL, 'w') as f:
        yaml.dump(list(goal), f, default_flow_style=False)

    Robot.go_to_pose_goal(goal)
    time.sleep(1)
    image = Camera.trigger(BASE + 'img/init.bmp')

    # Compute companation
    cmd = 'python ' + path['ACenter'] + 'autoCenter.py ' + str(SIM) + ' ' + str(DEBUG) 
    subprocess.call(cmd, shell=True)
    with open(ACenter_GOAL) as f:
        goal = yaml.load(f)

    # Move to center position
    Robot.go_to_pose_goal(goal)
    time.sleep(1)
    image = Camera.trigger(BASE + 'img/center.bmp')
    print("============ End Auto center process ============")

def AutoFocus(Robot, Camera, path, SIM, DEBUG):
    ###############################
    ### Step 2: Auto Focus
    ###############################
    BASE = path['AFocus'] if DEBUG else path['ROOT']
    AFocus_GOAL = BASE + 'goal/af_goal.yaml'
    BFocus_GOAL = BASE + 'goal/bf_goal.yaml'

    if not SIM:
        with open(AFocus_GOAL) as f:
            af_goals = yaml.load(f)

        for ind, goal in enumerate(af_goals):
            Robot.go_to_pose_goal(goal)
            time.sleep(1)
            img_name = BASE + 'img/af' + str(ind+1).zfill(2) + '.bmp'
            image = Camera.trigger(img_name)

    cmd = 'python ' + path['AFocus'] + 'autoFocus.py ' + str(SIM) + ' ' + str(DEBUG) 
    subprocess.call(cmd, shell=True)
    with open(BFocus_GOAL) as f:
        goal = yaml.load(f)

    # Move to best focus position
    Robot.go_to_pose_goal(goal)
    time.sleep(1)
    image = Camera.trigger(BASE + 'img/bestFocused.bmp')
    print("============ End Auto focus process ============")

def AutoPose(Robot, Camera, path, SIM, DEBUG):
    ###############################
    ### Step 3: Auto Pose
    # ###############################
    BASE = path['APose'] if DEBUG else path['ROOT']
    AF_BASE = path['AFocus'] if DEBUG else path['ROOT']
    POSE_GOAL = BASE + 'goal/ap_goal.yaml'
    BFocus_GOAL = AF_BASE + 'goal/bf_goal.yaml'
    
    cmd = 'python ' + path['APose'] + 'autoPose.py ' + str(SIM) + ' ' + str(DEBUG) 
    subprocess.call(cmd, shell=True)
    with open(POSE_GOAL) as f:
        pose_goals = yaml.load(f)

    for ind, goal in enumerate(pose_goals):
        Robot.go_to_pose_goal(goal)
        print("============ save as ap{}.bmp".format(ind+1))
        time.sleep(1)
        img_name = BASE + 'img/ap' + str(ind+1).zfill(2) + '.bmp'
        image = Camera.trigger(img_name)
    print("============ End Auto Pose process ============")
    
def CameraPoseEstimation(Robot, Camera, path, SIM, DEBUG):
    ###############################
    ### Step 4: Camera Pose Estimation
    ###############################
    # if SIM:
    #     cmd = 'python ' + path['PoseEstimation'] + 'cameraCalibration.py ' + str(DEBUG)
    # else:
    #     cmd = 'python ' + path['PoseEstimation'] + 'solvePnP.py ' + str(DEBUG)
    cmd = 'python ' + path['PoseEstimation'] + 'cameraCalibration.py ' + str(DEBUG)
    subprocess.call(cmd, shell=True)
    print("============ End Camera Pose Estimation process ============")

def SolveXZ(Robot, Camera, path, DEBUG):
    ###############################``
    ### Step 5: Solve XZ
    ###############################    
    cmd = 'python ' + path['solveXZ'] + 'solveXZ.py ' + str(DEBUG)
    subprocess.call(cmd, shell=True)
    print("============ End SolveXZ process ============")

def HoleSearching(Robot, Camera, path, SIM, DEBUG):
    ###############################
    ### Step 6: Hole searching
    ###############################
    BASE = path['holeSearching'] if DEBUG else path['ROOT']
    AF_BASE = path['AFocus'] if DEBUG else path['ROOT']
    Init_Hole_GOAL = BASE + 'goal/init_hole.yaml'
    HS_GOAL = BASE + 'goal/hs_goal.yaml'
    BFocus_GOAL = AF_BASE + 'goal/bf_goal.yaml'

    with open(BFocus_GOAL) as f:
        bf_goal = yaml.load(f)

    if SIM:
        # goals = np.array([[0.0989688762978, -0.290417812266, 0.409766763058, -0.977195181237, -0.0266658008638, 0.210581453508, 0.00582788734334],[-0.156218261102,-0.281085467775,0.432011039088,-0.493585475791,0.868994363991,-0.0170545137852,0.0305174867876], [-0.0618372168717,-0.349313734125,0.4180423475,-0.638058706016,0.766511282853,0.0496184328111,0.0536614578807]])
        goals = np.array([[-0.0550563022151,-0.364087315648,0.249035930652,-0.648106468421,0.758031072146,0.0488845538433,0.0543801401897],[-0.107294720034,-0.315941764418,0.253341511779,-0.493117443265,0.869257378346,-0.0174373169542,0.030376591576],[-0.064074742374, -0.271848230965, 0.246143270905, 0.801775847565, 0.597549447478, 0.00947959408198, 0.00053420643268]])
        with open(Init_Hole_GOAL, 'w') as f:
            yaml.dump(goals.tolist(), f, default_flow_style=False)
    else:
        # z = bf_goal[2] - 0.003 +0.03
        # goal = (0.11, 0.29, z, 0.0, -1.0, 0.0, 0.0)

        pose_goals = []
        goal = [0.163217244712,0.228229942802,0.367965525241,0.0935280302075,-0.995616646004,2.01592105624e-05,3.68547640211e-05]
        pose_goals.append(goal)
        goal = [0.061473640563, 0.208917543694, 0.354116877558,-0.0926641350729, 0.986460888941, 0.0125872959326, 0.13472131473]
        pose_goals.append(goal)

# ----------------- setting FOV(field of view)
    # with open(Init_Hole_GOAL, 'w') as f:
    #     yaml.dump(list(pose_goals), f, default_flow_style=False)

    # for ind, goal in enumerate(pose_goals):
    #     Robot.go_to_pose_goal(goal)
    #     rospy.sleep(1)
    #     img_name = BASE + 'img/hs' + str(ind+1).zfill(2) + '.bmp'
    #     image = Camera.trigger(img_name)

    # cmd = 'python ' + path['holeSearching'] + 'holeSearching.py ' + str(DEBUG)
    # subprocess.call(cmd, shell=True)

    # cmd = 'python ' + path['holeSearching'] + 'getShootingPose.py ' + str(DEBUG)
    # subprocess.call(cmd, shell=True)
# ----------------- 

    with open(Init_Hole_GOAL) as f:
        goals = yaml.load(f)

    # Move to center position
    for ind, goal in enumerate(goals):
        Robot.go_to_pose_goal(goal)
        rospy.sleep(2)
        img_name = BASE + 'img/hs' + str(ind+1).zfill(2) + '.bmp'
        image = Camera.trigger(img_name)


    cmd = 'python ' + path['holeSearching'] + 'holeSearching.py ' + str(DEBUG)
    subprocess.call(cmd, shell=True)

    # Testing: center to workpiece
    # with open(HS_GOAL) as f:
    #     hs_goal = yaml.load(f)
    # Robot.go_to_pose_goal(hs_goal)
    # rospy.sleep(1)
    # img_name = BASE + 'img/hs_center.bmp'
    # image = Camera.trigger(img_name)

    print("============ End HoleSearching process ============") 

def PegInHole(Robot, Camera, path, SIM, DEBUG):
    ###############################
    ### Step 6: Hole searching
    ###############################
    holeIDs = [5]

    for holeID in holeIDs:
        print('Target hole ID: {}'.format(holeID))
        cmd = 'python ' + path['pegInHole'] + 'pegInHole.py ' + str(DEBUG) + ' ' + str(holeID) 
        subprocess.call(cmd, shell=True)
        print('ready to move')
        BASE = path['pegInHole'] if DEBUG else path['ROOT']
        PIH_GOAL = BASE + 'goal/pih_goal.yaml'
        with open(PIH_GOAL) as f:
            goals = yaml.load(f)

        Robot.go_to_pose_goal(goals[0], 0.5)
        rospy.sleep(1)
        Robot.plan_cartesian_path(goals, 0.01)
        rospy.sleep(2)
        Robot.plan_cartesian_path(goals[-2::-1], 0.05)
   
    print("============ End Peg-in-Hole process ============")

def main(SIM, DEBUG=True): 
    # PATH SETTING
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)

    APose_GOAL = path['ROOT'] + 'goal/ap_goal.yaml'
    CCalibration_GOAL = path['ROOT'] + 'goal/cc_goal.yaml'

    Robot = hardward_controller.MoveGroupInteface()
    Camera = hardward_controller.camera_shooter()
    try:
        # AutoCenter(Robot, Camera, path, SIM, DEBUG)
        # AutoFocus(Robot, Camera, path, SIM, DEBUG)
        # AutoPose(Robot, Camera, path, SIM, DEBUG)
        # CameraPoseEstimation(Robot, Camera, path, SIM, DEBUG)
        # SolveXZ(Robot, Camera, path, DEBUG)
        HoleSearching(Robot, Camera, path, SIM, DEBUG)
        PegInHole(Robot, Camera, path, SIM, DEBUG)
        print("============ Calibration process complete!")
    
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    SIM = False
    # SIM = True
    main(SIM)
  

