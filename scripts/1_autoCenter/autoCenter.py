import time
from math import cos, sin, pi
import cv2
import numpy as np
import numpy.matlib
import quaternion
import glob
import yaml

def __get_axis_unit_vector__(p0, p1):
    vec = p1 - p0
    return vec/np.linalg.norm(vec)

def __get_pixel_coordinate_of_ARid__(ARcors, ARids, ptId):
    ptIndex = np.where(ARids==ptId)[0][0]
    return ARcors[ptIndex][0, 0]

def __get_angle_between_vectors__(v1, v2):
    uv1 = v1/np.linalg.norm(v1)
    uv2 = v2/np.linalg.norm(v2)
    return np.arctan2(uv2[1],uv2[0]) - np.arctan2(uv1[1],uv1[0])

def __get_angle_between_vectors_2__(v1, v2):
    uv1 = v1/np.linalg.norm(v1)
    uv2 = v2/np.linalg.norm(v2)
    a = np.dot(uv1, uv2)
    b = np.linalg.norm(uv1)*np.linalg.norm(uv2)
    return np.arccos(a/b)

def cal_center_axis_vector(ARcors, ARids):
    ptCen = __get_pixel_coordinate_of_ARid__(ARcors, ARids, 24)
    ptRight = __get_pixel_coordinate_of_ARid__(ARcors, ARids, 25)
    ptUp = __get_pixel_coordinate_of_ARid__(ARcors, ARids, 13)
    x_uvec = __get_axis_unit_vector__(ptCen, ptRight)
    y_uvec = __get_axis_unit_vector__(ptCen, ptUp)
    angle = __get_angle_between_vectors__(x_uvec, np.array([1, 0]))
    return x_uvec, y_uvec, ptCen, -angle

def get_cam_parameter(camBrand, FOV):
    FOV = np.array(FOV) # Field of view (mm)
    if camBrand == 'basler':
        cmos = np.array([5.7, 4.3]) # sensor size (mm)
        reso = np.array([2592, 1944]) # resolution (px)
        f = 12 # focal length (mm)
        (fx, fy) = f*reso/cmos # (px)
        (u0, v0) = reso/2 # (px)
        # intrinsic matrix
        cmtx = np.matrix([[fx, 0, u0],
                          [0, fy, v0],
                          [0, 0, 1]]) 
        M = np.mean(FOV/cmos) # Magnification
        WD = M*f  # Work distance (mm)
        # Magnification unit pixel to mm
        Mpx2mm = M*cmos/reso*np.array([1, 1])

    elif camBrand == 'toshiba':
        raise('Not supported camera') 

    elif camBrand == 'gazebo':
        reso = np.array([2592, 1944]) # resolution (px)
        (u0, v0) = reso/2 # (px)
        # intrinsic matrix
        cmtx = np.matrix([[0, 0, u0],
                          [0, 0, v0],
                          [0, 0, 1]]) 
        # Magnification unit pixel to mm
        Mpx2mm = np.array([110.0/1963.0, 80.0/1427])*np.array([1, 1])
    else:
        raise('Not supported camera')

    return (cmtx, Mpx2mm)

def convert_center_ROSgoal(dq, Init_GOAL):
    '''
    ROS goal: goal = (x, y, z, qx, qy, qz, qw)
    '''
    with open(Init_GOAL) as f:
        init_goal = np.array(yaml.load(f), dtype='float')

    q0 = np.quaternion(init_goal[6], init_goal[3], init_goal[4], init_goal[5])
 
    H = np.identity(4)
    H[0, 3] = init_goal[0] + dq[0]
    H[1, 3] = init_goal[1] + dq[1] 
    H[2, 3] = init_goal[2]
    H[0:2, 0:2] =  np.matrix([[cos(dq[2]), -sin(dq[2])], [sin(dq[2]), cos(dq[2])]])
    q = q0*quaternion.from_rotation_matrix(H[0:3, 0:3])
    Hq = quaternion.as_rotation_matrix(q)
    x = H[0, 3]
    y = H[1, 3]
    z = H[2, 3]
    goal = np.array([x, y, z, q.x, q.y, q.z, q.w])
    return goal

def convert_center_ROSgoal2(dq, Init_GOAL):
    '''
    ROS goal: goal = (x, y, z, qx, qy, qz, qw)
    '''
    with open(Init_GOAL) as f:
        init_goal = np.array(yaml.load(f), dtype='float')

    q0 = np.quaternion(init_goal[6], init_goal[3], init_goal[4], init_goal[5])
    x = np.matrix([[1.0,0.0,-0.035],[0.0,1.0,0.0],[0.0,0.0,1.0]])
    # x = np.matrix([[1.0,0.0,-0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
    y0 = np.matrix([[cos(dq[2]), -sin(dq[2]), dq[0]], [sin(dq[2]), cos(dq[2]), dq[1]], [0.0, 0.0, 1.0]])
    y = x*y0*np.linalg.inv(x)
    H = np.matlib.identity(4)
    H[0:2, 0:2] = y[0:2, 0:2]
    H[0:2, 3] = y[0:2, 2]

    H0 = np.matlib.identity(4)
    H0[0, 3] = init_goal[0]
    H0[1, 3] = init_goal[1]
    H0[2, 3] = init_goal[2]
    H0[0:3, 0:3] =  quaternion.as_rotation_matrix(q0)
    q = q0*quaternion.from_rotation_matrix(H[0:3, 0:3])
    H1 = H0*H
    goal = np.array([H1[0, 3], H1[1, 3], H1[2, 3], q.x, q.y, q.z, q.w])
    return goal

def convert_center_ROSgoal3(dq, Init_GOAL):
    '''
    ROS goal: goal = (x, y, z, qx, qy, qz, qw)
    '''
    with open(Init_GOAL) as f:
        init_goal = np.array(yaml.load(f), dtype='float')

    q0 = np.quaternion(init_goal[6], init_goal[3], init_goal[4], init_goal[5])

    # H0 = H camera
    H0 = np.matlib.identity(4)
    H0[0, 3] = init_goal[0] + 0.040
    H0[1, 3] = init_goal[1]
    H0[2, 3] = init_goal[2]
    H0[0:3, 0:3] =  quaternion.as_rotation_matrix(q0)
    # print(H0[0,3], H0[1,3])
    # Center of Pattern  
    Px = H0[0, 3] + dq[0]
    Py = H0[1, 3] + dq[1]

    # print(Px,Py)

    # Flange
    Fx = Px - 0.040*cos(-dq[2])
    Fy = Py - 0.040*sin(-dq[2])
    # print(Fx,Fy)

    H =  np.array([[cos(dq[2]), -sin(dq[2]), 0.0], [sin(dq[2]), cos(dq[2]), 0.0], [0.0, 0.0, 1.0]])
    q = q0*quaternion.from_rotation_matrix(H)

    goal = np.array([Fx, Fy, init_goal[2], q.x, q.y, q.z, q.w])
    return goal

def compute_compensation(BASE, IMAGE_PATH, Mpx2mm):
    # Generate ChArUco pattern
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    # board = cv2.aruco.CharucoBoard_create(11, 9, 0.010, 0.005, dictionary)
    # img = board.draw((100*11,100*9))
    # cv2.imwrite(BASE + 'img/charuco.png',img)

    # Compute center coordinate in pixel
    frame = cv2.imread(IMAGE_PATH)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ARcors, ARids, _ = cv2.aruco.detectMarkers(gray, dictionary)

    # Draw markers in figure
    img_markers = cv2.aruco.drawDetectedMarkers(frame, ARcors, ARids)
    # cv2.namedWindow('Markers', cv2.WINDOW_NORMAL)
    # cv2.resizeWindow('Markers', 1200, 800)
    # cv2.imshow('Markers', img_markers)
    # cv2.waitKey()

    # Compensate of pixel and mm
    x_uvec, y_uvec, ptCen, angle = cal_center_axis_vector(ARcors, ARids)
    dU = np.array(gray.shape)[::-1]/2 - ptCen    
    dQ = Mpx2mm*dU[::-1]
    # convert unit to meter
    dq = np.array([dQ[0]/1000, dQ[1]/1000, angle]) # dx, dy, angle
    print('[AC RESULT] Image center: {0} px, Angle: {1:.2f} deg, Compensate in XY axis: ({2:.2f}, {3:.2f}) mm'.format(tuple(ptCen), np.degrees(angle), dQ[0], dQ[1]))
    return dq

def main_gazebo(DEBUG):
    # PATH SETTING
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)
        
    BASE = path['ACenter'] if DEBUG else path['ROOT']
    AF_BASE = path['AFocus'] if DEBUG else path['ROOT']
    IMAGE_PATH = BASE + 'img/init.bmp'
    Init_GOAL = BASE + 'goal/init_goal.yaml'
    AC_GOAL = BASE + 'goal/ac_goal.yaml'
    AF_GOAL = AF_BASE + 'goal/af_goal.yaml'
    CAMERA_MAT = BASE + 'goal/camera_mat.yaml'

    # Compute compensation in meter
    FOV = (0.0, 0.0) # does not important
    (cmtx, Mpx2mm) = get_cam_parameter('gazebo', FOV) # Camera parameter
    dq = compute_compensation(BASE, IMAGE_PATH, Mpx2mm)

    # Convert compensation to ROS goal
    ac_goal = convert_center_ROSgoal3(dq, Init_GOAL)
    
    # Set auto focus goal moved along z-axis of center goal
    n = 11 # number of goal
    z = 0.20 # interval of movement in meter
    af_goals = np.ones((n, 7)) * ac_goal
    for ind, dz in enumerate(np.linspace(-z/2, z/2, 11)):
        af_goals[ind, 2] = ac_goal[2] + dz

    with open(AC_GOAL, 'w') as f:
        yaml.dump(ac_goal.tolist(), f, default_flow_style=False)
    
    with open(AF_GOAL, 'w') as f:
        yaml.dump(af_goals.tolist(), f, default_flow_style=False)

    with open(CAMERA_MAT, 'w') as f:
        yaml.dump(cmtx.tolist(), f, default_flow_style=False)

def main_denso(DEBUG):
    # PATH SETTING
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)
        
    BASE = path['ACenter'] if DEBUG else path['ROOT']
    AF_BASE = path['AFocus'] if DEBUG else path['ROOT']
    IMAGE_PATH = BASE + 'img/init.bmp'
    Init_GOAL = BASE + 'goal/init_goal.yaml'
    AC_GOAL = BASE + 'goal/ac_goal.yaml'
    AF_GOAL = AF_BASE + 'goal/af_goal.yaml'
    CAMERA_MAT = BASE + 'goal/camera_mat.yaml'

    # Compute compensation in meter
    FOV = (133.7, 100.0) # (mm)
    (cmtx, Mpx2mm) = get_cam_parameter('basler', FOV) # Camera parameter
    dq = compute_compensation(BASE, IMAGE_PATH, Mpx2mm)

    # Convert compensation to ROS goal
    ac_goal = convert_center_ROSgoal3(dq, Init_GOAL)
    
    # Set auto focus goal moved along z-axis of center goal
    n = 11 # number of goal
    z = 0.06 # interval of movement in meter
    af_goals = np.ones((n, 7)) * ac_goal
    for ind, dz in enumerate(np.linspace(-z/2, z/2, 11)):
        af_goals[ind, 2] = ac_goal[2] + dz

    with open(AC_GOAL, 'w') as f:
        yaml.dump(ac_goal.tolist(), f, default_flow_style=False)
    
    with open(AF_GOAL, 'w') as f:
        yaml.dump(af_goals.tolist(), f, default_flow_style=False)

    with open(CAMERA_MAT, 'w') as f:
        yaml.dump(cmtx.tolist(), f, default_flow_style=False)

if __name__ == '__main__':
    '''
    Input: Initial image
    Output: Compensate value of x, y, Rz direction
    SIM mode: Run in Gazebo enviroment
    DEBUG mode: Result save in the local folder
    '''
    import sys
    def str2bool(s):
      return s.lower() in ("yes", "true", "t", "1")

    if len(sys.argv) >= 2:
        SIM = str2bool(sys.argv[1])
        # DEBUG = str2bool(sys.argv[2])
        DEBUG = True
    else:
        SIM = True
        DEBUG = True
    SIM=False
    main_gazebo(DEBUG=DEBUG) if SIM else main_denso(DEBUG=DEBUG)
