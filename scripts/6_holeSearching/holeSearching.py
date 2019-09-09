# import sys
# sys.path.append('/usr/local/python/3.5')


import numpy as np
import numpy.matlib
from math import sin, cos
from scipy.optimize import least_squares
import quaternion
import cv2
import glob
import yaml
import frame3D
import matplotlib.pyplot as plt

def as_homogeneous_mat(rvec, tvec):
    # Extrinsic parameter from Camera to Object
    mat = np.matlib.identity(4)
    mat[0:3, 0:3] = cv2.Rodrigues(rvec)[0]
    mat[0:3 ,3] = tvec.reshape(3,1)
    return mat

def as_homogeneous_mats(rvecs, tvecs):
    pose_amount = len(rvecs)
    # Extrinsic parameter from Camera to Pattern
    Hs = np.zeros((pose_amount, 4, 4))
    for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        Hs[i, 0:3, 0:3] = cv2.Rodrigues(rvec)[0]
        Hs[i, 0:3 ,3] = tvec.flatten()
        Hs[i, 3, 3] = 1.0
    
    return Hs

def __cal_axis_angle_from_q__(q):
    theta = 2*np.arccos(q.w)
    v = np.array([q.x, q.y, q.z])/np.sin(theta/2)

    return theta, v

def calPositionAndOrientationError(H, H_exact):
    H1 = np.array(H)
    H2 = np.array(H_exact)
    pError = np.linalg.norm(H1[0:2, 3] - H2[0:2, 3])
    q1 = quaternion.from_rotation_matrix(H1[0:3, 0:3])
    q2 = quaternion.from_rotation_matrix(H2[0:3, 0:3])
    q = q1.inverse()*q2
    theta, v = __cal_axis_angle_from_q__(q)
    oError = np.degrees(theta) 
    if oError > 180.0:
        oError = 360.0 - oError 
    return pError, oError

def convertToOrigin(W, x, y):
    Worigin = np.copy(W)
    po = np.matrix([[x], [y], [0.0], [1.0]])
    Worigin[:, 3] = np.array(W*po).flatten()
    return Worigin

def as_ROSgoal(Homo_mat):
    '''
    ROS goal: goal = (x, y, z, qx, qy, qz, qw)
    '''
    q = quaternion.from_rotation_matrix(Homo_mat[0:3, 0:3])
    return np.array([Homo_mat[0, 3], Homo_mat[1, 3], Homo_mat[2, 3], q.x, q.y, q.z, q.w])

def H(x, y, z, Rx, Ry, Rz):
    RotX = np.matrix([[1, 0, 0, 0], [0, cos(Rx), -sin(Rx), 0], [0, sin(Rx), cos(Rx), 0], [0, 0, 0, 1]])
    RotY = np.matrix([[cos(Ry), 0, sin(Ry), 0], [0, 1, 0, 0], [-sin(Ry), 0, cos(Ry), 0], [0, 0, 0, 1]])
    RotZ = np.matrix([[cos(Rz), -sin(Rz), 0, 0], [sin(Rz), cos(Rz), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    P = np.matrix([[0, 0, 0, x], [0, 0, 0, y], [0, 0, 0, z], [0, 0, 0, 0]])
    return  RotZ*RotY*RotX + P

def objMatrix(X, Bs, HX, Ds):
    HW = H(X[0], X[1], X[2], X[3], X[4], X[5])

    fval = np.zeros((len(Bs), 12))
    for i, (B, D) in enumerate(zip(Bs, Ds)):
        residual = HW - B*HX*D
        fval[i] = np.array(residual[0:3, :]).flatten()
    return fval.flatten()

def solveWbylstsq(Bs, HX, Ds):

    # x0 = np.array([-0.04, 0.0, 0.04, 0, 0, np.radians(-90), 0.255, 0.055, 0.002, 0.0, 0.0, np.radians(-90.0)])
    
    x0 = np.random.rand(6)
    r0 = np.linalg.norm(objMatrix(x0, Bs, HX, Ds))
    print('r0: {}'.format(r0))
    res = least_squares(objMatrix, x0, args=(Bs, HX, Ds), method='lm',
                         verbose=1, ftol=1e-15, xtol=1e-15)
    
    # # Check
    HW = H(*res['x'][0:6])
    rf = np.linalg.norm(objMatrix(res['x'], Bs, HX, Ds))
    print('r_final: {}'.format(rf))

    return rf, HW

def getWbyMatrixProduct(Bs, HX, Ds):
    Ws = np.zeros((len(Bs), 4, 4))
    for i, (B, D) in enumerate(zip(Bs, Ds)):
        Ws[i] = B*HX*D
    return Ws

def main(DEBUG):
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)

    BASE = path['holeSearching'] if DEBUG else path['ROOT']
    PE_BASE = path['PoseEstimation'] if DEBUG else path['ROOT']
    XZ_BASE = path['solveXZ'] if DEBUG else path['ROOT']

    Init_Hole_GOAL = BASE + 'goal/init_hole.yaml'
    IMAGE_PATH = BASE + 'img/hs0*.bmp'
    CAMERA_MAT = PE_BASE + 'goal/camera_mat.yaml'
    HX_PATH = XZ_BASE + 'goal/HX.yaml'
    D_MAT = BASE + 'goal/D.yaml'
    HW_MAT = BASE + 'goal/HW.yaml'
    Bc_PATH = BASE + 'goal/Bc.yaml'
    HS_GOAL_PATH = BASE + 'goal/hs_goal.yaml'
    HS_RESULT = BASE + 'goal/hs_result.yaml'
    
    squareLength = 0.01   # Here, our measurement unit is m.
    markerLength = 0.25/40.0   # Here, our measurement unit is m.
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250 )
    arucoParams = cv2.aruco.DetectorParameters_create()
    with open(CAMERA_MAT) as f:
        cameraMatrix = np.array(yaml.load(f))
        distCoeffs = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]]) # Assume to zero

    rvecs = []
    tvecs = []

    images = sorted(glob.glob(IMAGE_PATH))
    for ind, fname in enumerate(images):
        frame = cv2.imread(fname)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=arucoParams)

        if len(corners)>0:
            diamondCorners, diamondIds = cv2.aruco.detectCharucoDiamond(gray, corners, ids, squareLength/markerLength)

            if len(diamondCorners) >= 1: 
                img_with_diamond = cv2.aruco.drawDetectedDiamonds(frame, diamondCorners, diamondIds, (0,255,0))
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(diamondCorners, squareLength, cameraMatrix, distCoeffs)
                img_with_diamond = cv2.aruco.drawAxis(img_with_diamond, cameraMatrix, distCoeffs, rvec, tvec, 1)    # axis length 100 can be changed according to your requirement
                
                rvecs.append(rvec)
                tvecs.append(tvec)

        else:
            img_with_diamond = gray

    #     cv2.namedWindow('diamond', cv2.WINDOW_NORMAL)
    #     cv2.resizeWindow('diamond', 1200, 800)
    #     cv2.imshow("diamond", img_with_diamond)   # display
    #     cv2.waitKey(30)
    # cv2.destroyAllWindows()

    Ds = as_homogeneous_mats(rvecs, tvecs) # from Camera to Object
    with open(D_MAT, 'w') as f:
        yaml.dump(Ds.tolist(), f, default_flow_style=False)

    #################
    # Calculate workpiece 
    #################
    with open(HX_PATH) as f:
        X = np.matrix(yaml.load(f))

    with open(Init_Hole_GOAL) as f:
        ihs = np.array(yaml.load(f)) # ROS goal: (x, y, z, qx, qy,qz, qw)
        Bs = np.zeros((len(ihs), 4, 4))
        for i, ih in enumerate(ihs):
            q = np.quaternion(ih[6], ih[3], ih[4], ih[5])
            
            B = np.identity(4)
            B[0:3, 0:3] = quaternion.as_rotation_matrix(q)
            B[0:3, 3] = ih[0:3]
            Bs[i] = B
    Ws = getWbyMatrixProduct(Bs, X, Ds)
    rf, W = solveWbylstsq(Bs, X, Ds)


    (x, y) = (-0.015, 0.015)
    # (x, y) = (0, 0)
    # (x, y) = (-0.0149+0.0725, 0.0153-0.050)
    Worigin = convertToOrigin(W, x, y)
 
    with open(HW_MAT, 'w') as f:
        yaml.dump(Worigin.tolist(), f, default_flow_style=False)
    
    World = frame3D.Frame(np.matlib.identity(4))
    Base = frame3D.Frame(World.pose)
    Flange = frame3D.Frame(Base.pose)
    Camera = frame3D.Frame(Base.pose)
    Workpiece = frame3D.Frame(Base.pose)
    orientation = frame3D.Orientation()
    ax = frame3D.make_3D_fig(axSize=0.45)
    
    # {Base as World}
    poseID = 0
    Base.plot_frame(ax, 'Base')
    Flange.transform_by_rotation_mat(Bs[poseID], refFrame=Base.pose)
    Camera.transform_by_rotation_mat(X, refFrame=Flange.pose)
    Workpiece.transform_by_rotation_mat(Ds[poseID], refFrame=Camera.pose)
    Base.plot_frame(ax, 'Base')
    Camera.plot_frame(ax, 'Camera') 
    Flange.plot_frame(ax, 'Flange') 
    Workpiece.plot_frame(ax, 'Workpiece')

    SIM = False
    if SIM:
        W_ideal = np.array([[-1.0, 0.0, 0.0, -0.06],
                            [0.0, -1.0, 0.0, -0.30],
                            [0.0, 0.0, 1.0, 0.01],
                            [0.0, 0.0, 0.0, 1.0]])
        pError, oError = calPositionAndOrientationError(W, W_ideal)
        print(pError*1000, oError)
        for w in Ws:
            pError, oError = calPositionAndOrientationError(w, W_ideal)
            print(pError*1000, oError)
    else:
        W_assumed = np.identity(4)
        _, oError = calPositionAndOrientationError(W, W_assumed)
        print(oError)
        for w in Ws:
            _, oError = calPositionAndOrientationError(w, W_assumed)
            print(oError)
    hs_res = np.array([np.copy(W)])
    hs_res[0,3,3] = rf

    with open(HS_RESULT, 'a') as f:
        yaml.safe_dump(hs_res.tolist(), f, default_flow_style=False)
        # yaml.safe_dump()

    # Centering command validation
    WD = 0.32692938211698336
    cmd_Dc = orientation.asRad((0.0, 0.0, WD, 0.0, 180, 0.0)).cmd()
    Dc = Workpiece.transform(cmd_Dc, refFrame=World.pose)
    Bc = Worigin*np.linalg.inv(Dc)*np.linalg.inv(X)
    
    goal = as_ROSgoal(Bc)
    with open(HS_GOAL_PATH, 'w') as f:
        yaml.dump(goal.tolist(), f, default_flow_style=False)

    # plt.show()

if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 2:
        DEBUG = sys.argv[1]
    else:
        DEBUG = True
    main(DEBUG=DEBUG)