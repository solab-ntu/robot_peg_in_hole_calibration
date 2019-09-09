
import numpy as np
import numpy.matlib
import quaternion
import cv2
import glob
import yaml
import frame3D
import matplotlib.pyplot as plt

def as_ROSgoal(Homo_mat):
    '''
    ROS goal: goal = (x, y, z, qx, qy, qz, qw)
    '''
    q = quaternion.from_rotation_matrix(Homo_mat[0:3, 0:3])
    return np.array([Homo_mat[0, 3], Homo_mat[1, 3], Homo_mat[2, 3], q.x, q.y, q.z, q.w])

def holeRegister(holeID):
    holesCoodinate = np.array([
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
    
    return np.matrix(holesCoodinate[holeID]).reshape(4,1)

def convertToHole(Worigin, holeID):
    Whole = np.copy(Worigin)
    hole = holeRegister(holeID)
    Whole[:, 3] = np.array(Worigin*hole).flatten()
    return Whole

def main(DEBUG, holeID):
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)

    BASE = path['pegInHole'] if DEBUG else path['ROOT']
    XZ_BASE = path['solveXZ'] if DEBUG else path['ROOT']
    HS_BASE = path['holeSearching'] if DEBUG else path['ROOT']
    TCP_BASE = path['TCPCalibration'] if DEBUG else path['ROOT']

    PIH_goal_PATH = BASE + 'goal/pih_goal.yaml'
    Y_PATH = TCP_BASE + 'goal/Y.yaml'
    X_PATH = XZ_BASE + 'goal/HX.yaml'
    W_PATH = HS_BASE + 'goal/HW.yaml'

    with open(W_PATH) as f:
        W = np.matrix(yaml.load(f))
    with open(Y_PATH) as f:
        Y = np.matrix(yaml.load(f))
    with open(X_PATH) as f:
        X = np.matrix(yaml.load(f))
    
    # holeID from most precision 1 to lowest 9
    Whole = convertToHole(W, holeID=holeID)

    # WDs = [0.03, 0.02, 0.01, 0.0, -0.01, -0.02]
    WDs = [0.03, 0.02, 0.01, 0.0]
    WDs = [0.0]
    wayposes = np.zeros((len(WDs),7))
    for i, WD in enumerate(WDs):
        C = np.matrix([[0.0, 1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, -1.0, WD], [0.0, 0.0, 0.0, 1.0]])
        B = Whole*np.linalg.inv(C)*np.linalg.inv(Y)
        pih_goal = as_ROSgoal(B)
        wayposes[i] = pih_goal

    with open(PIH_goal_PATH, 'w') as f:
        yaml.dump(wayposes.tolist(), f, default_flow_style=False)
    print('pih ok')
    # Data visualization
    # World = frame3D.Frame(np.matlib.identity(4))
    # Base = frame3D.Frame(World.pose)
    # Workpiece = frame3D.Frame(Base.pose)
    # Flange = frame3D.Frame(Base.pose)
    # Tool = frame3D.Frame(Base.pose)
    # Camera = frame3D.Frame(Base.pose)
    # orientation = frame3D.Orientation()
    # ax = frame3D.make_3D_fig(axSize=0.45)
  
    # Flange.transform_by_rotation_mat(B, refFrame=Base.pose)
    # Tool.transform_by_rotation_mat(Y, refFrame=Flange.pose)
    # Camera.transform_by_rotation_mat(X, refFrame=Flange.pose)
    # Workpiece.transform_by_rotation_mat(Whole, refFrame=Base.pose)

    # Base.plot_frame(ax, 'Base')
    # Flange.plot_frame(ax, 'Flange')
    # Tool.plot_frame(ax, 'Tool')
    # Camera.plot_frame(ax, 'Camera')
    # Workpiece.plot_frame(ax, 'Workpiece ')

    # plt.show()

if __name__ == "__main__":
    import sys
    def str2bool(s):
      return s.lower() in ("yes", "true", "t", "1")

    if len(sys.argv) >= 2:
        DEBUG = str2bool(sys.argv[1])
        holeID = int(sys.argv[2])
    else:
        DEBUG = True
        holeID = 5

    main(DEBUG=DEBUG, holeID=holeID)