import numpy as np
import numpy.matlib
import quaternion
import cv2
import glob
import yaml
import frame3D
import matplotlib.pyplot as plt

def TCPCalibration():
    Y = np.identity(4)
    Y[2, 3] = 0.13 # 
    return Y

def main(DEBUG):
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)

    BASE = path['TCPCalibration'] if DEBUG else path['ROOT']
    HS_BASE = path['holeSearching'] if DEBUG else path['ROOT']
    XZ_BASE = path['solveXZ'] if DEBUG else path['ROOT']
    Y_PATH = BASE + 'goal/Y.yaml'


    Y = TCPCalibration()
    with open(Y_PATH, 'w') as f:
        yaml.dump(Y.tolist(), f, default_flow_style=False)
    
    # Data visualization
    X_PATH = XZ_BASE + 'goal/HX.yaml'
    Bc_PATH = HS_BASE + 'goal/Bc.yaml'
    WHole_PATH = HS_BASE + 'goal/Whole.yaml'
    with open(Bc_PATH) as f:
        Bc = np.array(yaml.load(f))
    with open(WHole_PATH) as f:
        Wh = np.array(yaml.load(f))
    with open(X_PATH) as f:
        X = np.array(yaml.load(f))

    World = frame3D.Frame(np.matlib.identity(4))
    Base = frame3D.Frame(World.pose)
    Workpiece = frame3D.Frame(Base.pose)
    Flange = frame3D.Frame(Base.pose)
    Tool = frame3D.Frame(Base.pose)
    Camera = frame3D.Frame(Base.pose)
    orientation = frame3D.Orientation()
    ax = frame3D.make_3D_fig(axSize=0.45)
  
    Flange.transform_by_rotation_mat(Bc, refFrame=Base.pose)
    Tool.transform_by_rotation_mat(Y, refFrame=Flange.pose)
    Camera.transform_by_rotation_mat(X, refFrame=Flange.pose)
    Workpiece.transform_by_rotation_mat(Wh, refFrame=Base.pose)

    Base.plot_frame(ax, 'Base')
    Flange.plot_frame(ax, 'Flange')
    Tool.plot_frame(ax, 'Tool')
    Camera.plot_frame(ax, 'Camera')
    Workpiece.plot_frame(ax, 'Workpiece ')

    plt.show()
    
    


if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 2:
        DEBUG = sys.argv[1]
    else:
        DEBUG = True
    main(DEBUG=DEBUG)