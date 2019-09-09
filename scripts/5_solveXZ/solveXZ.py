from math import sin, cos, pi
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
import numpy as np
import numpy.matlib
import quaternion
import yaml



def __get_ABXZ__(A_PATH, B_PATH, X_PATH, Z_PATH):
    with open(A_PATH) as f:
        As = np.array(yaml.load(f))
    with open(B_PATH) as f:
        Bs = np.array(yaml.load(f))
    with open(X_PATH) as f:
        X = np.array(yaml.load(f))
    with open(Z_PATH) as f:
        Z = np.array(yaml.load(f))

    Z_corner = np.array([[ 0.0, 1.0, 0.0, 0.255],
                        [-1.0, 0.0, 0.0, 0.055],
                        [ 0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0]])
    
    return As, Bs, X, Z, Z_corner

def __create_unit_vector__(thetaS, phiS, r=1):
    '''
    Create unit vector base on Spherical coordinate.
    '''
    v = np.zeros(3)
    v[0] = r*sin(thetaS)*cos(phiS)
    v[1] = r*sin(thetaS)*sin(phiS)
    v[2] = r*cos(thetaS)
    return v

def __create_unit_quaternion__(theta, thetaS, phiS):
    '''
    `theta` is the rotation angle around a vector
    `thetaS` and `phiS` is the angle base on Spherical coordinate.
    '''
    t = theta
    v = __create_unit_vector__(thetaS, phiS)
    return np.quaternion(cos(t), v[0]*sin(t), v[1]*sin(t), v[2]*sin(t))

def objRotQuaternion(X, As, Bs):
    HX = np.matlib.identity(4)
    HZ = np.matlib.identity(4)
    qX = __create_unit_quaternion__(*X[0:3])
    qZ = __create_unit_quaternion__(*X[3:6])
    HX[0:3, 0:3] = quaternion.as_rotation_matrix(qX)
    HZ[0:3, 0:3] = quaternion.as_rotation_matrix(qZ)
    HX[0:3, 3] = np.reshape(X[6:9], (3,1))
    HZ[0:3, 3] = np.reshape(X[9:12], (3,1))
    fval = np.zeros((len(Bs), 12))
    for i, (A, B) in enumerate(zip(As, Bs)):
        # error = HBi - HZ*HAi*HX
        error = HZ - B*HX*A
        fval[i, :] = np.array(error[0:3, :]).flatten()
    
    return fval.flatten()

def __solveXZbyQuaternion__(As, Bs):

    # x0 = np.random.rand(12)
    x0 = np.array([-8.98312741e-01, -5.66731629e-03,  4.64156886e+00,  2.73101774e+00,     -9.45400758e-03,  6.80454534e+00, -3.95480793e-02, -2.62769118e-03,     4.00426225e-02,  2.41447274e-01,  5.84820302e-02,  2.08623381e-03])
    lb = (-2*pi, -2*pi, -2*pi, -2*pi, -2*pi, -2*pi, -10, -10, -5, -10, -10, -5)
    ub = (2*pi, 2*pi, 2*pi, 2*pi, 2*pi, 2*pi, 10, 10, 10, 10, 10, 10)
    res = least_squares(objRotQuaternion, x0, args=(As, Bs), method='lm',
                         verbose=2, ftol=1e-15, xtol=1e-15)
    
    # Check
    HX = np.matlib.identity(4)
    HZ = np.matlib.identity(4)
    qX = __create_unit_quaternion__(*res['x'][0:3])
    qZ = __create_unit_quaternion__(*res['x'][3:6])
    HX[0:3, 0:3] = quaternion.as_rotation_matrix(qX)
    HZ[0:3, 0:3] = quaternion.as_rotation_matrix(qZ)
    HX[0:3, 3] = np.reshape(res['x'][6:9], (3,1))
    HZ[0:3, 3] = np.reshape(res['x'][9:12], (3,1))

    return HX, HZ, res['x']

def H(x, y, z, Rx, Ry, Rz):
    RotX = np.matrix([[1, 0, 0, 0], [0, cos(Rx), -sin(Rx), 0], [0, sin(Rx), cos(Rx), 0], [0, 0, 0, 1]])
    RotY = np.matrix([[cos(Ry), 0, sin(Ry), 0], [0, 1, 0, 0], [-sin(Ry), 0, cos(Ry), 0], [0, 0, 0, 1]])
    RotZ = np.matrix([[cos(Rz), -sin(Rz), 0, 0], [sin(Rz), cos(Rz), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    P = np.matrix([[0, 0, 0, x], [0, 0, 0, y], [0, 0, 0, z], [0, 0, 0, 0]])
    return  RotZ*RotY*RotX + P

def objMatrix(X, As, Bs):
    HX = H(X[0], X[1], X[2], X[3], X[4], X[5])
    HZ = H(X[6], X[7], X[8], X[9], X[10], X[11])

    fval = np.zeros(len(Bs))
    for i, (A, B) in enumerate(zip(As, Bs)):
        # error = HBi - HZ*HAi*HX
        residual = HZ - B*HX*A
        fval[i] = np.linalg.norm(residual, ord=2)
    
    return fval

def __solveXZbyMatrix__(As, Bs):

    x0 = np.array([-0.04, 0.0, 0.04, 0, 0, np.radians(-90), 0.255, 0.055, 0.002, 0.0, 0.0, np.radians(-90.0)])
    # x0 = np.random.rand(12)
    r0 = np.linalg.norm(objMatrix(x0, As, Bs))
    print('r0: {}'.format(r0))
    lb = (-2*pi, -2*pi, -2*pi, -2*pi, -2*pi, -2*pi, -10, -10, -5, -10, -10, -5)
    ub = (2*pi, 2*pi, 2*pi, 2*pi, 2*pi, 2*pi, 10, 10, 10, 10, 10, 10)
    res = least_squares(objMatrix, x0, args=(As, Bs), method='lm',
                         verbose=1, ftol=1e-15, xtol=1e-15)
    
    # # Check
    HX = H(*res['x'][0:6])
    HZ = H(*res['x'][6:12])

    return HX, HZ, res['x']

def calResidual(As, Bs, X, Z):
    residual = np.zeros(len(As))
    for i, (A, B) in enumerate(zip(As, Bs)):
        # residual[i] = np.linalg.norm(B - Z*A*X)
        residual[i] = np.linalg.norm(Z - B*X*A)
    
    return residual

def __cal_axis_angle_from_q__(q):
    theta = 2*np.arccos(q.w)
    v = np.array([q.x, q.y, q.z])/sin(theta/2)

    return theta, v

def calPositionAndOrientationError(H, H_exact):
    H1 = np.array(H)
    H2 = np.array(H_exact)
    pH = np.sort(np.abs(H1[0:3, 3] - H2[0:3, 3]))

    pError = np.linalg.norm(pH[0:2])
    q1 = quaternion.from_rotation_matrix(H1[0:3, 0:3])
    q2 = quaternion.from_rotation_matrix(H2[0:3, 0:3])
    q = q1.inverse()*q2
    theta, v = __cal_axis_angle_from_q__(q)
    oError = np.degrees(theta) 
    if oError > 180.0:
        oError = 360.0 - oError 
    return pError, oError


def main_gazebo(DEBUG):
        # PATH SETTING
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)
    
    BASE = path['solveXZ'] if DEBUG else path['ROOT']
    AP_BASE = path['APose'] if DEBUG else path['ROOT']
    PE_BASE = path['PoseEstimation'] if DEBUG else path['ROOT']

    A_PATH = PE_BASE + 'goal/As.yaml'
    B_PATH = AP_BASE + 'goal/Bs.yaml'
    X_PATH = AP_BASE + 'goal/X.yaml' # initial guess
    Z_PATH = AP_BASE + 'goal/Z.yaml' # initial guess, center

    HX_PATH = BASE + 'goal/HX.yaml' # optimal solution
    HZ_PATH = BASE + 'goal/HZ.yaml' # optimal solution, left-up point

    As, Bs, X0, Z0, Z0_corner = __get_ABXZ__(A_PATH, B_PATH, X_PATH, Z_PATH)

    # Quaternion base solver
    HX1, HZ1, res1 =__solveXZbyQuaternion__(As, Bs)
    rf1 = np.linalg.norm(objRotQuaternion(res1, As, Bs))
    XpError, XoError = calPositionAndOrientationError(X0, HX1)
    ZpError, ZoError = calPositionAndOrientationError(Z0_corner, HZ1)
    print(rf1)
    print(XpError, XoError)
    print(ZpError, ZoError)

    # # Rotation matrix base solver
    # HX, HZ, res = __solveXZbyMatrix__(As, Bs)
    # rf = np.linalg.norm(objMatrix(res, As, Bs))
    # pError, oError = calPositionAndOrientationError(X0, HX)
    # print(pError, oError)

    # print('r_final: {}'.format(rf))
    # print('Postion error: {} mm, Orientation error: {} degree'.format(pError*1000, oError))

    with open(HX_PATH, 'w') as f:
        yaml.dump(HX1.tolist(), f, default_flow_style=False)
    with open(HZ_PATH, 'w') as f:
        yaml.dump(HZ1.tolist(), f, default_flow_style=False)

def main_denso(DEBUG):
    # PATH SETTING
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)
    
    BASE = path['solveXZ'] if DEBUG else path['ROOT']
    AP_BASE = path['APose'] if DEBUG else path['ROOT']
    PE_BASE = path['PoseEstimation'] if DEBUG else path['ROOT']

    A_PATH = PE_BASE + 'goal/As.yaml'
    B_PATH = AP_BASE + 'goal/Bs.yaml'
    X_PATH = AP_BASE + 'goal/X.yaml' # initial guess
    Z_PATH = AP_BASE + 'goal/Z.yaml' # initial guess, center

    HX_PATH = BASE + 'goal/HX.yaml' # optimal solution
    HZ_PATH = BASE + 'goal/HZ.yaml' # optimal solution, left-up point

    As, Bs, X0, Z0, _ = __get_ABXZ__(A_PATH, B_PATH, X_PATH, Z_PATH)
    
    # Quaternion base solver
    HX1, HZ1, res1 =__solveXZbyQuaternion__(As, Bs)
    rf1 = np.linalg.norm(objRotQuaternion(res1, As, Bs))
    
    # Rotation matrix base solver
    HX, HZ, res = __solveXZbyMatrix__(As, Bs)
    rf = np.linalg.norm(objMatrix(res, As, Bs))
    print('r_final: {}'.format(rf))


    with open(HX_PATH, 'w') as f:
        yaml.dump(HX.tolist(), f, default_flow_style=False)
    with open(HZ_PATH, 'w') as f:
        yaml.dump(HZ.tolist(), f, default_flow_style=False)


if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 2:
        DEBUG = sys.argv[1]
    else:
        DEBUG = True
    # main_gazebo(DEBUG=DEBUG) 
    main_denso(DEBUG=DEBUG)