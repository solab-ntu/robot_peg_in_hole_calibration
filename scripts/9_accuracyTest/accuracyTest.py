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
import matplotlib.pyplot as plt

def convert_to_vector(matrix_vector):
    '''
    Convert a matrix_vector, which is a 4*1 homogeneous matrix, to a 3*1 array vector
    '''
    return np.array(matrix_vector).flatten()[0:3]

def get_normal_vector(refFrame, axis='z'):
    origin = refFrame*np.matrix([[0], [0], [0], [1]])
    if axis == 'z':
        normal = refFrame*np.matrix([[0], [0], [1], [1]])
    elif axis == 'x':
        normal = refFrame*np.matrix([[1], [0], [0], [1]])
    elif axis == 'y':
        normal = refFrame*np.matrix([[0], [1], [0], [1]])
    return convert_to_vector(normal - origin)

def get_angle_between_vectors(n0, n1):
    numerator = np.dot(n0, n1)
    denominator = np.linalg.norm(n0) * np.linalg.norm(n1)
    
    if np.isclose(numerator, denominator):
        # print('numerator:{},denominator:{}'.format(numerator, denominator))
        angle = 0
    else:
        angle = np.arccos(numerator/denominator)
    return angle

def __cal_axis_angle_from_q__(q):
    theta = 2*np.arccos(q.w)
    v = np.array([q.x, q.y, q.z])/np.sin(theta/2)

    return theta, v

def calOrientationError(R):
    R1 = np.array(R)
    R2 = np.identity(3)
    q1 = quaternion.from_rotation_matrix(R1)
    q2 = quaternion.from_rotation_matrix(R2)
    q = q1.inverse()*q2
    theta, v = __cal_axis_angle_from_q__(q)
    oError = np.degrees(theta) 
    if oError > 180.0:
        oError = 360.0 - oError 
    return oError

def calPositionError(p1, p0):
    pError = np.linalg.norm(p1 - p0)
    return pError

def ZScoreStandardization(data):
    mu = np.mean(data)*np.ones(len(data))
    sigma = np.std(data)*np.ones(len(data))
    return (data - mu)

def minMaxNormalization(data):
    mi = np.min(data)*np.ones(len(data))
    ma = np.max(data)*np.ones(len(data))
    return (data - mi)/(ma - mi)

def get_data(PLANE_PATH, SLASH_PATH, HANDEYE_CAL, HANDEYE_PLANE):
    with open(PLANE_PATH) as f:
        res_plane = np.array(yaml.load(f))
    with open(SLASH_PATH) as f:
        res_slash = np.array(yaml.load(f))
    with open(HANDEYE_CAL) as f:
        res_hanyeye_cal = np.array(yaml.load(f))
    with open(HANDEYE_PLANE) as f:
        res_handeye_plane = np.array(yaml.load(f))
    
    return res_plane, res_slash, res_hanyeye_cal, res_handeye_plane

def plotTestGraph_test1(plane_data, slash_data):
    # Plane
    tol_data = [0.25, 0.3, 0.35, 0.4, 0.45, 0.5]
    plane = [100, 100, 100, 100, 100, 100]
    slash = [57.1, 100, 100, 100, 100, 100] 
    plt.figure('AccuracyTest_onlineOperation')
    plt.plot(tol_data, plane, '-.or', label='Horizontal pose')
    plt.plot(tol_data, slash, '-.^k', label='Skewed pose')
    plt.ylim([-5, 105])
    plt.xticks([0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55])
    plt.yticks([0, 20, 40, 57.1, 60, 80, 100])
    plt.title('Accuracy test for online operation(35 test for each)')
    plt.xlabel('Allowance(mm)')
    plt.ylabel('Rate of success(%)')
    plt.legend(loc=0)

    # Test condition for Plane
    plt.figure('AccuracyTest_onlineOperation_planePose')
    sorted_pp = np.sort(plane_data[:, 1])
    ordList = np.arange(len(sorted_pp))
    plt.plot(ordList, sorted_pp, 'ro', label='success')
    plt.title('Test condition of 0.25 mm Allowance \n(Horizontal rotation)')
    plt.xlabel('Times of test(#)')
    plt.ylabel('Horizontal rotation angle(deg)')
    plt.legend(loc=0)

    # Test condition for Skew
    slash_poses = []
    slash_poses_plot = []
    for ind, sp in enumerate(slash_data):
        slash_pose = np.copy(sp)
        slash_pose[3, 2] = 0
        slash_pose[3, 3] = 1
        n1 = get_normal_vector(slash_pose)
        n0 = np.array([0, 0, 1])
        ang = get_angle_between_vectors(n1, n0)
        slash_poses.append((bool(sp[3,2]), np.degrees(ang)))
        slash_poses_plot.append(np.degrees(ang))
    plt.figure('AccuracyTest_onlineOperation_skewPose')
    dtype = [('name', bool), ('angle', float)]
    a = np.array(slash_poses, dtype=dtype)
    sorted_sp = np.sort(a, order='angle') 
    ordList = np.arange(len(sorted_sp))
    okList = np.zeros(len(sorted_sp)).astype(bool)
    sorted_spose = np.sort(slash_poses_plot)
    for ind, ele in enumerate(sorted_sp):
        if ele[0]:
            okList[ind] = True
        else:
            okList[ind] = False
    plt.plot(ordList[okList], sorted_spose[okList], 'ro', label='success')
    plt.plot(ordList[~okList], sorted_spose[~okList], 'bx', label='fail')
    plt.title('Test condition of 0.25 mm Allowance \n(Skew rotation)')
    plt.xlabel('Times of test(#)')
    plt.ylabel('Skew angle w.r.t. Z axis(deg)')
    plt.legend(loc=0)
 
def plotTestGraph_test2(hanyeye_cal, handeye_planes):
    # Hand Eye stability test
    cam_RMS = np.array(hanyeye_cal)[:, 0]
    cam_RMS_mean = np.mean(cam_RMS)*np.ones(len(cam_RMS))
    handEye_SysError = np.array(hanyeye_cal)[:, 1]
    handEye_SysError_mean = np.mean(handEye_SysError)*np.ones(len(handEye_SysError))
    ordList = np.arange(1, len(cam_RMS)+1)

    plt.figure('Stability_test_Cam_Calibration')
    plt.plot(ordList, cam_RMS, '-.ob', label='test data')
    plt.plot(ordList, cam_RMS_mean, '-k', label='average value')
    plt.yticks([0.691, 0.692, 0.693, 0.694, 0.695,0.6954, 0.696, 0.697, 0.698])
    plt.legend(loc=0)
    plt.title('Stability test for Camera Calibration/Solve PnP')
    plt.xlabel('Times of test(#)')
    plt.ylabel('Reprojection RMS error(px)')

    plt.figure('Stability_test_HE_Calibration')
    plt.plot(ordList, handEye_SysError, '-.ob', label='test data')
    plt.plot(ordList, handEye_SysError_mean, '-k', label='average value')
    plt.yticks([0.011, 0.012, 0.013, 0.014, 0.015, 0.016, 0.017, 0.018])
    plt.legend(loc=0)
    plt.title('Stability test for Hand/Eye Calibration')
    plt.xlabel('Times of test(#)')
    plt.ylabel('System error(dimensionless)')
    
    # HandEye Plane
    handeye_poses = []
    for ind, hp in enumerate(handeye_planes):
        handeye_plane = np.copy(hp)
        handeye_plane[3, 2] = 0
        handeye_plane[3, 3] = 1
        n1 = get_normal_vector(handeye_plane)
        n0 = np.array([0, 0, 1])
        ang = get_angle_between_vectors(n1, n0)
        p1 = handeye_plane[0:2, 3].flatten()
        p0 = np.array([0.0, 0.0])
        dis = calPositionError(p1, p0)
        handeye_poses.append([dis*1000, np.degrees(ang)])
    # a = minMaxNormalization(np.array(handeye_poses)[:, 0])

    ordList = np.arange(len(handeye_planes))
    plt.figure('Stability_test_workpiece_positoin_std')
    pError = np.array(handeye_poses)[:, 0]
    pError_norm = ZScoreStandardization(pError)
    pError_mean = np.zeros(len(pError))
    plt.plot(ordList, pError_norm,'-.ob', label='Position data')
    plt.plot(ordList, pError_mean,'-k', label='Average')
    plt.legend(loc=0)
    plt.title('Position Error of Workpiece Localization(Standardization)')
    plt.xlabel('Times of test(#)')
    plt.ylabel('distance beteen workpiece\n origin and robot base (mm)')

    plt.figure('Stability_test_workpiece_orientation_std')
    oError = np.array(handeye_poses)[:, 1]
    oError_norm = ZScoreStandardization(oError)
    oError_mean = np.zeros(len(oError))
    plt.plot(ordList, oError_norm,'-.ob', label='Orientation data')
    plt.plot(ordList, oError_mean,'-k', label='Average')
    plt.legend(loc=0)
    plt.title('Orientation Error of Workpiece Localization(Standardization)')
    plt.xlabel('Times of test(#)')
    plt.ylabel('Skew angle w.r.t. Z axis(deg)')
    
    plt.figure('Stability_test_workpiece_positoin')
    pError = np.array(handeye_poses)[:, 0]
    pError_norm = ZScoreStandardization(pError)
    pError_mean = np.mean(pError)*np.ones(len(pError))
    plt.plot(ordList, pError,'-.ob', label='Position data')
    plt.plot(ordList, pError_mean,'-k', label='Average')
    plt.legend(loc=0)
    plt.title('Position Error of Workpiece Localization')
    plt.xlabel('Times of test(#)')
    plt.ylabel('distance beteen workpiece\n origin and robot base (mm)')

    plt.figure('Stability_test_workpiece_orientation')
    oError = np.array(handeye_poses)[:, 1]
    oError_norm = ZScoreStandardization(oError)
    oError_mean = np.mean(oError)*np.ones(len(oError))
    plt.plot(ordList, oError,'-.ob', label='Orientation data')
    plt.plot(ordList, oError_mean,'-k', label='Average')
    plt.legend(loc=0)
    plt.title('Orientation Error of Workpiece Localization')
    plt.xlabel('Times of test(#)')
    plt.ylabel('Skew angle w.r.t. Z axis(deg)')
    
def main(DEBUG):
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)

    BASE = path['accuracyTest'] if DEBUG else path['ROOT']

    RES_HANDEYE_CAL = BASE + 'data/hs_result_handEye_calibration.yaml'
    RES_HANDEYE_PLANE = BASE + 'data/hs_result_handEye_plane.yaml'
    RES_PLANE = BASE + 'data/hs_result_plane.yaml'
    RES_SLASH = BASE + 'data/hs_result_slash.yaml'

    res_plane, res_slash, res_hanyeye_cal, res_handeye_plane = get_data(RES_PLANE, RES_SLASH, RES_HANDEYE_CAL, RES_HANDEYE_PLANE)
    plotTestGraph_test1(res_plane, res_slash)
    plotTestGraph_test2(res_hanyeye_cal, res_handeye_plane)
    plt.show()

if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 2:
        DEBUG = sys.argv[1]
    else:
        DEBUG = True
    main(DEBUG=DEBUG)