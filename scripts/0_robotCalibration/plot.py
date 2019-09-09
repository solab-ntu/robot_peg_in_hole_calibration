import numpy as np
import yaml
import matplotlib.pyplot as plt
import scipy.signal
import glob
def plotJointState(path, test_joint):
    JSPosition_PATH = path['robotCalibration'] + 'test/test_with_700g_payload/jsposition.yaml'
    TIME_PATH = path['robotCalibration'] + 'test/test_with_700g_payload/js_time.yaml'
    
    with open(JSPosition_PATH) as f:
        jip = np.array(yaml.load(f))[test_joint-1]

    with open(TIME_PATH) as f:
        time = yaml.load(f)

    n = 30  # the larger n is, the smoother curve will be
    b = [1.0 / n] * n
    a = 1

    J3pRad = np.radians(jip)
    J3v = np.gradient(J3pRad, time)
    J3vF = scipy.signal.lfilter(b, a, J3v)
    J3a = np.gradient(J3vF, time)
    
    plt.figure()
    plt.plot(time, jip, '.r')
    plt.figure()
    plt.plot(time, J3vF)
    plt.figure()
    plt.plot(time, J3a)

def plotTrajectoryFeedback(path, test_joint):
    Desired_Position_PATH = path['robotCalibration'] + 'test/test_with_700g_payload/desired_position.yaml'
    Desired_Velocity_PATH = path['robotCalibration'] + 'test/test_with_700g_payload/desired_velocity.yaml'
    Desired_Acceleration_PATH = path['robotCalibration'] + 'test/test_with_700g_payload/desired_acceleration.yaml'
    Actual_Position_PATH = path['robotCalibration'] + 'test/test_with_700g_payload/actual_position.yaml'
    Error_Position_PATH = path['robotCalibration'] + 'test/test_with_700g_payload/error_position.yaml'
    TF_TIME_PATH = path['robotCalibration'] + 'test/test_with_700g_payload/tf_time.yaml'

    JSPosition_PATH = path['robotCalibration'] + 'test/test_with_700g_payload/jsposition.yaml'
    JS_TIME_PATH = path['robotCalibration'] + 'test/test_with_700g_payload/js_time.yaml'
    
    with open(JSPosition_PATH) as f:
        Jip = np.array(yaml.load(f))[:, test_joint-1]
    with open(JS_TIME_PATH) as f:
        js_time = yaml.load(f)

    with open(Desired_Position_PATH) as f:
        desired_position = np.array(yaml.load(f))
    with open(Desired_Velocity_PATH) as f:
        desired_velocity = np.array(yaml.load(f))
    with open(Desired_Acceleration_PATH) as f:
        desired_acceralation = np.array(yaml.load(f))
    with open(Actual_Position_PATH) as f:
        actual_position = np.array(yaml.load(f))
    with open(Error_Position_PATH) as f:
        error_position = np.array(yaml.load(f))
    with open(TF_TIME_PATH) as f:
        tf_time = np.array(yaml.load(f))

    Jip_desired = desired_position[:, test_joint-1]
    Jiv_desired = desired_velocity[:, test_joint-1]
    Jia_desired = desired_acceralation[:, test_joint-1]

    Jiv_desired_m = np.gradient(Jip_desired, tf_time)
    Jia_desired_m = np.gradient(Jiv_desired_m, tf_time)

    Jip_actual = actual_position[:, test_joint-1]
    Jiv_actual = np.gradient(Jip_actual, tf_time)
    Jia_actual = np.gradient(Jiv_actual, tf_time)
    Jip_error = error_position[:, test_joint-1]
    cut_i = 20
    cut_f = 10

    plt.figure()
    plt.plot(tf_time, Jip_desired, '.k')
    plt.plot(tf_time, Jip_actual, '.r')
    # plt.plot(js_time[:-(cut_i+cut_f)], Jip[cut_i:-cut_f], '.g')

    plt.figure()
    plt.plot(tf_time, Jiv_desired, 'k')
    plt.plot(tf_time, Jiv_actual, 'r')  
    plt.plot(tf_time, Jiv_desired_m, 'b')
    plt.figure()
    plt.plot(tf_time, Jia_desired, 'k')
    plt.plot(tf_time, Jia_actual, 'r')
    plt.plot(tf_time, Jia_desired_m, 'b')
    plt.figure()
    plt.plot(tf_time, Jip_error, 'k')

def plotMultiTrajectory(path, test_joint, series_number='*'):
    # series_number is defined as %year%month%day%hour%minute$second
    Desired_Position_PATH = sorted(glob.glob(path['robotCalibration'] + 'test/' + series_number + '_desired_position.yaml'))
    Desired_Velocity_PATH = sorted(glob.glob(path['robotCalibration'] + 'test/' + series_number + '_desired_velocity.yaml'))
    Desired_Acceleration_PATH = sorted(glob.glob(path['robotCalibration'] + 'test/' + series_number + '_desired_acceleration.yaml'))
    Error_Position_PATH = sorted(glob.glob(path['robotCalibration'] + 'test/' + series_number + '_error_position.yaml'))
    TF_TIME_PATH = sorted(glob.glob(path['robotCalibration'] + 'test/' + series_number + '_tf_time.yaml'))
    Actual_Position_PATH = sorted(glob.glob(path['robotCalibration'] + 'test/' + series_number + '_actual_position.yaml'))
    
    desired_positions = []
    desired_velocitys = []
    desired_acceralations = []
    actual_positions = []
    tf_times = []
    for i in range(len(TF_TIME_PATH)):

        # with open(Desired_Position_PATH[i]) as f:
        #     desired_positions.append(np.array(yaml.load(f))[:, test_joint-1])
        # with open(Desired_Velocity_PATH[i]) as f:
        #     desired_velocitys.append(np.array(yaml.load(f))[:, test_joint-1])
        # with open(Desired_Acceleration_PATH[i]) as f:
        #     desired_acceralations.append(np.array(yaml.load(f))[:, test_joint-1])
        with open(Actual_Position_PATH[i]) as f:
            actual_positions.append(np.array(yaml.load(f))[:, test_joint-1])
        with open(TF_TIME_PATH[i]) as f:
            tf_times.append(np.array(yaml.load(f)))
    
    plt.figure()
    stablePosition = []
    for i, tf_time in enumerate(tf_times):
        Jip_actual = actual_positions[i]
        stablePosition.append(Jip_actual[-1])
   
        # plt.plot(tf_time, Jip_actual, '-')
        # plt.title('Joint Test (20 Test for J3)')
        # plt.xlabel('Execute time(sec)')
        # plt.ylabel('Anglur position of J3 motor(rad)')
    SysError = np.array(stablePosition) - np.mean(stablePosition)*np.ones(len(stablePosition))
    print(SysError)

def plotMultiJointState(path, test_joint, series_number='*'):
    # series_number is defined as %year%month%day%hour%minute$second
    JSPosition_PATH = sorted(glob.glob(path['robotCalibration'] + 'test/' + series_number + '_jsposition.yaml'))
    JS_TIME_PATH = sorted(glob.glob(path['robotCalibration'] + 'test/' + series_number + '_js_time.yaml'))
    
    js_positions = []
    js_times = []
    for i in range(len(JS_TIME_PATH)):

        with open(JSPosition_PATH[i]) as f:
            js_positions.append(np.array(yaml.load(f))[:, test_joint-1])
        with open(JS_TIME_PATH[i]) as f:
            js_times.append(np.array(yaml.load(f)))
    
    plt.figure()
    for i, js_time in enumerate(js_times):
        Jip_actual = js_positions[i]
        plt.plot(js_time, Jip_actual, 'o')
   

if __name__ == "__main__":
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)

    # plotJointState(path)
    # plotTrajectoryFeedback(path, test_joint=3)
    # plotMultiTrajectory(path, test_joint=3, series_number='190704*')
    for test_joint in range(6):
        plotMultiJointState(path, test_joint=test_joint, series_number='190807*')
    
    plt.show()