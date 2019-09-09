import numpy as np
from math import cos, sin
import matplotlib.pyplot as plt
import frame3D
import scipy.optimize
import yaml
import quaternion

########################
# Calculate camera pose
########################
def get_position_from_spherical(refFrame, rho, theta, phi):
    '''
    Get photo position(x,y,z) in spherical cooridnate and convert it to world frame coordinate
    - a_0: x-value with respact to frame 0
    - a_w: x-value with respact to world frame
    '''
    origin = refFrame*np.matrix([[0], [0], [0], [1]])
    a_0 = rho*sin(phi)*cos(theta)
    b_0 = rho*sin(phi)*sin(theta)
    c_0 = rho*cos(phi)
    p_photo = refFrame*np.matrix([[a_0], [b_0], [c_0], [1]])
    (a_w, b_w, c_w) = convert_to_vector(p_photo)
    normal_to_origin_vector = convert_to_vector(origin - p_photo)
    return (a_w, b_w, c_w, normal_to_origin_vector)

def objFun_normal_vector(X, position_1, refFrame):
    (Rx, Ry, Rz) = X
    (a, b, c, n1) = position_1
    orientation_1 = (a, b, c, Rx, Ry, Rz)
    tarFrame = frame3D.H(orientation_1)
    # Constraint 1
    n1_hat = get_normal_vector(tarFrame)
    angle = get_angle_between_vectors(n1, n1_hat)
    # Constraint 2
    n0_z = get_normal_vector(refFrame, axis='z')
    n0_y = get_normal_vector(refFrame, axis='y')
    n1_y = get_normal_vector(tarFrame, axis='y')
    n1_y_proj = get_project_vector_on_plane(n1_y, n0_z)
    angle2 = get_angle_between_vectors(n0_y, n1_y_proj)
    
    return np.array([angle, angle2, 0])

def get_angle_between_vectors(n0, n1):
    numerator = np.dot(n0, n1)
    denominator = np.linalg.norm(n0) * np.linalg.norm(n1)
    
    if np.isclose(numerator, denominator):
        # print('numerator:{},denominator:{}'.format(numerator, denominator))
        angle = 0
    else:
        angle = np.arccos(numerator/denominator)
    return angle

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

def get_project_vector_on_plane(vector, normal_vector):
    proj_on_normal_vector = np.dot(vector, normal_vector) / np.linalg.norm(normal_vector)**2 * normal_vector
    proj_on_plane = vector - proj_on_normal_vector
    return proj_on_plane

def get_photo_orientation_by_given_shoot_angle(guess_orientation, refFrame, focal_length, shoot_angle, theta):
    position_1 = get_position_from_spherical(refFrame, rho = focal_length, theta = theta, phi = shoot_angle)

    res = scipy.optimize.leastsq(objFun_normal_vector, guess_orientation, 
                        args=(position_1, refFrame), full_output=1)

    (Rx, Ry, Rz) = res[0] 
    (a, b, c, _) = position_1
    return [a, b, c, Rx, Ry, Rz]

def main_calculateCameraPose(WD, pose_amount, polar_ang):
    # ax = frame3D.make_3D_fig(axSize=200)
    World = frame3D.Frame(np.matlib.identity(4))
    Camera = frame3D.Frame(World.pose)
    orientation = frame3D.Orientation()

    thetas = np.linspace(0, 360, pose_amount, endpoint=False) # azimuthal angle
    X0 = (np.radians(45), np.radians(45), np.radians(45)) # initial guess
    camOrientations = np.zeros((pose_amount,6)) 

    if polar_ang == 0:
        for ind, theta in enumerate(thetas) :
            t = theta + 60
            camOrientations[ind, :] = (0.0, 0.0, WD, np.pi, 0.0, np.radians(t))
    else:
        for ind, theta in enumerate(thetas) :
            orientation_2 = get_photo_orientation_by_given_shoot_angle(X0,\
                        World.pose, WD, np.radians(polar_ang), np.radians(theta))
            X0 = orientation_2[3:6]
            cmd = orientation.asItself(orientation_2).cmd()
            Camera.transform(cmd, refFrame=World.pose)
            camOrientations[ind, :] = Camera.get_orientation_from_pose(Camera.pose, refFrame=World.pose)
    
    return camOrientations

def as_ROSgoal(Homo_mats):
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
    H[0:3, 3] = rosGoal[0:3].reshape(3,1)
    q = np.quaternion(rosGoal[6], rosGoal[3], rosGoal[4], rosGoal[5])
    H[0:3, 0:3] = quaternion.as_rotation_matrix(q)
    return H

def main_gazebo(DEBUG):
    # import os.path
    # PATH SETTING
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)

    BASE = path['APose'] if DEBUG else path['ROOT']
    AF_BASE = path['AFocus'] if DEBUG else path['ROOT']
    BF_GOAL = AF_BASE + 'goal/bf_goal.yaml'
    CAMERA_POSE_PATH = BASE + 'goal/A_desired.yaml'
    ROS_GOAL_PATH = BASE + 'goal/ap_goal.yaml'
    Bs_PATH = BASE + 'goal/Bs.yaml'
    X_PATH = BASE + 'goal/X.yaml'
    Z_PATH = BASE + 'goal/Z.yaml'
    Z2_PATH = BASE + 'goal/Z2.yaml'

    with open(BF_GOAL) as f:
        bf_goal = np.array(yaml.load(f))

    cam_pose = [-0.040, 0, 0.040, 0, 0, -90] # pose wrt Flange in meter: (x,y,z,Rx,Ry,Rz)
    WD = bf_goal[2] - cam_pose[2] # work distence from camera to pattern(m)

    # First layer of poses
    phi = 0 # polarangle 
    camOrien1 = main_calculateCameraPose(WD, 6, phi)

    # Second layer of poses
    phi = 15 # polarangle 
    camOrien2 = main_calculateCameraPose(WD, 12, phi) 

    # Third layer of poses
    phi = 20 # polarangle 
    camOrien3 = main_calculateCameraPose(WD, 6, phi) 

    camOrientations = np.concatenate((camOrien1, camOrien2, camOrien3), axis=0)

    # data visualization
    World = frame3D.Frame(np.matlib.identity(4))
    Pattern = frame3D.Frame(World.pose)
    Image = frame3D.Frame(Pattern.pose)
    Camera = frame3D.Frame(Pattern.pose)
    Flange = frame3D.Frame(Pattern.pose)
    Base = frame3D.Frame(Pattern.pose)
    Flange_test = frame3D.Frame(Pattern.pose)
    orientation = frame3D.Orientation()
    ax = frame3D.make_3D_fig(axSize=0.45)

    ##################
    # Determine Z
    ##################    
    # Base as World coordinate
    Base.plot_frame(ax, 'Base')
    # Base to Flange 
    B0 = as_MatrixGoal(bf_goal)
    Flange.transform_by_rotation_mat(B0, refFrame=Base.pose)
    # Flange.plot_frame(ax, 'initFlange')
    # Flange to Camera
    cmd_X = orientation.asRad(cam_pose).cmd()
    X = Camera.transform(cmd_X, refFrame=World.pose)
    Camera.transform(cmd_X, refFrame=Flange.pose)
    # Camera.plot_frame(ax, 'initCamera')
    # Camera to Pattern
   
    cmd_A0 = orientation.asRad((0, 0, WD, 0, 180, 0)).cmd()
    A0 = Pattern.transform(cmd_A0, refFrame=World.pose)
    Pattern.transform(cmd_A0, refFrame=Camera.pose)
    Pattern.plot_frame(ax, 'Pattern')
    # Base to Pattern
    Z = B0*X*A0
    
    ##################
    # Pose calculation
    ##################
    pose_amount = len(camOrientations)
    iAs = np.zeros((pose_amount,4,4))
    As = np.zeros((pose_amount,4,4))
    Bs = np.zeros((pose_amount,4,4))
    iX = np.linalg.inv(X)
    for i, camOrientation in enumerate(camOrientations):
        cmd_iA = orientation.asItself(camOrientation).cmd()
        iAs[i] = Camera.transform(cmd_iA, refFrame=World.pose)
        As[i] = np.linalg.inv(iAs[i])
        Camera.transform(cmd_iA, refFrame=Pattern.pose)
        Camera.plot_frame(ax, '')    

        Flange.transform_by_rotation_mat(iX, refFrame=Camera.pose)
        # Flange.plot_frame(ax, '')
        # {Test Flange calculate from Z*A*X}
        Bs[i] = Z*iAs[i]*iX
    plt.show()
    # plt.show(block=False)
    # plt.pause(0.5)
    # plt.close()

    with open(Bs_PATH, 'w') as f:
        yaml.dump(Bs.tolist(), f, default_flow_style=False)
        print('Save the Bs to yaml data file.')
    
    with open(X_PATH, 'w') as f:
        yaml.dump(X.tolist(), f, default_flow_style=False)
        print('Save the X to yaml data file.')
    
    with open(Z_PATH, 'w') as f:
        yaml.dump(Z.tolist(), f, default_flow_style=False)
        print('Save the Z to yaml data file.')
    
    with open(CAMERA_POSE_PATH, 'w') as f:
        yaml.dump(As.tolist(), f, default_flow_style=False)
        print('Save the desired A to yaml data file.')

    #################
    # Save as moveit command
    #################
    goals = as_ROSgoal(Bs)
    with open(ROS_GOAL_PATH, 'w') as f:
        yaml.dump(goals.tolist(), f, default_flow_style=False)
        print('Save the ROS goal to yaml data file.')

def main_denso(DEBUG):
    # PATH SETTING
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)

    BASE = path['APose'] if DEBUG else path['ROOT']
    AF_BASE = path['AFocus'] if DEBUG else path['ROOT']
    BF_GOAL = AF_BASE + 'goal/bf_goal.yaml'
    CAMERA_POSE_PATH = BASE + 'goal/camera_pose.yaml'
    ROS_GOAL_PATH = BASE + 'goal/ap_goal.yaml'
    Bs_PATH = BASE + 'goal/Bs.yaml'
    X_PATH = BASE + 'goal/X.yaml'
    Z_PATH = BASE + 'goal/Z.yaml'
    Z2_PATH = BASE + 'goal/Z2.yaml'

    with open(BF_GOAL) as f:
        bf_goal = np.array(yaml.load(f))

    cam_pose = [-0.040, 0, 0.040, 0, 0, -90] # pose wrt Flange in meter: (x,y,z,Rx,Ry,Rz)
    WD = bf_goal[2] - cam_pose[2] # work distence from camera to pattern(m)

    # First layer of poses
    phi = 0 # polarangle 6
    camOrien1 = main_calculateCameraPose(WD, 6, phi)

    # Second layer of poses
    phi = 10 # polarangle 
    camOrien2 = main_calculateCameraPose(WD, 12, phi) 

    # Third layer of poses
    phi = 15 # polarangle 
    camOrien3 = main_calculateCameraPose(WD, 6, phi) 

    camOrientations = np.concatenate((camOrien1, camOrien2, camOrien3), axis=0)

    # data visualization
    World = frame3D.Frame(np.matlib.identity(4))
    Pattern = frame3D.Frame(World.pose)
    Image = frame3D.Frame(Pattern.pose)
    Camera = frame3D.Frame(Pattern.pose)
    Flange = frame3D.Frame(Pattern.pose)
    Base = frame3D.Frame(Pattern.pose)
    Flange_test = frame3D.Frame(Pattern.pose)
    orientation = frame3D.Orientation()
    ax = frame3D.make_3D_fig(axSize=0.45)

    ##################
    # Determine Z
    ##################    
    # Base as World coordinate
    Base.plot_frame(ax, 'Base')
    # Base to Flange 
    B0 = as_MatrixGoal(bf_goal)
    Flange.transform_by_rotation_mat(B0, refFrame=Base.pose)
    Flange.plot_frame(ax, 'initFlange')
    # Flange to Camera
    cmd_X = orientation.asRad(cam_pose).cmd()
    X = Camera.transform(cmd_X, refFrame=World.pose)
    Camera.transform(cmd_X, refFrame=Flange.pose)
    Camera.plot_frame(ax, 'initCamera')
    # Camera to Pattern
   
    cmd_A0 = orientation.asRad((0, 0, WD, 0, 180, 0)).cmd()
    A0 = Pattern.transform(cmd_A0, refFrame=World.pose)
    Pattern.transform(cmd_A0, refFrame=Camera.pose)
    Pattern.plot_frame(ax, 'Pattern')
    # Base to Pattern
    Z = B0*X*A0
    
    ##################
    # Pose calculation
    ##################
    pose_amount = len(camOrientations)
    iAs = np.zeros((pose_amount,4,4))
    Bs = np.zeros((pose_amount,4,4))
    iX = np.linalg.inv(X)
    for i, camOrientation in enumerate(camOrientations):
        cmd_iA = orientation.asItself(camOrientation).cmd()
        iAs[i] = Camera.transform(cmd_iA, refFrame=World.pose)
        Camera.transform(cmd_iA, refFrame=Pattern.pose)
        Camera.plot_frame(ax, '')    

        Flange.transform_by_rotation_mat(iX, refFrame=Camera.pose)
        # Flange.plot_frame(ax, '')
        # {Test Flange calculate from Z*A*X}
        Bs[i] = Z*iAs[i]*iX
    # plt.show()
    plt.show(block=False)
    plt.pause(0.5)
    plt.close()

    with open(Bs_PATH, 'w') as f:
        yaml.dump(Bs.tolist(), f, default_flow_style=False)
        print('Save the Bs to yaml data file.')
    
    with open(X_PATH, 'w') as f:
        yaml.dump(X.tolist(), f, default_flow_style=False)
        print('Save the X to yaml data file.')
    
    with open(Z_PATH, 'w') as f:
        yaml.dump(Z.tolist(), f, default_flow_style=False)
        print('Save the Z to yaml data file.')

    #################
    # Save as moveit command
    #################
    goals = as_ROSgoal(Bs)
    with open(ROS_GOAL_PATH, 'w') as f:
        yaml.dump(goals.tolist(), f, default_flow_style=False)
        print('Save the ROS goal to yaml data file.')

if __name__ =='__main__':
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
    main_gazebo(DEBUG=DEBUG) if SIM else main_denso(DEBUG=DEBUG)
