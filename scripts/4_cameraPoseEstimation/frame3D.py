from math import cos, sin, pi
import numpy as np
import quaternion
import numpy.matlib
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import scipy.optimize
'''
v1.0
'''
class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)
    
    def plot(self, ax):
        ax.add_artist(self)

class Orientation():
    def __init__(self):
        self.orientation = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    def cmd(self):
        return self.orientation

    def asItself(self, orientation):
        '''
        Return origin input
        '''
        (x, y, z, Rx, Ry, Rz) = orientation
        self.orientation = (x, y, z, Rx, Ry, Rz)
        return self

    def asRad(self, orientation):
        '''
        Convert Rx, Ry, Rz from degree to radians.
        '''
        (x, y, z, Rx, Ry, Rz) = orientation
        Rx, Ry, Rz = np.radians([Rx, Ry, Rz])
        self.orientation = (x, y, z, Rx, Ry, Rz)
        return self
    
    def asDeg(self, orientation):
        '''
        Convert Rx, Ry, Rz from radians to degree.
        '''
        (x, y, z, Rx, Ry, Rz) = orientation
        Rx, Ry, Rz = np.degrees([Rx, Ry, Rz])
        self.orientation = (x, y, z, Rx, Ry, Rz)
        return self

class Frame():
    def __init__(self, init_pose):
        self.pose = init_pose

# General method
    def __cal_direct_vectors_from_Frame__(self, refFrame, len_uv=1):
        '''
        The orientation of `refFrame` is with respact to Base Frame
        '''
        # Basic defination
        H = refFrame
        uv_0 = np.matrix([[0], [0], [0], [1]])
        uv_x = np.matrix([[len_uv], [0], [0], [1]])
        uv_y = np.matrix([[0], [len_uv], [0], [1]])
        uv_z = np.matrix([[0], [0], [len_uv], [1]])
        # Transform
        (x0, y0, z0, _) = np.array(H*uv_0).flatten()
        (i_x, i_y, i_z, _) = np.array(H*uv_x).flatten()
        (j_x, j_y, j_z, _) = np.array(H*uv_y).flatten()
        (k_x, k_y, k_z, _) = np.array(H*uv_z).flatten()
        # Result record
        q_unit_vecotr = np.array([[x0, y0, z0], [i_x, i_y, i_z], [j_x, j_y, j_z], [k_x, k_y, k_z]])
        vx = (i_x-x0, i_y-y0, i_z-z0)
        vy = (j_x-x0, j_y-y0, j_z-z0)
        vz = (k_x-x0, k_y-y0, k_z-z0)
        return (vx, vy, vz), q_unit_vecotr

    def __as_homogenous_matrix__(self, RotMat, TransVec):
        RotMat = np.zeros((3, 3)) if RotMat is None else RotMat
        TransVec = np.zeros((1, 3)) if TransVec is None else TransVec
        retMat = np.matlib.zeros((4, 4))
        retMat[0:3, 0:3] = RotMat
        retMat[0:3, 3] = TransVec.reshape(3, 1)
        retMat[3, 3] = 1.0
        return retMat

    def __objFun_get_orientation_from_pose__(self, X, qFramePose, refFrame):
        '''
        In `refFrame` coordinate, Rotate about fixed X -> fixed Y -> fixed Z by `Rx, Ry, Rz` radians.
        '''
        (vx, vy, vz), _ = self.__cal_direct_vectors_from_Frame__(refFrame)
        qRef = quaternion.from_rotation_matrix(refFrame[0:3, 0:3])
        tx = X[0]/2
        ty = X[1]/2
        tz = X[2]/2
        qx = self.__quaternion_by_axis__(tx, vx)
        qy = self.__quaternion_by_axis__(ty, vy)
        qz = self.__quaternion_by_axis__(tz, vz)
        q = qz*qy*qx*qRef
        return quaternion.as_float_array(q - qFramePose)

# Translatetion method  
    def translate(self, Px, Py, Pz, refFrame):
        trans = np.matrix([[Px], [Py], [Pz], [1]])
        TransVec = refFrame*trans

        Rot_ref = refFrame[0:3, 0:3]
        Trans_ref = refFrame[0:3, 3]
        TransVec2 = Trans_ref + Rot_ref*trans[0:3]
        return np.array(TransVec).flatten()[0:3]

# Rotation method
    def __quaternion_by_axis__(self, theta, v):
        t = theta
        return np.quaternion(cos(t), v[0]*sin(t), v[1]*sin(t), v[2]*sin(t))

    def qRotate_XYZ(self, Rx, Ry, Rz, refFrame):
        '''
        In `refFrame` coordinate, Rotate about fixed X -> fixed Y -> fixed Z by `Rx, Ry, Rz` radians.
        '''
        (vx, vy, vz), _ = self.__cal_direct_vectors_from_Frame__(refFrame)
        qRef = quaternion.from_rotation_matrix(refFrame[0:3, 0:3])
        tx = Rx/2
        ty = Ry/2
        tz = Rz/2
        qx = self.__quaternion_by_axis__(tx, vx)
        qy = self.__quaternion_by_axis__(ty, vy)
        qz = self.__quaternion_by_axis__(tz, vz)
        q = qz*qy*qx*qRef
        return quaternion.as_rotation_matrix(q)

    def matRotate_XYZ(self, H, refFrame):
        return refFrame[0:3, 0:3]*H[0:3, 0:3]
    
# Transformation operation
    def transform(self, orientation, refFrame, rotDef='FixedXYZ'):
        Px, Py, Pz = orientation[0:3]
        TransVec = self.translate(Px, Py, Pz, refFrame)
        
        try:
            Rx, Ry, Rz = orientation[3:6]
            if rotDef == 'FixedXYZ':
                RotMat = self.qRotate_XYZ(Rx, Ry, Rz, refFrame)
        except:
            print('The rotation defination did not exist.')
        self.pose = self.__as_homogenous_matrix__(RotMat, TransVec)
        return self.pose

    def transform_by_rotation_mat(self, transMat, refFrame):
        Px, Py, Pz = np.array(transMat)[0:3, 3].flatten()
        TransVec = self.translate(Px, Py, Pz, refFrame)
        RotMat = self.matRotate_XYZ(transMat, refFrame)
        self.pose = self.__as_homogenous_matrix__(RotMat, TransVec)
        return self.pose

    def get_transform_matrix(self, orientation, rotDef='FixedXYZ'):
        localFrame = np.matlib.identity(4)
        Px, Py, Pz = orientation[0:3]
        TransVec = self.translate(Px, Py, Pz, localFrame)
        
        try:
            Rx, Ry, Rz = orientation[3:6]
            if rotDef == 'FixedXYZ':
                RotMat = self.qRotate_XYZ(Rx, Ry, Rz, localFrame)
        except:
            print('The rotation defination did not exist.')

        return self.__as_homogenous_matrix__(RotMat, TransVec)

    def get_orientation_from_pose(self, framePose, refFrame, rotDef='FixedXYZ'):
        '''
        The inverse function of XYZ transformation, There exist multiple set of solution. But the final orientation of frmae is same
        '''
        qPose = quaternion.from_rotation_matrix(framePose)
        X0 = np.ones(3)
        res = scipy.optimize.least_squares(self.__objFun_get_orientation_from_pose__, 
                        X0, args=(qPose, refFrame))
        orientation = (framePose[0, 3], framePose[1, 3], framePose[2, 3],\
                        res['x'][0], res['x'][1], res['x'][2])
        if res['success']:
            if res['cost'] < 1e-12:
                # print(res['cost'])
                return orientation
            else:
                raise('Optimization error')
        else:
            raise('Optimization error')

# Special Method
    def sequenceCmd(self, orientations, rotDef = 'FixedXYZ'):
        n = len(orientations)
        Hs = np.zeros((n, 4, 4))
        for i, orientation in enumerate(orientations):
            Hs[i, 0:4, 0:4] = self.get_transform_matrix(orientation)
        return Hs

# Frame Visualization
    def __draw_frame__(self, q_unit_vector):
        ((x0, y0, z0), (x_i, x_j, x_k), (y_i, y_j, y_k), (z_i, z_j, z_k)) = q_unit_vector
        X = Arrow3D([x0,x_i],[y0,x_j],[z0,x_k], mutation_scale=20, lw=1, arrowstyle="-|>", color="r")
        Y = Arrow3D([x0,y_i],[y0,y_j],[z0,y_k], mutation_scale=20, lw=1, arrowstyle="-|>", color="g")
        Z = Arrow3D([x0,z_i],[y0,z_j],[z0,z_k], mutation_scale=20, lw=1, arrowstyle="-|>", color="b")
        X.plot(self.ax)
        Y.plot(self.ax)
        Z.plot(self.ax)
        self.ax.scatter(x0, y0, z0, zdir='z', s=30, c='black', depthshade=True)
        self.ax.set_xlabel('x_values')
        self.ax.set_ylabel('y_values')
        self.ax.set_zlabel('z_values')
        self.ax.text(x0, y0, z0, s=self.frameName, zdir=None)

    def plot_frame(self, plotAx, frameName, axisLen=0.300):
        self.ax = plotAx
        self.frameName = frameName
        
        _, q_unit_vecotr = self.__cal_direct_vectors_from_Frame__(self.pose, len_uv=axisLen)
        q_unit_vecotr[1:] = q_unit_vecotr[1:]
        self.__draw_frame__(q_unit_vecotr)


def make_3D_fig(axSize=250):
    fig = plt.figure(figsize=(15,15)) 
    ax = fig.gca(projection='3d')
    set_frame_lim(ax, axSize)
    ax.view_init(elev=45, azim=-135)
    return ax

def set_frame_lim(ax, axSize):
    bottom = -axSize
    top = axSize
    ax.set_zlim3d(0.0, top)
    ax.set_ylim3d(bottom, top)
    ax.set_xlim3d(bottom, top)

def RotX(theta):
    t = theta
    return np.matrix([[1, 0, 0, 0], [0, cos(t), -sin(t), 0], [0, sin(t), cos(t), 0], [0, 0, 0, 1]])

def RotY(theta):
    t = theta
    return np.matrix([[cos(t), 0, sin(t), 0], [0, 1, 0, 0], [-sin(t), 0, cos(t), 0], [0, 0, 0, 1]])

def RotZ(theta):
    t = theta
    return np.matrix([[cos(t), -sin(t), 0, 0], [sin(t), cos(t), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def P(x, y, z):
    return np.matrix([[0, 0, 0, x], [0, 0, 0, y], [0, 0, 0, z], [0, 0, 0, 0]])

def H(orientation):
    (x, y, z, Rx, Ry, Rz) = orientation
    return  RotZ(Rz)*RotY(Ry)*RotX(Rx) + P(x, y, z)

if __name__ == '__main__':
# # Frame operation
    # # Create object
    # ax = make_3D_fig(axSize=200)
    # Base = Frame(np.matlib.identity(4))
    # Wrist = Frame(Base.pose)
    # Camera = Frame(Base.pose)
    # Board = Frame(Base.pose)
    # Board2 = Frame(Base.pose)
    # orientation = Orientation()

    # cmd = orientation.asRad((100, 20, 0, 0, 0, 45)).cmd()
    # Wrist.transform(cmd, refFrame=Base.pose)
    # B = Wrist.get_transform_matrix(cmd)

    # cmd = orientation.asRad((50, -40, 0, 0, 0, 195)).cmd()
    # Camera.transform(cmd, refFrame=Wrist.pose)
    # X = Camera.get_transform_matrix(cmd)
    
    # A = np.matrix([ [-0.0052, -0.9927, 0.1203,   61.4368],
    #                 [ 0.9998, -0.0027, 0.0215, -305.2288],
    #                 [-0.0211,  0.1204, 0.9925,   96.9816],
    #                 [      0,       0,      0,    1.0000]])
    # Board.transform_by_rotation_mat(A, refFrame=Camera.pose)

    # Z = B*X*A
    # Board2.transform_by_rotation_mat(Z, refFrame=Base.pose)
    
    # # Frame plot
    # Base.plot_frame(ax, 'Base', axisLen=150)
    # Wrist.plot_frame(ax, 'Wrist', axisLen=150)
    # Camera.plot_frame(ax, 'Camera')
    # Board.plot_frame(ax, 'Board')
    # Board2.plot_frame(ax, 'Board2')
    # plt.show()

# Get_eular_anlge_from_pose test
    plt.ion()
    fig = plt.figure(figsize=(15,15)) 
    Base = Frame(np.matlib.identity(4))
    Frame1 = Frame(Base.pose)
    orientation = Orientation()

    residual = []
    for i in range(10):
        plt.clf()
        ax = fig.gca(projection='3d')
        set_frame_lim(ax, 300)
        orientation_i = np.random.rand(6)*2*np.pi
        cmd_i = orientation.asItself(orientation_i).cmd()
        Frame1.transform(cmd_i, refFrame=Base.pose)
        Frame1.plot_frame(ax, 'Frame {}'.format(i))
        pose_i = Frame1.pose

        
        orientation_i_hat = Frame1.get_orientation_from_pose(Frame1.pose, Base.pose)
        cmd_hat = orientation.asItself(orientation_i_hat).cmd()
        Frame1.transform(cmd_hat, refFrame=Base.pose)
        Frame1.plot_frame(ax, 'Frame_hat {}'.format(i))
        pose_hat = Frame1.pose
        residual.append(np.linalg.norm(pose_i - pose_hat))
        if i % 100 == 0:
            print(i)
        plt.waitforbuttonpress(1)
    # plt.plot(residual)
    plt.show()



