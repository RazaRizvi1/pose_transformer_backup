#!/usr/bin/env python3
import os
import shutil
import cv2
# import cv2.aruco as aruco
import numpy as np
#import scipy
from geometry_msgs.msg import Transform, TransformStamped, PoseStamped
from scipy.spatial.transform import Rotation
#from scipy.optimize import least_squares
#import Equirec2Perspec as E2P
import rospy
import tf

def make_dir(path,delete_if_exists=False):   
    if not os.path.exists(path):
        os.makedirs(path)
    elif delete_if_exists:
        shutil.rmtree(path)
        os.makedirs(path)

def pinhole_model(pose,K):
    """Loads projection and extrensics information for a pinhole camera model

    Arguments
    -------
        pose (numpy.array): px, py, cx, qx, qy, qz, qw
        K (numpy.array): camera matrix

    Returns
    -------
        P (numpy.array): projection matrix
        R (numpy.array): rotation matrix (camera coordinates)
        C (numpy.array): camera center (world coordinates)
    """

    pose = pose.reshape(-1)
    C = np.array(pose[1:4]).reshape(-1,1)

    # Convert camera rotation from quaternion to matrix
    q = pose[4:]
    Rot = Rotation.from_quat(q)
    
    # Find camera extrinsics (R,t)
    R = Rot.T
    t = Rot.T.dot(-C)

    # Construct projection matrix (P)
    P = np.dot(K, np.hstack([R, t]))

    return P,R,C  

def ProjectToImage(projectionMatrix, pos):
    """Project 3D world coordinates to 2D image coordinates using a pinhole camera model
    
    Arguments
    -------
        P (numpy.array): projection matrix
        pos (numpy.array): 3D world coordinates (3xN)

    Returns
    -------
        uv (numpy.array): 2D pixel coordinates (2xN)
    """    
    pos = np.array(pos).reshape(3,-1)
    pos_ = np.vstack([pos, np.ones((1, pos.shape[1]))])

    uv_ = np.dot(projectionMatrix, pos_)
    #uv_ = uv_[:, uv_[-1, :] > 0]
    uv = uv_[:-1, :]/uv_[-1, :]

    return uv

def ProjectToWorld(projectionMatrix, uv, R, C):
    """Back-project 2D image coordinates to rays in 3D world coordinates using a pinhole camera model
    
    Arguments
    -------
        P (numpy.array): projection matrix
        uv (numpy.array): 2D pixel coordinates (2xN)
        R (numpy.array): rotation matrix (camera coordinates)
        C (numpy.array): camera center (world coordinates)       

    Returns
    -------
        pos (numpy.array): [3D world coordinates (3xN)]
    """       
    uv_ = np.vstack([uv[0,:], uv[1,:], np.ones((1, uv.shape[1]))])
    pinvProjectionMatrix = np.linalg.pinv(projectionMatrix)

    pos2_ = np.dot(pinvProjectionMatrix, uv_)
    pos2_[-1,pos2_[-1,:]==0] = 1
    pos2 = pos2_[:-1,:]/pos2_[-1,:]
    rays = pos2-C

    # check that rays project forwards
    rays_local = np.dot(R , rays)
    rays[:,rays_local[2,:]<0] = -1*rays[:,rays_local[2,:]<0]    
    rays = rays/np.linalg.norm(rays,axis=0)

    return rays


def T_inv(Tmat):
    if Tmat.shape == (4,4):
        R = Tmat[:3,:3]
        t = Tmat[:3,3]
        R_inv = R.T
        t_inv = -R_inv.dot(t)     
        Tmat_inv = np.eye(4)
        Tmat_inv[:3,:3] = R_inv
        Tmat_inv[:3,3] = t_inv
        return Tmat_inv
    else:
        raise 'Error: input should be 4x4 transformation matrix'

def create_transform(p,q):
    
    T = Transform()
    T.translation.x = p[0]
    T.translation.y = p[1]
    T.translation.z = p[2]
    
    T.rotation.x = q[0]
    T.rotation.y = q[1]
    T.rotation.z = q[2]
    T.rotation.w = q[3]
    
    return T

def create_transform_stamped(p,q,t,child_frame_id,frame_id):
    
    T = TransformStamped()
    T.header.stamp = t
    T.header.frame_id = frame_id
    T.child_frame_id = child_frame_id

    T.transform.translation.x = p[0]
    T.transform.translation.y = p[1]
    T.transform.translation.z = p[2]

    T.transform.rotation.x = q[0]
    T.transform.rotation.y = q[1]
    T.transform.rotation.z = q[2]
    T.transform.rotation.w = q[3]
    
    return T

def create_pose(T_m2_m1, query_odom_frame, timestamp_query=None):
    t_unity = T_m2_m1[:3,3]
    q_unity = tf.transformations.quaternion_from_matrix(T_m2_m1)

    if timestamp_query is None:
        timestamp_query = rospy.Time.now()
        
    ps = PoseStamped()
    ps.header.stamp = timestamp_query
    ps.header.frame_id = query_odom_frame
    ps.pose.position.x = t_unity[0]
    ps.pose.position.y = t_unity[1]
    ps.pose.position.z = t_unity[2]
    ps.pose.orientation.x = q_unity[0]
    ps.pose.orientation.y = q_unity[1]
    ps.pose.orientation.z = q_unity[2]
    ps.pose.orientation.w = q_unity[3]    

    return ps

def unpack_transform(T):
    p = [T.translation.x, T.translation.y, T.translation.z]
    q = [T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w]
    return p,q

def unpack_pose(T):
    p = [T.position.x, T.position.y, T.position.z]
    q = [T.orientation.x, T.orientation.y, T.orientation.z, T.orientation.w]
    return p,q    

def points2numpy(pl):
    return np.array([[p.x,p.y,p.z] for p in pl])

def quaterions2numpy(ql):
    return np.array([[q.x,q.y,q.z,q.w] for q in ql])    


def project_2d_to_3d(m,K,D,center=False, h=0):
    u = m[0,:]
    v = m[1,:]
         
    fx = K[0,0]
    fy = K[1,1]
    cx = K[0,2]
    cy = K[1,2]

    d = []

    if center:
        d0 = D[int(v.mean()),int(u.mean())]   
        d = [d0 for _ in range(u.shape[0])]  
    else:
        for ui,vi in zip(u,v):
            di = D[int(vi)-h:int(vi)+h+1,int(ui)-h:int(ui)+h+1]
            if len(di)>0:
                di = di[di>0].mean()
                d.append(di)

    Z = np.array(d,dtype=np.float)/1000
    X = Z*(u-cx)/fx
    Y = Z*(v-cy)/fy    

    return X,Y,Z      

def countour2mask(m,I):
    
    mask = np.zeros(I.shape)
    contours=m
    mask=cv2.drawContours(mask, contours, -1, (255),1)    
    return mask

def cloud_inside_polygon(m,K,D):
    mask = countour2mask(m,D)
    cloud = project_2d_to_3d(mask,K,D)
    return cloud

def detect_markers(I,find_ids=None,xy_array=False,ignore_zeros=True):

    gray = cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)

    w = int(I.shape[1])
    h = int(I.shape[0])

    # define names of each possible ArUco tag OpenCV supports
    ARUCO_DICT = {
        # "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        # "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        # "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        # "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        # "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        # "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        # "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        # "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        # "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        # "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        # "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        # "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        # "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        # "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        # "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        # "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
        # "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        # "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
        # "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        # "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
    }

    # loop over the types of ArUco dictionaries
    for (arucoName, arucoDict) in ARUCO_DICT.items():
        
        # load the ArUCo dictionary, grab the ArUCo parameters, and
        # attempt to detect the markers for the current dictionary
        arucoDict = cv2.aruco.Dictionary_get(arucoDict)
        arucoParams = cv2.aruco.DetectorParameters_create()
        arucoParams.adaptiveThreshConstant = 15
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
            gray, arucoDict, parameters=arucoParams)    

        if ids is not None:
            # print(arucoName)
            break

    if ids is None:
        return []
    ids = list(ids.reshape(-1))

    # if specified, only find markers with ids in find_ids list
    if isinstance(find_ids,list):
        corners2 = []
        ids2 = []
        for i,c in zip(ids,corners):
            if i in find_ids:
                ids2.append(i)
                corners2.append(c)
    else:
        corners2 = corners
        ids2 = ids

    objects = []
    for c,i in zip(corners2,ids2):
        con = c.reshape(-1, 2)
        if con.shape[0] < 3:
            continue

        if ignore_zeros and ids==0:
            continue

        obj = dict([])
        obj['id'] = 'ID:'+str(i)
        obj['confidence'] = 1.0

        if xy_array:
            coords = np.array([[],[]])
            for pt in con:
                x = float(pt[0])
                y = float(pt[1])
                coords = np.hstack([coords,[[x],[y]]])
            obj['coords'] = coords
        else:
            obj['coords'] = []
            for pt in con:
                coords = dict([])
                coords['x'] = float(pt[0])
                coords['y'] = float(pt[1])
                obj['coords'].append(coords)

        objects.append(obj)     

    return objects


def detect_manual(I,m,s,h=0.05):

    W = int(I.shape[1])
    H = int(I.shape[0])

    objects = []

    obj = dict([])
    obj['id'] = 'ID:'+str(s)
    obj['confidence'] = 1.0

    coords = np.array([[],[]])
    for i in range(4):
        if i == 2:
            h1 = 1
            h2 = -1
        elif i == 1:
            h1 = 1
            h2 = 1
        elif i == 0:
            h1 = -1
            h2 = 1
        elif i == 3:
            h1 = -1
            h2 = -1                                    
        x = (float(m[1])+h1*h)*W
        y = (float(m[0])+h2*h)*H
        coords = np.hstack([coords,[[x],[y]]])
    obj['coords'] = coords


    objects.append(obj)     

    return objects


def multiviewSolvePnPRansac(pts3D_l, pts2D_l, poses_l, K2, max_reproj_error=25, max_iterations=1000):

    n_test = 4
    n_imgs = len(pts2D_l)
    iterations = 0
    T_m1_m2 = None
    if len(poses_l[0])>7:
        k0 = len(poses_l[0])-6
    else:
        k0 = 0
    T_m2_c_l = [pose2matrix(pose[k0:]) for pose in poses_l]
    best_inlier_idxs = [[] for _ in range(n_imgs)]  
    best_err = None
    # for i_img in range(n_imgs):
    iterations = 0
    best_inlier_idxs = [[] for _ in range(n_imgs)]  
    while iterations < max_iterations:
        i_img = np.random.randint(0,n_imgs)
        n_data = pts2D_l[i_img].shape[0]
        if n_data == 0:
            continue
        test_idxs = np.random.randint(0,n_data,n_test)
        pts2D_test = pts2D_l[i_img][test_idxs,:]
        pts3D_test = pts3D_l[i_img][test_idxs,:]
        retval,rvecs_test,tvecs_test = cv2.solvePnP(pts3D_test,pts2D_test,K2,None,flags=cv2.SOLVEPNP_P3P)
        if not retval:
            continue
        T_m1_c_test = poses2matrix(tvecs_test,rvecs_test)
        T_m2_c = T_m2_c_l[i_img]
        T_c_m2 = T_inv(T_m2_c)
        T_m1_m2_test = np.dot(T_m1_c_test,T_c_m2)

        test_err = multiview_pnp_error(pts3D_l, pts2D_l, K2, T_m1_m2_test, T_m2_c_l)
        inlier_idxs = [np.where([e < max_reproj_error])[1].tolist() for e in test_err] # select indices of rows with accepted points

        if len_subelems(inlier_idxs) > len_subelems(best_inlier_idxs):
            T_m1_m2 = T_m1_m2_test
            best_inlier_idxs = inlier_idxs
            best_err = [np.array(e,dtype=np.int) for e in test_err]
            # print(T_m1_m2[:3,3])
        iterations+=1
        
    tvecs_l = [matrix2poses(T_m1_m2.dot(T_m2_c))[0] for T_m2_c in T_m2_c_l]
    rvecs_l = [matrix2poses(T_m1_m2.dot(T_m2_c))[1] for T_m2_c in T_m2_c_l]

    return T_m1_m2,tvecs_l,rvecs_l,best_inlier_idxs


def multiviewSolvePnPOptimization(pts3D_l, pts2D_l, poses_l, K2, T_m1_m2_init):

    if len(poses_l[0])>7:
        k0 = len(poses_l[0])-6
    else:
        k0 = 0
    T_m2_c_l = [pose2matrix(pose[k0:]) for pose in poses_l]

    x0 = matrix2pose(T_m1_m2_init)
    res=least_squares(optim_multiview_pnp_error,x0, verbose=2, x_scale='jac', ftol=1e-4, method='trf',args=(pts3D_l, pts2D_l, K2, T_m2_c_l))

    T_m1_m2 = pose2matrix(res.x)

    tvecs_l = [matrix2poses(T_m1_m2.dot(T_m2_c))[0] for T_m2_c in T_m2_c_l]
    rvecs_l = [matrix2poses(T_m1_m2.dot(T_m2_c))[1] for T_m2_c in T_m2_c_l]

    return T_m1_m2,tvecs_l,rvecs_l

def optim_multiview_pnp_error(x,pts3D_l, pts2D_l, K2, T_m2_c_l):
    T_m1_m2_test = np.eye(4)
    p0 = x[:3]
    q0 = x[3:]
    T_m1_m2_test[:3,3] = p0
    T_m1_m2_test[:3,:3] = Rotation.from_quat(q0).as_matrix()

    test_err = multiview_pnp_error(pts3D_l, pts2D_l, K2, T_m1_m2_test, T_m2_c_l)
    return np.sum([np.sum(e) for e in test_err]) / len_subelems(test_err)

def pnp_error(pts3D,pts2D,rvecs,tvecs,K2):
    pts2D_reproj = cv2.projectPoints(pts3D,rvecs,tvecs,K2,None)[0].reshape(-1,2)
    e = np.linalg.norm(pts2D-pts2D_reproj,axis=1)    
    return e

def multiview_pnp_error(pts3D_l, pts2D_l, K2, T_m1_m2_test, T_m2_c_l):
    n = len(pts2D_l)
    e_l = [None for _ in range(n)]
    for i in range(n):
        nfeatures = pts2D_l[i].shape[0]
        if nfeatures > 0:
            pts3D_i = pts3D_l[i]
            pts2D_i = pts2D_l[i]
            T_m1_ci = T_m1_m2_test.dot(T_m2_c_l[i])
            tvecs,rvecs = matrix2poses(T_m1_ci)
            e_l[i] = pnp_error(pts3D_i,pts2D_i,rvecs,tvecs,K2)
        else:
            e_l[i] = np.zeros(0)
    return e_l

def poses2matrix(tvecs,rvecs):
    R_ = cv2.Rodrigues(rvecs)[0]
    R = R_.T
    C = -R_.T.dot(tvecs)      
    T_m_c = np.eye(4)
    T_m_c[:3,:3] = R
    T_m_c[:3,3] = C.reshape(-1)
    return T_m_c

def pose2matrix(pose):
    p = pose[:3]
    q = pose[3:]
    R = Rotation.from_quat(q)
    T_m_c = np.eye(4)
    T_m_c[:3,:3] = R.as_matrix()
    T_m_c[:3,3] = p
    return T_m_c

def pq2matrix(pq):
    p = pq[0]
    q = pq[1]
    R = Rotation.from_quat(q)
    T_m_c = np.eye(4)
    T_m_c[:3,:3] = R.as_matrix()
    T_m_c[:3,3] = p
    return T_m_c

def matrix2poses(T_m_c):
    R = T_m_c[:3,:3]
    C = T_m_c[:3,3]

    rvecs = cv2.Rodrigues(R.T)[0]
    tvecs = -R.T.dot(C.reshape(3,1))      

    return tvecs,rvecs    

def matrix2pose(T_m_c):
    R = T_m_c[:3,:3]
    p = T_m_c[:3,3]
    q = Rotation.from_matrix(R).as_quat()

    pose = np.concatenate([p,q])

    return pose


def len_subelems(l):
    return np.sum([len(sl) for sl in l])    

def quat2matrix(q):
    Rot = Rotation.from_quat(q)
    R = Rot.as_matrix()
    return R

def T2rt(T_m_c1):
    R = T_m_c1[:3,:3].T
    tvec = (- R @ T_m_c1[:3,3]).reshape(-1)
    rvec = cv2.Rodrigues(R.T)[0]
    return tvec,rvec

def matrix2quat(R):
    Rot = Rotation.from_matrix(R)
    q = Rot.as_quat()
    return q
    
def fun_rectify_views(I_p,fov):

    equ = E2P.Equirectangular(I_p)    # Load equirectangular image
    
    phi = 0
    width = fov*equ._width/360
    height = fov*equ._height/180

    theta_1 = 0
    theta_2  = 180
    theta_3 = 90
    theta_4  = 270

    T1 = np.eye(4)
    T2 = np.eye(4)
    T3 = np.eye(4)
    T4 = np.eye(4)

    T1[:3,:3] = cv2.Rodrigues(theta_1*np.pi/180*np.array([0,1,0]))[0]
    T2[:3,:3] = cv2.Rodrigues(theta_2*np.pi/180*np.array([0,1,0]))[0]
    T3[:3,:3] = cv2.Rodrigues(theta_3*np.pi/180*np.array([0,1,0]))[0]
    T4[:3,:3] = cv2.Rodrigues(theta_4*np.pi/180*np.array([0,1,0]))[0]

    I1,K1 = equ.GetPerspective(fov, theta_1, phi, height, width)        
    I2,_  = equ.GetPerspective(fov, theta_2, phi, height, width)        
    I3,_  = equ.GetPerspective(fov, theta_3, phi, height, width)        
    I4,_  = equ.GetPerspective(fov, theta_4, phi, height, width)        

    I_list = [I1,I2,I3,I4]
    T_list = [T1,T2,T3,T4]

    return I_list, T_list, K1
