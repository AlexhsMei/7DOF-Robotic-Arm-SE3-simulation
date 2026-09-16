import numpy as np # Linear Algebra
import matplotlib.pyplot as plt # Plotting
from mpl_toolkits.mplot3d import Axes3D # For 3D plotting
from math import pi, radians
from scipy.linalg import expm, logm
from time import sleep

def transform_point(pb, R, p = np.zeros((3, 1))):
    return p + R @ pb

def transform_vector(vb, R):
    return R @ vb

def homogeneous(R, p = np.zeros((3, 1))):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3:] = p

    return T
def hom_inv(T):
    
    R = T[:3, :3]
    t = T[:3, 3:]

    Tn = np.eye(4)
    Tn[:3, :3] = R.T
    Tn[:3, 3:] = -R.T @ t

    return Tn
def hat(vec):
    v = vec.reshape((3,))
    return np.array([
        [0., -v[2], v[1]],
        [v[2], 0., -v[0]],
        [-v[1], v[0], 0.]
    ])
def exp_rotation(p):
    phi = p.reshape((3, 1))
    theta = np.linalg.norm(phi)
    if theta < 1e-12:
        return np.eye(3, 3)
    a = phi / theta
    return np.eye(3) * np.cos(theta) + (1. - np.cos(theta)) * a @ a.T + np.sin(theta) * hat(a)

def log_rotation(R):
    theta = np.arccos(max(-1., min(1., (np.trace(R) - 1.) / 2.)))
    if theta < 1e-12:
        return np.zeros((3, 1))
    mat = R - R.T
    r = np.array([mat[2, 1], mat[0, 2], mat[1, 0]]).reshape((3, 1))
    return theta / (2. * np.sin(theta)) * r

def exp_pose(tau):
    theta = np.linalg.norm(tau[:3, :])

    R = np.eye(3)
    p = np.zeros((3, 1))

    if not np.isclose(theta, 0.):
        r = tau[:3, :] / theta
        rho = tau[3:, :] / theta
        rh = hat(r)
        R = exp_rotation(tau[:3, :])
        p = (np.eye(3) * theta + (1. - np.cos(theta)) * rh + (theta - np.sin(theta)) * (rh @ rh)) @ rho
    else:
        p = tau[3:, :]
    return np.block([[R, p], [np.zeros((1, 3)), 1.]])

def log_pose(T):
    R = T[:3, :3]
    p = T[:3, 3:]
    rt = log_rotation(R)
    theta = np.linalg.norm(rt)
    if np.allclose(theta, 0.):
        return np.block([[np.zeros((3, 1))], [p]])
    rh = hat(rt / theta)
    return np.block([[rt], [(1./theta * np.eye(3) - 0.5 * rh + (1./theta - 0.5 / np.tan(theta/2.)) * (rh @ rh)) @ p * theta]])

def s_to_hat(S):
    S = np.array(S).flatten()
    w = np.array(S[:3])
    v = np.array(S[3:]).reshape(3,1)

    w_hat = hat(w)

    upper = np.hstack((w_hat, v))
    lower = np.array([[0,0,0,0]])

    return np.vstack((upper, lower))

# Adjoint Representation
def Adj(T):
    R = T[:3, :3]
    p = T[:3, 3]
    Tadj = np.zeros((6, 6))
    Tadj[:3, :3] = R
    Tadj[3:, 3:] = R
    Tadj[3:, :3] = hat(p) @ R

    return Tadj
def vee(X):
    w_hat = X[:3, :3]
    omega = np.array([
        w_hat[2,1],
        w_hat[0,2],
        w_hat[1,0]
    ]).reshape(3,1)
    v = X[:3, 3].reshape(3,1)
    return np.vstack((omega, v))

def clip(a):
    a[0] = np.clip(a[0], -160, 160)
    a[1] = np.clip(a[1], -70, 115)
    a[2] = np.clip(a[2], -170, 170)
    a[3] = np.clip(a[3], -113, 75)
    a[4] = np.clip(a[4], -170, 170)
    a[5] = np.clip(a[5], -115, 115)
    a[6] = np.clip(a[6], -180, 180)
    return a

def RotX(theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    R = np.eye(3, 3)
    R[1, 1] = ct
    R[1, 2] = -st
    R[2, 1] = st
    R[2, 2] = ct
    return R

def RotY(theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    R = np.eye(3, 3)
    R[0, 0] = ct
    R[0, 2] = st
    R[2, 0] = -st
    R[2, 2] = ct
    return R

def RotZ(theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    R = np.eye(3, 3)
    R[0, 0] = ct
    R[0, 1] = -st
    R[1, 0] = st
    R[1, 1] = ct
    return R

def compute_Jw(expS, S):
    Jw = []
    for i in range(len(S)):
        if i==0:
            Jw.append(S[i].reshape(6, 1))
        else:
            J=np.eye(4)
            for j in range(i): 
                J = J @ expS[j]
            Jw.append(Adj(J) @ S[i].reshape(6, 1))
    Jw_mat = np.hstack(Jw)
    return Jw_mat

def calculate_coeffs(ts, tg, Vs, Vg, T=1):
    c0 = np.copy(ts)
    c1 = np.copy(Vs)
    c2 = 3.*tg/(T**2) - 3.*ts/(T**2) - 2.*Vs/T - Vg/T
    c3 = -2.*tg/(T**3) + 2.*ts/(T**3) + Vs/(T**2) + Vg/(T**2)

    return c0, c1, c2, c3

def twist(wx=0, wy=0, wz=0, vx=0, vy=0, vz=0):
    w = np.array([[wx], [wy], [wz]])
    v = np.array([[vx], [vy], [vz]])
    Vs = np.vstack((w, v))
    return Vs

def loadList(list=1):
    angleslist = []
  
    with open(f"angleslist{list}.txt", "r") as f:
            for line in f:
                angleslist.append(np.fromstring(line, sep=" "))

    return angleslist