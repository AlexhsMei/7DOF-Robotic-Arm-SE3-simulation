from functions import *

# Setting origin


origin=[0,0,0]
x_axis=[1,0,0]
y_axis=[0,1,0]
z_axis=[0,0,1]
axes=[x_axis, y_axis, z_axis]
np.set_printoptions(suppress=True)

# Transformation matrix of cube at its origin


x_cube=17.5
y_cube=2.5
z_cube=2.5
pc = np.array([[x_cube], [y_cube], [z_cube]])
T_wc = homogeneous(np.eye(3), pc)

# Transformation matrix of red object at its origin


x_red=10
y_red=12.5
z_red=10
pr = np.array([[x_red], [y_red], [z_red]])
T_wr = homogeneous(np.eye(3), pr)

# Transformation matrix of effector relative to world when grasping the cube

R_w_eff2 = RotY(pi-pi/2)
p_w_eff2 = np.array([[17.5+5, 2.5, 5]]).T
p_w_eff2 = np.array([[15+5+2.5, 2.5+2.5, 2.5]]).T

T_w_eff2 = homogeneous(R_w_eff2, p_w_eff2)
# T_w_eff2 = np.round(T_w_eff2, 2)

# Transformation matrix of the effector relative to the cube


T_eff_cube = hom_inv(T_w_eff2) @ T_wc


# Transformation matrix of CUBE when just OUT OF THE HOLE


R_cube_out2 = RotZ(-pi/4) @ RotX(-pi/2) #@RotZ(-pi/4)
p_cube_out2 = np.array([[10, 7.5, 10]]).T
T_cube_out2 = homogeneous(R_cube_out2, p_cube_out2)
# T_cube_out2 = np.round(T_cube_out2, 2)

# Transformation matrix of CUBE when IN THE HOLE


R_cube_in2 = R_cube_out2
p_cube_in2 = p_cube_out2 + np.array([[0, 5, 0]]).T
T_cube_in2 = homogeneous(R_cube_in2, p_cube_in2)
# T_cube_in2 = np.round(T_cube_in2, 2)


# Transformation matrix of effector when grasping the cube outside the hole


# T_w_eff_out = T_cube_out2 @ hom_inv(T_eff_cube)
# print(T_w_eff_out)

R_w_eff_out = RotZ(-3*pi/4) @ RotX(pi/2)
R_w_eff_out = RotX(-pi/2)
p_w_eff_out = np.array([[15+2.5, 10, 10]]).T
T_w_eff_out = homogeneous(R_w_eff_out, p_w_eff_out)
# T_w_eff_out = np.round(T_w_eff_out, 2)

# Transformation matrix of base to effector
R_base_eff = np.eye(3)
p_base_eff = np.array([[0.8, 0, 55.38]]).T
T_base_eff = homogeneous(R_base_eff, p_base_eff)
# T_base_eff = np.round(T_base_eff)

# Screw Axes of Robot

# S0
d0=0
r0 = np.array([[0, 0, 1]])
q0 = np.array([[0, 0, 0]])
p0 = -np.cross(r0, q0)
# print(p0)
S0 = np.hstack((r0, p0))

#S1
d1=16.95
r1 = r0
q1 = np.array([[0, 0, d1]])
p1 = -np.cross(r1, q1)
# print(p1)
S1 = np.hstack((r1, p1))

#S2
d2=d1
r2 = np.array([[0, 1, 0]])
q2 = np.array([[0, 0, d2]])
p2 = -np.cross(r2, q2)
S2 = np.hstack((r2, p2))

#S3
d3=d2+6+5.55
r3 = r0
q3 = np.array([[0, 0, d3]])
p3 = -np.cross(r3, q3)
S3 = np.hstack((r3, p3))

#S4
d4=d3
r4 = -r2
q4 = np.array([[0, 0, d4]])
p4 = -np.cross(r4, q4)
S4 = np.hstack((r4, p4))

#S5
d5=d4+7.5+5.28
r5 = r0
q5 = np.array([[0, 0, d5]])
p5 = -np.cross(r5, q5)
S5 = np.hstack((r5, p5))

#S6
d6=d5
r6 = r4
q6 = np.array([[0, 0, d6]])
p6 = -np.cross(r6, q6)
S6 = np.hstack((r6, p6))

#S7
d7=d6+6.6
r7 = r0
q7 = np.array([[0, 0, d7]])
p7 = -np.cross(r7, q7)
S7 = np.hstack((r7, p7))

#Sef
d_eff=[0.8, 0, d7+7.5]
r_eff = r0
q_eff = np.array([d_eff])
p_eff = -np.cross(r_eff, q_eff)
S_eff = np.hstack((r_eff, p_eff))
# print(d0, d1, d2, d3, d4, d5, d6, d7)
# print(r0, r1, r2, r3, r4, r5, r6, r7)

# exponentials for forward kinematics


#gwnies joints
a0, a1, a2, a3, a4, a5, a6, a7 = [0 for i in range(8)]
 
S = [S1, S2, S3, S4, S5, S6, S7]

a = np.array([a1, a2, a3, a4, a5, a6, a7])
a1 = np.array([0.2600291611950587, 1.753087471643317, 0.26629046503525766, -1.744087596705986, 0.22176977478215992, 1.9794014290417326, -0.1781377332152562])
a2 = np.array([-1.2820644025498162, -0.03016311291972594, 0.0163355659390607, -0.057426690923012114, -0.010865026390405944, 0.027334303178858745, 1.2766036039333968])
S_hat = [s_to_hat(S1), s_to_hat(S2), s_to_hat(S3), s_to_hat(S4), s_to_hat(S5), s_to_hat(S6), s_to_hat(S7)]


expS = [exp_pose((S[i]*a[i]).reshape(6,1)) for i in range(len(S))]
# print(expS)

# Desired transformation matrix from exp, θ


T_forward = expS[0] @ expS[1] @ expS[2] @ expS[3] @ expS[4] @ expS[5] @ expS[6] @ T_base_eff
# T_forward = np.round(T_forward, 2)


# Jacobians


Jw_mat = compute_Jw(expS, S)
Jb = Adj(np.linalg.inv(T_forward)) @ Jw_mat

rank = np.linalg.matrix_rank(Jw_mat)
