import pybullet as p
import pybullet_data
import time
import numpy as np
import os
from functions import loadList
from hw2 import a, a1, a2
from math import pi, dist

os.chdir("C:\\Users\\meint\\Desktop\\Alexis")


physicsClient = p.connect(p.GUI)           
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
timeStep = 0.05
p.setTimeStep(timeStep)
p.resetDebugVisualizerCamera(
    cameraDistance=0.6,      # smaller = zoom in
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.1]
)

# Load robot
robot_urdf = "myarm_300_pi.urdf" 
start_pos = [-0.05-0.025, -0.025, 0]
start_orientation = p.getQuaternionFromEuler([0,0,0])
robot_id = p.loadURDF(robot_urdf, start_pos, start_orientation, useFixedBase=True)

# ----------------- LOAD OBJECTS -----------------
cube = 1
if cube:
    cube_size = 0.025  # half-extents
    cubeCollision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[cube_size, cube_size, cube_size])
    cubeVisual = p.createVisualShape(p.GEOM_BOX, halfExtents=[cube_size, cube_size, cube_size], rgbaColor=[0,0,1,1])
    cubeStartPos = [0.175, 0.025, cube_size] #[0.175, 0.05, cube_size]  # place on top of plane
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    cubeId = p.createMultiBody(
        baseMass=0.1,  # mass >0 if you want it to fall
        baseCollisionShapeIndex=cubeCollision,
        baseVisualShapeIndex=cubeVisual,
        basePosition=cubeStartPos,
        baseOrientation=cubeStartOrientation
    )

box = 1
if box:
    box_width = 0.025
    box_length = 0.1
    box_height = box_length
    #boxCollision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_length, box_width, box_height])
    boxVisual = p.createVisualShape(p.GEOM_BOX, halfExtents=[box_length, box_width, box_height], rgbaColor=[1,0,0,1])
    boxStartPos = [10e-02, 15e-02, 10e-02]
    boxStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    boxId = p.createMultiBody(
        baseMass=0,  # mass >0 if you want it to fall
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=boxVisual,
        basePosition=boxStartPos,
        baseOrientation=boxStartOrientation
    )

# ---------------------------
# Get controllable joints
# ---------------------------
joint_indices = [i for i in range(p.getNumJoints(robot_id))
                 if p.getJointInfo(robot_id, i)[2] != p.JOINT_FIXED]
print(joint_indices)
lastLink = joint_indices[-1]
grasped = False
constraint_id = None
parentFramePos = [0.008, 0, 0.1]        # translation from last link  to cube (in link frame)
parentFrameOrn = p.getQuaternionFromEuler([0, -pi/2, 0])

n_joints = len(joint_indices)
current_angles = np.zeros(n_joints)  # absolute angles

# ---------------------------
# Run the simulation following trajectory
# ---------------------------

os.chdir(os.path.dirname(os.path.abspath(__file__)))

fullTraj = 1
if fullTraj:
    angleslist = loadList(1)
    for i, joint_index in enumerate(joint_indices):
        p.resetJointState(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            targetValue=a[i]
        )
    for _ in range(25):
        p.stepSimulation()
        time.sleep(timeStep/2)


    for angles in angleslist: 
        for i, joint_index in enumerate(joint_indices): 
            p.setJointMotorControl2( 
                bodyIndex=robot_id, 
                jointIndex=joint_index, 
                controlMode=p.POSITION_CONTROL, #VELOCITY_CONTROL??
                targetPosition=angles[i], 
                maxVelocity = 3,
                force=500) 

        # Step simulation and wait
        for _ in range(3):  
            p.stepSimulation() 
            time.sleep(timeStep/2)
    for _ in range(10):  
        p.stepSimulation()
        time.sleep(timeStep)

    grasped = dist(p.getLinkState(robot_id, 6)[0], p.getBasePositionAndOrientation(cube)[0]) <= 0.15  # basic distance to check for errors

    if grasped:
        constraint_id = p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=lastLink,
        childBodyUniqueId=cubeId,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0,0,0],
        parentFramePosition=parentFramePos,
        childFramePosition=[0,0,0],
        parentFrameOrientation=parentFrameOrn,
        childFrameOrientation=[0,0,0,1]
    )
    
    time.sleep(2)
    for angles in reversed(angleslist): 
        for i, joint_index in enumerate(joint_indices): 
            p.setJointMotorControl2( 
                bodyIndex=robot_id, 
                jointIndex=joint_index, 
                controlMode=p.POSITION_CONTROL, 
                targetPosition=angles[i], 
                maxVelocity = 3,
                force=500) 

        # Step simulation and wait
        for _ in range(3):
            p.stepSimulation() 
            time.sleep(timeStep/2)
    for _ in range(20):  
        p.stepSimulation()
        time.sleep(timeStep)


    angleslist = loadList(2)
    time.sleep(2)
    for angles in angleslist: 
        angles[0] -= pi/2
        for i, joint_index in enumerate(joint_indices): 
            p.setJointMotorControl2( 
                bodyIndex=robot_id, 
                jointIndex=joint_index, 
                controlMode=p.POSITION_CONTROL, 
                targetPosition=angles[i], 
                maxVelocity = 3,
                force=500) 

        # Step simulation and wait
        for _ in range(5):  
            p.stepSimulation() 
            time.sleep(timeStep/2)

    p.setJointMotorControl2( 
                bodyIndex=robot_id, 
                jointIndex=0, 
                controlMode=p.POSITION_CONTROL, 
                targetPosition = p.getJointState(robot_id, 0)[0] + pi/2, 
                maxVelocity = 0.2,
                force=100) 
    
    p.setJointMotorControl2( 
                bodyIndex=robot_id, 
                jointIndex=6, 
                controlMode=p.POSITION_CONTROL, 
                targetPosition = p.getJointState(robot_id, 6)[0] + pi/4, 
                force=500) 
    
    

if not fullTraj:
    grasped = True
    angleslist = loadList(3)
    for i, joint_index in enumerate(joint_indices):
        p.resetJointState(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            targetValue=a[i]
        )
    p.stepSimulation()

    if grasped:
        constraint_id = p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=lastLink,
        childBodyUniqueId=cubeId,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0,0,0],
        parentFramePosition=parentFramePos,
        childFramePosition=[0,0,0],
        parentFrameOrientation=parentFrameOrn,
        childFrameOrientation=[0,0,0,1]
    )

    for angles in angleslist: 
        for i, joint_index in enumerate(joint_indices): 
            p.setJointMotorControl2( 
                bodyIndex=robot_id, 
                jointIndex=joint_index, 
                controlMode=p.POSITION_CONTROL, 
                targetPosition=angles[i], 
                force=500) 

        # Step simulation and wait
        p.stepSimulation() 
        time.sleep(timeStep)

# ---------------------------
# Keep GUI alive
# ---------------------------
print("Trajectory finished.")
while True:
    p.stepSimulation()
    time.sleep(timeStep)
    if p.getJointState(robot_id, 0)[1] <= 0.01 and grasped:
        time.sleep(0.5)
        grasped = False
        p.removeConstraint(constraint_id)
        p.setGravity(0, 0, -1)
