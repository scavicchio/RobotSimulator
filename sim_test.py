import pybullet as p
import time
import pybullet_data

# useful pybullet functions for us

#https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/preview#heading=h.4mzaihtg8vfd

# getBaseVelocity & resetBaseVelocity
# you get access to the linear and angular velocity of the body 
# (we can use this to measure the average speed) 
# returns a list of two vector3 values (3 floats in a list) representing xyz and wx wy wz
# for resetBaseVelocity the input parameters are (objectUniqueID - int (required), linearVelocity - vec3 (optional), angularVelocity - vec3 (optional), physicsclientid - int NOT NEEDED)

# saveWorld
# snapshot of the current world as a pybullet python file stored on the server
# input argument is (fileName - string)

# saveState (saves on memory so not permanant), saveBullet (saves on disk), restoreState
# does what they sounds like - input is (filename - string). 

#calculateInverseDynamics(2)
#calculateInverseDynamics will compute the forces needed to reach the given joint accelerations, starting from specified joint positions and velocities. The inverse dynamics is computed using the recursive Newton Euler algorithm (RNEA).


#The calculateInverseDynamics input parameters are:
#required
#bodyUniqueId
#int
#body unique id, as returned by loadURDF etc.
#required
#objPositions
#list of float
#joint positions (angles) for each degree of freedom (DoF). Note that fixed joints have 0 degrees of freedom. The base is skipped/ignored in all cases (floating base and fixed base).
#required
#objVelocities
#list of float
#joint velocities for each degree of freedom (DoF)
#required
#objAccelerations
#list of float
#desired joint accelerations for each degree of freedom (DoF)
#optional
#physicsClientId
#int
#if you are connected to multiple servers, you can pick one.
#calculateInverseDynamics returns a list of joint forces for each degree of freedom.
#Note that when multidof (spherical) joints are involved, the calculateInverseDynamics uses a different code path and is a bit slower. Also note that calculateInverseDynamics ignores the joint/link damping, while forward dynamics (in stepSimulation) includes those damping terms. So if you want to compare the inverse dynamics and forward dynamics, make sure to set those damping terms to zero using changeDynamics with jointDamping and link damping through linearDamping and angularDamping.


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("Robot.urdf",cubeStartPos, cubeStartOrientation)



# THIS IS WHERE THE MAGIC HAPPENS
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)

p.disconnect()
