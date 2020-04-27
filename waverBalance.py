import pybullet as p
import time
import pybullet_data
import numpy
import math
from scipy.spatial.transform import Rotation 



## Loading
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
RobotStartPosition = [0,0, 0.75]
RobotStartOrientation = p.getQuaternionFromEuler([0,0,0])
RoboBoi = p.loadURDF("Robot.urdf",RobotStartPosition, RobotStartOrientation)
# print("Number Joints", p.getNumJoints(RoboBoi))

# Create Parent-Child Map
parent_child_map = numpy.array([[0, 1], [1, 2], [0, 3], [3, 4], [4, 5], \
    [3, 6], [6, 7], [7, 8], [0, 9], [9, 10], [10, 11]])

for i in range(p.getNumJoints(RoboBoi)):
    link = p.getJointInfo(RoboBoi, i)
    print(link)

## Let's try to move the robot legs so it can stand
## Try moving back legs
p.setJointMotorControl2(RoboBoi, 9, p.POSITION_CONTROL, \
    -0.4)
p.setJointMotorControl2(RoboBoi, 6, p.POSITION_CONTROL, \
    -0.4)   
# p.setJointMotorControl2(RoboBoi, 10, p.POSITION_CONTROL, \
#     -0.25)
# p.setJointMotorControl2(RoboBoi, 7, p.POSITION_CONTROL, \
#     -0.25)   



a = 0.4 
b = 0.15
c = 0.01
omega1 = 0
omega2 = 2
omega3 = 0
omega4 = 2
## Simulate the thing
for i in range (10000):
    p.stepSimulation()

    if i > 500:
        target1 = a + b*math.sin(c*i + omega1)
        target2 = a + b*math.sin(c*i + omega1)
        target3 = a + b*math.sin(c*i + omega1)
        target4 = a + b*math.sin(c*i + omega1)

        p.setJointMotorControl2(RoboBoi, 0, p.POSITION_CONTROL, \
            target1)
        # p.setJointMotorControl2(RoboBoi, 2, p.POSITION_CONTROL, \
        #     target1)
        p.setJointMotorControl2(RoboBoi, 3, p.POSITION_CONTROL, \
            target2)
            
        # have to change URDF lines 216, 217 to allow for joint movement
        # p.setJointMotorControl2(RoboBoi, 2, p.POSITION_CONTROL, \
        #     0.05)


        p.setJointMotorControl2(RoboBoi, 9, p.POSITION_CONTROL, \
            target3)
        # p.setJointMotorControl2(RoboBoi, 10, p.POSITION_CONTROL, \
        #     target3)
        p.setJointMotorControl2(RoboBoi, 6, p.POSITION_CONTROL, \
            target4)
        # p.setJointMotorControl2(RoboBoi, 7, p.POSITION_CONTROL, \
        #     target4)




    time.sleep(1./240.)


cubePos, cubeOrn = p.getBasePositionAndOrientation(RoboBoi)

print(cubePos,cubeOrn)


p.disconnect()
