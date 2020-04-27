import pybullet as p
import time
import pybullet_data
import numpy
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation 



## Loading
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
RobotStartPosition = [0,0, 0.75]
RobotStartOrientation = p.getQuaternionFromEuler([0,0,0])
RoboBoi = p.loadURDF("Robot.urdf",RobotStartPosition, RobotStartOrientation)

# Create Parent-Child Map
parent_child_map = numpy.array([[0, 1], [1, 2], [0, 3], [3, 4], [4, 5], \
    [3, 6], [6, 7], [7, 8], [0, 9], [9, 10], [10, 11]])

## Print all the links
for i in range(p.getNumJoints(RoboBoi)):
    link = p.getJointInfo(RoboBoi, i)
    print(link)

print("Here")
## Let's try to move the robot legs so it can stand
## Try moving back legs
p.setJointMotorControl2(RoboBoi, 9, p.POSITION_CONTROL, -0.4)
p.setJointMotorControl2(RoboBoi, 6, p.POSITION_CONTROL, -0.4)   


a = numpy.zeros(p.getNumJoints(RoboBoi))
b = numpy.zeros(p.getNumJoints(RoboBoi))
c = numpy.zeros(p.getNumJoints(RoboBoi))
omega = numpy.zeros(p.getNumJoints(RoboBoi))

a[:] = 0.4 
b[:] = 0.2
c[:] = 0.01
omega[:] = 0

## Simulate the thing
maxstep = 10000

target = numpy.zeros(p.getNumJoints(RoboBoi))
#all_target = numpy.zeros()
for i in range (maxstep):

    p.stepSimulation()
## Do all our joint/link data handling before moving

    if i > 500:
        for j in range (p.getNumJoints(RoboBoi)):
            target[j] = a[j] + b[j]*math.sin(c[j]*i + omega[j])
          #  all_target[j][i] = target1
            p.setJointMotorControl2(RoboBoi,j,p.POSITION_CONTROL,target[j])

    else:
        for j in range (p.getNumJoints(RoboBoi)):
            target[j] = a[j] + b[j]*math.sin(c[j]*i + omega[j])
           # all_target[j][i] = target1

    time.sleep(1./240.)


cubePos, cubeOrn = p.getBasePositionAndOrientation(RoboBoi)
times = numpy.arange(maxstep)
print(cubePos,cubeOrn)

plt.figure(1)
labels = ['Joint 0']
plt.plot(times, all_target_1[:], 'r+', label = 'Joint 0')
plt.legend()
plt.ylabel('Joint Configuration, Radians')
plt.xlabel('Simulation Time')
plt.title('Joint Angles')
plt.show()

p.disconnect()

### REGULAR MOVEMENT COMMANDS
        # p.setJointMotorControl2(RoboBoi, 0, p.POSITION_CONTROL, \
        #     target1)

        # p.setJointMotorControl2(RoboBoi, 3, p.POSITION_CONTROL, \
        #     target2)


        # p.setJointMotorControl2(RoboBoi, 9, p.POSITION_CONTROL, \
        #     target3)

        # p.setJointMotorControl2(RoboBoi, 6, p.POSITION_CONTROL, \
        #     target4)

