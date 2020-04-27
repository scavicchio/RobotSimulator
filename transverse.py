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

## Create Parent-Child Map
parent_child_map = numpy.array([[0, 1], [1, 2], [0, 3], [3, 4], [4, 5], \
    [3, 6], [6, 7], [7, 8], [0, 9], [9, 10], [10, 11]])

## Print all the links
for i in range(p.getNumJoints(RoboBoi)):
    link = p.getJointInfo(RoboBoi, i)
    print(link)

print("Here")
## Let's try to move the robot legs so it can stand
## Try moving back legs
p.setJointMotorControl2(RoboBoi, 9, p.POSITION_CONTROL, 0.4)
p.setJointMotorControl2(RoboBoi, 6, p.POSITION_CONTROL, 0.4)   



a = 0.4 
b = 0.2
c = 0.01
omega1 = 0
omega2 = 2
omega3 = 0
omega4 = 2

## Simulate the thing
maxstep = 10000

all_target_1 = numpy.zeros((maxstep,2)) # joint 0
all_target_2 = numpy.zeros((maxstep,2)) # joint 3
all_target_3 = numpy.zeros((maxstep,2)) # joint 9
all_target_4 = numpy.zeros((maxstep,2)) # joint 6

for i in range (maxstep):

    p.stepSimulation()
## Do all our joint/link data handling before moving

    if i > 500:
        target1 = a + b*math.sin(c*i + omega1)
        target2 = a + b*math.sin(c*i + omega2)
        target3 = a + b*math.sin(c*i + omega3)
        target4 = a + b*math.sin(c*i + omega4)
        all_target_1[i,0] = target1
        all_target_2[i,0] = target2
        all_target_3[i,0] = target3
        all_target_4[i,0] = target4
        
        # TRANSVERSE MOVEMENT COMMANDS
        print("setting movement")
        p.setJointMotorControl2(RoboBoi, 2, p.POSITION_CONTROL, \
            targetPosition = (0.1 + 0.1*math.sin(0.1*i)))

        p.setJointMotorControl2(RoboBoi, 5, p.POSITION_CONTROL, \
            targetPosition = (0.1 + 0.1*math.sin(0.1*i)))

        p.setJointMotorControl2(RoboBoi, 8, p.POSITION_CONTROL, \
            targetPosition = (0.1 + 0.1*math.sin(0.1*i)))


    else:
        target1 = 0
        target2 = 0
        target3 = 0
        target4 = 0
        all_target_1[i] = target1
        all_target_2[i] = target2
        all_target_3[i] = target3
        all_target_4[i] = target4


    target1_speed, target2_speed, target3_speed, target4_speed = \
        p.getLinkStates(RoboBoi, [0, 3, 9, 6], computeLinkVelocity=1)
            

    link0_velocity_vec = target1_speed[6]
    link0_velocity_mag = numpy.linalg.norm(link0_velocity_vec)

    link3_velocity_vec = target2_speed[6]
    link3_velocity_mag = numpy.linalg.norm(link3_velocity_vec)

    link9_velocity_vec = target3_speed[6]
    link9_velocity_mag = numpy.linalg.norm(link9_velocity_vec)

    link6_velocity_vec = target4_speed[6]
    link6_velocity_mag = numpy.linalg.norm(link6_velocity_vec)
    all_target_1[i,1] = link0_velocity_mag
    all_target_2[i,1] = link3_velocity_mag
    all_target_3[i,1] = link9_velocity_mag
    all_target_4[i,1] = link6_velocity_mag 




    time.sleep(1./240.)


cubePos, cubeOrn = p.getBasePositionAndOrientation(RoboBoi)
times = numpy.arange(maxstep)
print(cubePos,cubeOrn)

plt.figure(1)
labels = ['Joint 0', 'Joint 3', 'Joint 9', 'Joint 6']
plt.plot(times, all_target_1[:,0], 'r+', label = 'Joint 0')
plt.plot(times, all_target_2[:,0], 'bx', label = 'Joint 3')
plt.plot(times, all_target_3[:,0], 'g.', label = 'Joint 9')
plt.plot(times, all_target_4[:,0], 'cd', label = 'Joint 6')
plt.legend()
plt.ylabel('Joint Configuration, Radians')
plt.xlabel('Simulation Time')
plt.title('Joint Angles')
plt.show()

plt.figure(2)
labels = ['Joint 0', 'Joint 3', 'Joint 9', 'Joint 6']
plt.plot(times, all_target_1[:,1], 'r+', label = 'Joint 0')
plt.plot(times, all_target_2[:,1], 'bx', label = 'Joint 3')
plt.plot(times, all_target_3[:,1], 'g.', label = 'Joint 9')
plt.plot(times, all_target_4[:,1], 'cd', label = 'Joint 6')
plt.legend()
plt.ylabel('Joint Linear Velocity, m/s')
plt.xlabel('Simulation Time')
plt.title('Joint Linear Speed')
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

