import pybullet as p
import time
import pybullet_data
import numpy
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation 
import random
import multiprocessing as mp
from multiprocessing import Pool
print("Number of processors: ", mp.cpu_count())

# this just gets the front left node for now
def calcDistance(RoboBoi):
	
	position, orientation = p.getBasePositionAndOrientation(RoboBoi);
	pTemp = numpy.square(position)
	distanceFromOrigin = numpy.sqrt(pTemp[0] + pTemp[1])

	return distanceFromOrigin

# speed of front left node
def currentSpeed(RoboBoi):
	linear, angular = p.getBaseVelocity(RoboBoi)
	speed =	numpy.sqrt(sum(numpy.square(linear)))
	return speed

def resetRobot(RoboBoi):
	for joint in p.getNumJoints(RoboBoi):
		p.resetJointState(RoboBoi,joint,0)



	return 


def randomizeParams(a,b,c,omega):
	maxOmega = 2*3.14159

	for i in range(len(a)):
		a[i] = random.uniform(-1,1)
		b[i] = random.uniform(-1,1)
		c[i] = random.uniform(-1,1)
		omega[i] = random.uniform(0, maxOmega)




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
INITSTATE = p.saveState()
originalPos, originalOrientation = p.getBasePositionAndOrientation(RoboBoi)


a = -0.4 
b = 0.2
c = 0.01
omega = 0

a = numpy.full(p.getNumJoints(RoboBoi),a)
b = numpy.full(p.getNumJoints(RoboBoi),b)
c = numpy.full(p.getNumJoints(RoboBoi),c)
omega = numpy.full(p.getNumJoints(RoboBoi),omega)

#previous state
a_best = a
b_best = b
c_best = c
omega_best = omega



## Simulate the thing
maxstep = 1000
evolutionIterations = 100

#measures
distanceTraveled = numpy.zeros(evolutionIterations)
finalSpeed = numpy.zeros(evolutionIterations)

target = numpy.zeros(p.getNumJoints(RoboBoi))
#all_target = numpy.zeros()



for k in range(evolutionIterations):
	print("Loop: " + str(k))
	
	#p.resetSimulation()
	p.restoreState(INITSTATE)

	#randomize the thing
	randomizeParams(a,b,c,omega)

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

	    time.sleep(1./120.)

	distanceTraveled[k] = calcDistance(RoboBoi)
	finalSpeed[k] = currentSpeed(RoboBoi)

	if (k != 0):
		if(distanceTraveled[k] > distanceTraveled[k-1]):
			print("found better solution on iteration: " + str(k))
			a_best = a
			b_best = b
			c_best = c
			omega_best = omega


plt.figure(1)
labels = ['Distance Traveled']
plt.plot(distanceTraveled)
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

