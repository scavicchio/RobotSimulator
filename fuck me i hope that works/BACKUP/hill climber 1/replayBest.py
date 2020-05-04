import pybullet as p
import time
import pybullet_data
import numpy
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation 
import random
import csv


with open('finalResultHILLsaveFlipped.csv', newline='') as csvfile:
    data = list(csv.reader(csvfile))

def setTarget(j,i):
	target[j] = a[j] + b[j]*math.sin(c[j]*i + omega[j])
	return

def setJoint(j,i,RoboBoi):
	setTarget(j,i)
	p.setJointMotorControl2(RoboBoi,j,p.POSITION_CONTROL,target[j])

def isTurnedOver(RoboBoi):
	sanity = 3.14159
	position, orientation = p.getBasePositionAndOrientation(RoboBoi);
	orin = p.getEulerFromQuaternion(orientation)
	if (abs(orin[0]) > sanity/2 or abs(orin[2]) > sanity):
			return True;

	return False;

def resetRobot(RoboBoi):
	p.resetBasePositionAndOrientation(RoboBoi,originalPos, originalOrientation)

	for joint in range(p.getNumJoints(RoboBoi)):
		p.resetJointState(RoboBoi,joint,0)
		p.setJointMotorControl2(RoboBoi,joint,p.VELOCITY_CONTROL,0)
		#p.setJointMotorControl2(RoboBoi,joint,p.TORQUE_CONTROL,0)

	p.setJointMotorControl2(RoboBoi, 9, p.POSITION_CONTROL, -0.4)
	p.setJointMotorControl2(RoboBoi, 6, p.POSITION_CONTROL, -0.4) 
	return 

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

a = numpy.zeros(p.getNumJoints(RoboBoi))
b = numpy.zeros(p.getNumJoints(RoboBoi))
c = numpy.zeros(p.getNumJoints(RoboBoi))
omega = numpy.zeros(p.getNumJoints(RoboBoi))
resetRobot(RoboBoi)
for i in range (p.getNumJoints(RoboBoi)):
	a[i] = float(data[0][i])
	b[i] = float(data[1][i])
	c[i] = float(data[2][i])
	omega[i] = float(data[3][i])

print(a)
print(b)
print(c)
print(omega)
print("-------------------------------------")
print(data)

## Simulate the thing
maxstep = 10000
target = numpy.zeros(p.getNumJoints(RoboBoi))

for i in range (maxstep):
	p.stepSimulation()
## Do all our joint/link data handling before moving
	if i > 500:
		tuenover = isTurnedOver(RoboBoi)
		if (tuenover):
			print("FLIPPED")
			break
		else:
			for j in range (p.getNumJoints(RoboBoi)):
				#pool.apply_async(worker, setJoint(j,i,RoboBoi))
				setJoint(j,i,RoboBoi)
	time.sleep(1./240.)