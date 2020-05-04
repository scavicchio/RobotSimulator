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
results_pos = numpy.zeros((p.getNumJoints(RoboBoi),maxstep))
results_speed_1 = numpy.zeros(maxstep)
results_speed_2 = numpy.zeros(maxstep)
results_speed_3 = numpy.zeros(maxstep)
results_speed_4 = numpy.zeros(maxstep)

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
				results_pos[j][i] = target[j] = a[j] + b[j]*math.sin(c[j]*i + omega[j])



	target1_speed, target2_speed, target3_speed, target4_speed = p.getLinkStates(RoboBoi, [0, 3, 9, 6], computeLinkVelocity=1)
            

	link0_velocity_vec = target1_speed[6]
	link0_velocity_mag = numpy.linalg.norm(link0_velocity_vec)

	link3_velocity_vec = target2_speed[6]
	link3_velocity_mag = numpy.linalg.norm(link3_velocity_vec)

	link9_velocity_vec = target3_speed[6]
	link9_velocity_mag = numpy.linalg.norm(link9_velocity_vec)

	link6_velocity_vec = target4_speed[6]
	link6_velocity_mag = numpy.linalg.norm(link6_velocity_vec)
	results_speed_1[i] = link0_velocity_mag
	results_speed_2[i] = link3_velocity_mag
	results_speed_3[i] = link9_velocity_mag
	results_speed_4[i] = link6_velocity_mag 
	time.sleep(1./2400.)

times = numpy.arange(maxstep)

plt.figure(1)
labels = ['Joint 0', 'Joint 3', 'Joint 9', 'Joint 6']
plt.plot(times, results_pos[0,:], 'r+', label = 'Joint 0')
plt.plot(times, results_pos[3,:], 'bx', label = 'Joint 3')
plt.plot(times, results_pos[9,:], 'g.', label = 'Joint 9')
plt.plot(times, results_pos[6,:], 'cd', label = 'Joint 6')
plt.legend()
plt.ylabel('Joint Configuration, Radians')
plt.xlabel('Simulation Time')
plt.title('Joint Angles')
plt.show()


plt.figure(2)
labels = ['Joint 0', 'Joint 3', 'Joint 9', 'Joint 6']
plt.plot(times, results_speed_1, 'r+', label = 'Joint 0')
plt.plot(times, results_speed_2, 'bx', label = 'Joint 3')
plt.plot(times, results_speed_3, 'g.', label = 'Joint 9')
plt.plot(times, results_speed_4, 'cd', label = 'Joint 6')
plt.legend()
plt.ylabel('Joint Linear Velocity, m/s')
plt.xlabel('Simulation Time')
plt.title('Joint Linear Speed')
plt.show()