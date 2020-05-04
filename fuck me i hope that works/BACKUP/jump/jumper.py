import pybullet as p
import time
import pybullet_data
import numpy
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation 
import random
import csv
from multiprocessing import Pool
pool_size = 1
pool = Pool(pool_size)

# define worker function before a Pool is instantiated
def worker(item):
    try:
        api.my_operation(item)
    except:
        print('error with item')

def setTarget(j,i):
	target[j] = a[j] + b[j]*math.sin(c[j]*i + omega[j])
	return

def setJoint(j,i,RoboBoi):
	setTarget(j,i)
	p.setJointMotorControl2(RoboBoi,j,p.POSITION_CONTROL,target[j])

def getHeight(RoboBoi):
	position, orientation = p.getBasePositionAndOrientation(RoboBoi);
	return position[2]

# this just gets the front left node for now
def calcDistance(RoboBoi):
	
	position, orientation = p.getBasePositionAndOrientation(RoboBoi);
	pTemp = numpy.square(position)
	distanceFromOrigin = numpy.sqrt(pTemp[0] + pTemp[1])
	return distanceFromOrigin

def isTurnedOver(RoboBoi):
	sanity = 3.14159
	position, orientation = p.getBasePositionAndOrientation(RoboBoi);
	orin = p.getEulerFromQuaternion(orientation)
	if (abs(orin[0]) > sanity/2 or abs(orin[2]) > sanity):
			return True;

	return False;



# speed of front left node
def currentSpeed(RoboBoi):
	linear, angular = p.getBaseVelocity(RoboBoi)
	speed =	numpy.sqrt(sum(numpy.square(linear)))
	return speed

def resetRobot(RoboBoi):
	p.resetBasePositionAndOrientation(RoboBoi,originalPos, originalOrientation)

	for joint in range(p.getNumJoints(RoboBoi)):
		p.resetJointState(RoboBoi,joint,0)
		p.setJointMotorControl2(RoboBoi,joint,p.VELOCITY_CONTROL,0)
		#p.setJointMotorControl2(RoboBoi,joint,p.TORQUE_CONTROL,0)

	p.setJointMotorControl2(RoboBoi, 9, p.POSITION_CONTROL, -0.4)
	p.setJointMotorControl2(RoboBoi, 6, p.POSITION_CONTROL, -0.4) 
	return 

def hillClimbSingle(a,b,c,omega):
	omega_max = 2*3.14159
	a_max = 1
	b_max = 1
	c_max = 0.1
	a_min = 0
	b_min = 0
	c_min = 0
	omega_min = 0
	
	j = random.randrange(2)
	i = random.randrange(len(a))

	if (j == 0):
		a_tmp = a[i]
		num = random.uniform(-0.05,0.05)
		a[i] = a[i]+a[i]*num
		if (a[i] > a_max):
			a[i] = a_tmp-a_tmp*num
		elif (a[i] < a_min):
			a[i] = a_tmp+a_tmp*num

		if (i == 3): 
			a[0] == a[i]
		elif (i == 0):
			a[3] == a[i]
		elif (i == 8):
			a[5] = a[i]
		elif (i == 5):
			a[8] == a[i]

	elif (j == 1):
		b_tmp = b[i]
		num = random.uniform(-0.05,0.05)
		b[i] = b[i]+b[i]*num
		if (b[i] > b_max):
			b[i] = b_tmp-b_tmp*num
		elif (b[i] < b_min):
			b[i] = b_tmp+b_tmp*num

		if (i == 3): 
			b[0] == b[i]
		elif (i == 0):
			b[3] == b[i]
		elif (i == 8):
			b[5] = b[i]
		elif (i == 5):
			b[8] == b[i]

	else:
		c_tmp = c[i]
		num = random.uniform(-0.05,0.05)
		c[i] = c[i]+c[i]*num
		if (c[i] > c_max):
			c[i] = c_tmp-c_tmp*num
		elif (c[i] < c_min):
			c[i] = c_tmp+c_tmp*num

		if (i == 3): 
			c[0] == c[i]
		elif (i == 0):
			c[3] == c[i]
		elif (i == 8):
			c[5] = c[i]
		elif (i == 5):
			c[8] == c[i]

	return

def hillClimb(a,b,c,omega):
	omega_max = 2*3.14159
	a_max = 1
	b_max = 1
	c_max = 0.1
	a_min = 0
	b_min = 0
	c_min = 0
	omega_min = 0
	
	for i in range(len(a)):
		if (i == 3): 
			a[i] = a[0]
			b[i] = b[0]
			c[i] = c[0]
			omega[i] = omega[0]
		if (i == 8):
			a[i] = a[5]
			b[i] = b[5]
			c[i] = c[5]
			omega[i] = omega[5]
		else: 
			a_tmp = a[i]
			num = random.uniform(-0.05,0.05)
			a[i] = a[i]+a[i]*num
			if (a[i] > a_max):
				a[i] = a_tmp-a_tmp*num
			elif (a[i] < a_min):
				a[i] = a_tmp+a_tmp*num

			b_tmp = b[i]
			num = random.uniform(-0.05,0.05)
			b[i] = b[i]+b[i]*num
			if (b[i] > b_max):
				b[i] = b_tmp-b_tmp*num
			elif (b[i] < b_min):
				b[i] = b_tmp+b_tmp*num

			c_tmp = c[i]
			num = random.uniform(-0.05,0.05)
			c[i] = c[i]+c[i]*num
			if (c[i] > c_max):
				c[i] = c_tmp-c_tmp*random.uniform(-0.05,0.05)
			elif (c[i] < c_min):
				c[i] = c_tmp+c_tmp*num

	return

def randomizeParams(a,b,c,omega):
	maxOmega = 2*3.14159

	for i in range(len(a)):
		if (i == 3): 
			a[i] = a[0]
			b[i] = b[0]
			c[i] = c[0]
			omega[i] = omega[0]
		if (i == 8):
			a[i] = a[5]
			b[i] = b[5]
			c[i] = c[5]
			omega[i] = omega[5]
		else: 
			a[i] = random.uniform(0,1)
			b[i] = random.uniform(0,1)
			c[i] = random.uniform(0,.1)
			omega[i] = random.uniform(0, maxOmega)
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


a = -0.4 
b = 0.2
c = 0.01
omega = 0

a = numpy.full(p.getNumJoints(RoboBoi),a)
b = numpy.full(p.getNumJoints(RoboBoi),b)
c = numpy.full(p.getNumJoints(RoboBoi),c)
omega = numpy.full(p.getNumJoints(RoboBoi),omega)

#best state
a_best = a
b_best = b
c_best = c
omega_best = omega



## Simulate the thing
maxstep = 5000
evolutionIterations = 10000

intermediateResults = numpy.zeros([4,p.getNumJoints(RoboBoi)])
print(intermediateResults)

intermediateResultList = []

#measures
distanceTraveled = numpy.zeros(evolutionIterations)
finalSpeed = numpy.zeros(evolutionIterations)
averageSpeeds = numpy.zeros(evolutionIterations)
weightedScores = numpy.zeros(evolutionIterations)
target = numpy.zeros(p.getNumJoints(RoboBoi))
maxHeights = numpy.zeros(p.getNumJoints(RoboBoi))
#all_target = numpy.zeros()
bestIter = 0;

for k in range(evolutionIterations):
	print("Loop: " + str(k))
	average = 0
	maxH = 0
	# save every 1000 bots
	if (k % 1000 == 0):

		print("saving every 1000...")
		for i in range(len(a)):
			intermediateResults[0][i] = a[i]
			intermediateResults[1][i] = a[i]
			intermediateResults[2][i] = a[i]
			intermediateResults[3][i] = a[i]

		intermediateResultList.append(intermediateResults)

	#p.resetSimulation()
	#p.restoreState(INITSTATE)
	resetRobot(RoboBoi)

	#randomize the thing
	hillClimbSingle(a,b,c,omega)
	tuenover = False
	for i in range (maxstep):
		height = getHeight(RoboBoi)
		if (height >= maxH):
			maxH = height
		average += currentSpeed(RoboBoi)
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
					
	average /= maxstep
	distanceTraveled[k] = calcDistance(RoboBoi)
	averageSpeeds[k] = average
	finalSpeed[k] = currentSpeed(RoboBoi)
	score = (average*.25) + (calcDistance(RoboBoi))*.75
	weightedScores[k] = score
	maxHeights[k] = maxH
	if (k != 0):
		if(weightedScores[k] > weightedScores[bestIter]):
			bestIter = k
			print("found better solution on iteration: " + str(k))
			a_best = a
			b_best = b
			c_best = c
			omega_best = omega
		else:
			a = a_best
			b = b_best
			c = c_best
			omega = omega_best



plt.figure(1)
labels = ['Distance Traveled']
plt.plot(distanceTraveled)
plt.show()
p.disconnect()

print("OPTIMAL SETTINGS FOUND AT:")
print(a_best)
print(b_best)
print(c_best)
print(omega_best)



print("INTERMEDIATE RESULTS")
print(intermediateResultList)

pool.close()
pool.join()

with open("averageSpeedsHILLsaveFlipped.csv", 'w', newline='') as myfile:
     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
     wr.writerow(averageSpeeds)

with open("maxHeightsHILLsaveFlipped.csv", 'w', newline='') as myfile:
     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
     wr.writerow(maxHeights)

with open("weightedscoresHILLsaveFlipped.csv", 'w', newline='') as myfile:
     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
     wr.writerow(weightedScores)

with open("intermediateResultsHILLsaveFlipped.csv", 'w', newline='') as myfile:
     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
     wr.writerow(intermediateResults)

with open("finalResultHILLsaveFlipped.csv", 'w', newline='') as myfile:
     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
     wr.writerow(a_best)
     wr.writerow(b_best)
     wr.writerow(c_best)
     wr.writerow(omega_best)

with open("DistanceHILLsaveFlipped.csv", 'w', newline='') as myfile:
     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
     wr.writerow(distanceTraveled)


with open("finalSpeedHILLsaveFlipped.csv", 'w', newline='') as myfile:
     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
     wr.writerow(finalSpeed)



### REGULAR MOVEMENT COMMANDS
        # p.setJointMotorControl2(RoboBoi, 0, p.POSITION_CONTROL, \
        #     target1)

        # p.setJointMotorControl2(RoboBoi, 3, p.POSITION_CONTROL, \
        #     target2)


        # p.setJointMotorControl2(RoboBoi, 9, p.POSITION_CONTROL, \
        #     target3)

        # p.setJointMotorControl2(RoboBoi, 6, p.POSITION_CONTROL, \
        #     target4)

