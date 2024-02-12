import pybullet as p
import time
import math
import random
import pybullet_data
from datetime import datetime
import numpy as np

######################################################### Simulation Setup ############################################################################

clid = p.connect(p.GUI)
if (clid < 0):
	p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf", [0, 0, -1])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
sawyerId = p.loadURDF("./sawyer_robot/sawyer_description/urdf/sawyer.urdf", [0, 0, 0], [0, 0, 0, 3],
					  useFixedBase=1)  # load sawyer robot


tableId = p.loadURDF("./table/table.urdf", [1.1, 0.000000, -0.3],
					 p.getQuaternionFromEuler([(math.pi / 2), 0, (math.pi / 2)]), useFixedBase=1, flags=8)


######################################################### Load Object Here!!!!#############################################################################

# load object, change file name to load different objects
# p.loadURDF(finlename, position([X,Y,Z]), orientation([a,b,c,d]))
# Example:
#objectId = p.loadURDF("random_urdfs/001/001.urdf", [1.25 ,0.25,-0.1], p.getQuaternionFromEuler([0,0,1.56])) # pi*0.5

xpos = 1.23
ypos = 0
ang = 3.14 * 0.5
orn = p.getQuaternionFromEuler([0, ang, 0])

object_path ="random_urdfs/065/065.urdf"
objectId = p.loadURDF(object_path, xpos, ypos, -0.03, orn[0], orn[1], orn[2], orn[3])


######################################################### Load tray Here!!!!#############################################################################

tray_x =  1.15
tray_y =  0.45615519873065125

trayId = p.loadURDF("./tray/tray.urdf", [tray_x, tray_y, 0], [0, 0, 0, 3])



###########################################################################################################################################################


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.resetBasePositionAndOrientation(sawyerId, [0, 0, 0], [0, 0, 0, 1])

sawyerEndEffectorIndex = 16
numJoints = p.getNumJoints(sawyerId)  # 65 with ar10 hand

# useRealTimeSimulation = 0
# p.setRealTimeSimulation(useRealTimeSimulation)
# p.stepSimulation()
# all R joints in robot
js = [3, 4, 8, 9, 10, 11, 13, 16, 21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36, 37, 39, 40, 41, 44, 45, 46, 48, 49, 50,
	  53, 54, 55, 58, 61, 64]
# lower limits for null space
ll = [-3.0503, -5.1477, -3.8183, -3.0514, -3.0514, -2.9842, -2.9842, -4.7104, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17,
	  0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.85, 0.34,
	  0.17]
# upper limits for null space
ul = [3.0503, 0.9559, 2.2824, 3.0514, 3.0514, 2.9842, 2.9842, 4.7104, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57,
	  0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 2.15, 1.5, 1.5]
# joint ranges for null space
jr = [0, 0, 0, 0, 0, 0, 0, 0, 1.4, 1.4, 1.4, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4,
	  0, 1.4, 1.4, 0, 1.3, 1.16, 1.33]
# restposes for null space
rp = [0] * 35
# joint damping coefficents
jd = [1.1] * 35


######################################################### Inverse Kinematics Function ##########################################################################

# Finger tip ID: index:51, mid:42, ring: 33, pinky:24, thumb 62
# Palm ID: 20
# move palm (center point) to reach the target postion and orientation
# input: targetP --> target postion
#        orientation --> target orientation of the palm
# output: joint positons of all joints in the robot
#         control joint to correspond joint position
def palmP(targetP, orientation):
	jointP = [0] * 65
	jointPoses = p.calculateInverseKinematics(sawyerId, 19, targetP, targetOrientation=orientation, jointDamping=jd)
	j = 0
	for i in js:
		jointP[i] = jointPoses[j]
		j = j + 1

	for i in range(p.getNumJoints(sawyerId)):
		p.setJointMotorControl2(bodyIndex=sawyerId,
								jointIndex=i,
								controlMode=p.POSITION_CONTROL,
								targetPosition=jointP[i],
								targetVelocity=0,
								force=50000,
								positionGain=0.03,
								velocityGain=1)
	return jointP


######################################################### Hand Direct Control Functions ##########################################################################

# control the lower joint and middle joint of pinky finger, range both [0.17 - 1.57]

#hand = [21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36 ,37, 39, 40, 41, 44, 45, 46, 48, 49, 50, 53, 54, 55, 58, 61, 64]
# handReading = [0.2196998776260993, 0.9841056922424084, 0.16991782178342238, 0.21967883521345558, 0.9846229478397389, 0.1699958046620013, 0.5711534611694058, 0.5914229523765463, 0.16999954970542672, 0.573730600144428, 0.5902151809391006, 0.17000660753266578, 0.9359158730554522, 0.265116872922352, 0.170003190706592, 0.9361250259528252, 
# 0.2652466938834658, 0.17003347470289248, 0.9068051254489781, 0.2490975329073341, 0.17008149880963058, 0.9066050389575453, 0.2502858674912193, 0.16999999999999976, 
# 1.5698468053021237, 0.34006621802344955, 0.3400508342876441]

def pinkyF(lower, middle):
	p.setJointMotorControlArray(bodyIndex=sawyerId,
								jointIndices=[21, 26, 22, 27],
								controlMode=p.POSITION_CONTROL,
								targetPositions=[lower, lower, middle, middle],
								targetVelocities=[0, 0, 0, 0],
								forces=[500, 500, 500, 500],
								positionGains=[0.03, 0.03, 0.03, 0.03],
								velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of ring finger, range both [0.17 - 1.57]
def ringF(lower, middle):
	p.setJointMotorControlArray(bodyIndex=sawyerId,
								jointIndices=[30, 35, 31, 36],
								controlMode=p.POSITION_CONTROL,
								targetPositions=[lower, lower, middle, middle],
								targetVelocities=[0, 0, 0, 0],
								forces=[500, 500, 500, 500],
								positionGains=[0.03, 0.03, 0.03, 0.03],
								velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of mid finger, range both [0.17 - 1.57]
def midF(lower, middle):
	p.setJointMotorControlArray(bodyIndex=sawyerId,
								jointIndices=[39, 44, 40, 45],
								controlMode=p.POSITION_CONTROL,
								targetPositions=[lower, lower, middle, middle],
								targetVelocities=[0, 0, 0, 0],
								forces=[500, 500, 500, 500],
								positionGains=[0.03, 0.03, 0.03, 0.03],
								velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of index finger, range both [0.17 - 1.57]
def indexF(lower, middle):
	p.setJointMotorControlArray(bodyIndex=sawyerId,
								jointIndices=[48, 53, 49, 54],
								controlMode=p.POSITION_CONTROL,
								targetPositions=[lower, lower, middle, middle],
								targetVelocities=[0, 0, 0, 0],
								forces=[500, 500, 500, 500],
								positionGains=[0.03, 0.03, 0.03, 0.03],
								velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of thumb, range: low [0.17 - 1.57], mid [0.34, 1.5]
def thumb(lower, middle):
	p.setJointMotorControlArray(bodyIndex=sawyerId,
								jointIndices=[58, 61, 64],
								controlMode=p.POSITION_CONTROL,
								targetPositions=[lower, middle, middle],
								targetVelocities=[0, 0, 0],
								forces=[500, 500, 500],
								positionGains=[0.03, 0.03, 0.03],
								velocityGains=[1, 1, 1])


##################################################################### Information from CLI ######################################################

def info():
	palmContact = []
	thumbContact = []
	indexContact = []
	midContact = []
	ringContact = []
	pinkyContact = []
	palmLinks = [19, 20, 25, 29, 34, 38, 43, 47, 52, 56, 57]
	thumbLinks = [58, 59, 60, 61, 62, 63, 64]
	indexLinks = [48, 49, 50, 51, 53, 54, 55]
	middleLinks = [39, 40, 41, 42, 44, 45, 46]
	ringLinks = [30, 31, 32, 33, 35, 36, 37]
	pinkyLinks = [21, 22, 23, 24, 26, 27, 28]

	contact = p.getContactPoints(sawyerId, objectId) # pubullet quick guide
	nums = len(contact)
	if (nums == 0):
		print ("There are no contact points")
		time.sleep(500)
		return [], [], [], [], [], []

	for i in range(nums):
		temp = []
		if(contact[i][3] in palmLinks):
			temp.append(contact[i][3])
			temp.append(contact[i][6])
			temp.append(contact[i][9])
			temp.append(contact[i][10])
			temp.append(contact[i][11])
			temp.append(contact[i][12])
			temp.append(contact[i][13])
			palmContact.append(temp)

		if(contact[i][3] in thumbLinks):
			temp.append(contact[i][3])
			temp.append(contact[i][6])
			temp.append(contact[i][9])
			temp.append(contact[i][10])
			temp.append(contact[i][11])
			temp.append(contact[i][12])
			temp.append(contact[i][13])
			thumbContact.append(temp)

		if(contact[i][3] in indexLinks):
			temp.append(contact[i][3])
			temp.append(contact[i][6])
			temp.append(contact[i][9])
			temp.append(contact[i][10])
			temp.append(contact[i][11])
			temp.append(contact[i][12])
			temp.append(contact[i][13])
			indexContact.append(temp)

		if(contact[i][3] in middleLinks):
			temp.append(contact[i][3])
			temp.append(contact[i][6])
			temp.append(contact[i][9])
			temp.append(contact[i][10])
			temp.append(contact[i][11])
			temp.append(contact[i][12])
			temp.append(contact[i][13])
			midContact.append(temp)

		if(contact[i][3] in ringLinks):
			temp.append(contact[i][3])
			temp.append(contact[i][6])
			temp.append(contact[i][9])
			temp.append(contact[i][10])
			temp.append(contact[i][11])
			temp.append(contact[i][12])
			temp.append(contact[i][13])
			ringContact.append(temp)

		if(contact[i][3] in pinkyLinks):
			temp.append(contact[i][3])
			temp.append(contact[i][6])
			temp.append(contact[i][9])
			temp.append(contact[i][10])
			temp.append(contact[i][11])
			temp.append(contact[i][12])
			temp.append(contact[i][13])
			pinkyContact.append(temp)


	return palmContact, thumbContact, indexContact, midContact, ringContact, pinkyContact

##################################################################### Simu Info ##################################################################

currentP = [0]*65
k = 0

##################################################################### Input Value Here############################################################

# Step 1: move robot to the initial position 

handInitial = [1.3487097919328548, 1.2325016390042327, 0.3281524633769494, 1.3492255020362145, 1.2357844011848487, 0.2573948624971588, 1.2977772696042083, 1.1882177328629302, 0.24699882259068173, 1.294638576896465, 1.1951816118835707, 0.1700063391219669, 1.266854770927521, 1.1474511023694083, 0.1700054741547499, 1.2674432857876725, 1.1481727969168507, 0.17007067318530492, 1.2530500683945291, 1.1223522859804513, 0.1701734172481422, 1.2525739180549518, 1.1252881033260587, 0.16999999999999996, 1.569755495022963, 0.40810448915685316, 0.40811508584048745]
orientation = [1.1912902355194093, 3.130738870288698, 1.4550535678863525]
palmPosition = [1.23, -0.035, -0.12]

# Step 2: move robot to the grasping position 

startPos = [handInitial[24], handInitial[25], handInitial[18], handInitial[19], handInitial[12], handInitial[13], handInitial[6], handInitial[7], handInitial[0], handInitial[1]]

final = np.zeros(10)
for i in range(10):
	final[i] = startPos[i]

p.setGravity(0, 0, -10)

while 1:
	k = k + 1

# Step 3: grasp object 

	i=0
	while 1:
		i+=1
		p.stepSimulation()
		currentP = palmP([1.15, -0.038, -0.075], p.getQuaternionFromEuler([orientation[0]+0.45, orientation[1], orientation[2]]))
		time.sleep(0.0003)
		if(i==1000):
			break

	i=0
	while 1:
		i+=1
		p.stepSimulation()
		time.sleep(0.003)
		if(i==250):
			indexF(final[2], final[3])	# 48, 53, 49, 54	# tip 50, 55 handInitial[20], handInitial[23]
			midF(final[4], final[5])	# 39, 44, 40, 45	# tip 41, 46 handInitial[14], handInitial[17]
			ringF(final[6], final[7])	# 30, 35, 31, 36	# tip 32, 37 handInitial[8], handInitial[11]
			pinkyF(final[8], final[9])	# 21, 26, 22, 27	# tip 23, 28 handInitial[2], handInitial[5]
			thumb(startPos[0], final[1])
		if(i==400):
			break

# Step 4: pick up the object

	i=0
	while 1:
		i+=1
		p.stepSimulation()
		currentP = palmP([1.15, -0.035, 0.3], p.getQuaternionFromEuler([orientation[0]+1, orientation[1], orientation[2]]))
		time.sleep(0.003)
		if(i==300):
			break
# Step 5: move the object to tray

	i=0
	while 1:
		i+=1
		p.stepSimulation()
		currentP = palmP([tray_x, tray_y, -0.12], p.getQuaternionFromEuler([orientation[0]+1, orientation[1], orientation[2]]))
		time.sleep(0.003)
		if(i==300):
			break
	i=0
	while 1:
		i+=1
		p.stepSimulation()
		time.sleep(0.003)
		if(i==250):
			p.stepSimulation()
			currentP = palmP([1.0, -0.035, 0.3], [tray_x, tray_y, -0.12])
			time.sleep(0.003)
		if(i==300):
			break
	
	# contact point information for the fingers and the palm
	# For example list palm includes all the contact points of your current grasping status
	# the format of each contact point is [linkId, positionOnB, normalForce, lateralFriction1, lateralFrictionDir1, lateralFriction2, lateralFrictionDir2]
	# use this info to test whether the hand closed porperly based on the determined grasp type.

	palm, thumb, index, mid, ring, pinky = info()
	print('palm', palm)
	print('thumb', thumb)
	print('index', index)
	print('mid', mid)
	print('ring', ring)
	print('pinky', pinky)
	break
p.disconnect()
print("disconnected")








