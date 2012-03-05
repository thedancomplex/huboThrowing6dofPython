from openravepy import *
from huboJointNum import huboJointNum as hNum
import numpy as np
from SixDofMatrix import SixDofMatrix 

env = Environment()
env.SetViewer('qtcoin')

sDof = SixDofMatrix()


#env.Load('hubo.xml')
env.Load('../openHubo/jaemiHubo.robot.xml')

robot = env.GetRobots()[0]
print robot
#robot.SetDOFValues([0, 1, 0 ],[0,1, 2 ])


robot.WaitForController(0) # waiyyt

raw_input('press enter to start')
robot.SetActiveManipulator('rightArmManip') # set the manipulator to leftarm + torso
#robot.SetActiveManipulator('rightArmManipRtFoot') # set the manipulator to leftarm + torso

# 6D IK
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)

# 3D IK
#ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
if not ikmodel.load():
    ikmodel.autogenerate()

manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
#Tgoal = numpy.array([[0,-1,0,0.2],[-1,0,0,-0.2],[0,0,-1,0.0],[0,0,0,1]])
Tgoal = sDof.get6DofMatrix(0.2, -0.2, 0.0, 0.0, 0.0, 0.0)


res = manipprob.MoveToHandPosition(matrices=[Tgoal],seedik=10) # call motion planner with goal joint angles
robot.WaitForController(0) # waiyyt

d = 0.2

st = 1
while( st < 10):
#	raw_input("press enter for next pos")
	#d = d - 0.01
	
	try:

		DDx = raw_input("press enter for next pos, Dx = ")
		DDy = raw_input("press enter for next pos, Dy = ")
		DDz = raw_input("press enter for next pos, Dz = ")
		
		Dx = float(DDx)
		Dy = float(DDy)
		Dz = float(DDz)
		print 'Dx = ',Dx,'  Dy = ', Dy, '  Dz = ', Dz	
		Tgoal = sDof.get6DofMatrix(0.2, -0.2, -0.2, Dx,Dy,Dz)
		print Tgoal
		res = manipprob.MoveToHandPosition(matrices=[Tgoal],seedik=10) # call motion planner with goal joint angles
		robot.WaitForController(0) # waiyyt
		print Tgoal
		
	except:
		print "No joint pos"

	st = st+1


#for x in range(27):
#	robot.SetDOFValues([0.3],[x])

#robot.SetDOFValues([1],[hNum.REB])


#manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs

# this checks to see if there is a colision 
#res = manipprob.MoveManipulator(goal=[-0.75,1.24,-0.064,2.33,-1.16,-1.548,1.19]) # call motion planner with goal joint angl
#robot.SetJointValues([12, 25, 24],[0,1, 2 ])


#env.UpdatePublishedBodies()
#print res
#print "press ENTER"
#raw_input()

#res = manipprob.MoveManipulator(goal=[0.75,1.24,-0.164,2.33,-1.16,-1.548,1.19]) # call motion planner with goal joint angl
#print res
#env.UpdatePublishedBodies()

raw_input("press enter")
