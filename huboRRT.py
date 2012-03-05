from openravepy import *
from huboJointNum import huboJointNum as hubo
import numpy as np
#import huboJointNum as hubo
#from SixDofMatrix import SixDofMatrix 
from str2num import *
from numpy import *
from rodrigues import *
from TransformMatrix import *
from TSR import *
import time
import sys



#def hubo():
#        WST = 0



def WaitForcontroller(robot):
	robot.WaitForController(0)

if __name__ == "__main__":
	#chainlength = 2
	chainlength = 1

	# load the enviroment if it is not already
	try:
		orEnv
	except NameError:
		orEnv = Environment()
		orEnv.SetViewer('qtcoin')
	
	orEnv.Reset()
	orEnv.Load('../openHubo/jaemiHubo.robot.xml')
	robot = orEnv.GetRobots()[0]
	
	# set printing and display options
	orEnv.SetDebugLevel(DebugLevel.Info)
	#colchecker = RaveCreateCollisionChecker(orEnv,'ode')
	colchecker = RaveCreateCollisionChecker(orEnv,'bullet')
	orEnv.SetCollisionChecker(colchecker)

	# create problem instance
	probs_cbirrt = RaveCreateProblem(orEnv,'CBiRRT')
	orEnv.LoadProblem(probs_cbirrt,'jaemiHubo')

	# set up joint indices
	activedofs =   [hubo.RSP, hubo.RSR, hubo.RSY, hubo.REB, hubo.RWY, hubo.RWP, hubo.RWR, hubo.WST]
	initdofvals = r_[ 0.01  ,   0.01  ,   0.01  ,  0.01   ,  0.01   ,  0.01   ,  0.01   ,  0.01   ]
	allDOF = r_[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26]

	# put the bottle somewhere
	targobject = orEnv.ReadKinBodyXMLFile('/share/comps/ormodels/objects/household/juice_bottle_model.kinbody.xml')
	orEnv.AddKinBody(targobject)

	#T0_object = MakeTransform(mat(eye(3)),mat([0.3602,  0.2226, 0.9214]).T)
	T0_object = MakeTransform(mat(eye(3)),mat([0.2,  0.0, 0.0]).T)
	targobject.SetTransform(array(T0_object[0:3][:,0:4]))

	robot = orEnv.GetRobots()[0]

	# Start robot in a resonable pos
	robot.SetActiveDOFs(activedofs)
	robot.SetActiveDOFValues(initdofvals)	

	# define two TSR chains for the task they will differ in one rotation of the hand
	
	# first TSR
	# Place the reference frame of the world at that of the target object
	T0_w = T0_object
	
	# get the TSR's offset fraim in w coordinates
	Tw_e1 = MakeTransform(rodrigues([pi/2,0,0]),mat([0,0.20,0.1]).T)
	#Tw_e1 = MakeTransform(rodrigues([1,1,1]),mat([1,1,1]).T)

	# define bounds to only allow rotation of the hand about the z axis and a small deviation in translation along the z axis
	Bw = mat([0,0,0,0,-0.02, 0.02, 0,0,0,0,-pi,pi])
	
	TSRstring1 = SerializeTSR(.01,'NULL',T0_w,Tw_e1,Bw)
	TSRChainString1 = SerializeTSRChain(0,1,0,1,TSRstring1,'NULL',[])

	# now define the second TSR chain
	# this is the same as the first chaing except the Tw_e is different (the hand is rotated by 180 deg about the z)
	Tw_e2 = MakeTransform(rodrigues([0,pi,0])*rodrigues([pi/2,0,0]), mat([0,0.20,0.1]).T)
	#Tw_e2 = Tw_e1
	TSRstring2 = SerializeTSR(0,'NULL',T0_w,Tw_e2,Bw)
	TSRChainString2 = SerializeTSRChain(0,1,0,1,TSRstring2,'NULL',[])

	# call the cbirrt planner, it will generate a file with the trajectory called 'cmovetraj.txt'
	resp = probs_cbirrt.SendCommand('RunCBiRRT psample 0.25 %s%s'%(TSRChainString1,TSRChainString2))
	#resp = probs_cbirrt.SendCommand('RunCBiRRT psample 0.001 %s'%(TSRChainString1))
	probs_cbirrt.SendCommand('traj cmovetraj.txt')
	robot.WaitForController(0)

	raw_input("press enter")
	sys.stdin.readline()
