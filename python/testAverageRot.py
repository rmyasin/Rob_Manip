
from collections import deque
import PyKDL
import numpy as np
import rospy
import ipdb

def averageFrames(frameDeque):
    N=len(frameDeque)
    matOut=np.matrix(np.zeros((3,N)))
    quatmatOut=np.matrix(np.zeros((4,N)))
    for index in range(N):
      matOut[0,index]=frameDeque[index].p[0]
      matOut[1,index]=frameDeque[index].p[1]
      matOut[2,index]=frameDeque[index].p[2]

      # xyzw in each column
      quat=np.matrix(frameDeque[index].M.GetQuaternion()).transpose()
      quat=quat/(quat.transpose()*quat)
      quatmatOut[0,index]=quat[0]
      quatmatOut[1,index]=quat[1]
      quatmatOut[2,index]=quat[2]
      quatmatOut[3,index]=quat[3]
    
    meanP=np.mean(matOut,1)
    meanRot=averageRotations(quatmatOut) #Returns a PyKDL Rotation object
    aveFrame=PyKDL.Frame(meanRot,PyKDL.Vector(meanP[0],meanP[1],meanP[2]))
    return aveFrame


def averageRotations(quatMat):
	M=np.matrix(np.zeros((4,4)))
	for i in range(quatMat.shape[1]):
		M = M + quatMat[:,i] * quatMat[:,i].transpose()
	w,v = np.linalg.eig(M)
	index=np.argmax(w)
	quatAve=v[:,index]
	return PyKDL.Rotation.Quaternion(quatAve[0],quatAve[1],quatAve[2],quatAve[3])


def run():
	frame1 = PyKDL.Frame(PyKDL.Rotation.RotX(2),PyKDL.Vector(1,2,3))
	frame2 = PyKDL.Frame(PyKDL.Rotation.RotX(1),PyKDL.Vector(-1,-2,-3))
	frame3 = PyKDL.Frame(PyKDL.Rotation.RotX(1),PyKDL.Vector(-1,-2,-3))
	frameD = deque()
	frameD.append(frame1)
	frameD.append(frame2)
	frameD.append(frame3)
	aveFrame = averageFrames(frameD)
	print(aveFrame)

if __name__ == '__main__':
  run()