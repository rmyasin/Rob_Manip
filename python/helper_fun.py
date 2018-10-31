  def readRegistrationYaml(self):
    rpack=rospkg.RosPack()
    armalib_path=rpack.get_path('armalib')

    filePath = os.path.join(armalib_path,'include','basic_udp_example','tip_calibration.yaml')
    with open(filePath, 'r') as f:
      data = yaml.load(f)
    self.nextFrame=list()
    self.tipFrame=list()
    for i in range(1,5):
      markerS='marker_'+str(i)
      rot=PyKDL.Rotation.Quaternion(data[markerS]['quat'][1],data[markerS]['quat'][2],data[markerS]['quat'][3],data[markerS]['quat'][0])
      pos=PyKDL.Vector(data[markerS]['pos'][0],data[markerS]['pos'][1],data[markerS]['pos'][2])
      pos=pos/1000 # Convert from mm to meter
      self.tipFrame.append(PyKDL.Frame(rot,pos))

def readOrganRegTxt(filename):
  rpack=rospkg.RosPack()
  nripath=rpack.get_path('dvrk_nri_robot')
  saveFolder=os.path.abspath(os.path.join(nripath,'data','user_study'))
  pointListFilename=os.path.join(saveFolder,filename)

  with open(pointListFilename, 'r') as f:
    lines=f.readlines() #position Label
    posStr=re.sub(' +',' ',lines[1]).split(' ')
    quatStr=re.sub(' +',' ',lines[3]).split(' ')
  pos = PyKDL.Vector(float(posStr[0]),float(posStr[1]),float(posStr[2]))
  rot = PyKDL.Rotation.Quaternion(float(quatStr[0]),float(quatStr[1]),float(quatStr[2]),float(quatStr[3]))
  registration2Robot = PyKDL.Frame(rot,pos)

# write to file
def writePositionandQuat(pos,quat,saveFolder,filename='PSM2Phantom'):
  if type(pos)==PyKDL.Vector:
    pos=np.array((pos[0],pos[1],pos[2]))
  file_obj = open(os.path.join(saveFolder,filename+'.txt'),"w+")
  file_obj.write("Position\n")
  file_obj.write(re.sub('[\[\]]', '', np.array_str(pos.transpose()))+"\n")
  file_obj.write("Quaternion\n")
  file_obj.write(re.sub('[(),]','',str(quat)))
  file_obj.close()

def writePositionandQuatYaml(pos,quat):
  rpack=rospkg.RosPack()
  visionpath=rpack.get_path('dvrk_vision')
  saveFolder=os.path.abspath(os.path.join(visionpath,'defaults'))
  # folderName=os.path.expanduser('~') +'/catkin_ws/src/dvrk_vision/defaults/'
  file_obj = open(os.path.join(saveFolder,'PSM2Phantom.yaml'),"w+")
  file_obj.write("position: [")
  file_obj.write(re.sub('[\[\]]','',np.array2string(pos.transpose(),separator=', '))+"]\n")
  file_obj.write("# XYZW\n")
  file_obj.write("quaternion: [")
  file_obj.write(', '.join(str(item) for item in quat))
  file_obj.write("]")
  file_obj.close()

def writeRegistrationPoints(pos_input,quat_input,file_obj):
  for index in range(pos_input.shape[1]):
    file_obj.write(re.sub('[\[\]]','',np.array_str(pos_input[:,index].transpose()))+"\n")
    file_obj.write(re.sub('[\[\]]','',np.array_str(quat_input[:,index].transpose()))+"\n")

# x and y are both numpy matrices
def rigidRegistration(x,y):
  x0=np.mean(x,1)
  y0=np.mean(y,1)

  xtild=x-x0
  ytild=y-y0

  cMat=xtild*ytild.transpose()
  U,_,vh=np.linalg.svd(cMat)
  I=np.identity(xtild.shape[0])
  s=np.sign(np.linalg.det(vh.getH()*U))
  I[2,2]=s #TO GENERALIZE, WOULD USE LAST ELEMENT, NOT 2,2

  R=vh.getH()*I*U.transpose() 
  t=y0-R*x0
  return R,t

def convertPose2Pos(pose_input):
  N=len(pose_input)
  matOut=np.matrix(np.zeros((3,N)))
  quatmatOut=np.matrix(np.zeros((4,N)))
  for index in range(N):
    matOut[0,index]=pose_input[index].position.x
    matOut[1,index]=pose_input[index].position.y
    matOut[2,index]=pose_input[index].position.z

    quatmatOut[0,index]=pose_input[index].orientation.w
    quatmatOut[1,index]=pose_input[index].orientation.y
    quatmatOut[2,index]=pose_input[index].orientation.z
    quatmatOut[3,index]=pose_input[index].orientation.z
  return matOut,quatmatOut
