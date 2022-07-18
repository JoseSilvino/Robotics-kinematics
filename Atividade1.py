import time
from zmqRemoteApi import RemoteAPIClient
import numpy as np
time_start = time.time()
def stopsim(sim):
    while(True):
        time_tillnow = time.time()
        if(time_tillnow-time_start >= 10): break
    sim.stopSimulation()
client = RemoteAPIClient()
sim = client.getObject('sim')
sim.startSimulation()
dummy = sim.getObject('/Dummy')
pos = sim.getObjectPosition(dummy,-1)
q = sim.getObjectQuaternion(dummy,-1)
orientation = sim.getObjectOrientation(dummy,-1)
print(f'Position = {pos}')
print(f'Quaternion(h1,h2,h4,h1) = {q}')
print(f'Orientation = {orientation}')
joints = [-1,-1,-1,-1]
for i in range(4):
    joints[i] = sim.getObject('/axis',{'index':i})
print(joints)
sim.setJointPosition(joints[0],np.pi/2)
sim.setJointPosition(joints[1],np.pi/3)
sim.setJointPosition(joints[2],0.01)
sim.setJointPosition(joints[3],np.pi/2)
rop = sim.getObjectPosition(joints[2],-1)
sim.setObjectPosition(dummy,-1,[rop[0],rop[1],pos[2]])
stopsim(sim)


