import roboticstoolbox as rtb
import numpy as np
import time
from zmqRemoteApi import RemoteAPIClient
from matplotlib import pyplot as plt
pi = np.pi
x = np.eye(4,4)
def createRobot(l1,l2):
    L1 = rtb.RevoluteDH(a=l1)
    L2 = rtb.RevoluteDH(a=l2)
    L3 = rtb.RevoluteDH()
    LDigital =rtb.RevoluteDH(qlim=[0,0],alpha=pi)
    L4 = rtb.PrismaticDH(qlim=[0,0.1])
    robot = rtb.DHRobot([L1,L2,LDigital,L3,L4])
    return robot

def Jacobian(robot:rtb.DHRobot,q):
    s = np.sin
    c = np.cos
    l1 = robot.links[0].a
    l2 = robot.links[1].a
    c1 = c(q[0])
    s1 = s(q[0])
    c12 = c(q[0]+q[1])
    s12 = s(q[0]+q[1])

    """
    Parte 1 de J
    -l1s1-l2c12  -l2c12  0 0 0
    l1c1+l2s12    l2s12  0 0 0
        0           0    0 0 1

    Parte 2 de J
        0   0   0   0   0
        0   0   0   0   0
        1   1   1   1   0
    """
    J = np.array([[-l1*s1-l2*c12,-l2*c12,0,0,0],
                    [l1*c1+l2*s12,l2*s12,0,0,0],
                    [0,0,0,0,-1],
                    [0,0,0,0,0],
                    [0,0,0,0,0],
                    [1,1,1,1,0]])
    #print(J)
    return J
def InvKine(Ts:np.float32,robot:rtb.DHRobot,sim):
    joints = [-1,-1,-1,-1]
    for i in range(4):
        joints[i] = sim.getObject('/axis',{'index':i})
    dummy = sim.getObject("/Dummy")
    currentpos = [x for x in sim.getObjectPosition(joints[3],-1)]+[0,np.pi,sim.getObjectOrientation(joints[3],-1)[0]]
    q = []
    for i in range(5):
        if i<2:
            q += [sim.getJointPosition(joints[i])]
        if i == 2:
            q += [sim.getJointPosition(joints[i+1])]
        if i == 3:
            q += [0]
        if i == 4:
            q += [sim.getJointPosition(joints[i-1])]
    Xc = currentpos    
    errorlimit = 0.001
    xyzrpy = [x for x in sim.getObjectPosition(dummy,-1)] + [0,np.pi,sim.getObjectOrientation(dummy,-1)[0]]
    Xd = np.array(xyzrpy)
    Xc = np.array(Xc)
    error = np.linalg.norm(Xd-Xc)
    X_e = []
    jointvalues = [[],[],[],[]]
    while np.abs(error)>=errorlimit:
        xyzrpy = [x for x in sim.getObjectPosition(dummy,-1)] + [np.pi,0,sim.getObjectOrientation(dummy,-1)[0]]
        Xd = np.array(xyzrpy)
        print(np.abs(error))
        J = Jacobian(robot,q)
        J_inv = np.linalg.pinv(J)
        dq = J_inv@(Xd-Xc)
        q = q + dq *Ts
        params = toCoppelia(q)
        for i in range(len(params)):
            jointvalues[i] += [params[i]]
            sim.setJointPosition(joints[i],params[i])
        Xc =  [x for x in sim.getObjectPosition(joints[3],-1)]+[np.pi,0,sim.getObjectOrientation(joints[3],-1)[0]]
        Xc = np.array(Xc)
        error = np.linalg.norm(Xd-Xc)
        X_e += [error]
    plt.subplot(2,3,1)
    plt.title("Erro")
    plt.plot(X_e)
    for i in range(4):
        plt.subplot(2,3,i+2)
        plt.title(f"Junta {i}")
        plt.plot(jointvalues[i])
    plt.show()
    return q
def toCoppelia(params:list[np.float32]):
    return [params[0],params[1],params[4],-params[2]]
l1 = 0.475
l2 = 0.4
robot = createRobot(l1,l2)
time_start = time.time()
def stopsim(sim):
    while(True):
        time_tillnow = time.time()
        if(time_tillnow-time_start >= 5): break
    sim.stopSimulation()
client = RemoteAPIClient()
sim = client.getObject('sim')
sim.startSimulation()
#Posição de exemplo

params = InvKine(0.01,robot,sim)

stopsim(sim)
