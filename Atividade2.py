from time import sleep
import roboticstoolbox as rtb
import numpy as np
from matplotlib import pyplot as plt
from zmqRemoteApi import RemoteAPIClient
pi = np.pi
L1 = rtb.RevoluteDH(a=0.475)
L2 = rtb.RevoluteDH(a=0.4,offset=0*-pi/2)
L3 = rtb.RevoluteDH()
DummyLink = rtb.RevoluteDH(alpha=-pi,qlim=[-pi,-pi])
L4 = rtb.PrismaticDH(qlim=[0,0.1],offset=0.1)
links = [L1,L2,L3,DummyLink,L4]
robot = rtb.DHRobot(links)
print("Parametros DH:\n",robot)

#Matriz de rotação em Z
def rotMatrixZ(theta):
    return np.array([[np.cos(theta),-np.sin(theta),0,0],[np.sin(theta),np.cos(theta),0,0],[0,0,1,0],[0,0,0,1]])
#Matriz de translação em Z
def transMatrixZ(d):
    return np.array([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]])
#Matriz de rotação em X
def rotMatrixX(alpha):
    return np.array([[1,0,0,0],[0,np.cos(alpha),-np.sin(alpha),0],[0,np.sin(alpha),np.cos(alpha),0],[0,0,0,1]])
#Matriz de translação em X
def transMatrixX(a):
    return np.array([[1,0,0,a],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
#Matriz de transformação de j-1 até j
def TMatrix(Link:rtb.DHLink,value = 0):
    theta = Link.theta
    d = Link.d
    a = Link.a
    alpha = Link.alpha
    if Link.isprismatic: d += Link.offset + value
    if Link.isrevolute: theta += Link.offset + value
    rz = rotMatrixZ(theta)
    tz = transMatrixZ(d)
    tx = transMatrixX(a)
    rx = rotMatrixX(alpha)
    print(f"RotZ:\t",end='\t')
    print(f"TransZ:\t",end='\t')
    print(f"TransX:\t",end='\t')
    print(f"RotX:\n")
    for i in range(4):
        print(f'{rz[i]}\t\t{tz[i]}\t\t{tx[i]}\t\t{rx[i]}')
    return rz@tz@tx@rx
#Fkine
def myfkine(links:list[rtb.DHLink],params:list):
    res = np.eye(4,4)
    if len(links)!=len(params): return None
    for i in range(len(links)):
        m = links[i]
        p = params[i]
        tm = TMatrix(m,p)
        print(f'T({i},{i+1}): \n{tm}\n\n')
        res = res@tm
    return res
#Robô gerado



#client = RemoteAPIClient()
#sim = client.getObject('sim')
#sim.startSimulation()
#joints = [-1,-1,-1,-1]
#for i in range(4):
#    joints[i] = sim.getObject('/axis',{'index':i})
#Comparação dos Fkines
dummyvalue=0
def moveJoints(sim,joints,param):
    for i in range(4):
        if i==3: 
            sim.setJointPosition(joints[i],param[i+1])
        else:
            sim.setJointPosition(joints[i],param[i])
#Letra (a)
param = [0,0,0,dummyvalue,0]
param = [-0.5315870873440669, 2.653488878266732, -1.336503627525217, 0, 0.015]
implemented = robot.fkine(param)
my = myfkine(links,param)
print(implemented,'\n',my)
print()
print('robot.fkine-myfkine = \n',implemented-my)
robot.teach(param)
#moveJoints(sim,joints,param)
sleep(10)

#Letra (b)
param = [pi/2,-pi/2,0,dummyvalue,0]
param = [-0.6362057661901569, 1.907727213595434, -0.48612328400782867, 0, 0.015]
implemented = robot.fkine(param)
my = myfkine(links,param)
print(implemented,'\n',my)
print()
print('robot.fkine-myfkine = \n',implemented-my)
robot.teach(param)
#moveJoints(sim,joints,param)
sleep(10)
#Letra (c)
param = [pi/2,-pi/2,0,dummyvalue,0.05]
param = [-0.2043048091038574, 2.6823344700137253, 0.6635629926799251, 0, 0]
implemented = robot.fkine(param)
my = myfkine(links,param)
print(implemented,'\n',my)
print()
print('robot.fkine-myfkine = \n',implemented-my)
#moveJoints(sim,joints,param)
robot.teach(param)
sleep(10)
#sim.stopSimulation()
