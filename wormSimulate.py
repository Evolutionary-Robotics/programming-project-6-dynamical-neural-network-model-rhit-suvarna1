import pybullet as p
import pybullet_data
import pyrosim.pyrosim as ps
import numpy as np
import time 
import ctrnn
import matplotlib.pyplot as plt

#Neural Network stuff
size = 10
duration = 100  
stepsize = 0.01
sim_time = np.arange(0.0, duration, stepsize)
outputs = np.zeros(len(sim_time))
nn = ctrnn.CTRNN(size)
nn.load("ctrnn.npz")
nn.initializeState(np.zeros(size))

# for step in range(len(sim_time)):
#     nn.step(stepsize)
#     outputs[step] = nn.Outputs[2]

# plt.plot(sim_time, outputs, label = "Neuron 0")
# plt.xlabel("Time")
# plt.ylabel("Output of Neuron")
# plt.show()

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")

p.setGravity(0,0,-9.8)

# p.loadSDF("box.sdf")
# p.loadSDF("box2.sdf")
wormSim = p.loadURDF("worm.urdf")
# p.changeDynamics(wormSim, 4, lateralFriction = 10.0)

duration = 10000

joint_lower_limit = 0
joint_upper_limit = np.pi / 4  
back_joint_index = 4
front_joint_index = 0
ps.Prepare_To_Simulate(wormSim)
x = np.linspace(0,20*np.pi, duration)
# y = np.sin(x)*np.pi/2

for i in range(duration):

    nn.step(stepsize)
    neuronOutput = nn.Outputs[8]
    # print("neuron 0 Output: %i", neuronOutput)
    # target_position = y[i]
    
    # if target_position < joint_lower_limit:
    #     target_position = joint_lower_limit
    # elif target_position > joint_upper_limit:
    #     target_position = joint_upper_limit
    target_position = (joint_lower_limit + (neuronOutput * (joint_upper_limit - joint_lower_limit)))
    # print("target Position: %i", target_position)
    p.changeDynamics(wormSim, front_joint_index, lateralFriction=10)
    p.changeDynamics(wormSim, back_joint_index, lateralFriction=9)
    p.setJointMotorControl2(bodyIndex=wormSim,
                            jointIndex=0,  # Index of the joint to control (0 for the first joint)
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=target_position,
                            force=500)
    p.setJointMotorControl2(bodyIndex=wormSim,
                            jointIndex=1,  # Index of the joint to control (0 for the first joint)
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=-target_position,
                            force=500)
    p.setJointMotorControl2(bodyIndex=wormSim,
                            jointIndex=2,  # Index of the joint to control (0 for the first joint)
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=target_position,
                            force=500)
    p.setJointMotorControl2(bodyIndex=wormSim,
                            jointIndex=3,  # Index of the joint to control (0 for the first joint)
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=target_position,
                            force=500)
    
    # if target_position < joint_upper_limit / 2:
    #     p.changeDynamics(wormSim, front_joint_index, lateralFriction=0.1)
    #     p.changeDynamics(wormSim, back_joint_index, lateralFriction=10) 
    # else:
    #     p.changeDynamics(wormSim, back_joint_index, lateralFriction=0.1)  
    #     p.changeDynamics(wormSim, front_joint_index, lateralFriction=10)

    p.stepSimulation()
    time.sleep(1/1000)


p.disconnect()

