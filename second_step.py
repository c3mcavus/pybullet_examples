import pybullet as p
import time
import pybullet_data

physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
kukaId = p.loadURDF("kuka_iiwa/model.urdf")

num_joints = p.getNumJoints(kukaId)
print(f"Total Joints: {num_joints}")

for i in range(num_joints):
    joint_info = p.getJointInfo(kukaId,i)
    print(f"{i} joint info: {joint_info}")

for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(kukaId)
print(cubePos, cubeOrn)
p.disconnect()
