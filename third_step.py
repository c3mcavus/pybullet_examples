import pybullet as p
import time
import pybullet_data

physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
kukaId = p.loadURDF("kuka_iiwa/model.urdf")

p.setRealTimeSimulation(0)
orientation = p.getQuaternionFromEuler([3.14, 0, 3.14,])
targetPositionsJoints = p.calculateInverseKinematics(kukaId, 6, [0.2, 0.2, 0.4], targetOrientation = orientation)
p.setJointMotorControlArray(kukaId, range(7), p.POSITION_CONTROL, targetPositions = targetPositionsJoints)

for i in range(10000):
    p.stepSimulation()
    time.sleep(1./9.)

p.disconnect()
