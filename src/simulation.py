import pybullet as p
import pybullet_data
import time

def main():
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.loadURDF("plane.urdf")
    robot = p.loadURDF("r2d2.urdf", [0, 0, 0.5])

    for _ in range(500):
        p.stepSimulation()
        time.sleep(1./240.)

    p.disconnect()

if __name__ == "__main__":
    main()
