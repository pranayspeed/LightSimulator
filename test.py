import pybullet as p
import time
import pybullet_data

# Start PyBullet simulation
physicsClient = p.connect(p.GUI)  # Or p.DIRECT for non-graphical version
p.setGravity(0, 0, -10)  # Set gravity for the simulation

def setup_environment():
    """
    Sets up the basic simulation environment including the ground and gravity.
    """
    # Optionally, set the camera to a default view that suits your needs
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.5, -0.5, 0.5])

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # plane_id = p.loadURDF("plane.urdf")


    # Example of setting higher friction for the ground plane
    plane_id = p.loadURDF("plane.urdf")
    # p.changeDynamics(plane_id, -1, lateralFriction=0.2)  # Increase lateralFriction as needed


    p.setGravity(0, 0, -10)
    return plane_id

setup_environment()

# Load robot URDF
carId = p.loadURDF("robot/mycar.urdf", basePosition=[0,0,0.1])

# Identify joint indices (you may need to adjust these based on your URDF)
front_left_wheel_joint_index = 0
front_right_wheel_joint_index = 1
rear_left_wheel_joint_index = 2
rear_right_wheel_joint_index =3


# Main simulation loop
for i in range(10000):
    # Set rear wheels velocity for driving
    p.setJointMotorControl2(carId, rear_left_wheel_joint_index, p.VELOCITY_CONTROL, targetVelocity=5)
    p.setJointMotorControl2(carId, rear_right_wheel_joint_index, p.VELOCITY_CONTROL, targetVelocity=5)
    
    # Set front wheels position for steering
    # Adjust the targetPosition for steering. 0 means straight, adjust the value for left/right
    p.setJointMotorControl2(carId, front_left_wheel_joint_index, p.POSITION_CONTROL, targetPosition=0.1)
    p.setJointMotorControl2(carId, front_right_wheel_joint_index, p.POSITION_CONTROL, targetPosition=0.1)
    
    
    # Step simulation
    p.stepSimulation()
    time.sleep(1./240.)  # Slow down the loop to real-time so we can observe it

# Disconnect from the simulation
p.disconnect()
