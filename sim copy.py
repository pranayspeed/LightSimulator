import pybullet as p
import pybullet_data
import time


PLANE_FRIC = 1.2
CAR_FRIC = 0.2



def create_2d_constraint(body_id):
    """
    Creates a constraint for the body to limit its motion in the 2D plane.
    """
    p.createConstraint(body_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [0, 0, 0])


def create_2d_constraint_car(body_id):
    """
    Modifies the constraint for the body to allow it to move in the XY plane and rotate around the Z-axis,
    effectively limiting its motion to a 2D plane.
    """
    # Create a temporary non-colliding base link at the origin to act as the world anchor
    temp_anchor = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0, 0, 0])

    # Constraint that allows rotation around the Z-axis and free movement in the XY plane
    # The constraint is between the temporary anchor and the body
    constraint_id = p.createConstraint(parentBodyUniqueId=temp_anchor,
                                       parentLinkIndex=-1,
                                       childBodyUniqueId=body_id,
                                       childLinkIndex=-1,
                                       jointType=p.JOINT_PRISMATIC,
                                       jointAxis=[0, 0, 1],
                                       parentFramePosition=[0, 0, 0],
                                       childFramePosition=[0, 0, 0])
    
    # Allow movement along X and Y by setting limits very high
    p.changeConstraint(constraint_id, maxForce=0, gearRatio=1, erp=0.5,  gearAuxLink=-1)





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
    p.changeDynamics(plane_id, -1, lateralFriction=PLANE_FRIC)  # Increase lateralFriction as needed


    p.setGravity(0, 0, -10)
    return plane_id

def create_car():
    """
    Creates a simple car object. In this example, it's represented by a box.
    """
    # car_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5])
    car_id = p.loadURDF("robot/simple_car.urdf", basePosition=[0, 0, 0.5])
    # create_2d_constraint(car_id)

    # Increase wheel friction
    wheel_radius=0.05
    wheel_width=0.02
    wheels = [0, 1, 2, 3]  # Example indices for the wheel links

    # print("setting wheels constraint")
    # for wheel in wheels:
    #     wheel_material_id = p.getVisualShapeData(car_id)[wheel][0]
    #     print(p.getVisualShapeData(car_id)[wheel])
    #     p.changeDynamics(car_id, wheel_material_id, lateralFriction=CAR_FRIC)  # Adjust this value as needed


        # p.changeDynamics(wheel, -1, anisotropicFriction=[1.0, 0.5, 0.5])  # Example values, adjust as needed


    # Example: Setting anisotropic friction for wheels
    for wheel_joint_index in wheels:
        p.changeDynamics(car_id, wheel_joint_index, anisotropicFriction=[CAR_FRIC, 2.0, 0.5])  # Adjust these values as needed



    return car_id

    # car_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5])
    # create_2d_constraint(car_id)
    # return car_id


def create_obstacle_constraint(obstacle_id, position=[4, 0, 0.5]):

    # Create a fixed constraint to make the object static
    p.createConstraint(parentBodyUniqueId=obstacle_id,
                    parentLinkIndex=-1,
                    childBodyUniqueId=-1,
                    childLinkIndex=-1,
                    jointType=p.JOINT_FIXED,
                    jointAxis=[0, 0, 0],
                    parentFramePosition=[0, 0, 0],
                    childFramePosition=position,
                    childFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]))

def create_obstacles():
    """
    Creates simple obstacles in the environment.
    """

    objects_pos = [[2, 0, 0], [2, 4, 0]]
    for obj_pos in objects_pos:
        # Create a fixed constraint to make the object static
        # obstacle1_id = p.loadURDF("cube_small.urdf", basePosition=obj_pos)
        obstacle1_id = p.loadURDF("robot/obstacle.urdf", basePosition=obj_pos, useFixedBase=True)
        # scale_factor = 2.0  # Example scale factor
        # p.changeVisualShape(obstacle1_id, -1, [scale_factor, scale_factor, scale_factor])
        # create_2d_constraint(obstacle1_id)
        create_obstacle_constraint(obstacle1_id, position=obj_pos)


# def apply_force_to_move_forward(car_id):
#     """
#     Applies a force to the car to move it forward.
#     """
#     force_magnitude = 100  # Adjust as necessary for your simulation
#     forward_direction = [force_magnitude, force_magnitude,0]  # Assuming the car's forward direction aligns with the world's x-axis

#     # forward_direction *= force_magnitude
#     p.applyExternalForce(objectUniqueId=car_id, linkIndex=-1, forceObj=forward_direction, posObj=[0, 0, 0], flags=p.WORLD_FRAME)






def apply_force_to_move_forward_old(car_id):
    """
    Controls the car based on "ASDW" keys for forward, left, backward, and right movement.
    """
    keys = p.getKeyboardEvents()
    force_magnitude = 100
    force_steer = 1
    
    forward = [force_magnitude, 0, 0]
    backward = [-force_magnitude, 0, 0]
    left = [0, force_steer, 0]
    right = [0, -force_steer, 0]
    
    # Forward - 'W' key
    if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
        p.applyExternalForce(car_id, -1, forward, [0, 0, 0], p.WORLD_FRAME)
    
    # Backward - 'S' key
    if ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
        p.applyExternalForce(car_id, -1, backward, [0, 0, 0], p.WORLD_FRAME)
    
    # Left - 'A' key
    if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
        p.applyExternalForce(car_id, -1, left, [0, 0, 0], p.WORLD_FRAME)
    
    # Right - 'D' key
    if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
        p.applyExternalForce(car_id, -1, right, [0, 0, 0], p.WORLD_FRAME)



def apply_velocity_and_steering(car_id, wheel_velocity, steering_angle, drive_force=100, steering_force=100, steering_velocity=5.0):

    # Assuming `car_id` is the ID of your loaded car model
    # Define joint indices for your wheels
    joint_indices = {
        'front_left_wheel': 0,  # Update with actual joint index
        'front_right_wheel': 1,  # Update with actual joint index
        'rear_left_wheel': 2,  # Update with actual joint index
        'rear_right_wheel': 3  # Update with actual joint index
    }

    # # Set target velocity for driving wheels (rear wheels)
    # wheel_velocity = 10.0  # rad/s, adjust as needed
    # drive_force = 100  # Max force to apply, adjust as needed

    # Apply velocity control to rear wheels for driving
    for wheel in ['rear_left_wheel', 'rear_right_wheel']:
        joint_index = joint_indices[wheel]
        p.setJointMotorControl2(bodyUniqueId=car_id,
                                jointIndex=joint_index,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=wheel_velocity,
                                force=drive_force)

    # # Set target steering angle for front wheels
    # steering_angle = 0.5  # radians, adjust as needed for desired steering angle
    # steering_velocity = 5.0  # rad/s, maximum velocity to reach the steering angle
    # steering_force = 100  # Max force to apply for steering

    # Apply position control to front wheels for steering
    for wheel in ['front_left_wheel', 'front_right_wheel']:
        joint_index = joint_indices[wheel]
        p.setJointMotorControl2(bodyUniqueId=car_id,
                                jointIndex=joint_index,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=steering_angle,
                                maxVelocity=steering_velocity,
                                force=steering_force)




# Keycode constants for arrow keys
KEYCODE_LEFT_ARROW = 65295
KEYCODE_RIGHT_ARROW = 65296
KEYCODE_UP_ARROW = 65297
KEYCODE_DOWN_ARROW = 65298




def apply_force_to_move_forward(car_id):
    keys = p.getKeyboardEvents()
    wheel_velocity =5.0  # Speed at which wheels should rotate
    steering_angle = 0.8  # Angle for steering, in radians
    
    # IDs of the wheel joints and steering joints
    wheel_joints = [0, 1, 2, 3]  # Example indices for wheel rotation joints
    steering_joints = [0, 1]  # Example indices for front wheel steering joints

    drive_joints = [2, 3]  # Example indices for wheel rotation joints
    
    # Reset steering to straight when no left/right input is detected
    straight = True
    


    acceleration = 0.1
    force = 100

    if KEYCODE_UP_ARROW in keys and keys[KEYCODE_UP_ARROW] & p.KEY_IS_DOWN:
        wheel_velocity = wheel_velocity
    elif KEYCODE_DOWN_ARROW in keys and keys[KEYCODE_DOWN_ARROW] & p.KEY_IS_DOWN:
        wheel_velocity = -wheel_velocity
    else:
        wheel_velocity = 0.0

    if KEYCODE_LEFT_ARROW in keys and keys[KEYCODE_LEFT_ARROW] & p.KEY_IS_DOWN:
        steering_angle = steering_angle
        straight = False
    elif KEYCODE_RIGHT_ARROW in keys and keys[KEYCODE_RIGHT_ARROW] & p.KEY_IS_DOWN:
        steering_angle = -steering_angle
        straight = False
    else:
        steering_angle = 0.0
    

    apply_velocity_and_steering(car_id, wheel_velocity, steering_angle)

    # for joint in drive_joints:
    #     p.setJointMotorControl2(car_id, joint, p.VELOCITY_CONTROL, targetVelocity=wheel_velocity, force=force)

    # for joint in steering_joints:
    #     p.setJointMotorControl2(car_id, joint, p.POSITION_CONTROL, targetPosition=steering_angle)


    # # Check if wheels are making contact with the ground
    # for wheel_joint_index in [2,3]:
    #     print(p.getContactPoints(bodyA=car_id, linkIndexA=wheel_joint_index), end='\r')




def simulate_car_movement(car_id):
    """
    Simulates basic car movement by applying forces.
    """
    for i in range(100000):
        p.stepSimulation()
        # p.applyExternalForce(car_id, -1, [100, 0, 0], [0, 0, 0], p.WORLD_FRAME)
        
        
        apply_force_to_move_forward(car_id)
        time.sleep(1/240)  # Assuming 240 Hz simulation frequency

def main():
    p.connect(p.GUI)
    setup_environment()
    car_id = create_car()
    
    create_obstacles()



    simulate_car_movement(car_id)






    p.disconnect()

if __name__ == "__main__":
    main()
