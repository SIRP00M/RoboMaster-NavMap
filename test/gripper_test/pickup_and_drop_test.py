from robomaster import robot
import time

# Arm coordinates configured for your robot
PICK_POSITION = (180, -50)
LIFT_POSITION = (0, 100)

def main():
    print("Initializing robot...")
    ep_robot = robot.Robot()
    
    # Initialize connection (using 'ap' which is the default for direct Wi-Fi)
    ep_robot.initialize(conn_type="ap")
    
    try:
        ep_arm = ep_robot.robotic_arm
        ep_gripper = ep_robot.gripper
        
        # 1. Reset/Ready state
        print("1. Moving to LIFT position and opening gripper...")
        ep_arm.moveto(x=LIFT_POSITION[0], y=LIFT_POSITION[1]).wait_for_completed()
        ep_gripper.open(power=50)
        time.sleep(1.5)
        ep_gripper.pause()
        
        # 2. Go down to pick
        print("2. Moving down to PICK position...")
        ep_arm.moveto(x=PICK_POSITION[0], y=PICK_POSITION[1]).wait_for_completed()
        time.sleep(0.5)
        
        # 3. Grab
        print("3. Closing gripper (Picking up)...")
        ep_gripper.close(power=50)
        time.sleep(1.5)
        # Note: We don't pause the gripper here so it keeps holding the object securely
        
        # 4. Lift up
        print("4. Lifting object...")
        ep_arm.moveto(x=LIFT_POSITION[0], y=LIFT_POSITION[1]).wait_for_completed()
        
        # 5. Wait
        print("5. Holding object for 3 seconds...")
        time.sleep(3.0)
        
        # 6. Put down
        print("6. Moving down to place object...")
        ep_arm.moveto(x=PICK_POSITION[0], y=PICK_POSITION[1]).wait_for_completed()
        time.sleep(0.5)
        
        # 7. Release
        print("7. Opening gripper (Dropping)...")
        ep_gripper.open(power=50)
        time.sleep(1.5)
        ep_gripper.pause()
        
        # 8. Return to ready state
        print("8. Returning to LIFT position...")
        ep_arm.moveto(x=LIFT_POSITION[0], y=LIFT_POSITION[1]).wait_for_completed()
        
        print("Sequence completed successfully.")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Closing connection...")
        ep_robot.close()

if __name__ == '__main__':
    main()
