from robomaster import robot
import time

def main():
    print("Initializing robot...")
    ep_robot = robot.Robot()
    
    # Initialize connection (using 'ap' which is the default for direct Wi-Fi)
    ep_robot.initialize(conn_type="ap")
    
    try:
        ep_gripper = ep_robot.gripper
        
        print("Opening gripper (power=50%)...")
        # Open the gripper with 50% power
        ep_gripper.open(power=50)
        time.sleep(1.5) # Wait for it to fully open
        
        # Pause to stop the motor from constantly pushing
        ep_gripper.pause()
        print("Gripper is now open.")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Closing connection...")
        ep_robot.close()

if __name__ == '__main__':
    main()
