'''
Control a servo motor and ESC drive motor using PCA9685 on Jetson Orin Nano
- ESC is a Hyric brushless motor controller - Brush-X60-RTR
    Arming: set to neutral for 2 seconds
    Switch to reverse: set to neutral, then min pulse, then neutral again
    Switch to forward: no change needed, just set to neutral
- pwm 0 is for steering servo
- pwm 1 is for drive motor (ESC)

Control range is from -1 to 0 to 1
- Steering - -1 is full left, 0 is center, 1 is full right
- Drive - -1 is full reverse, 0 is stop, 1 is full forward

Uses adafruit_pca9685 library for PWM control

Could get fancy and use a class for each channel, but this is simpler for now.
'''

import board
import busio
import adafruit_pca9685
import time



class SteeringDriveMotorController: # PCA9685 based controller for steering servo and ESC drive moto
    def __init__(self):
        self.Servoinitialized = False
        self.ESCinitialized = False

        i2c = busio.I2C(board.SCL, board.SDA)
        print("I2C 1 ok!")

        self.pca = adafruit_pca9685.PCA9685(i2c, address=0x40)
        self.pca.frequency = 50

        # Capture servo and drive motor channels
        self.servo_channel = self.pca.channels[0]
        self.drive_channel = self.pca.channels[1]

        # PWM values (adjust these based on your ESC)
        # Same for the servo as well
        self.min_pulse = 3277    # ~1ms (full reverse or minimum)
        self.neutral_pulse = 4915 # ~1.5ms (neutral/stop)
        self.max_pulse = 6553    # ~2ms (full forward or maximum)

        # Initialize steering servo
        print("Initializing steering servo...")
        self.servo_channel.duty_cycle = self.neutral_pulse
        self.Servoinitialized = True

        # Initialize ESC
        print("Initializing ESC...")
        self.drive_channel.duty_cycle = self.neutral_pulse
        time.sleep(2)  # ESCs typically need 2-3 seconds to initialize
        print("ESC initialized!")
        self.forward = True
        self.ESCinitialized = True

    # raw - set duty cycle directly - range from -1.0 to 0 to 1.0
    def set_MotorSpeed(self, channel, speed):
        speed = max(-1.0, min(1.0, speed))      # clamp
        if speed == 0:
            duty_cycle = self.neutral_pulse     # Neutral position
        elif speed > 0:
            # Forward direction
            duty_cycle = int(self.neutral_pulse + speed * (self.max_pulse - self.neutral_pulse))
        else:
            # Reverse direction (if ESC supports it - ours does)
            duty_cycle = int(self.neutral_pulse + speed * (self.neutral_pulse - self.min_pulse))
        
        channel.duty_cycle = duty_cycle         # send to actual motor
        print(f"Motor speed: {speed*100:.1f}% (duty cycle: {duty_cycle})")

    def done(self):
        # Turn off PWM signal to servo and ESC
        self.servo_channel.duty_cycle = 0
        self.drive_channel.duty_cycle = 0
        self.pca.deinit()
        print("Steering and Drive Motor Controller complete!")
 
    def set_SteerDrive(self, steer_angle, drive_speed):
        """
        Set both steering angle and drive speed
        steer_angle: 0-180 degrees for servo
        drive_speed: -1.0 to 1.0 for ESC (negative = reverse, positive = forward)
        """
        self.set_Servoangle(steer_angle)
        self.set_Drivespeed(drive_speed)

    def set_SteeringServo(self, steer):
        if not self.Servoinitialized:
            print("Servo not initialized!")
            return
        
        self.set_MotorSpeed(self.servo_channel, steer)  # Ensure ESC is stopped before changing steering
        print(f"Steering servo set to {steer}")


    def set_Drivespeed(self, speed):
        """
        Set motor speed through ESC
        speed: -1.0 to 1.0 
        - For bidirectional ESCs: negative = reverse, positive = forward
        - For unidirectional ESCs: 0 = stop, positive = forward
        """
        if not self.ESCinitialized:
            print("ESC not initialized!")
            return
        
        if speed < 0 and self.forward:
            print("Switching to reverse mode")
            self.forward = False
            # Some ESCs may require a neutral signal before changing direction
            self.drive_channel.duty_cycle = self.neutral_pulse
            time.sleep(.1)
            self.drive_channel.duty_cycle = self.min_pulse
            time.sleep(.1)
            self.drive_channel.duty_cycle = self.neutral_pulse
            time.sleep(.1)
   
        elif speed >= 0 and not self.forward:
            print("Switching to forward mode")
            self.forward = True
            # no change needed

        self.set_MotorSpeed(self.drive_channel, speed)  # Ensure ESC is stopped before changing steering
        print(f"Motor speed set to {speed}")
    
    def set_SteerDrive(self, steer, speed):
        self.set_Drivespeed(speed)
        self.set_SteeringServo(steer)

    def reset(self):
        self.set_SteerDrive(0, 0)



def servo_test():
    motors = SteeringDriveMotorController()

    """Test servo movement"""
    print("Testing servo movement...")
    time.sleep(1)
    
    try:
        # Move to 0 degrees
        motors.set_SteeringServo(-0.5)
        time.sleep(1)
        
        # Move to 90 degrees (center)
        motors.set_SteeringServo(0)
        time.sleep(1)
        
        # Move to 180 degrees
        motors.set_SteeringServo(0.5)
        time.sleep(1)
        
        # Return to center
        motors.set_SteeringServo(0)
        time.sleep(1)

         # Ready for sweep
        motors.set_SteeringServo(-0.6)
        time.sleep(1)
       
        # Sweep test
        print("Performing sweep test...")
        for angle in [x * 0.1 for x in range(-6, 7)]:  # Works with floats
            motors.set_SteeringServo(angle)
            time.sleep(0.1)

        for angle in [x * 0.1 for x in range(6, -7, -1)]:
            motors.set_SteeringServo(angle)
            time.sleep(0.1)
        
        # Return to center and stop
        motors.set_SteeringServo(0)
        print("Servo test completed!")
        
    except Exception as e:
        print(f"Servo test failed: {e}")
    finally:
        motors.done()


def motor_test():
    """Test motor movement with ESC"""
    print("Testing ESC motor control...")
    
    motors = SteeringDriveMotorController()

    
    try:
        # print("Testing forward speeds...")
        # # for speed in [0.2, 0.5, 0.8, 1.0]:
        for speed in [x * 0.1 for x in range(-3,4)]:
            print(f"Speed {speed*100}%...")
            motors.set_Drivespeed(speed)
            time.sleep(1)
        
        # print("Stopping ...")
        # motors.reset()
        # time.sleep(2)
        
        # # Test again (if your ESC supports bidirectional)
        # print("Testing reverse speeds...")
        # # for speed in [-0.2, -0.5, -0.8, -1.0]:
        # for speed in [x * 0.1 for x in range(3, -3,-1)]:
        #     print(f"Speed {abs(speed)*100}%...")
        #     motors.set_Drivespeed(speed)
        #     time.sleep(2)
        
        # print("Stopping")
        # motors.reset()
        # time.sleep(2)

        # # print("Testing forward speeds...")
        # # # for speed in [0.2, 0.5, 0.8, 1.0]:
        # for speed in [x * 0.1 for x in range(1)]:
        #     print(f"Forward {speed*100}%...")
        #     motors.set_Drivespeed(speed)
        #     time.sleep(2)
 

        print("Final stop...")
        motors.reset()
        time.sleep(1)
        
        print("ESC motor test completed!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        motors.reset()
    except Exception as e:
        print(f"Motor test failed: {e}")
        motors.reset()
    finally:
        motors.done()


def steerDrive_test():
    print("Testing steering and drive control...")
    
    motors = SteeringDriveMotorController()

    
    try:
        # print("Testing forward speeds...")
        # # for speed in [0.2, 0.5, 0.8, 1.0]:
        motors.set_SteerDrive(.1,0.10)
        motors.set_SteerDrive(1,0.10)
        time.sleep(2)
        
        print("Stopping ...")
        motors.reset()
        time.sleep(0.5)

        # print("reverse speeds...")
        # motors.set_SteerDrive(-.1,-0.10)
        # time.sleep(2)      

        # print("Stopping ...")
        # motors.reset()
        # time.sleep(0.5)

        # print("forward speeds...")
        # motors.set_SteerDrive(.1,0.10)
        # time.sleep(2)      

        # print("Stopping ...")
        # motors.reset()

        # time.sleep(0.5)


    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        motors.reset()
    except Exception as e:
        print(f"Motor test failed: {e}")
        motors.reset()

    finally:
        motors.done()



if __name__ == "__main__":
    # servo_test()
    # motor_test()
    steerDrive_test()
