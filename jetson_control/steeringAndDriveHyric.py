'''
Control a servo motor and ESC drive motor using PCA9685 on Jetson Orin Nano
- ESC is a Hyric brushless motor controller - Brush-X60-RTR
    Arming: set to neutral for 2 seconds
    Switch to reverse: set to neutral, then min pulse, then neutral again
    Switch to forward: no change needed, just set to neutral
- pwm 0 is for steering servo
- pwm 1 is for drive motor (ESC)

One class for both steering and drive motor control.

Uses adafruit_pca9685 library for PWM control
'''

import board
import busio
import adafruit_pca9685
import time

class SteeringDriveMotorController:
    def __init__(self):
        self.Servoinitialized = False
        self.ESCinitialized = False

        i2c = busio.I2C(board.SCL, board.SDA)
        print("I2C 1 ok!")

        self.pca = adafruit_pca9685.PCA9685(i2c, address=0x40)
        self.pca.frequency = 50

        print("pca=", self.pca)

        # Capture servo and drive motor channels
        self.servo_channel = self.pca.channels[0]
        self.drive_channel = self.pca.channels[1]

        # PWM values (adjust these based on your ESC)
        # There are the same for the servo as well
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

    def done(self):
        # self.stop_Drive()
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


    def set_Servoangle(self, angle):
        """
        Set servo to specific angle (0-180 degrees)
        Most servos use:
        - 1ms pulse (0 degrees) = ~3277 duty cycle
        - 1.5ms pulse (90 degrees) = ~4915 duty cycle  
        - 2ms pulse (180 degrees) = ~6553 duty cycle
        """
        if not self.Servoinitialized:
            print("Servo not initialized!")
            return
        # Convert angle to duty cycle
        # duty_cycle = int((angle / 180.0) * (6553 - 3277) + 3277)
        # min_duty = 3277   # 1ms pulse width
        # max_duty = 6553   # 2ms pulse width
        duty_cycle = int(self.min_pulse + (angle / 180.0) * (self.max_pulse - self.min_pulse))
        
        self.servo_channel.duty_cycle = duty_cycle
        print(f"Servo set to {angle}° (duty cycle: {duty_cycle})")

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
        
        # Clamp speed to valid range
        speed = max(-1.0, min(1.0, speed))

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
   
        elif speed > 0 and not self.forward:
            print("Switching to forward mode")
            self.forward = True
            # no change needed

        if speed == 0:
            # Neutral position
            duty_cycle = self.neutral_pulse
        elif speed > 0:
            # Forward direction
            duty_cycle = int(self.neutral_pulse + speed * (self.max_pulse - self.neutral_pulse))
        else:
            # Reverse direction (if ESC supports it)
            duty_cycle = int(self.neutral_pulse + speed * (self.neutral_pulse - self.min_pulse))
        
        self.drive_channel.duty_cycle = duty_cycle
        print(f"Motor speed: {speed*100:.1f}% (duty cycle: {duty_cycle})")
    
    def stop_Drive(self):
        """Stop ESC motor"""
        self.set_Drivespeed(0)


def servo_test():
    motors = SteeringDriveMotorController()

    """Test servo movement"""
    print("Testing servo movement...")
    time.sleep(1)
    
    try:
        # Move to 0 degrees
        motors.set_Servoangle(0)
        time.sleep(1)
        
        # Move to 90 degrees (center)
        motors.set_Servoangle(90)
        time.sleep(1)
        
        # Move to 180 degrees
        motors.set_Servoangle(180)
        time.sleep(1)
        
        # Return to center
        motors.set_Servoangle(90)
        time.sleep(1)
        
        # Sweep test
        print("Performing sweep test...")
        for angle in range(0, 181, 10):
            motors.set_Servoangle(angle)
            time.sleep(0.1)
        
        for angle in range(180, -1, -10):
            motors.set_Servoangle(angle)
            time.sleep(0.1)
        
        # Return to center and stop
        motors.set_Servoangle(90)
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
        for speed in [0.10]:
            print(f"Forward {speed*100}%...")
            motors.set_Drivespeed(speed)
            time.sleep(2)
        
        print("Stopping ...")
        motors.stop_Drive()
        time.sleep(2)
        
        # Test reverse (if your ESC supports bidirectional)
        print("Testing reverse speeds...")
        # for speed in [-0.2, -0.5, -0.8, -1.0]:
        for speed in [-0.10]:
            print(f"Reverse {abs(speed)*100}%...")
            motors.set_Drivespeed(speed)
            time.sleep(2)
        
        print("Stopping")
        motors.stop_Drive()
        time.sleep(2)

        # print("Testing forward speeds...")
        # # for speed in [0.2, 0.5, 0.8, 1.0]:
        for speed in [0.10]:
            print(f"Forward {speed*100}%...")
            motors.set_Drivespeed(speed)
            time.sleep(2)
 

        print("Final stop...")
        motors.stop_Drive()
        time.sleep(1)
        
        print("ESC motor test completed!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        motors.stop_Drive()
    except Exception as e:
        print(f"Motor test failed: {e}")
        motors.stop_Drive()
    finally:
        motors.done()


def steerDrive_test():
    print("Testing steering and drive control...")
    
    motors = SteeringDriveMotorController()

    
    try:
        # print("Testing forward speeds...")
        # # for speed in [0.2, 0.5, 0.8, 1.0]:
        motors.set_SteerDrive(70,0.10)
        time.sleep(2)
        
        print("Stopping ...")
        motors.set_SteerDrive(90,0.0)
        time.sleep(0.5)

        print("reverse speeds...")
        motors.set_SteerDrive(70,-0.10)
        time.sleep(2)      

        print("Stopping ...")
        motors.set_SteerDrive(90,0.0)
        time.sleep(0.5)

        print("forward speeds...")
        motors.set_SteerDrive(110,0.10)
        time.sleep(2)      

        print("Stopping ...")
        motors.set_SteerDrive(90,0.0)
        time.sleep(0.5)


    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        motors.stop_Drive()
    except Exception as e:
        print(f"Motor test failed: {e}")
        motors.stop_Drive()
    finally:
        motors.done()



if __name__ == "__main__":
    # servo_test()
    # motor_test()
    steerDrive_test()
