import board
import busio
import adafruit_pca9685
import time


# Hyric ESC

# arm - set to neutral pulse

# switch to reverse mode
# - neutral, full reverse, neutral

# switch to forward mode
# nothing needed

i2c = busio.I2C(board.SCL, board.SDA)
print("I2C 1 ok!")

pca = adafruit_pca9685.PCA9685(i2c, address=0x40)
pca.frequency = 50

print("pca=", pca)

motor_pwm = pca.channels[1]    # Speed control

class ESCMotorController:
    def __init__(self, channel):
        self.channel = channel
        self.initialized = False
        self.forward = True
        
        # ESC PWM values (adjust these based on your ESC)
        self.min_pulse = 3277    # ~1ms (full reverse or minimum)
        self.neutral_pulse = 4915 # ~1.5ms (neutral/stop)
        self.max_pulse = 6553    # ~2ms (full forward or maximum)
        
        # Initialize ESC
        self.initialize_esc()
    
    def initialize_esc(self):
        """Initialize ESC - send neutral signal for a few seconds"""
        print("Initializing ESC...")
        self.channel.duty_cycle = self.neutral_pulse
        time.sleep(2)  # ESCs typically need 2-3 seconds to initialize
        print("ESC initialized!")
        self.initialized = True
        self.forward = True
    
    def set_speed(self, speed):
        """
        Set motor speed through ESC
        speed: -1.0 to 1.0 
        - For bidirectional ESCs: negative = reverse, positive = forward
        - For unidirectional ESCs: 0 = stop, positive = forward
        """
        if not self.initialized:
            print("ESC not initialized!")
            return
        
        # Clamp speed to valid range
        speed = max(-1.0, min(1.0, speed))

        if speed < 0 and self.forward:
            print("Switching to reverse mode")
            self.forward = False
            # Some ESCs may require a neutral signal before changing direction
            self.channel.duty_cycle = self.neutral_pulse
            time.sleep(.1)
            self.channel.duty_cycle = self.min_pulse
            time.sleep(.1)
            self.channel.duty_cycle = self.neutral_pulse
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
        
        self.channel.duty_cycle = duty_cycle
        print(f"Motor speed: {speed*100:.1f}% (duty cycle: {duty_cycle})")
    
    def stop(self):
        """Stop motor"""
        self.set_speed(0)
    
    def arm_esc(self):
        """Arm the ESC (some ESCs require this sequence)"""
        print("Arming ESC...")
        # Send minimum signal
        # self.channel.duty_cycle = self.min_pulse
        # time.sleep(1)
        # Send maximum signal
        # self.channel.duty_cycle = self.max_pulse
        # time.sleep(1)
        # Return to neutral
        print("arm - neutral=", self.neutral_pulse)
        self.channel.duty_cycle = self.neutral_pulse
        time.sleep(1)
        print("ESC armed!")


def motor_test():
    """Test motor movement with ESC"""
    print("Testing ESC motor control...")
    
    # Create motor controller
    motor = ESCMotorController(motor_pwm)
    
    try:
        # Uncomment this if your ESC needs arming
        motor.arm_esc()
        
        # print("Testing forward speeds...")
        # # for speed in [0.2, 0.5, 0.8, 1.0]:
        for speed in [0.10]:
            print(f"Forward {speed*100}%...")
            motor.set_speed(speed)
            time.sleep(2)
        
        print("Stopping ...")
        motor.stop()
        time.sleep(2)
        
        # Test reverse (if your ESC supports bidirectional)
        print("Testing reverse speeds...")
        # for speed in [-0.2, -0.5, -0.8, -1.0]:
        for speed in [-0.10]:
            print(f"Reverse {abs(speed)*100}%...")
            motor.set_speed(speed)
            time.sleep(2)
        
        print("Stopping")
        motor.stop()
        time.sleep(2)

        # print("Testing forward speeds...")
        # # for speed in [0.2, 0.5, 0.8, 1.0]:
        for speed in [0.10]:
            print(f"Forward {speed*100}%...")
            motor.set_speed(speed)
            time.sleep(2)
 

        print("Final stop...")
        motor.stop()
        time.sleep(1)
        
        print("ESC motor test completed!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        motor.stop()
    except Exception as e:
        print(f"Motor test failed: {e}")
        motor.stop()
    finally:
        motor.stop()
        pca.deinit()

def simple_speed_test():
    """Simple speed ramping test"""
    motor = ESCMotorController(motor_pwm)
    
    try:
        print("Speed ramping test...")
        
        # Ramp up
        for i in range(0, 101, 10):
            speed = i / 100.0
            motor.set_speed(speed)
            time.sleep(0.5)
        
        # Ramp down
        for i in range(100, -1, -10):
            speed = i / 100.0
            motor.set_speed(speed)
            time.sleep(0.5)
        
        motor.stop()
        print("Speed ramping test completed!")
        
    except Exception as e:
        print(f"Speed test failed: {e}")
        motor.stop()
    finally:
        pca.deinit()

# For unidirectional ESC (forward only)
def unidirectional_test():
    """Test for ESCs that only support forward direction"""
    motor = ESCMotorController(motor_pwm)
    
    # Adjust pulse values for unidirectional ESC
    motor.min_pulse = 3277    # Minimum throttle
    motor.neutral_pulse = 3277 # Same as minimum (no reverse)
    motor.max_pulse = 6553    # Maximum throttle
    
    try:
        print("Unidirectional ESC test...")
        
        speeds = [0, 0.2, 0.5, 0.8, 1.0, 0.5, 0]
        for speed in speeds:
            print(f"Speed: {speed*100}%...")
            motor.set_speed(speed)
            time.sleep(2)
        
        print("Unidirectional test completed!")
        
    except Exception as e:
        print(f"Unidirectional test failed: {e}")
        motor.stop()
    finally:
        pca.deinit()

if __name__ == "__main__":
    # Choose which test to run:
    
    # Standard bidirectional ESC test
    motor_test()
    
    # Or simple speed ramping
    # simple_speed_test()
    
    # Or unidirectional ESC test
    # unidirectional_test()