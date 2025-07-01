import board
import busio
import adafruit_pca9685
import time


i2c = busio.I2C(board.SCL, board.SDA)
print("I2C 1 ok!")

pca = adafruit_pca9685.PCA9685(i2c, address=0x40)
pca.frequency = 50

print("pca=", pca)

servo_channel = pca.channels[0]
print("servo_channel=", servo_channel)

def set_servo_angle(channel, angle):
    """
    Set servo to specific angle (0-180 degrees)
    Most servos use:
    - 1ms pulse (0 degrees) = ~3277 duty cycle
    - 1.5ms pulse (90 degrees) = ~4915 duty cycle  
    - 2ms pulse (180 degrees) = ~6553 duty cycle
    """
    # Convert angle to duty cycle
    # duty_cycle = int((angle / 180.0) * (6553 - 3277) + 3277)
    min_duty = 3277   # 1ms pulse width
    max_duty = 6553   # 2ms pulse width
    duty_cycle = int(min_duty + (angle / 180.0) * (max_duty - min_duty))
    
    channel.duty_cycle = duty_cycle
    print(f"Servo set to {angle}° (duty cycle: {duty_cycle})")

def servo_test():
    """Test servo movement"""
    print("Testing servo movement...")
    
    try:
        # Move to 0 degrees
        set_servo_angle(servo_channel, 0)
        time.sleep(1)
        
        # Move to 90 degrees (center)
        set_servo_angle(servo_channel, 90)
        time.sleep(1)
        
        # Move to 180 degrees
        set_servo_angle(servo_channel, 180)
        time.sleep(1)
        
        # Return to center
        set_servo_angle(servo_channel, 90)
        time.sleep(1)
        
        # Sweep test
        print("Performing sweep test...")
        for angle in range(0, 181, 10):
            set_servo_angle(servo_channel, angle)
            time.sleep(0.1)
        
        for angle in range(180, -1, -10):
            set_servo_angle(servo_channel, angle)
            time.sleep(0.1)
        
        # Return to center and stop
        set_servo_angle(servo_channel, 90)
        servo_channel.duty_cycle = 0  # Turn off PWM signal
        
        print("Servo test completed!")
        
    except Exception as e:
        print(f"Servo test failed: {e}")
    finally:
        # Always turn off PWM when done
        servo_channel.duty_cycle = 0
        pca.deinit()

if __name__ == "__main__":
    servo_test()