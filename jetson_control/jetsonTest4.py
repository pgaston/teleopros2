import board
import busio
import digitalio
import pwmio
import time
import os

print("Initializing Motor Controller with Blinka...")

# Check available I2C devices first
print("Checking I2C devices...")
i2c_devices = []
for i in range(10):
    device_path = f"/dev/i2c-{i}"
    if os.path.exists(device_path):
        i2c_devices.append(device_path)
        print(f"Found: {device_path}")

if not i2c_devices:
    print("No I2C devices found in /dev/")

# Try to use PWM directly with Blinka
try:
    print("Testing PWM pins...")
    
    # Common Jetson Nano PWM pins
    # You may need to adjust these based on your wiring
    print("trying pwm 18")
    motor1_pwm = pwmio.PWMOut(board.D18, frequency=1000)  # GPIO 18

    print("worked 1")


    motor2_pwm = pwmio.PWMOut(board.D19, frequency=1000)  # GPIO 19
    
    # Direction control pins (if using H-bridge)
    motor1_dir1 = digitalio.DigitalInOut(board.D20)
    motor1_dir2 = digitalio.DigitalInOut(board.D21)
    motor2_dir1 = digitalio.DigitalInOut(board.D22)
    motor2_dir2 = digitalio.DigitalInOut(board.D23)
    
    # Set direction pins as outputs
    motor1_dir1.direction = digitalio.Direction.OUTPUT
    motor1_dir2.direction = digitalio.Direction.OUTPUT
    motor2_dir1.direction = digitalio.Direction.OUTPUT
    motor2_dir2.direction = digitalio.Direction.OUTPUT
    
    print("✅ PWM pins initialized successfully!")
    



except Exception as e:
    print(f"❌ PWM initialization failed: {e}")
    print("Trying alternative approach with MotorKit...")


