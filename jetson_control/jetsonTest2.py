from adafruit_motorkit import MotorKit
import board
import busio


print("starting motor test")

try:
    # Try different I2C buses
    kit = MotorKit()
except OSError as e:
    print(f"Default I2C failed: {e}")
    try:
        # Try specific I2C bus
        i2c = busio.I2C(board.SCL, board.SDA)
        kit = MotorKit(i2c=i2c)
    except Exception as e2:
        print(f"Alternative I2C also failed: {e2}")
        raise

print("done")