'''
Test LED light control.

60% or so is nice, not glaring.


Hook up:
nano / pca9685
 1 - VCC
 2 - V+
 3 - SDA
 4 - open
 5 - SCL
 6 - GND

 LED (headlight) is on channel 15

 Uses adafruit_pca9685 library for PWM control

Could get fancy and use a class for each channel, but this is simpler for now.
'''

import board
import busio
import adafruit_pca9685
import time

class PCA9685PWMlight: # PCA9685 based controller for LED light
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        print("I2C 1 ok!")

        self.pca = adafruit_pca9685.PCA9685(i2c, address=0x40)
        self.pca.frequency = 1000  # Higher frequency for LED dimming

        # Capture LED channel (15 for headlight)
        self.led_channel = self.pca.channels[15]

    def set_LightBrightness(self, brightness):
        """
        Set LED brightness
        brightness: 0.0 (off) to 1.0 (full brightness)
        """
        brightness = max(0.0, min(1.0, brightness))  # clamp
        duty_cycle = int(brightness * 65535)         # Scale to 16-bit
        self.led_channel.duty_cycle = duty_cycle
        print(f"LED brightness set to {brightness*100:.1f}% (duty cycle: {duty_cycle})")

    def done(self):
        # Turn off LED
        self.led_channel.duty_cycle = 0
        self.pca.deinit()
        print("PCA9685 PWM Light Controller complete!")

def light_test():
    light = PCA9685PWMlight()

    """Test LED brightness"""
    print("Testing LED brightness...")
    time.sleep(1)
    
    try:
        for brightness in [x * 0.1 for x in range(11)]:  # 0.0 to 1.0
            print(f"Setting brightness to {brightness*100}%...")
            light.set_LightBrightness(brightness)
            time.sleep(0.5)
        
        for brightness in [x * 0.1 for x in range(10, -1, -1)]:  # 1.0 to 0.0
            print(f"Setting brightness to {brightness*100}%...")
            light.set_LightBrightness(brightness)
            time.sleep(0.5)
        
        print("LED brightness test completed!")
        
    except Exception as e:
        print(f"LED brightness test failed: {e}")
    finally:
        light.done()



if __name__ == "__main__":
    light_test()
    # servo_test()
    # motor_test()
    # steerDrive_test()
