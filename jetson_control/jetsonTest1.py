#!/usr/bin/env python3

import os
import subprocess

def test_i2c_access():
    print("Testing I2C access...")
    
    # Check if I2C devices exist
    i2c_devices = [f"/dev/i2c-{i}" for i in range(8)]
    existing_devices = [dev for dev in i2c_devices if os.path.exists(dev)]
    
    print(f"Available I2C devices: {existing_devices}")
    
    if not existing_devices:
        print("ERROR: No I2C devices found!")
        return False
    
    # Test I2C detection
    for device in existing_devices:
        bus_num = device.split('-')[1]
        try:
            result = subprocess.run(['i2cdetect', '-y', bus_num], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                print(f"I2C bus {bus_num} is accessible")
                print(result.stdout)
            else:
                print(f"I2C bus {bus_num} failed: {result.stderr}")
        except Exception as e:
            print(f"Error testing I2C bus {bus_num}: {e}")
    
    return True

if __name__ == "__main__":
    test_i2c_access()