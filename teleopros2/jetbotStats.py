'''
Jetbot Stats
- Display Jetbot stats on the OLED display
    - Capture Platform, CPU, RAM, Disk, Temperature, Battery, and IP Address
- Publish stats on ROS2 node for teleop consumption

While psutil is a great library for getting system stats, it doesn't deliver GPU utilization, one of the interesting things.
So I'll just stick w/ the shell commands for now.

Most of the code is from github installPiOLED - though I've switched the up-to-date OLED drawing
'''

import subprocess       # to get info from command line calls
import os
import time
import json

import platform

## Access to I2C bus to write to the OLED display
import board
import busio
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont


# ros2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String     # for carrying json to/from the browser - ROS2 


# get platform - aarch64 for us will generally mean Jetson (Orin) (Nano)
def get_platform():
    return platform.machine()

# get's local IP address
def get_ip_address(interface):    # from https://forums.raspberrypi.com/viewtopic.php?t=79936
    aRoutes = os.popen("ip -j -4 route").read()
    routes = json.loads(aRoutes)
    for r in routes:
        if r.get("dev") == interface and r.get("prefsrc"):
            ip = r['prefsrc']
            return ip
    return "None"

def get_cpu_usage():
    # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
    cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
    CPU = subprocess.check_output(cmd, shell=True)
    return CPU.decode('utf-8')

def get_ram():
    # Get RAM information
    cmd = "free -m | awk 'NR==2{printf \"Mem:  %.0f%% %s/%s M\", $3*100/$2, $3,$2 }'"
    MemUsage = subprocess.check_output(cmd, shell=True)
    return MemUsage.decode('utf-8')

def get_disk():
    # Get disk information
    cmd = "df -h | awk '$NF==\"/\"{printf \"Disk: %d/%dGB %s\", $3,$2,$5}'"
    DiskUsage = subprocess.check_output(cmd, shell=True)
    return DiskUsage.decode('utf-8')

def get_temperature():
    # Get CPU temperature - in Celsius
    # cmd = "/opt/vc/bin/vcgencmd measure_temp | cut -d '=' -f2"
    cmd = "cat /sys/class/thermal/thermal_zone0/temp"
    temp = subprocess.check_output(cmd, shell=True)
    iTemp = int(temp)/1000.0
    return "{:.1f}".format(iTemp)

def get_battery():
    # Get Battery information
    # nah - copilot made this one up...
    # cmd = "cat /sys/class/power_supply/BAT0/capacity"
    # battery = subprocess.check_output(cmd, shell=True)
    battery = 0.85      # placeholder for now
    return battery

# Return a float representing the percentage of GPU in use.
# On the Jetson Nano, the GPU is GPU0
def get_gpu_usage():
    GPU = 0.0
    with open("/sys/devices/gpu.0/load", encoding="utf-8") as gpu_file:
        GPU = gpu_file.readline()
        GPU = int(GPU)/10
    return "{:.1f}".format(GPU)


## ROS2 Node to publish stats
# WebRTC node publish/subscribe
class PublishStats(Node):

    def __init__(self):
        super().__init__('jetbotstats')

        # parameters
        self.declare_parameter('json-topic', 'teleoppub')
        argsjsontopic = self.get_parameter('json-topic').value

        # publishers
        self.jPublisher = self.create_publisher(String, argsjsontopic, 10)

        timer_period = 5.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # OLED display
        # 128x32 display with hardware I2C:
        # setting gpio to 1 is hack to avoid platform detection
        i2c = busio.I2C(board.SCL, board.SDA)
        self.oled = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c)

        self.image = Image.new("1", (self.oled.width, self.oled.height))
        # Get drawing object to draw on image.
        self.draw = ImageDraw.Draw(self.image)
        self.font = ImageFont.load_default()


        # Draw a white background
        self.draw.rectangle((0, 0, self.oled.width, self.oled.height), outline=255, fill=255)

        BORDER = 5
        # Draw a smaller inner rectangle
        self.draw.rectangle(
            (BORDER, BORDER, self.oled.width - BORDER - 1, self.oled.height - BORDER - 1),
            outline=0,
            fill=0,
        )

        # Draw Some Text
        localIP = get_ip_address('wlan0')    # get local IP address
        text = localIP
        (font_width, font_height) = self.font.getsize(text)
        self.draw.text(
            (self.oled.width // 2 - font_width // 2, self.oled.height // 2 - font_height // 2),
            text,
            font=self.font,
            fill=255,
        )

        self.oled.image(self.image)
        self.oled.show()


    def timer_callback(self):
        # collect data
        machine = get_platform()
        localIP = get_ip_address('wlan0')    # get local IP address
        cpuUsage = get_cpu_usage()
        ramUsage = get_ram()
        diskUsage = get_disk()
        temperature = get_temperature()
        battery = get_battery()
        gpuUsage = get_gpu_usage()

        ##
        # OLED display
        # we want localIP - that's it for now
        # Erase
        self.draw.rectangle((0, 0, self.oled.width, self.oled.height), outline=0, fill=0)
        padding = -2
        top = padding
        bottom = self.oled.height-padding
        self.draw.text((0, top), "wlan0: "+localIP, font=self.font, fill=255)
        self.draw.text((0, top+8), cpuUsage, font=self.font, fill=255)
        self.draw.text((0, top+16), "GPU: "+gpuUsage+"%", font=self.font, fill=255)
        self.draw.text((0, top+24), "temp: "+temperature+"C", font=self.font, fill=255)

        self.oled.image(self.image)
        self.oled.show()

        ##
        # ROS2 message
        jStats = {
            'localIP': localIP, 
            'cpuUsage': cpuUsage, 
            'gpuUsage': gpuUsage,
            'temperature': temperature,
            'ramUsage': ramUsage, 
            'diskUsage': diskUsage, 
            'machine': machine, 
            'battery': battery,
            }
        
        msg = String()
        msg.data = json.dumps(jStats)
        self.jPublisher.publish(msg)


def main():

    # machine = get_platform()
    # localIP = get_ip_address('wlan0')    # get local IP address
    # cpuUsage = get_cpu_usage()
    # ramUsage = get_ram()
    # diskUsage = get_disk()
    # temperature = get_temperature()
    # battery = get_battery()
    # gpuUsage = get_gpu_usage()

    # print(f"Machine: {machine}")
    # print(f"Local IP: {localIP}")
    # print(f"CPU Usage: {cpuUsage}")
    # print(f"RAM Usage: {ramUsage}")
    # print(f"Disk Usage: {diskUsage}")
    # print(f"Temperature: {temperature}")
    # print(f"Battery: {battery}")
    # print(f"GPU Usage: {gpuUsage}")

    # print("type of cpuUsage: ", type(cpuUsage))
    # print("cpuUsage: ", cpuUsage)


    # exit()



    rclpy.init()
    StatsNode = PublishStats()     # do this sequentially, as it captures the parameters

    rclpy.spin(StatsNode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    StatsNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

