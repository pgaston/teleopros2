'''
To do's:
 - get to work, again
 - then w/ Isaac SIM
 - add https certificate (?)

 - Physically separate the Web server from WebRTC (this code) on bot
    - Allows usage of web site even when disconnected from bot
    - May not need https for WebRTC (?)

 - Python portal-esque framework for the web server
 - Better object architecture on web server allowing customization/enhancement

**************************

*Note* - for use over the internet STUN seems to be required.

teleopros2 - Browser interface to ROS2 robot
 - Web server - provides simple page with video and robot movement controls
 - Stream video back using WebRTC (thanks to aiortc)
 - Data channel - can send/receive JSON messages which are converted to/from ROS2 messages
   (yes, ros2-web-bridge exists - integrating here simplifies things)
 - Focused on supporting the NVidia stack - Jetson (Orin) Nano, Isaac Sim (for simulation)
   (will work elsewhere w/ some modifications)
 - Simplyfing assumptions - to be validated or changed over time.

Open issues:
 - performance - how to optimize for the Jetson Nano (Orin) - use of GPU for image processing.   
   resize, color conversion, h.264 encoding using Deepstream/gstreamer 
   - project ros_deep_learning could be a node to use
   - i.e., accept as topics either:
     - 'image' - and do the conversion on the CPU, or
     - 'image_gpu' - and use the already converted/encoded image
       see https://github.com/aiortc/aiortc/discussions/769
       oh, and if you see this topic, ignore all 'image' topics... (unsubscribe)
 - Interesting, that when the iPad starts running, the server shows that it's using the GPU,
           this doesn't happen for desktop or Android Chrome.   (I assume for h264 encoding and ???)
           The NVidia stack can do this, but it's not integrated into this code stack.   
           Worth looking into at some point.
         - Python - heck, the simplicity and robustness of aiortc have me sold.
 - https - throws up warning to user given the lack of a valid certificate.  
    - self-signed certificate is fine for local use, but not for general use.
        - create true certificate - https://letsencrypt.org/
        - use duckdns/other for dynamic DNS
    - also aiortc shows a couple of deprecation warnings around ssl - ignore for now...
 - logging - currently using a non ROS2 logger - hence logging doesn't show up when running as a ROS2 node.
           - for now, debug using 'python3 teleopros2.py'
 - watchdog - periodic message each way, but not sure what functionality this is adding at present - leaving it in


Limitations:
 - ROS2, Python, (NVidia stack)
 - Web connectivity to browser (have to play with Autonomous/Manual disconnect/reconnect...)

 *************
Requires https for remote access for mobile tilt.   This is turned on by default - you can always run http as desired.

Create your own local certificate/key and place in the certs directory.   
Google, or see https://deliciousbrains.com/ssl-certificate-authority-for-local-https-development/#how-it-
This throws security warnings when opening in a browser, but it's a start.   For real use, get a real certificate which 
requires a real domain name.   

One way to do this correctly:
    - duckdns.org - free dynamic DNS - get a domain name
    - using apache...  https://certbot.eff.org/, https://letsencrypt.org/

***************************

to run as python only...
cd ${ISAAC_ROS_WS}/src/TeleOpROS2/teleopros2
python3 teleopros2.py

then open your browser to https://localhost:8080

to run as ROS2 node

to start nvidia docker:
************************************************************************
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh
************************************************************************

source install/setup.bash
ros2 run teleopros2 teleopros2_node

*then realsense in another terminal window*
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py

*or, in a single launch file
source install/setup.bash
ros2 launch teleopros2 teleRSCamera_launch.py


*test if realsense seems off...*
realsense-viewer
*that, and use a third window to look at published topics - make sure all are there*

*that, and use a third window to look at published topics - make sure all are there*

to run in simulator - ISAAC SIM

Make sure Nucleus and all it's services are running - http://localhost:3080/ --> restart all
Start Isaac SIM
(I'm using 'simple room' - needs ROS2 to work...)
The code below shows the image topic Isaac SIM emites - you'll need to change for that.

*to use gstreamer to use your webcam*
- install gscam
- this assumes you have a camera.ini file in the cfg directory
- this assumes you have a gscam_params.yaml file in the cfg directory
  change the /dev/videoX to be your webcam (for this example)

source install/setup.bash
ros2 run gscam2 gscam_main --ros-args  --params-file src/gscam2/gscam_params.yaml -p camera_info_url:=file://$PWD/src/gscam2/cfg/my_camera.ini
-or-

source install/setup.bash
ros2 launch teleopros2 teleGSCamera_launch.py



*************
Build instructions: (docker on Jetson (Orin) Nano)

to start nvidia docker:

cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh

to build: (using symlink to make debugging easier...)

cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-select teleopros2
source install/setup.bash
ros2 run teleopros2 teleopros2
'''

import asyncio
import json
import logging
import os
import platform
import ssl
import uuid
import threading
import time
import fractions
import numpy as np
from typing import Tuple

import cv2
from aiohttp import web
from av import VideoFrame
from av.frame import Frame

from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaBlackhole, MediaPlayer, MediaRecorder

# ros2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String     # for carrying json to/from the browser - ROS2 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image   # image, internal to ROS2 (and eventually published to browser via WebRTC)
from geometry_msgs.msg import Twist


####################################
# Groups of code:
# - WebRTC
# - WebRTC datachannel
# - ROS2 node
# - Main
####################################

# Parameters we support via ROS2
argstwisttopic = None           # ROS2 topic for the incoming twist, (cmd_vel)
argsjsontopic = None           # ROS2 topic for the publishing json messages, (teleoppub)
argsimagetopic = None           # ROS2 topic for the incoming image, ()RealSense)
argsssl= False                  # use SSL, (true)
argscertfile = None             # (certs/server.cert)
argskeyfile = None              # (certs/server.key)
argshost = None                 # (0.0.0.0)
argsport = None                 # (8080) - note that Isaac SIM uses 8080, in with case, say use 8081
argsfps = None                  # frames per second (15)
argsverbose = False             # default logging verbosity (False - show Info, not Debug)


height, width = 480, 640
logger = None       # set in init of the ROS2 node

####################################
# WebRTC - Code derived from server.py in aiortc
####################################


# from mediastreams.py
AUDIO_PTIME = 0.020  # 20ms audio packetization
VIDEO_CLOCK_RATE = 90000
VIDEO_PTIME = 1 / 30  # 30fps
VIDEO_PTIME = 1 / 15  # 30fps       # this gets reset based on a ROS2 parameter
VIDEO_TIME_BASE = fractions.Fraction(1, VIDEO_CLOCK_RATE)

class MediaStreamError(Exception):
    pass

ROOT = os.path.dirname(__file__)
HOST_IP = os.getenv('HOST_IP', "0.0.0.0")
pcs = set()     # can support multiple connections - we only care about one
pc = None       # the one
dc = None       # data channel - None until we have one
Ros2PubSubNode = None       # set to a valid object when ROS2 node is running

# current image - to be sent to the browser
# either a static image, or the last image received from ROS2
# we defer image conversion until the last moment, as we only use half of the images (15fps use, generated at 30fps)
class currentImage():
    def __init__(self):
        self.ros2Image = None
        self.defaultImg = np.zeros((height, width, 3), np.uint8)
        self.resetImg()

    def resetImg(self):     # called when channel is closed
        self.ros2Image = None
        self.defaultImg[:, :] = (64, 64, 64)        # gray
        # self.img[:, :] = (0, 0, 200)         # different - this is red

    def setImg(self, ros2Image):
        # print("setImg",ros2Image)
        self.ros2Image = ros2Image

    def getImg(self):
        # print("getImg",self.ros2Image)    ## !!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if self.ros2Image is not None:
            # Note: currently GStreamer returns a 640x480x3 image in BGR format
            # we could convert there to RGB - but convention (?) says to use internally in BGR
 
            ## Conversion - BGR to RGB for nice display
            cv_image = cv2.cvtColor(self.ros2Image, cv2.COLOR_BGR2RGB)

            # check to see if need to resize ?
            if cv_image.shape[0] != height or cv_image.shape[1] != width:
                cv_image = cv2.resize(cv_image, (width, height))

            return cv_image
        else:
            return self.defaultImg
    
curImg = currentImage()

### send a video frame to the browser
# can set fps
class VideoStreamTrack(MediaStreamTrack):
    kind = "video"

    _start: float
    _timestamp: int

    def __init__(self):
        super().__init__()  # don't forget this!
        curImg.resetImg()   # so we don't get an older image

    async def next_timestamp(self) -> Tuple[int, fractions.Fraction]:
        if self.readyState != "live":
            raise MediaStreamError

        if hasattr(self, "_timestamp"):
            self._timestamp += int(VIDEO_PTIME * VIDEO_CLOCK_RATE)
            wait = self._start + (self._timestamp / VIDEO_CLOCK_RATE) - time.time()
            await asyncio.sleep(wait)
        else:
            self._start = time.time()
            self._timestamp = 0
        return self._timestamp, VIDEO_TIME_BASE

    def setImg(self, img):
        self.currentImg = img       # atomically (hopefully)...

    async def recv(self):
        img = curImg.getImg()
        frame = VideoFrame.from_ndarray(img,format="bgr24")
        pts, time_base = await self.next_timestamp()
        frame.pts = pts
        frame.time_base = time_base
        return frame

    def __del__(self):
        logger.debug("VideoStreamTrack deleted")

## Web server
async def index(request):       # a blank url defaults to index.html
    content = open(os.path.join(ROOT, "index.html"), "r").read()
    return web.Response(content_type="text/html", text=content)

async def javascript(request):      # not needed - seems to work w/ wildcards, no special content type
    content = open(os.path.join(ROOT, "client.js"), "r").read()
    return web.Response(content_type="application/javascript", text=content)

async def offer(request):           # WebRTC request from browser
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    global pc
    pc = RTCPeerConnection()
    pcs.add(pc)

    logger.info(f"PeerConnection created for {request.remote}")

    @pc.on("datachannel")
    def on_datachannel(channel):
        global dc
        logger.debug(f"on_datachannel: was {dc} now {channel}")
        dc = channel

        @channel.on("message")
        def on_message(message):
            if isinstance(message, str):
                dataChannelReceive(message)
            else:
                logger.info(f"on_datachannel - unknown message: {message}")  # wonder what will show up if binary?   :)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        global pc
        if pc is None:
            logger.debug("on_connectionstatechange: pc is None")
            pass
        else:
            logger.debug(f"on_connectionstatechange: {pc.connectionState}")

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        global pc
        if pc is None:
            logger.debug("iceconnectionstatechange: pc is None")
            pass
        else:
            logger.debug(f"iceconnectionstatechange: {pc.iceConnectionState}")
            if pc.iceConnectionState == "failed":
                await pc.close()
                pcs.discard(pc)
                pc = None

    @pc.on("track")
    def on_track(track):
        logger.debug(f"Track {track.kind} received")
        if track.kind == "audio":
            pass
        elif track.kind == "video":
            logger.error("on_track - should not get video track")
            pc.addTrack(VideoStreamTrack())
            logger.debug("Added video track to pc")

        @track.on("ended")
        async def on_ended():
            if track.kind == "audio":
                pass
            elif track.kind == "video":
                logger.info(f"Track {track.kind} ended")
            global dc
            dc = None

    pc.addTrack(VideoStreamTrack())
 
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": (pc.localDescription.type)}
        ),
    )

async def on_shutdown(app):
    global dc
    dc = None
    # close peer connections
    # Not sure why there are more than one?
    # Perhaps this server can support multiple?    Not our use case...
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()
    pc = None


####################################
# WebRTC - DATA CHANNEL interface
####################################

def dataChannelOpen():
    if pc is not None:
        if (pc.connectionState == "connected"):
            logger.debug("dataChannelOpen: dc=%s, readyState=%s" % (dc,dc.readyState))
            if dc.readyState == 'open':
                return True
    return False

async def sendMsg(msg):
    assert dataChannelOpen()
    try:
        logger.debug("sendMsg(%s) %s" % (dc.label, dc))
        dc.send(msg) 
    except Exception as e:
        # print("ERROR: ",e.message, e.args)
        logger.warning("error sending message %s   -   %s" % (e.message, e.args))
        # to do - kill channel?
        # dc = None

def dataChannelSend(jmsg):
    msg = json.dumps(jmsg)
    if dataChannelOpen():
        logger.debug("dataChannelSend(%s) %s" % (dc.label, msg))
        asyncio.run( sendMsg(msg) )
    else:
        pass
        logger.debug("data channel not open-not sending message")

def dataChannelReceive(msg):
    try:
        jmsg = json.loads(msg)
    except:
        logger.warning("error parsing received message: %s" % (msg))
        return
    
    if 'twist' in jmsg:
        # Vector3  linear  -> x,y,z
        # Vector3  angular -> rx,ry,rz (rotation about that axis - z is yaw)
        jTwist = jmsg['twist']
        logger.debug("twist: x=%.2f, rz=%.2f" % (jTwist['linear']['x'],jTwist['angular']['z']))
        # print("twist: x=%.2f, rz=%.2f" % (jTwist['linear']['x'],jTwist['angular']['z']))
        if Ros2PubSubNode is not None:      # should always be valid
            # and send to ROS...
            Ros2PubSubNode.publishTwist(jTwist)
    elif 'watchdog-browser' in jmsg:
        logger.debug("watchdog: %s" % (msg))
        pass
    else:
        logger.warning("unknown message: %s" % (msg))

def watchdog():     # actual watchdog, within it's own thread
    dataChannelSend( {"watchdog-server": int(time.time())} )
    threading.Timer(5, watchdog).start()

# Start watchdog - runs at all times
threading.Timer(5, watchdog).start()



####################################
# ROS2 - publish and subscribe
####################################

jsonTopic = 'teleoppub'
# twistTopic = 'cmd_vel'

# WebRTC node publish/subscribe
class WebRTCPubSub(Node):

    def __init__(self):
        super().__init__('teleopros2')

        global logger
        logger = self.get_logger()

        # parameters
        # some common default topics for images...
        kRealSenseImageTopic = "/camera/color/image_raw"    # 6/24/2025 on jetson
        # kRealSenseImageTopic = "/color/image_raw"    # 6/18/2025 on desktop
        # kIsaacSimImageTopic = "/front/stereo_camera/left/rgb"
        # kIsaacSimImageTopic = "/image_raw"
        kDefaultImageTopic = kRealSenseImageTopic

        self.declare_parameter('twist-topic', 'cmd_vel')
        self.declare_parameter('json-topic', 'teleoppub')
        self.declare_parameter('image-topic', kDefaultImageTopic)
        self.declare_parameter('ssl', True)
        self.declare_parameter('cert-file', 'certs/server.cert')
        self.declare_parameter('key-file', 'certs/server.key')
        # self.declare_parameter('cert-file', 'certs/test.cert')
        # self.declare_parameter('key-file', 'certs/test.key')        
        self.declare_parameter('host', HOST_IP)
        self.declare_parameter('port', 8080)      # Isaac SIM takes 8080...   so in that case use 8081
        self.declare_parameter('fps', 15)           # max video update rate
        self.declare_parameter('verbose', False)

        global argstwisttopic, argsjsontopic, argsimagetopic, argsssl, argscertfile, argskeyfile, argshost, argsport, argsfps, argsverbose
        argstwisttopic = self.get_parameter('twist-topic').value
        argsjsontopic = self.get_parameter('json-topic').value
        argsimagetopic = self.get_parameter('image-topic').value
        argsssl = self.get_parameter('ssl').value
        argscertfile = self.get_parameter('cert-file').value
        argskeyfile = self.get_parameter('key-file').value
        argshost = self.get_parameter('host').value
        argsport = self.get_parameter('port').value
        argsfps = self.get_parameter('fps').value
        argsverbose = self.get_parameter('verbose').value

        self.bridge = CvBridge()

        # subscribers
        logger.info("listening for images on: %s" % (argsimagetopic))
        print("listening for images on: %s" % (argsimagetopic))

        self.imageSubscription = self.create_subscription(Image,argsimagetopic,self.listener_image_callback,10)
        self.imageSubscription  # prevent unused variable warning

        # JSON - exact uses t.b.d.
        self.subscription = self.create_subscription(String,jsonTopic,self.json_listener_callback,10)
        self.subscription  # prevent unused variable warning

        # publishers
        self.jPublisher_ = self.create_publisher(String, argsjsontopic, 10)
        self.twistPublisher_ = self.create_publisher(Twist, argstwisttopic, 10)

        logger.debug('WebRTCPubSub initialized')
        
    def publishTwist(self, jTwist):
        x = float(jTwist['linear']['x'])
        y = float(jTwist['linear']['y'])
        z = float(jTwist['linear']['z'])
        rx = float(jTwist['angular']['x'])
        ry = float(jTwist['angular']['y'])
        rz = float(jTwist['angular']['z'])

        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        twist.angular.x = rx
        twist.angular.y = ry
        twist.angular.z = rz
        self.twistPublisher_.publish(twist)

    def json_listener_callback(self, msg):
        # publish the json to the browser via the data channel
        logger.debug('JSON: I heard: "%s"' % msg.data)

    def listener_image_callback(self, image_message):
        cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        # Defer conversion till needed, seeing as we only use half of the images (15fps use, generated at 30fps)
        # print("caught image")
        curImg.setImg(cv_image)

    def json_pub(self,payload):        
        msg = String()
        msg.data = payload
        self.jsonpublisher_.publish(msg)
        logger.debug('Published: "%s"' % msg.data)

def runROSNode():
     rclpy.spin(Ros2PubSubNode)


####################################
# Main - needed to launch w/ ROS2
####################################

def main():
    global Ros2PubSubNode, logger, argsverbose
    rclpy.init()
    Ros2PubSubNode = WebRTCPubSub()     # do this sequentially, as it captures the parameters

    # Set up Python logger if not running as ROS2 node
    if logger is None:
        import logging
        logger = logging.getLogger("teleopros2")
        if argsverbose:
            logger.basicConfig(level=logging.DEBUG)
        else:
            logger.basicConfig(level=logging.INFO)

    t = threading.Thread(target=runROSNode)
    t.start()

    global VIDEO_PTIME
    VIDEO_PTIME = 1 / argsfps           # use parameterized fps



    if argsssl:
        logger.debug("creating SSL context...")

        certFP = argscertfile
        keyFP = argskeyfile
        if not os.path.isfile(argscertfile):            # either absolute, or relative
            certFP = os.path.join(ROOT,argscertfile)
            keyFP = os.path.join(ROOT,argskeyfile)
        if not os.path.isfile(certFP):
            logger.error("cert file not found: %s" % (certFP))
            return
        if not os.path.isfile(keyFP):
            logger.error("key file not found: %s" % (keyFP))
            return

        ssl_context = ssl.SSLContext()
        ssl_context.load_cert_chain(certFP, keyFP)

    else:
        logger.debug("Not SSL")
        ssl_context = None

    import warnings
    warnings.filterwarnings("ignore")       # , category=DeprecationWarning)

    # print("Starting WebRTC server on %s:%d" % (argshost, argsport))

    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)                  # default - no file specified, use index.html
    # app.router.add_get("/client.js", javascript)  # not needed - seems to not need the mime type text/javascript
    app.router.add_post("/offer", offer)            # startup ICE offer - rest is through data channel
    app.router.add_static('/', path=ROOT, follow_symlinks=True)       # and all else, including subfolders - js, css, png, ico, etc.
    web.run_app(
        app, access_log=None, host=argshost, port=argsport, ssl_context=ssl_context
    )

if __name__ == "__main__":
    main()