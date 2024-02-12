'''
teleopros2 - Browser interface to ROS2 robot
 - Web server - provides simple page with video and robot movement controls
 - Stream video back using WebRTC (thanks to aiortc)
 - Data channel - can send/receive JSON messages which are converted to/from ROS2 messages
 - Focused on supporting the NVidia stack - Jetson (Orin) Nano, Isaac Sim (for simulation)
   (will work elsewhere w/ some modifications)
 - Simplyfing assumptions - to be validated or changed over time.

Open issues:
 - getting the video element to nicely show on mobile
 - https - throws up warning to user given the lack of a valid certificate.   Oh well. 

Limitations:
 - ROS2, Python, NVidia stack
 - Web connectivity to browser (have to play with Autonomous/Manual disconnect/reconnect...)

 *************
Requires https for remote access.   See https://deliciousbrains.com/ssl-certificate-authority-for-local-https-development/#how-it-works
to run...
cd ~/workspaces/isaac_ros-dev/src/TeleOpROS2/teleopros2

python3 teleopros2.py --cert-file certs/server.cert --key-file certs/server.key

https://localhost:8081/testTilt.html
https://192.168.86.220:8081/testTilt.html
https://192.168.86.220:8081/                    -- doesn't work from mobile?
https://192.168.86.220:8081/index.html  

*************
Build instructions: (docker on Jetson (Orin) Nano)

to start nvidia docker:

cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh

to build:

cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-select teleopros2
source install/setup.bash
ros2 run ros2webrtc ros2webrtc

* this tells you the browser URL to use - http://localhost:8080

*then realsense in another terminal*
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py

*test if realsense seems off...*
realsense-viewer

*that, and use a third window to look at published topics - make sure all are there*

to run in simulator - ISAAC SIM

Make sure Nucleus and all that is running - http://localhost:3080/ --> restart all
Start Isaac SIM
(I'm using 'simple room' - needs ROS2 to work...)


*use docker, per above for ROS2*
'''

import argparse
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

# from mediastreams.py
AUDIO_PTIME = 0.020  # 20ms audio packetization
VIDEO_CLOCK_RATE = 90000
VIDEO_PTIME = 1 / 30  # 30fps
VIDEO_PTIME = 1 / 15  # 30fps
# reset given parameters
VIDEO_TIME_BASE = fractions.Fraction(1, VIDEO_CLOCK_RATE)





class MediaStreamError(Exception):
    pass

# aiortc stuff
ROOT = os.path.dirname(__file__)
height, width = 480, 640
logger = logging.getLogger("pc")
pcs = set()     # can support multiple connections - we only care about one
pc = None       # the one
dc = None       # data channel - None until we have one
vst = None      # video stream track - None until we have one
Ros2PubSubNode = None       # set to a valid object when ROS2 node is running

# current image - to be sent to the browser
# either a static image, or the last image received from ROS2
class currentImage():
    def __init__(self):
        self.img = np.zeros((height, width, 3), np.uint8)
        self.resetImg()

    def resetImg(self):
        # self.img[:, :] = (64, 64, 64)        # gray
        self.img[:, :] = (0, 0, 200)         # different - this is red

    def setImg(self, img):
        self.img = img

    def getImg(self):
        return self.img
    
curImg = currentImage()

### send a video frame to the browser
# can set fps
# do transforms as needed (?)
class VideoStreamTrack(MediaStreamTrack):
    """
    A dummy video track which reads green frames.
    """

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
        self.currentImg = img       # hopefully atomically...

    async def recv(self):
        img = curImg.getImg()
        frame = VideoFrame.from_ndarray(img,format="bgr24")
        pts, time_base = await self.next_timestamp()
        frame.pts = pts
        frame.time_base = time_base
        # logging.debug("sending frame")
        return frame

    def __del__(self):
        logging.info("VideoStreamTrack deleted")

## Web server
async def index(request):
    # logging.info("index.html request")
    content = open(os.path.join(ROOT, "index.html"), "r").read()
    # content = open(os.path.join(ROOT, "index-1.html"), "r").read()
    return web.Response(content_type="text/html", text=content)

async def javascript(request):      # not used - seems to work w/ wildcards, no special content type
    content = open(os.path.join(ROOT, "client.js"), "r").read()
    return web.Response(content_type="application/javascript", text=content)

async def offer(request):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    global pc
    pc = RTCPeerConnection()
    pcs.add(pc)

    # Tried to add here instead of in on_track - didn't work
    # pc.addTrack(VideoStreamTrack())

    logger.info("PeerConnection created for %s", request.remote)

    # prepare local media
    # player = MediaPlayer(os.path.join(ROOT, "demo-instruct.wav"))
    # if args.write_audio:
    #     recorder = MediaRecorder(args.write_audio)
    # else:
    #     recorder = MediaBlackhole()


    @pc.on("datachannel")
    def on_datachannel(channel):
        global dc
        logger.debug("on_datachannel: was %s now %s" % (dc,channel))
        dc=channel

        @channel.on("message")
        def on_message(message):
            if isinstance(message, str):
                dataChannelReceive(message)
            else:
                print(">>> unknown message??: ",message)    # not sure what would print?

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        global pc
        if pc is None:
            # logger.info("on_connectionstatechange: pc is None")
            pass
        else:
            logger.info("on_connectionstatechange: %s", pc.connectionState)


    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        global pc
        if pc is None:
            # logger.info("iceconnectionstatechange: pc is None")
            pass
        else:
            logger.info("iceconnectionstatechange: %s", pc.iceConnectionState)
            if pc.iceConnectionState == "failed":
                await pc.close()
                pcs.discard(pc)
                pc = None

    @pc.on("track")
    def on_track(track):
        logger.info("Track %s received", track.kind)

        if track.kind == "audio":
            pass
            # pc.addTrack(player.audio)     # not used
            # recorder.addTrack(track)      # not used
        elif track.kind == "video":
            # add our locally generated video track here...
            pc.addTrack(VideoStreamTrack())
            logger.info("Added video track to pc")


        @track.on("ended")
        async def on_ended():
            if track.kind == "audio":
                pass
            elif track.kind == "video":
                logger.info("Track %s ended", track.kind)

            global dc
            dc = None       # data channel no longer valid

            # await recorder.stop()       # not used - this is MediaBlackHole

    # logger.info("before setRemoteDescription")
    # handle offer
    await pc.setRemoteDescription(offer)
    # await recorder.start()              # not used - this is MediaBlackHole

    # logger.info("manually adding VideoStreamTrack")

    pc.addTrack(VideoStreamTrack())
 
    # send answer
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )

'''
GOOD
INFO:pc:PeerConnection created for 192.168.86.220
INFO:pc:Track video received
INFO:pc:XXXXXXXXXXXXXXXX    track.kind == video
INFO:aioice.ice:Connection(0) Check CandidatePair(('192.168.86.220', 45126) -> ('192.168.86.220', 44607)) State.FROZEN -> State.WAITING
INFO:aioice.ice:Connection(0) Check CandidatePair(('172.17.0.1', 53593) -> ('192.168.86.220', 44607)) State.FROZEN -> State.WAITING
INFO:pc:iceconnectionstatechange: checking
INFO:pc:on_connectionstatechange: connecting
INFO:aioice.ice:Connection(0) Check CandidatePair(('192.168.86.220', 45126) -> ('192.168.86.220', 44607)) State.WAITING -> State.IN_PROGRESS
INFO:aioice.ice:Connection(0) Check CandidatePair(('192.168.86.220', 45126) -> ('192.168.86.220', 44607)) State.IN_PROGRESS -> State.SUCCEEDED
INFO:aioice.ice:Connection(0) Check CandidatePair(('172.17.0.1', 53593) -> ('192.168.86.220', 44607)) State.WAITING -> State.FAILED
INFO:aioice.ice:Connection(0) ICE completed
INFO:pc:iceconnectionstatechange: completed
INFO:pc:on_connectionstatechange: connected
'''

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
# ROS2 - publish and subscribe
####################################

jsonTopic = 'teleoppub'
twistTopic = 'cmd_vel'

# Arguments
argsimagetopic = None
argsssl= False
argscertfile = None
argskeyfile = None
argshost = None
argsport = None
argsfps = None
argsverbose = False

# WebRTC node publish/subscribe
class WebRTCPubSub(Node):

    def __init__(self):
        super().__init__('pywebrtc')

        # parameters
        # Isaac SIM - "/front/stereo_camera/left/rgb"
        # Realsense - "/camera/color/image_raw"
        self.declare_parameter('image-topic', "/camera/color/image_raw")
        self.declare_parameter('ssl', True)
        self.declare_parameter('cert-file', 'certs/server.cert')
        self.declare_parameter('key-file', 'certs/server.key')
        self.declare_parameter('host', "0.0.0.0")
        self.declare_parameter('port', 8080)      # Isaac SIM takes 8080...   so in that case use 8081
        self.declare_parameter('fps', 15)           # max video update rate
        self.declare_parameter('verbose', False)

        global argsimagetopic, argsssl, argscertfile, argskeyfile, argshost, argsport, argsfps, argsverbose
        argsimagetopic = self.get_parameter('image-topic').value
        argsssl = self.get_parameter('ssl').value
        argscertfile = self.get_parameter('cert-file').value
        argskeyfile = self.get_parameter('key-file').value
        argshost = self.get_parameter('host').value
        argsport = self.get_parameter('port').value
        argsfps = self.get_parameter('fps').value
        argsverbose = self.get_parameter('verbose').value

        # print("argsimagetopic: ",argsimagetopic)
        # print("argsssl: ",argsssl)
        # print("argscertfile: ",argscertfile)
        # print("argskeyfile: ",argskeyfile)
        # print("argshost: ",argshost)
        # print("argsport: ",argsport)
        # print("argsfps: ",argsfps)
        # print("argsverbose: ",argsverbose)

        self.bridge = CvBridge()

        # subscriber - t.b.d.
        self.subscription = self.create_subscription(String,jsonTopic,self.json_listener_callback,10)
        self.subscription  # prevent unused variable warning

        self.imageSubscription = self.create_subscription(Image,argsimagetopic,self.listener_image_callback,10)
        self.imageSubscription  # prevent unused variable warning

        # publishers
        self.jPublisher_ = self.create_publisher(String, jsonTopic, 10)
        self.twistPublisher_ = self.create_publisher(Twist, twistTopic, 10)

        logger.info('WebRTCPubSub initialized')
        
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
        logger.debug("twist: x=%f, y=%f, z=%f, rx=%f, ry=%f, rz=%f" % (x,y,z,rx,ry,rz))

    def json_listener_callback(self, msg):
        # publish the json to the browser via the data channel
        logger.info('JSON: I heard: "%s"' % msg.data)

    def listener_image_callback(self, image_message):

        # we could defer conversion till needed, seeing as we only use half of the images (15fps use, generated at 30fps)

        cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')

        ## Conversion - perhaps this can be done on the GPU using NVidia stack on Jetson Orin...
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # check to see if need to resize
        # print("ros image shape: ",cv_image.shape)
        cv_image = cv2.resize(cv_image, (width, height))

        # and push to the object holding the current image
        curImg.setImg(cv_image)
        # logger.info('Rcvd new image')

    def json_pub(self,payload):        
        msg = String()
        msg.data = payload
        self.jsonpublisher_.publish(msg)
        logger.info('Published: "%s"' % msg.data)

def runROSNode(args=None):
    # global Ros2PubSubNode
    # rclpy.init(args=args)
    # Ros2PubSubNode = WebRTCPubSub()
    rclpy.spin(Ros2PubSubNode)




### hmmm, perhaps something stronger than this?
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
        logger.warning("error parsing received message" + msg)
        return
    
    if 'twist' in jmsg:
        # Vector3  linear  -> x,y,z
        # Vector3  angular -> rx,ry,rz (rotation about that axis - z is yaw)
        jTwist = jmsg['twist']
        logger.info("twist: x=%f, rz=%f" % (jTwist['linear']['x'],jTwist['angular']['z']))
        if Ros2PubSubNode is not None:      # should always be valid
            Ros2PubSubNode.publishTwist(jTwist)
        # and send to ROS...
    elif 'watchdog-browser' in jmsg:
        pass
        logger.debug("watchdog:" + msg)
    else:
        logger.warning("unknown message:" + msg)

def watchdog():     # actual watchdog, within it's own thread
    dataChannelSend( {"watchdog-server": int(time.time())} )
    threading.Timer(5, watchdog).start()

# Start watchdog - runs at all times
threading.Timer(5, watchdog).start()



# Need main function to get launched by ROS2
def main():
    global Ros2PubSubNode
    rclpy.init()
    Ros2PubSubNode = WebRTCPubSub()     # do this sequentially, as it captures the parameters
    t = threading.Thread(target=runROSNode)
    t.start()

    global VIDEO_PTIME
    VIDEO_PTIME = 1 / argsfps           # use parameterized fps

    # parser = argparse.ArgumentParser(
    #     description="WebRTC audio / video / data-channels demo"
    # )
    # parser.add_argument("--cert-file", help="SSL certificate file (for HTTPS)")
    # parser.add_argument("--key-file", help="SSL key file (for HTTPS)")
    # parser.add_argument(
    #     "--host", default="0.0.0.0", help="Host for HTTP server (default: 0.0.0.0)"
    # )
    # parser.add_argument(
    #     # Isaac SIM takes 8080...   so in that case use 8081
    #     "--port", type=int, default=8080, help="Port for HTTP server (default: 8080)"
    # )
    # parser.add_argument("--verbose", "-v", action="count")
    # parser.add_argument("--write-audio", help="Write received audio to a file")
    # args = parser.parse_args()

    if argsverbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    if argsssl:
        logging.info("creating SSL context...")
        ssl_context = ssl.SSLContext()
        ssl_context.load_cert_chain(argscertfile, argskeyfile)
    else:
        # logging.info("Not SSL")
        ssl_context = None

    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)                  # default - no file specified, use index.html
    # app.router.add_get("/client.js", javascript)  # not used - seems to not need the mime type text/javascript
    app.router.add_post("/offer", offer)            # startup ICE offer - rest is through data channel
    app.router.add_static('/', path=ROOT, follow_symlinks=True)       # and all else, including subfolders - js, css, png, ico, etc.
    web.run_app(
        app, access_log=None, host=argshost, port=argsport, ssl_context=ssl_context
    )

if __name__ == "__main__":
    main()