import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from aiortc import VideoStreamTrack
from av import VideoFrame
import logging

# Configurer la journalisation
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

class VideoStreamTrackFromROS(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.frame = None

    def update_frame(self, frame):
        self.frame = frame
        logger.debug("Frame updated")

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        if self.frame is None:
            logger.warning("Received None frame, returning a blank frame")
            return VideoFrame.from_ndarray(np.zeros((480, 640, 3), np.uint8), format='bgr24')

        logger.debug("Converting frame to VideoFrame")
        video_frame = VideoFrame.from_ndarray(self.frame, format='bgr24')
        video_frame.pts = pts
        video_frame.time_base = time_base

        logger.debug(f"Sending frame with PTS: {pts} and time_base: {time_base}")
        return video_frame

class ROSVideoStream(Node):
    def __init__(self, video_track):
        super().__init__('ros_video_stream')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            lambda msg: self.ros_image_callback(msg, video_track),
            10
        )

    def ros_image_callback(self, msg, video_track):
        logger.debug("Received image from ROS topic")
        
        try:
            # Utiliser cv_bridge pour convertir le message ROS en image OpenCV
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            logger.debug(f"Converted image shape: {image.shape}")

            if image is None:
                logger.warning("Failed to decode image from ROS message")
                return

            video_track.update_frame(image)
            logger.debug("Updated frame from ROS image callback")
        except Exception as e:
            logger.error(f"Error converting ROS image message to OpenCV: {e}")

