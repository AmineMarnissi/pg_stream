#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RTSPStreamPublisher:
    def __init__(self):
        rospy.init_node('rtsp_stream_publisher', anonymous=True)
        self.rtsp_stream_url = rospy.get_param('~rtsp_stream_url', 'rtsp://your_rtsp_stream_url')
        self.frame_rate = rospy.get_param('~frame_rate', 30)
        self.image_pub = rospy.Publisher('rtsp_stream', Image, queue_size=10)
        self.bridge = CvBridge()

    def publish_rtsp_stream(self):
        while not rospy.is_shutdown():
            cap = cv2.VideoCapture(self.rtsp_stream_url)
            if cap.isOpened():
                rospy.loginfo("Opened RTSP stream successfully.")
                break  # Exit loop if stream is opened successfully
            else:
                rospy.logwarn("Failed to open RTSP stream. Retrying...")
                rospy.sleep(1)  # Wait for a second before retrying

        rate = rospy.Rate(self.frame_rate)
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(ros_image)
            rate.sleep()

if __name__ == '__main__':
    try:
        rtsp_publisher = RTSPStreamPublisher()
        rtsp_publisher.publish_rtsp_stream()
    except rospy.ROSInterruptException:
        pass

