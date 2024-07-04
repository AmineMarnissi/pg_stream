#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

class RTSPStreamPublisher:
    def __init__(self, rtsp_stream_urls, topic_names, frame_rate=30):
        self.rtsp_stream_urls = rtsp_stream_urls
        self.topic_names = topic_names
        self.frame_rate = frame_rate
        self.publishers = []
        self.bridge = CvBridge()

        for topic in self.topic_names:
            publisher = rospy.Publisher(topic, Image, queue_size=10)
            self.publishers.append(publisher)

    def publish_rtsp_stream(self, url, publisher):
        cap = cv2.VideoCapture(url)
        if not cap.isOpened():
            rospy.logerr(f"Failed to open RTSP stream: {url}")
            return
        
        rospy.loginfo(f"Opened RTSP stream successfully: {url}")

        rate = rospy.Rate(self.frame_rate)
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                publisher.publish(ros_image)
            else:
                rospy.logwarn(f"No frame received from RTSP stream: {url}")
                break  # Exit loop if no frame received
            
            rate.sleep()

        cap.release()

    def start_publishing(self):
        threads = []
        for url, publisher in zip(self.rtsp_stream_urls, self.publishers):
            thread = threading.Thread(target=self.publish_rtsp_stream, args=(url, publisher))
            thread.start()
            threads.append(thread)

        for thread in threads:
            thread.join()

if __name__ == '__main__':
    try:
        rospy.init_node('rtsp_stream_publisher', anonymous=True)
        
        # Read RTSP stream URLs and topic names from ROS parameters
        rtsp_stream_urls = rospy.get_param('~rtsp_stream_urls', [])
        topic_names = rospy.get_param('~topic_names', [])
        
        if len(rtsp_stream_urls) != len(topic_names):
            rospy.logerr("Number of RTSP stream URLs and topic names must be the same.")
            exit(1)

        rtsp_publisher = RTSPStreamPublisher(rtsp_stream_urls, topic_names)
        rtsp_publisher.start_publishing()
        
    except rospy.ROSInterruptException:
        pass
