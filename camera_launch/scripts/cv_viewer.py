#!/usr/bin/env python3
"""OpenCV-based image viewer for ROS."""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys


class CvViewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.window_name = rospy.get_param("~window_name", "Camera View")
        image_topic = rospy.get_param("~image", "/camera/image_raw")

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        rospy.Subscriber(image_topic, Image, self.image_callback)
        rospy.loginfo(f"Subscribing to: {image_topic}")

    def image_callback(self, msg):
        try:
            if msg.encoding == "rgb8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)

            cv2.imshow(self.window_name, cv_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # q or ESC
                rospy.signal_shutdown("User requested shutdown")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()


def main():
    rospy.init_node("cv_viewer", anonymous=True)
    viewer = CvViewer()
    viewer.run()


if __name__ == "__main__":
    main()
