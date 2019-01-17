import rosbag
import cv2
from cv_bridge import CvBridge


def play_bag(fin, pause=False):
    bag = rosbag.Bag(fin, "r")
    bridge = CvBridge()
    for _, msg, _ in bag:
        cv_img = bridge.imgmsg_to_cv2(msg)
        cv2.imshow('image', cv_img)
        if pause:
            cv2.waitKey(0)
        else:
            cv2.waitKey(20)
