import rosbag
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm


def rosbag_playback(fin, pause=False):

    bag = rosbag.Bag(fin, "r")
    bridge = CvBridge()

    for _, msg, _ in tqdm(bag, total=bag.get_message_count()):
        cv_img = bridge.imgmsg_to_cv2(msg)
        cv2.imshow('image', cv_img)
        if pause:
            cv2.waitKey(0)
        else:
            cv2.waitKey(20)
    cv2.destroyAllWindows()


rosbag_playback('rkf_test01.bag')
