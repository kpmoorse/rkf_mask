import rosbag
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm


def rosbag_backsub(fin, pause=False):

    bag = rosbag.Bag(fin, "r")
    bridge = CvBridge()
    backSub = cv2.createBackgroundSubtractorKNN(detectShadows=False)

    for _, msg, _ in tqdm(bag, total=bag.get_message_count()):
        cv_img = bridge.imgmsg_to_cv2(msg)
        fgMask = backSub.apply(cv_img)
        cv2.imshow('image', fgMask)
        if pause:
            cv2.waitKey(0)
        else:
            cv2.waitKey(20)
    cv2.destroyAllWindows()


rosbag_backsub('rkf_test01.bag', pause=True)
