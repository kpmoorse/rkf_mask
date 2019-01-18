import rosbag
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm
import numpy as np
from time import time

def rosbag_rmtether(fin, pause=False, fps=30):

    # Open bag file and initialize CvBridge
    bag = rosbag.Bag(fin, "r")
    bridge = CvBridge()
    while True:
        for _, msg, _ in tqdm(bag, total=bag.get_message_count()):

            tframe = time()

            # Load image from rosbag message
            img = bridge.imgmsg_to_cv2(msg).copy()

            # Find tether polygon
            lsd = cv2.createLineSegmentDetector()
            ly, lx = img.shape
            lines = lsd.detect(img[:int(ly/2), int(lx/3):int(2*lx/3)])
            d = np.sqrt((lines[0][:, 0, 3] - lines[0][:, 0, 1])**2 + (lines[0][:, 0, 3] - lines[0][:, 0, 1])**2)
            edges = lines[0][np.argsort(d)[-2:]]
            points = edges.copy().reshape(-1, 2)
            points[:, 0] += lx/3

            # Extract non-artifact region
            nzx = sum(img[-1, :] != 0)
            nzy = sum(img[:, 1] != 0)
            img_tmp = img[-nzy:, :nzx]

            # Generate tether mask and dilate
            mask = img_tmp * 0
            cv2.fillPoly(mask, [points.astype('int32')], 255)
            mask = cv2.dilate(mask, np.ones((21, 15)))

            # Inpaint over tether mask
            img[-nzy:, :nzx] = cv2.inpaint(img[:nzy, :nzx], mask, 10, cv2.INPAINT_TELEA)
            cv2.imshow('image', img)

            if pause:
                cv2.waitKey(0)
            else:
                t2 = time()
                cv2.waitKey(max(1, int((1./fps - (t2-tframe))*1000)))

        cv2.destroyAllWindows()


# Restrict values to (-pi, pi]
def pi_mod(theta):
    return (theta + np.pi) % (2*np.pi) - np.pi


rosbag_rmtether('rkf_test01.bag', pause=False, fps=30)
