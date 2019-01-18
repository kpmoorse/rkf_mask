import rosbag
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm
import imutils
import numpy as np


def rosbag_rmtether(fin, pause=False):

    bag = rosbag.Bag(fin, "r")
    bridge = CvBridge()

    init = True
    for _, msg, _ in tqdm(bag, total=bag.get_message_count()):

        img = bridge.imgmsg_to_cv2(msg).copy()

        if init:

            # Extract and sharpen stabilization border
            lower = 0
            upper = 1
            edgeMask = cv2.inRange(img, lower, upper)
            edgeMask = cv2.dilate(edgeMask, np.ones((3, 3)))
            img_tmp = cv2.bitwise_and(img, img, mask=255 - edgeMask)

            nzx = sum(img[-1, :] != 0)
            nzy = sum(img[:, 1] != 0)
            img_tmp = img[-nzy:, :nzx]

            # Extract foreground (dark)
            lower = 1
            upper = 160
            fgMask = cv2.inRange(img_tmp, lower, upper)

            # Calculate inpainted background image
            bg = cv2.inpaint(img_tmp, cv2.dilate(fgMask, np.ones((20, 20))), 15, cv2.INPAINT_TELEA)

            init = False

        # Extract tether polygon
        lsd = cv2.createLineSegmentDetector()
        ly, lx = img.shape
        lines = lsd.detect(img[:int(ly/2), int(lx/3):int(2*lx/3)])
        d = np.sqrt((lines[0][:, 0, 3] - lines[0][:, 0, 1])**2 + (lines[0][:,0,3] - lines[0][:,0,1])**2)
        edges = lines[0][np.argsort(d)[-2:]]
        points = edges.copy().reshape(-1, 2)
        points[:, 0] += lx/3

        # Convert poly to mask and dilate
        mask = bg * 0
        cv2.fillPoly(mask, [points.astype('int32')], 255)
        mask = cv2.dilate(mask, np.ones((3, 11)))

        # Apply bg tether mask and show
        img[:nzy, :nzx][mask == 255] = bg[mask == 255]
        cv2.imshow('image', img)

        if pause:
            cv2.waitKey(0)
        else:
            cv2.waitKey(20)

    cv2.destroyAllWindows()


# Restrict values to (-pi, pi]
def pi_mod(theta):
    return (theta + np.pi) % (2*np.pi) - np.pi


rosbag_rmtether('rkf_test01.bag', pause=False)
