import cv2
from cv_bridge import CvBridge
import numpy as np
from time import time


class TetherMask(object):

    def __init__(self):
        self.bridge = CvBridge()

    # Extract message contents and convert to image
    def msg_to_img(self, msg):

        imgmsg = msg[1]
        img = self.bridge.imgmsg_to_cv2(imgmsg).copy()
        return img

    # Find and mask rectangular tether in image
    def tmask(self, img):

        # Find tether polygon
        lsd = cv2.createLineSegmentDetector()
        ly, lx = img.shape
        lines = lsd.detect(img[:int(ly / 2), int(lx / 3):int(2 * lx / 3)])
        d = np.sqrt((lines[0][:, 0, 3] - lines[0][:, 0, 1]) ** 2 + (lines[0][:, 0, 3] - lines[0][:, 0, 1]) ** 2)
        edges = lines[0][np.argsort(d)[-2:]]
        points = edges.copy().reshape(-1, 2)
        points[:, 0] += lx / 3

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

        return img

    # Display bag contents as video with tether mask applied
    def masked_playback(self, bag, framelock=30):

        for msg in bag:

            if framelock > 0: tframe = time()
            img = self.msg_to_img(msg)
            img = self.tmask(img)

            cv2.imshow('image', img)
            if framelock > 0:
                cv2.waitKey(max(1, int((1. / framelock - (time() - tframe)) * 1000)))
            else: cv2.waitKey(10)
