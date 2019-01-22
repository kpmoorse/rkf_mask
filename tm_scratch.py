import rosbag
from tethermask import TetherMask
from rosbagstream import RosbagStream
from cvdisplay import CvDisplay

# Load bag file and pass to TetherMask class
# tm = TetherMask()
# bag = rosbag.Bag('rkf_test01.bag', 'r')
# tm.masked_playback(bag, 30)

if __name__ == "__main__":

    # Init stream, mask, and display nodes
    rbs = RosbagStream('rkf_test01.bag')
    tm = TetherMask()
    cvd = CvDisplay()

    # Start image stream
    rbs.img_stream(loop=False)

