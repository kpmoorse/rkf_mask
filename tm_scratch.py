import rosbag
from tethermask import TetherMask
from rosbagstream import RosbagStream

# Load bag file and pass to TetherMask class
# tm = TetherMask()
# bag = rosbag.Bag('rkf_test01.bag', 'r')
# tm.masked_playback(bag, 30)

if __name__ == "__main__":

    rbs = RosbagStream('rkf_test01.bag')
    tm = TetherMask()
    rbs.img_stream()

